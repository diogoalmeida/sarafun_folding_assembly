#include <folding_assembly_controller/controller.hpp>

foldingController::foldingController()
{
  v1_ = Eigen::Vector3d::Zero();
  w1_ = Eigen::Vector3d::Zero();
  pc_ = Eigen::Vector3d::Zero();
  p1_ = Eigen::Vector3d::Zero();
  f2_ = Eigen::Vector3d::Zero();
  t2_ = Eigen::Vector3d::Zero();
  fRef_ = 0.0;
  thetaC_ = M_PI;

  estimation_type_ = NO_ESTIMATION;
  force_control_type_ = NO_FORCE_CONTROL;
  estimator_.initialize(p1_);

  n_ = ros::NodeHandle("~");
  getParams();

  monitorPub_ = n_.advertise<folding_assembly_controller::monitorMsg>("foldingController/signals", 1);
  // Subscribe to ft sensor
  wrench_sub_ = n_.subscribe(wrench_topic_name_, 1, &foldingController::wrenchCallback, this);
}

/*
  Computes the skew-symmetric matrix of the provided vector
*/
Eigen::Matrix3d foldingController::computeSkewSymmetric(Eigen::Vector3d v)
{
  Eigen::Matrix3d S;

  S << 0,    -v(2),  v(1),
       v(2),  0   , -v(0),
      -v(1),  v(0),  0;

  return S;
}

void foldingController::control(const double &vd, const double &wd, const double &contact_force, const double &final_angle, Eigen::Vector3d &vOut, Eigen::Vector3d &wOut, const double d_t)
{
  tf::StampedTransform ft_sensor_transform;
  Eigen::Matrix3d S;
  Eigen::Vector3d rotationAxis, omegaD, velD;

  dt_ = d_t;
  fRef_ = contact_force;

  // Need to be able to get surface tangent and normal. These are defined w.r.t the sensor frame
  try{
    tf_listener_.lookupTransform(ft_sensor_frame_name_, base_frame_, ros::Time(0), ft_sensor_transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("Transform exception when trying to get the sensor frame: %s", ex.what());
    ros::shutdown();
  }

  tf::transformTFToKDL(ft_sensor_transform, ft_sensor_frame_);
  tf::vectorKDLToEigen(ft_sensor_frame_.M.UnitX(), surfaceTangent_);
  tf::vectorKDLToEigen(ft_sensor_frame_.M.UnitZ(), surfaceNormal_);
  tf::vectorKDLToEigen(ft_sensor_frame_.p, p2_); // "surface piece end-effector"

  updateContactPoint();
  updateTheta();
  orientation_error_ = final_angle - thetaC_;

  rotationAxis = surfaceTangent_.cross(surfaceNormal_);

  omegaD = wd*saturateAngle(orientation_error_)*rotationAxis;
  velD = vd*surfaceTangent_;

  // HACK: Due to calibration mismatches between the measured sensor frame and
  //       the actual one, the ft measurements might not be exactly correct.
  //       We can measure the offset in the world frame and add it to the contact
  //       point estimate
  pc_ = pc_ + ft_sensor_measured_offset_;

  w1_ = omegaD;

  r1_ = pc_ - p1_;

  vf_ = computeForceControl();

  S = computeSkewSymmetric(w1_);
  v1_ = - S * r1_ + velD + vf_;

  if (abs(v1_.dot(surfaceTangent_)) > saturationV_)
  {
    v1_ = v1_.normalized() * saturationV_;
    ROS_WARN("V1 IS SATURATED");
  }

  if (w1_.norm() > saturationW_)
  {
    w1_ = w1_.normalized() * saturationW_;
    ROS_WARN("OMEGA1 IS SATURATED");
  }

    vOut = v1_;
    wOut = w1_;
}

/*
  Sets the current contact point estimate to the initial guess
*/
void foldingController::resetEstimate()
{
  estimator_.reset(p1_);
}

/*
  Return x < 1 for |error| < breaking_error and 1 otherwise
*/
double foldingController::saturateAngle(double error)
{
  if (breaking_error_ == 0) // disabled
  {
    return 1;
  }

  if (std::abs(error) > breaking_error_)
  {
    return 1;
  }

  return std::abs(error/breaking_error_);
}

/*
  Gives the current contact point and angle with surface estimates
*/
void foldingController::getEstimates(Eigen::Vector3d &pc, double &thetac, double &theta_error, KDL::Frame &pc_frame)
{
  pc = pc_;
  thetac = thetaC_;
  pc_frame = pc_frame_;
  theta_error = orientation_error_;
}

/* Computes the velocity of the end-effector along the surface normal,
   which is a result of a force control component

   The controller is assuming that the tangent direction points in the direction
   where we will apply forces
*/
Eigen::Vector3d foldingController::computeForceControl()
{
  // Requires access to sensor force measurements
  double v_f, force_norm, force_error;
  Eigen::Vector3d forceDirection;

  switch(force_control_type_)
  {
    case NO_FORCE_CONTROL:
      forceDirection = Eigen::Vector3d::Zero();
      break;
    case NORMAL_FORCE_CONTROL:
      forceDirection = surfaceNormal_;
      break;
    case TANGENTIAL_FORCE_CONTROL:
      forceDirection = -surfaceTangent_;
      break;
    case ROD_FORCE_CONTROL:
      forceDirection = r1_/r1_.norm(); //The robot will apply force in the direction of the rod piece
      break;
    case DEBUG_ROD_FORCE_CONTROL:
      tf::vectorKDLToEigen(eef_frame_.M.UnitZ(), forceDirection); // uses the direction of the end-effector rather than the estimate
      break;
  }
  // Eigen::Vector3d forceDirection = surfaceNormal_; // The robot will apply force in the direction of the rod piece

  force_norm = f2_.norm();
  force_error = fRef_ - force_norm;

  v_f = kf_ * force_error;

  // ROS_INFO("VF: %.5f", v_f);
  // ROS_INFO("forceDirection: %.5f, %.5f, %.5f", forceDirection(0), forceDirection(1), forceDirection(2));

  return v_f*forceDirection;
}

/*
  Computes the angle between rod and surface parts
*/
void foldingController::updateTheta()
{
  thetaC_ = atan2(r1_.dot(surfaceNormal_), r1_.dot(surfaceTangent_));
}

/*
  Uses force and torque information to update the contact point estimate
*/
void foldingController::updateContactPoint()
{
  Eigen::Vector3d pc_temp;
  KDL::Vector pc_kdl;
  switch(estimation_type_)
  {
    case NO_ESTIMATION:
      // code for having a pre-determined contact point
      {
        KDL::Vector translational_direction = eef_frame_.M.UnitZ();
        pc_kdl = eef_frame_.p + known_pc_distance_*translational_direction;
        pc_frame_.M = eef_frame_.M;
        pc_frame_.p = pc_kdl;
        tf::vectorKDLToEigen(pc_kdl, pc_);
      }
      break;
    case DIRECT_COMPUTATION:
      pc_temp = -t2_(1)/f2_(2)*surfaceNormal_; // This is in the sensor frame
      tf::vectorEigenToKDL(pc_temp, pc_kdl);
      pc_kdl = ft_sensor_frame_.Inverse()*pc_kdl;
      pc_frame_.p = pc_kdl;
      pc_frame_.M = eef_frame_.M;
      tf::vectorKDLToEigen(pc_kdl, pc_);
      break;
    case KALMAN_FILTER:
      pc_temp = estimator_.estimate(measured_v1_, measured_w1_, f2_, t2_, p1_, p2_, dt_); // This is in the sensor frame
      tf::vectorEigenToKDL(pc_temp, pc_kdl);
      pc_kdl = ft_sensor_frame_.Inverse()*pc_kdl;
      pc_frame_.p = pc_kdl;
      pc_frame_.M = eef_frame_.M; // TODO: Make function of computed thetaC_
      tf::vectorKDLToEigen(pc_kdl, pc_);
      break;
  }
}

/*
  Loads parameters from the parameter server
*/
bool foldingController::getParams()
{
  if(!n_.getParam("/folding_controller/saturation/v", saturationV_))
  {
    ROS_WARN("No saturation value set for the linear velocity! Using default (/folding_controller/saturation/v)");
    saturationV_ = 0.01;
  }

  if(!n_.getParam("/folding_controller/saturation/w", saturationW_))
  {
    ROS_WARN("No saturation value set for the angular velocity! Using default (/folding_controller/saturation/w)");
    saturationW_ = 0.01;
  }

  if(!n_.getParam("/folding_controller/gains/kf", kf_))
  {
    ROS_WARN("No force control gain set! Will set to zero (/folding_controller/gains/kf)");
    kf_ = 0.0;
  }

  if(!n_.getParam("/config/ft_sensor_topic", wrench_topic_name_))
  {
    ROS_WARN("FT sensor topic name not defined! Will set to default (/config/ft_sensor_topic)");
    wrench_topic_name_ = std::string("/ft_sensor");
  }

  if(!n_.getParam("/folding_controller/known_pc_distance", known_pc_distance_))
  {
    ROS_WARN("Hardcoded contact point distance not defined! Will set to zero (/folding_controller/known_pc_distance)");
    known_pc_distance_ = 0.0;
  }

  if(!n_.getParam("/folding_controller/base_frame", base_frame_))
  {
    ROS_WARN("Base frame name not given! Will set to /base_frame (/folding_controller/base_frame)");
    base_frame_ = "/base_frame";
  }

  if(!n_.getParam("/folding_controller/ft_frame", ft_sensor_frame_name_))
  {
    ROS_WARN("Sensor frame name not given! Will set to /ft (/folding_controller/ft_frame)");
    ft_sensor_frame_name_ = "/ft";
  }

  std::vector<double> sensor_offset(3,0.0);
  if(!n_.getParam("/folding_controller/ft_sensor_offset", sensor_offset))
  {
    ROS_WARN("No sensor offset defined! Will use zero (/folding_controller/ft_sensor_offset)");
    ft_sensor_measured_offset_ << 0, 0, 0;
  }
  else
  {
    if(sensor_offset.size() != 3)
    {
      ROS_WARN("Sensor offset has an incorrect dimension (should be 3)");
      ft_sensor_measured_offset_ << 0, 0, 0;
    }
    else
    {
      ft_sensor_measured_offset_ << sensor_offset[0], sensor_offset[1], sensor_offset[2];
    }
  }

  if(!n_.getParam("/folding_controller/breaking_error", breaking_error_))
  {
    ROS_WARN("Missing breaking error (/folding_controller/breaking_error)");
    breaking_error_ = M_PI/8;
  }

  if(breaking_error_ == 0)
  {
    ROS_WARN("Breaking error set to 0! This will disable angular feedback");
  }
}

/*
  Updates the available wrench values
*/
void foldingController::wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  Eigen::Matrix<double, 6, 1> wrench_eigen;

  tf::wrenchMsgToEigen(msg->wrench, wrench_eigen);

  f2_ = wrench_eigen.block<3,1>(0,0);
  t2_ = wrench_eigen.block<3,1>(3,0);
}

/*
  Gets the current end-effector position and velocities computed from KDL
*/
void foldingController::updateState(const KDL::Frame p1_kdl, const Eigen::MatrixXd measured_twist_eig)
{
  eef_frame_ = p1_kdl;
  tf::vectorKDLToEigen(p1_kdl.p, p1_);
  measured_v1_ = measured_twist_eig.block<3,1>(0,0);
  measured_w1_ = measured_twist_eig.block<3,1>(3,0);
}

//////////  Methods to assist debugging the controller /////////////

void foldingController::disableEstimate()
{
  estimation_type_ = NO_ESTIMATION;
}

void foldingController::enableDirectEstimate()
{
  estimation_type_ = DIRECT_COMPUTATION;
}

void foldingController::enableKF()
{
  estimation_type_ = KALMAN_FILTER;
}

void foldingController::disableForceControl()
{
  force_control_type_ = NO_FORCE_CONTROL;
}

void foldingController::normalForceControl()
{
  force_control_type_ = NORMAL_FORCE_CONTROL;
}

void foldingController::tangentForceControl()
{
  force_control_type_ = TANGENTIAL_FORCE_CONTROL;
}

void foldingController::rodForceControl()
{
  force_control_type_ = ROD_FORCE_CONTROL;
}

void foldingController::debugRodForceControl()
{
  force_control_type_ = DEBUG_ROD_FORCE_CONTROL;
}
