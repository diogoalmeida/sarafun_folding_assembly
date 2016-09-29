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

void foldingController::control(const double &vd, const double &wd, const double &contact_force, Eigen::Vector3d &vOut, Eigen::Vector3d &wOut, const double d_t)
{
  Eigen::Matrix3d S;
  Eigen::Vector3d rotationAxis, omegaD, velD;

  dt_ = d_t;
  fRef_ = contact_force;

  // Need to be able to get surface tangent and normal
  surfaceTangent_ << 1, 0, 0;
  surfaceNormal_ << 0, 0, 1;

  // updateSurfaceTangent();
  // updateSurfaceNormal();
  updateTheta();

  rotationAxis = surfaceTangent_.cross(surfaceNormal_);

  omegaD = wd*rotationAxis;
  velD = vd*surfaceTangent_;

  updateContactPoint();

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
  Gives the current contact point and angle with surface estimates
*/
void foldingController::getEstimates(Eigen::Vector3d &pc, double &thetac, KDL::Frame &pc_frame)
{
  pc = pc_;
  thetac = thetaC_;
  pc_frame = pc_frame_;
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
      forceDirection = -cos(thetaC_)*surfaceTangent_ + sin(thetaC_)*surfaceNormal_; // The robot will apply force in the direction of the rod piece
      break;
  }
  // Eigen::Vector3d forceDirection = surfaceNormal_; // The robot will apply force in the direction of the rod piece

  force_norm = f2_.norm();
  force_error = fRef_ - force_norm;

  v_f = -kf_ * force_error;

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
  switch(estimation_type_)
  {
    case NO_ESTIMATION:
      // code for having a pre-determined contact point
      {
        KDL::Vector translational_direction = eef_frame_.M.UnitZ();
        KDL::Vector pc_kdl = eef_frame_.p + known_pc_distance_*translational_direction;
        pc_frame_.M = eef_frame_.M;
        pc_frame_.p = pc_kdl;
        tf::vectorKDLToEigen(pc_kdl, pc_);
      }
      break;
    case DIRECT_COMPUTATION:
      // code for computing the contact point directly from measured force and torque
      break;
    case KALMAN_FILTER:
      pc_ = estimator_.estimate(measured_v1_, measured_w1_, f2_, t2_, p1_, p2_, dt_);
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
