#include <folding_assembly_controller/ects.hpp>

namespace folding_algorithms{
  ECTSController::ECTSController(const std::string &rod_eef, const std::string &surface_eef, std::shared_ptr<generic_control_toolbox::KDLManager> kdl_manager)
{
  nh_ = ros::NodeHandle("~");
  kdl_manager_ = kdl_manager;
  rod_eef_ = rod_eef;
  surface_eef_ = surface_eef;

  if (!getParams())
  {
    throw std::logic_error("ECTS controller failed to initialize");
  }
}

ECTSController::~ECTSController(){}

Vector14d ECTSController::control(const sensor_msgs::JointState &state, const Vector3d &r1, const Vector3d &r2, const Vector6d &twist_a, const Vector6d &twist_r) const
{
  MatrixECTS J = MatrixECTS::Zero();
  Matrix14d I = Matrix14d::Identity();
  Matrix12d damped_inverse;
  Vector12d total_twist;
  Vector14d q_dot, epsilon = Vector14d::Zero(), proj, out;

  total_twist.block<6,1>(0,0) = twist_a;
  total_twist.block<6,1>(6,0) = twist_r;

  J = computeECTSJacobian(state, r1, r2);

  damped_inverse = (J*J.transpose() + damping_*Matrix12d::Identity());

  proj = (I  - J.transpose()*(J*J.transpose() + damping_*Matrix12d::Identity()).inverse()*J)*epsilon;

  out = J.transpose()*damped_inverse.colPivHouseholderQr().solve(total_twist) + proj;

  return out;
}

void ECTSController::setAlpha(double alpha)
{
  if (alpha >= 0 && alpha <= 1)
  {
    alpha_ = alpha;
  }
  else
  {
    std::stringstream errMsg;
    errMsg << "Got alpha " << alpha << ", must be within the range [0, 1]";
    throw std::logic_error(errMsg.str().c_str());
  }
}

double ECTSController::getAlpha() const
{
  return alpha_;
}

MatrixECTS ECTSController::computeECTSJacobian(const sensor_msgs::JointState &state, const Vector3d &r_1, const Vector3d &r_2) const
{
  Matrix12d C = Matrix12d::Zero(), W = Matrix12d::Identity();
  MatrixECTS J_e, J = MatrixECTS::Zero();
  KDL::Jacobian J_1_kdl(7), J_2_kdl(7);

  kdl_manager_->getJacobian(rod_eef_, state, J_1_kdl);
  kdl_manager_->getJacobian(surface_eef_, state, J_2_kdl);

  C.block<6,6>(0,0) = alpha_*Matrix6d::Identity();
  C.block<6,6>(0,6) = (1 - alpha_)*Matrix6d::Identity();
  C.block<6,6>(6,0) = -beta_*Matrix6d::Identity();
  C.block<6,6>(6,6) = Matrix6d::Identity();

  W.block<3, 3>(0, 3) = -matrix_parser_.computeSkewSymmetric(r_1);
  W.block<3, 3>(6, 9) = -matrix_parser_.computeSkewSymmetric(r_2);

  J.block<6,7>(0,0) = J_1_kdl.data;
  J.block<6,7>(6,7) = J_2_kdl.data;

  J_e = C*W*J;

  return J_e;
}

bool ECTSController::getParams()
{
  if (!nh_.getParam("ects_controller/alpha", alpha_))
  {
    ROS_ERROR("Missing alpha gain (ects_controller/alpha)");
    return false;
  }

  if (!nh_.getParam("ects_controller/beta", beta_))
  {
    ROS_ERROR("Missing beta value (ects_controller/beta)");
    return false;
  }

  if (!nh_.getParam("ects_controller/inverse_damping", damping_))
  {
    ROS_ERROR("Missing damping value (ects_controller/inverse_damping)");
    return false;
  }

  setAlpha(alpha_);

  if (beta_ != 0 && beta_ != 1)
  {
    std::stringstream errMsg;
    errMsg << "Got beta " << beta_ << ", must be either 0 or 1";
    throw std::logic_error(errMsg.str().c_str());
  }

  return true;
}
}
