#include <folding_assembly_controller/adaptive_velocity_controller.hpp>

namespace folding_algorithms{

  AdaptiveController::AdaptiveController()
  {
    int_force_ = Eigen::Vector3d::Zero();
    int_torque_ = Eigen::Vector3d::Zero();
    v_f_ = Eigen::Vector3d::Zero();
    w_f_ = Eigen::Vector3d::Zero();
    t_ << 1, 0, 0;
    r_ << 0, 0, 1;
    max_force_ = 0;
    max_torque_ = 0;
    nh_ = ros::NodeHandle("~");

    if (!getParams())
    {
      throw std::logic_error("Adaptive velocity controller missing initialization parameters");
    }
  }

  AdaptiveController::~AdaptiveController(){}

  Vector6d AdaptiveController::control(const Vector6d &wrench, double v_d, double w_d, double dt, const Eigen::Vector3d &force_direction)
  {
    Eigen::Vector3d normal, torque_d, force;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Vector6d ref_twist;

    normal = t_.cross(r_);
    torque_d = Eigen::Vector3d::Zero();
    force = wrench.block<3,1>(0,0);

    if (force_direction == Eigen::Vector3d::Zero())
    {
      force_error_ = force - f_d_*normal;
    }
    else
    {
      force_error_ = force - f_d_*force_direction;
      ROS_DEBUG_STREAM_THROTTLE(2, "Current wrench: " << force << ". Desired wrench: " << f_d_*force_direction);
    }

    if (force_error_.norm() < force_slack_)
    {
      ROS_DEBUG_STREAM_THROTTLE(10, "Measured force norm is " << force_error_.norm() << ". Setting to zero");
      force_error_ = Eigen::Vector3d::Zero();
    }
    else
    {
      force_error_ = force_error_ - force_slack_*force_error_.normalized();
    }

    torque_error_ = wrench.block<3,1>(3,0);

    if (torque_error_.norm() < torque_slack_)
    {
      ROS_DEBUG_STREAM_THROTTLE(10, "Measured torque norm is " << torque_error_.norm() << ". Setting to zero");
      torque_error_ = Eigen::Vector3d::Zero();
    }
    else
    {
      torque_error_ = torque_error_ - torque_slack_*torque_error_.normalized();
    }

    int_force_ = computeIntegralTerm(int_force_, t_, force_error_, dt);
    v_f_ = alpha_force_*force_error_ + beta_force_*int_force_;

    // TODO: Avoid windup
    if (v_f_.norm() > max_force_)
    {
      ROS_WARN("Saturated v_f");
      v_f_ = v_f_*max_force_/v_f_.norm();
    }

    // ref_twist.block<3,1>(0,0) = v_d*t_ - (I - t_*t_.transpose())*v_f_;
    ref_twist.block<3,1>(0,0) = v_d*t_ - v_f_;
    t_ = t_ - alpha_adapt_t_*v_d*(I - t_*t_.transpose())*v_f_*dt;
    t_ = t_/t_.norm();

    int_torque_ = computeIntegralTerm(int_torque_, r_, torque_error_, dt);
    // apply different torque gains along different directions
    w_f_ = (alpha_torque_t_*(t_*t_.transpose())+ alpha_torque_n_*(normal*normal.transpose()))*torque_error_
         + (beta_torque_t_*(t_*t_.transpose()) + beta_torque_n_*(normal*normal.transpose()))*int_torque_;

    if (w_f_.norm() > max_torque_)
    {
      ROS_WARN("Saturated w_f");
      w_f_ = w_f_*max_torque_/w_f_.norm();
    }

    ref_twist.block<3,1>(3,0) = w_d*r_ - (I - r_*r_.transpose())*w_f_;
    r_ = r_ - alpha_adapt_r_*w_d*w_f_*dt;
    r_ = r_/r_.norm();

    return ref_twist;
  }

  void AdaptiveController::reset()
  {
    int_force_ = Eigen::Vector3d::Zero();
    int_torque_ = Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d AdaptiveController::computeIntegralTerm(const Eigen::Vector3d &prev, const Eigen::Vector3d &v, const Eigen::Vector3d &error, double dt) const
  {
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    return prev + (I - v*v.transpose())*error*dt;
  }

  void AdaptiveController::getErrors(Eigen::Vector3d &force_e, Eigen::Vector3d &torque_e, Eigen::Vector3d &desired_force) const
  {
    Eigen::Vector3d normal = t_.cross(r_);
    force_e = force_error_;
    torque_e = torque_error_;
    desired_force = f_d_*normal;
  }

  void AdaptiveController::initEstimates(const Eigen::Vector3d &t, const Eigen::Vector3d &r)
  {
    t_ = t;
    r_ = r;
  }

  void AdaptiveController::setReferenceForce(double f_d)
  {
    f_d_ = f_d;
  }

  void AdaptiveController::getEstimates(Eigen::Vector3d &t, Eigen::Vector3d &r) const
  {
    t = t_;
    r = r_;
  }

  void AdaptiveController::getForceControlValues(Eigen::Vector3d &v_f, Eigen::Vector3d &w_f) const
  {
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    v_f = (I - t_*t_.transpose())*v_f_;
    w_f = w_f_;
  }

  bool AdaptiveController::getParams()
  {
    if (!nh_.getParam("adaptive_estimator/alpha_force", alpha_force_))
    {
      ROS_ERROR("Missing force gain (adaptive_estimator/alpha_force)");
      return false;
    }

    if (!nh_.getParam("adaptive_estimator/beta_force", beta_force_))
    {
      ROS_ERROR("Missing beta force value (adaptive_estimator/beta_force)");
      return false;
    }

    if (!nh_.getParam("adaptive_estimator/force_slack", force_slack_))
    {
      ROS_ERROR("Missing force slack value (adaptive_estimator/force_slack)");
      return false;
    }

    if (!nh_.getParam("adaptive_estimator/alpha_adapt_t", alpha_adapt_t_))
    {
      ROS_ERROR("Missing translational dof adaptation value (adaptive_estimator/alpha_adapt_t)");
      return false;
    }

    if (!nh_.getParam("adaptive_estimator/alpha_torque_t", alpha_torque_t_))
    {
      ROS_ERROR("Missing translation dir torque gain (adaptive_estimator/alpha_torque_t)");
      return false;
    }

    if (!nh_.getParam("adaptive_estimator/alpha_torque_n", alpha_torque_n_))
    {
      ROS_ERROR("Missing normal dir torque gain (adaptive_estimator/alpha_torque_n)");
      return false;
    }

    if (!nh_.getParam("adaptive_estimator/beta_torque_t", beta_torque_t_))
    {
      ROS_ERROR("Missing translational beta torque value (adaptive_estimator/beta_torque_t)");
      return false;
    }

    if (!nh_.getParam("adaptive_estimator/beta_torque_n", beta_torque_n_))
    {
      ROS_ERROR("Missing normal beta torque value (adaptive_estimator/beta_torque_n)");
      return false;
    }

    if (!nh_.getParam("adaptive_estimator/alpha_adapt_r", alpha_adapt_r_))
    {
      ROS_ERROR("Missing translational dof adaptation value (adaptive_estimator/alpha_adapt_r)");
      return false;
    }

    if (!nh_.getParam("adaptive_estimator/torque_slack", torque_slack_))
    {
      ROS_ERROR("Missing torque slack value (adaptive_estimator/torque_slack)");
      return false;
    }

    if (!nh_.getParam("adaptive_estimator/max_force", max_force_))
    {
      ROS_ERROR("Missing max force value (adaptive_estimator/max_force)");
      return false;
    }

    if (!nh_.getParam("adaptive_estimator/max_torque", max_torque_))
    {
      ROS_ERROR("Missing max torque value (adaptive_estimator/max_torque)");
      return false;
    }

    return true;
  }
}
