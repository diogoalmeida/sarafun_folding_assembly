#ifndef __ADAPTIVE_CONTROL_ALGORITHM__
#define __ADAPTIVE_CONTROL_ALGORITHM__

#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <limits>
#include <stdexcept>

namespace folding_algorithms{
typedef Eigen::Matrix<double, 6, 1> Vector6d;
  /**
    Class that implements the adaptive control algorithm that enables DOF identification.
  **/
  class AdaptiveController
  {
  public:
    AdaptiveController();
    ~AdaptiveController();

    /**
      Computes the relative twist reference for the mechanism and updates the
      estimates.

      @param wrench Estimation wrench: estimated contact point force and wrench measurement .
      @param v_d The desired linear velocity.
      @param w_d The desired angular velocity.
      @param dt The elapsed time between calls.
      @return The control twist for the relative velocity between parts.
    **/
    Vector6d control(const Vector6d &wrench, double v_d, double w_d, double dt);

    /**
      Initialize the adaptive controller estimates.

      @param t Initial translational DOF estimate.
      @param r Initial rotational DOF estimate.
    **/
    void initEstimates(const Eigen::Vector3d &t, const Eigen::Vector3d &r);

    /**
      Sets the magnitude of the desired contact force.

      @param f_d Desired force magnitude.
    **/
    void setReferenceForce(double f_d);

    /**
      Return the current estimates.
      @param t Current translational DOF estimate.
      @param r Current rotational DOF estimate.
    **/
    void getEstimates(Eigen::Vector3d &t, Eigen::Vector3d &r) const;

    /**
      Returns the current value of the force control components.

      @param v_f The current value of the force control component.
      @param w_f The current value of the torque control component.
    **/
    void getForceControlValues(Eigen::Vector3d &v_f, Eigen::Vector3d &w_f) const;

    /**
      Returns the current measured force and torque errors.

      @param force_e The force error.
      @param torque_e The torque error.
      @param desired_force Force setpoint.
    **/
    void getErrors(Eigen::Vector3d &force_e, Eigen::Vector3d &torque_e, Eigen::Vector3d &desired_force) const;

  private:
    double alpha_force_, beta_force_, alpha_torque_, beta_torque_;
    double f_d_, alpha_adapt_t_, alpha_adapt_r_, torque_slack_, max_force_, max_torque_;
    Eigen::Vector3d t_, r_, int_force_, int_torque_, v_f_, w_f_, force_error_, torque_error_;
    ros::NodeHandle nh_;

    /**
      Compute the integral term in the wrench feedback component of the adaptive controller.

      @param prev The previous value of the integral term.
      @param v The projection vector.
      @param error The force/torque error.
      @param dt The elapsed time.
      @return The updated integral value.
    **/
    Eigen::Vector3d computeIntegralTerm(const Eigen::Vector3d &prev, const Eigen::Vector3d &v, const Eigen::Vector3d &error, double dt) const;

    bool getParams();
  };
}
#endif
