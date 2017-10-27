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

      @param wrench The required wrench for the estimation. This can be at the end-effector, as the force will match the contact point and the angular velocity has the same restrictions as the contact's.
      @param virtual_stick The estimated virtual stick that allows to obtain the desired torque value.
      @param v_d The desired linear velocity.
      @param w_d The desired angular velocity.
      @param dt The elapsed time between calls.
      @return The control twist for the relative velocity between parts.
    **/
    Vector6d control(const Vector6d &wrench, const Eigen::Vector3d &virtual_stick, double v_d, double w_d, double dt);

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
    void getEstimates(Eigen::Vector3d &t, Eigen::Vector3d &r);

    /**
      Returns the current value of the force control components.

      @param v_f The current value of the force control component.
      @param w_f The current value of the torque control component.
    **/
    void getForceControlValues(Eigen::Vector3d &v_f, Eigen::Vector3d &w_f);

    /**
      Returns the current measured force and torque errors.

      @param force_e The force error.
      @param torque_e The torque error.
      @param desired_force Force setpoint.
      @param desired_torque Torque setpoint.
    **/
    void getErrors(Eigen::Vector3d &force_e, Eigen::Vector3d &torque_e, Eigen::Vector3d &desired_force, Eigen::Vector3d &desired_torque);

  private:
    double alpha_force_, beta_force_, alpha_torque_, beta_torque_, f_d_, v_d_amp_, w_d_amp_, v_freq, w_freq, alpha_adapt_t_, alpha_adapt_r_, torque_slack_, normal_bias_, max_force_, max_torque_;
    Eigen::Vector3d t_, r_, int_force_, int_torque_, v_f_, w_f_, force_error_, torque_error_, stick_;
    ros::NodeHandle nh_;

    /**
      Compute the integral term in the wrench feedback component of the adaptive controller.

      @param prev The previous value of the integral term.
      @param v The projection vector.
      @param error The force/torque error.
      @param dt The elapsed time.
      @return The updated integral value.
    **/
    Eigen::Vector3d computeIntegralTerm(const Eigen::Vector3d &prev, const Eigen::Vector3d &v, const Eigen::Vector3d &error, double dt);

    bool getParams();
  };
}
#endif
