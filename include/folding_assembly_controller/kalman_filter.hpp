#ifndef __FOLDING_KALMAN_FILTER__
#define __FOLDING_KALMAN_FILTER__

#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <limits>
#include <stdexcept>
#include <utils/matrix_parser.hpp>

namespace folding_algorithms{
typedef Eigen::Matrix<double, 6, 1> Vector6d;

  /**
    Class that implements the folding kalman filter.
    Quantities estimated in the world frame.
  **/
  class KalmanEstimator
  {
  public:
    KalmanEstimator();
    ~KalmanEstimator();

    /**
      Estimate the joint location from the measured reaction forces.
      Neglects inertial effects and friction.

      @param p_e1 The position of the rod piece end-effector.
      @param x_dot_e1 The rod piece end-effector twist.
      @param p_e2 The position of the surface piece end-effector.
      @param wrench The measured wrench. Accepts 6 or 12 dimensional units for 1 or 2 measurement points.
      @param dt The elapsed time between estimate steps.
      @throw logic_error in case the received wrench is not compatible with Q.
      @output The estimated state.
    **/
    Eigen::VectorXd estimate(const Eigen::Vector3d &p_e1, const Vector6d &x_dot_e1, const Eigen::Vector3d &p_e2, const Eigen::VectorXd &wrench, double dt);

    /**
      Initialize the estimator

      @param x The initial state estimate.
      @throw logic_error in case the initial estimate size collides with the algorithm initialization parameters.
      @return False for when initialization is not possible (e.g., due to missing parameters)
    **/
    bool initialize(const Eigen::VectorXd &x);

    /**
      Load the estimator parameters from the ROS parameter server.

      @throw logic_error in case some parameter is incorrectly set.
      @return False in case some parameter is missing.
    **/
    bool getParams();

  private:
    ros::NodeHandle n_;
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_, Q_, R_;
    folding_utils::MatrixParser matrix_parser_;

    /**
      Computed the skew-symmetric matrix of a 3-dimensional vector.

      @param v The 3-dimensional vector
      @return The skew-symmetric matrix
    **/
    Eigen::Matrix3d computeSkewSymmetric(const Eigen::Vector3d &v);
  };
}
#endif
