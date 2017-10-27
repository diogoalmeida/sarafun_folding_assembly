#ifndef __ECTS_CONTROL_ALGORITHM__
#define __ECTS_CONTROL_ALGORITHM__

#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <limits>
#include <stdexcept>
#include <utils/matrix_parser.hpp>
#include <cmath>


namespace folding_algorithms{
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 7> Matrix67d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 14, 14> Matrix14d;
typedef Eigen::Matrix<double, 6, 14> MatrixECTSr;
typedef Eigen::Matrix<double, 12, 14> MatrixECTS;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 14, 1> Vector14d;

  /**
    Class that implements an extended cooperative task space (ECTS) control algorithm.

    Hardcoded dimensions to fit two 7 DOF manipulators.
  **/
  class ECTSController
  {
  public:
    ECTSController(const KDL::Chain &chain_1, const KDL::Chain &chain_2);
    ~ECTSController();

    /**
      Computes the ECTS reference joint velocities for the two manipulators.

      @param qi The manipulators' joint states
      @param ri The virtual sticks connecting the manipulators end-effectors to the task C-Frame.
      @param twist_a The commanded absolute motion twist.
      @param twist_r The commanded relative motion twist.
      @return The 14 dimensional joint velocities vector.
    **/
    Vector14d control(const Vector3d &r1, const Vector3d &r2, const KDL::JntArray &q1, const KDL::JntArray &q2, const Vector6d &twist_a, const Vector6d &twist_r);

    /**
      Return the current alpha value that determines the degree of colaboration between arms.
    **/
    double getAlpha();

    /**
      Sets the alpha value.

      @param alpha The desired ECTS alpha value.
      @throw logic_error if alpha outside range [0, 1]
    **/
    void setAlpha(double alpha);

  private:
    ros::NodeHandle nh_;
    double alpha_, damping_;
    int beta_;
    std::vector<boost::shared_ptr<KDL::ChainJntToJacSolver> > jac_solver_;
    folding_utils::MatrixParser matrix_parser_;

    /**
      Computes the ECTS jacobian that maps joints to task space twists.

      @param qi Joint state of arm i
      @param r_i virtual stick connecting eef i to task frame
      @return The ECTS jacobian.
    **/
    MatrixECTS computeECTSJacobian(const KDL::JntArray &q1, const KDL::JntArray &q2, const Vector3d &r_1, const Vector3d &r_2);

    /**
      Loads the ECTS controller parameters from the ros parameter server.

      @throw logic_error for parameters with unsuitable values
      @return True in case of success, False if some parameter is not available.
    **/
    bool getParams();
  };
}
#endif
