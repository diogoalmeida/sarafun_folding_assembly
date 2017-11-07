#include <folding_assembly_controller/kalman_filter.hpp>

namespace folding_algorithms{

  KalmanEstimator::KalmanEstimator()
  {
    n_ = ros::NodeHandle("~");
  }

  KalmanEstimator::~KalmanEstimator(){}

  bool KalmanEstimator::getParams()
  {
    if(!matrix_parser_.parseMatrixData(Q_, "kf_estimator/Q", n_))
    {
      return false;
    }

    if(!matrix_parser_.parseMatrixData(R_, "kf_estimator/R", n_))
    {
      return false;
    }

    return true;
  }

  bool KalmanEstimator::initialize(const Eigen::VectorXd &x)
  {
    if(!getParams())
    {
      return false;
    }

    // Check for data consistency
    if(x.rows() != R_.rows() || x.rows() != R_.cols())
    {
      std::stringstream errMsg;
      errMsg << "R matrix is not from the proper size (got: <" << R_.rows() << "," << R_.cols() << "> and state vector is size "<< x.rows() << ")";
      throw std::logic_error(errMsg.str().c_str());
    }

    x_ = x;
    P_ = Eigen::MatrixXd::Identity(x.rows(), x.rows());

    return true;
  }

  Eigen::VectorXd KalmanEstimator::estimate(const Eigen::Vector3d &p_e1, const Vector6d &x_dot_e1, const Eigen::Vector3d &p_e2, const Eigen::VectorXd &wrench, double dt)
  {
    if(wrench.rows() != 6 && wrench.rows() != 12)
    {
      std::stringstream errMsg;
      errMsg << "Received wrench has wrong dimensions (got: <"<< wrench.rows() << " > and expected 6 or 12)";
      throw std::logic_error(errMsg.str().c_str());
    }

    if(wrench.rows()/2 != Q_.rows() || wrench.rows()/2 != Q_.cols())
    {
      std::stringstream errMsg;
      errMsg << "Q matrix is not compatible with the received wrench (got: <" << Q_.rows() << "," << Q_.cols() << "> and wrench vector is size "<< wrench.rows() << ")";
      throw std::logic_error(errMsg.str().c_str());
    }

    Eigen::MatrixXd A(x_.rows(), x_.rows()), I(x_.rows(), x_.rows()), P_hat(x_.rows(), x_.rows()),
                    S(wrench.rows(), wrench.rows()), C(wrench.rows(), x_.rows()), K(x_.rows(), wrench.rows()), innov(wrench.rows(), 1), y(wrench.rows(), 1);

    I = Eigen::MatrixXd::Identity(x_.rows(), x_.rows());

    A = matrix_parser_.computeSkewSymmetric(x_dot_e1.block<3,1>(3,0));

    if (wrench.rows() == 6)
    {
      C = -matrix_parser_.computeSkewSymmetric(wrench.block<3,1>(0,0));
      y = wrench.block<3,1>(3,0) - matrix_parser_.computeSkewSymmetric(wrench.block<3,1>(0,0))*p_e1;
    }
    else
    {
      C.block<3,3>(0,0) = -matrix_parser_.computeSkewSymmetric(wrench.block<3,1>(0,0));
      C.block<3,3>(3,0) = -matrix_parser_.computeSkewSymmetric(wrench.block<3,1>(6,0));
      y.block<3,1>(0,0) = wrench.block<3,1>(3,0) - matrix_parser_.computeSkewSymmetric(wrench.block<3,1>(0,0))*p_e1;
      y.block<3,1>(0,0) = wrench.block<3,1>(9,0) - matrix_parser_.computeSkewSymmetric(wrench.block<3,1>(6,0))*p_e2;
    }

    // process model
    P_hat = A*P_.selfadjointView<Eigen::Upper>()*A.transpose() + R_;
    x_ = x_ + (A*(x_ - p_e1) + x_dot_e1.block<3,1>(0,0))*dt;
    innov = y - C*x_;
    S = C*P_hat.selfadjointView<Eigen::Upper>()*C.transpose() + Q_;

    K = P_hat.selfadjointView<Eigen::Upper>()*C.transpose()*S.llt().solve(I);
    x_ = x_ + K*innov;
    P_= (I - K*C)*P_hat.selfadjointView<Eigen::Upper>();

    return x_;
  }
}
