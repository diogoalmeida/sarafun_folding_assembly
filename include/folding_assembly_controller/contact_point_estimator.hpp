#ifndef __CONTACT_POINT_ESTIMATOR__
#define __CONTACT_POINT_ESTIMATOR__
#include <ros/ros.h>
#include <Eigen/Dense>


/*
  Estimator Interface

  Generic interface that an estimation method has to comply with in order to
  be used with the folding controller
*/
class estimatorInterface
{
  public:
    estimatorInterface(){}
    virtual bool initialize(const Eigen::Vector3d mu_init) = 0;
    virtual Eigen::Vector3d estimate() = 0;

  protected:
    ros::NodeHandle privateNh;
};

/*
  KF Estimator Base

  Implements the estimator interface for the particular case of the
  Kalman filter
*/
class KFEstimatorBase : public estimatorInterface
{
  public:
    KFEstimatorBase(){}
    bool initialize(const Eigen::Vector3d mu_init);
    virtual Eigen::Vector3d estimate(){}
    virtual Eigen::Vector3d estimate(const Eigen::Vector3d v1, const Eigen::Vector3d w1,
      const Eigen::Vector3d force, const Eigen::Vector3d torque,
      const Eigen::Vector3d p1, const Eigen::Vector3d p2, const double dt)=0;
    void reset(const Eigen::Vector3d mu_init);
  protected:
    virtual void predict(Eigen::Vector3d &mu_bar, Eigen::Matrix3d &sigma_bar,
      const Eigen::Vector3d v1, const Eigen::Vector3d w1,
      const Eigen::Vector3d pEef, const double dt) = 0;
    virtual void update(const Eigen::Vector3d mu_bar, const Eigen::Matrix3d sigma_bar,
      const Eigen::Matrix3d H) = 0;
    void initializeEigenMatrix(Eigen::Matrix3d &M, int rows, int columns, const std::vector<double> vals);
    bool parseMatrixData(Eigen::Matrix3d &M, const std::string configName);
    void skew(Eigen::Matrix3d &S, const Eigen::Vector3d w);

    Eigen::Matrix3d R, Q, sigma;
    Eigen::Vector3d nu, mu;
    double z_offset_;
};

/*
  KF Estimator 1

  Implements the KF estimator based on the force torque model and using
  one force torque measurement
*/
class KFEstimator1 : public KFEstimatorBase
{
  public:
    KFEstimator1(){}
    virtual Eigen::Vector3d estimate(const Eigen::Vector3d v1, const Eigen::Vector3d w1,
      const Eigen::Vector3d force, const Eigen::Vector3d torque,
      const Eigen::Vector3d p1, const Eigen::Vector3d p2, const double dt);
  protected:
    virtual void predict(Eigen::Vector3d &mu_bar, Eigen::Matrix3d &sigma_bar,
      const Eigen::Vector3d v1, const Eigen::Vector3d w1,
      const Eigen::Vector3d pEef, const double dt);
    virtual void update(const Eigen::Vector3d mu_bar, const Eigen::Matrix3d sigma_bar,
      const Eigen::Matrix3d H);
};

#endif
