#include <folding_assembly_controller/contact_point_estimator.hpp>


bool EKFEstimatorBase::initialize(const Eigen::Vector3d mu_init)
{
  std::string estimatorName;
  if(privateNh.hasParam("estimatorName"))
  {
    privateNh.getParam("estimatorName", estimatorName);
  }
  else
  {
    ROS_ERROR("The parameter server should have an estimatorName parameter. Shutting down...");
    privateNh.shutdown();
    return false;
  }

  if(estimatorName != std::string("EKF"))
  {
    ROS_ERROR("The estimator config file does not have configuration values for the"
      "EKF estimator (estimatorName: EKF). Shutting down...");
      privateNh.shutdown();
      return false;
  }

  if(privateNh.hasParam("config"))
  {
    if(privateNh.hasParam("config/noiseMatrices"))
    {
      if(!parseMatrixData(R, std::string("config/noiseMatrices/process")))
      {
        ROS_ERROR("Error processing the process noise covariance matrix in the"
          "input file (config/noiseMatices/process). Shutting down...");
          privateNh.shutdown();
          return false;
      }
      if(!parseMatrixData(Q, std::string("config/noiseMatrices/measurement")))
      {
        ROS_ERROR("Error processing the process noise covariance matrix in the"
          "input file (config/noiseMatices/measurment). Shutting down...");
          privateNh.shutdown();
          return false;
      }
    }
    else
    {
      ROS_ERROR("No noise matrices information in the config file for the"
        "EKF estimator (config/noiseMatrices). Shutting down...");
        privateNh.shutdown();
        return false;
    }
    if(!parseMatrixData(sigma, std::string("config/initialCov")))
    {
      ROS_ERROR("No initial covariance matrix provided for the EKF estimator"
        "(config/initialCov). Shutting down...");
        privateNh.shutdown();
        return false;
    }
    mu = mu_init;
    ROS_INFO("INITIAL MU:");
    for(int i = 0; i < 3; i ++)
    {
      std::cout << mu(i) << " ";
    }
    std::cout << std::endl;
    std::cin.get();
  }
  else
  {
    ROS_ERROR("No config section exists in the EKF estimator config file."
      "Shutting down...");
      privateNh.shutdown();
      return false;
  }

  return true;
}

bool EKFEstimatorBase::parseMatrixData(Eigen::Matrix3d &M, const std::string configName)
{
  int rows = 0, columns = 0;
  std::vector<double> vals;

  if(privateNh.hasParam(configName.c_str()))
  {
    if(privateNh.hasParam((configName + std::string("/rows")).c_str()))
    {
      privateNh.getParam((configName + std::string("/rows")).c_str(), rows);
    }
    else
    {
      ROS_ERROR("Matrix definition %s has no rows value (%s)!. Shutting down..."
      , configName.c_str(), (configName + std::string("/rows")).c_str());
      return false;
    }
    if(privateNh.hasParam((configName + std::string("/columns")).c_str()))
    {
      privateNh.getParam((configName + std::string("/columns")).c_str(), columns);
    }
    else
    {
      ROS_ERROR("Matrix definition %s has no rows value (%s)!. Shutting down..."
      , configName.c_str(), (configName + std::string("/columns")).c_str());
      return false;
    }
    if(privateNh.hasParam((configName + std::string("/data").c_str())))
    {
      privateNh.getParam((configName + std::string("/data")).c_str(), vals);
      initializeEigenMatrix(M, rows, columns, vals);
    }
    else
    {
      ROS_ERROR("Matrix definition %s has no data values (%s)! Shutting down..."
      , configName.c_str(), (configName + std::string("/data")).c_str());
      return false;
    }
  }

  return true;
}

void EKFEstimatorBase::initializeEigenMatrix(Eigen::Matrix3d &M, int rows,
  int columns, const std::vector<double> vals)
{
  for(int i = 0; i < rows; i++)
  {
    for(int j = 0; j < columns; j++)
      {
        M(i,j) = vals[i*rows + j];
      }
  }
}

void EKFEstimatorBase::skew(Eigen::Matrix3d &S, const Eigen::Vector3d w)
{
  S << 0, -w(2), w(1),
       w(2), 0, -w(0),
       -w(1), w(0), 0;
}

//////////////////////////////////////////////

Eigen::Vector3d EKFEstimator1::estimate(const Eigen::Vector3d v1, const Eigen::Vector3d w1,
  const Eigen::Vector3d force, const Eigen::Vector3d torque,
  const Eigen::Vector3d p1, const Eigen::Vector3d p2, const double dt)
{
  Eigen::Vector3d mu_bar, y;
  Eigen::Matrix3d sigma_bar, S;

  predict(mu_bar, sigma_bar, v1, w1, p1, dt);
  skew(S, force);

  y = torque - S*p2;

  nu = y - (-S*mu_bar);

  update(mu_bar, sigma_bar, -S);

  return mu;
}

void EKFEstimator1::predict(Eigen::Vector3d &mu_bar, Eigen::Matrix3d &sigma_bar,
  const Eigen::Vector3d v1, const Eigen::Vector3d w1,
  const Eigen::Vector3d pEef, const double dt)
{
  Eigen::Matrix3d S;

  skew(S, w1);

  mu_bar = mu + (S*(mu - pEef) + v1)*dt;

  S = Eigen::Matrix3d::Identity() + S*dt;

  sigma_bar.triangularView<Eigen::Upper>() = S*sigma.selfadjointView<Eigen::Upper>()*S.transpose() + R;
}

void EKFEstimator1::update(const Eigen::Vector3d mu_bar, const Eigen::Matrix3d sigma_bar,
  const Eigen::Matrix3d H)
{
  Eigen::Matrix3d Kalman;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  I.setIdentity();

  Kalman = sigma_bar.selfadjointView<Eigen::Upper>()*H.transpose()*(H*sigma_bar.selfadjointView<Eigen::Upper>()*H.transpose() + Q).inverse();
  mu = mu_bar + Kalman*nu;
  sigma.triangularView<Eigen::Upper>() = (I - Kalman*H)*sigma_bar.selfadjointView<Eigen::Upper>();
}
