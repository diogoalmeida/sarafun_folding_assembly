#include <folding_assembly_controller/controller.hpp>

foldingController::foldingController()
{
  v1 = Eigen::Vector3d::Zero();
  w1 = Eigen::Vector3d::Zero();

  // Default values. Can be overriden by input file
  saturationV = 0.01;
  saturationW = 0.01;

  _n = ros::NodeHandle("~");

  monitorPub = _n.advertise<folding_assembly_controller::monitorMsg>("foldingController/signals", 1);
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

// void foldingController::setDesiredNormalForce(double newNormalForce)
// {
//   fnRef = newNormalForce;
//
//   ROS_INFO("Reference force for the folding assembly controller changed to %.3f!", fnRef);
// }

void foldingController::control(const double &vd, const double &wd, Eigen::Vector3d &vOut, Eigen::Vector3d &wOut, const double d_t)
{
  Eigen::Matrix3d S;
  Eigen::Vector3d rotationAxis, omegaD, velD;

  dt = d_t;
  // updateSurfaceTangent();
  // updateSurfaceNormal();
  // updateTheta();

  rotationAxis = surfaceTangent.cross(surfaceNormal);

  omegaD = wd*rotationAxis;
  velD = vd*surfaceTangent;

  // updateContactPoint();

  w1 = omegaD;

  r1 = pc - p1;

  // vf = computeForceControl();

  S = computeSkewSymmetric(w1);
  v1 = - S * r1 + velD + vf;

  if (abs(v1.dot(surfaceTangent)) > saturationV)
  {
    v1 = v1/v1.norm() * saturationV;
    ROS_WARN("V1 IS SATURATED");
  }
  if (w1.norm() > saturationW)
  {
    w1 = w1/w1.norm() * saturationW;
    ROS_WARN("OMEGA1 IS SATURATED");
  }

    vOut = v1;
    wOut = w1;
}
