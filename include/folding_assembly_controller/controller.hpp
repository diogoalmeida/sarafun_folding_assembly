#ifndef __FOLDING_CONTROLLER__
#define __FOLDING_CONTROLLER__
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
// #include <folding_assembly_controller/contact_point_estimator.hpp>
// #include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <math.h>
// #include <robot_arm/robot_arm.hpp>
#include <tf_conversions/tf_kdl.h>
#include <folding_assembly_controller/monitorMsg.h>
#include <vector>

/*
Folding Controller

Implements the folding controller.
*/
class foldingController
{
  public:
    foldingController();
    void control(const double &vd, const double &wd, Eigen::Vector3d &vOut, Eigen::Vector3d &wOut, const double d_t);

  protected:
    Eigen::Vector3d surfaceNormal, surfaceTangent, p1, p2, r1,
      r2, omega1, pc, pd, thetaD, v1, w1, vref, wref, vf, realPc, f1, f2,
      t1, t2;
    double saturationV, saturationW, dt;
    double thetaC, fnRef, forceAvg;
    int deltaState;

    ros::NodeHandle _n;
    ros::Publisher monitorPub;

    Eigen::Matrix3d computeSkewSymmetric(Eigen::Vector3d v);

    void updateForces();
    folding_assembly_controller::monitorMsg publishInfo();
    void getPoints(Eigen::Vector3d &realPc, Eigen::Vector3d &p1, Eigen::Vector3d &p2);
};
#endif
