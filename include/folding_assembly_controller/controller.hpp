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
#include <eigen_conversions/eigen_msg.h>

/*
Folding Controller

Implements the folding controller.
*/
class foldingController
{
  public:
    foldingController();
    void control(const double &vd, const double &wd, const double &contact_force, Eigen::Vector3d &vOut, Eigen::Vector3d &wOut, const double d_t);
    void getEstimates(Eigen::Vector3d &pc, double &thetac);

  protected:
    Eigen::Vector3d surfaceNormal_, surfaceTangent_,
                    p1_, p2_, r1_,
                    r2_, omega1_, pc_,
                    pd_, thetaD_, v1_,
                    w1_, vref_, wref_,
                    vf_, realPc_, f1_,
                    f2_, t1_, t2_;
    double saturationV_, saturationW_, dt_;
    double thetaC_, fRef_, kf_;
    bool estimate_;
    std::string wrench_topic_name_;

    ros::NodeHandle n_;
    ros::Publisher monitorPub_;
    ros::Subscriber wrench_sub_;

    Eigen::Matrix3d computeSkewSymmetric(Eigen::Vector3d v);

    void updateForces();
    Eigen::Vector3d computeForceControl();
    void updateContactPoint();
    void updateTheta();
    folding_assembly_controller::monitorMsg publishInfo();
    void getPoints(Eigen::Vector3d &realPc, Eigen::Vector3d &p1, Eigen::Vector3d &p2);
    bool getParams();
    void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
};
#endif
