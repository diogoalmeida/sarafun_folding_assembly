#ifndef __FOLDING_POSE_CONTROLLER__
#define __FOLDING_POSE_CONTROLLER__

#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <limits>
#include <stdexcept>

namespace folding_algorithms{
/**
  Implements the position feedback controller responsible for generating velocity
  references to the folding controller. Given a desired relative contact position
**/
class FoldingPoseController
{
public:
  FoldingPoseController(const std::string &ns = "pose_controller");
  ~FoldingPoseController();

  /**
    Computes the control commands for a given contact state.

    @param pc Contact point position in the local frame.
    @param thetac Relative angle in the local frame.
    @param pd Desired position.
    @param thetad Desired angle.
    @param vd Computed relative linear velocity.
    @param wd Computed relative angular velocity.
  **/
  void computeControl(double pc, double thetac, double pd, double thetad, double &vd, double &wd);
private:
  double vd_max_, wd_max_, position_gain_, orientation_gain_;
  std::string ns_;
  ros::NodeHandle nh_;

  /**
    Initializes the controller with the controller parameters.
    Uses default values in case of error.
  **/
  void getParams();
};
}

#endif
