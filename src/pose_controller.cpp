#include <folding_assembly_controller/pose_controller.hpp>

namespace folding_algorithms{

    FoldingPoseController::FoldingPoseController(const std::string &ns) : ns_(ns)
    {
      nh_ = ros::NodeHandle("~");
      getParams();
    }

    FoldingPoseController::~FoldingPoseController(){}

    void FoldingPoseController::getParams()
    {
      if(!nh_.getParam(ns_ + "/vd_max", vd_max_))
      {
        ROS_WARN_STREAM("Missing vd_max in pose controller. Using default (" << ns_ << "/vd_max)");
        vd_max_ = 0.01;
      }

      if(!nh_.getParam(ns_ + "/wd_max", wd_max_))
      {
        ROS_WARN_STREAM("Missing wd_max in pose controller. Using default (" << ns_ << "/wd_max)");
        wd_max_ = 0.05;
      }

      if(!nh_.getParam(ns_ + "/position_gain", position_gain_))
      {
        ROS_WARN_STREAM("Missing position_gain in pose controller. Using default (" + ns_ + "/position_gain)");
        position_gain_ = 1;
      }

      if(!nh_.getParam(ns_ + "/orientation_gain", orientation_gain_))
      {
        ROS_WARN_STREAM("Missing orientation_gain in pose controller. Using default (" << ns_ << "/orientation_gain)");
        orientation_gain_ = 1;
      }
    }

    void FoldingPoseController::computeControl(double pc, double thetac, double pd, double thetad, double &vd, double &wd)
    {
      vd = position_gain_*(pc - pd);
      wd = orientation_gain_*(thetad - thetac);

      if (vd > vd_max_)
      {
        ROS_WARN("Vd is bigger than the threshold");
        vd = vd_max_;
      }

      if (vd < -vd_max_)
      {
        ROS_WARN("Vd is smaller than the threshold");
        vd = -vd_max_;
      }

      if (wd > wd_max_)
      {
        ROS_WARN("Wd is bigger than the threshold");
        wd = wd_max_;
      }

      if (wd < -wd_max_)
      {
        ROS_WARN("Wd is smaller than the threshold");
        wd = -wd_max_;
      }
    }
}
