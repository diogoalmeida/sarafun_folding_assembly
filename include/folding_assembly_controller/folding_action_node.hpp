#ifndef __FOLDING_ACTION_NODE__
#define __FOLDING_ACTION_NODE__

#include <ros/ros.h>
#include <stdexcept>
#include <generic_control_toolbox/kdl_manager.hpp>
#include <generic_control_toolbox/wrench_manager.hpp>
#include <generic_control_toolbox/controller_template.hpp>
#include <folding_assembly_controller/ects.hpp>
#include <folding_assembly_controller/kalman_filter.hpp>
#include <folding_assembly_controller/pose_controller.hpp>
#include <folding_assembly_controller/FoldingControllerAction.h>
#include <folding_assembly_controller/adaptive_velocity_controller.hpp>

using namespace folding_assembly_controller;
class FoldingController : public generic_control_toolbox::ControllerTemplate<FoldingControllerAction,
                                                                                FoldingControllerGoal,
                                                                                FoldingControllerFeedback,
                                                                                FoldingControllerResult>
{
public:
  FoldingController(const std::string &action_name);
  virtual ~FoldingController();


private:
  bool parseGoal(boost::shared_ptr<const FoldingControllerGoal> goal);
  void resetController();
  sensor_msgs::JointState controlAlgorithm(const sensor_msgs::JointState &current_state, const ros::Duration &dt);

  /**
    Get the required parameters for initializing the folding controller
    elements.

    @return False if something goes wrong, true otherwise.
  **/
  bool init();

  /**
    Initializes the manager classes with the given arm info.

    @param msg The arm info message for one arm.
    @return False if something goes wrong, true otherwise.
  **/
  bool setArm(const generic_control_toolbox::ArmInfo &msg);

  ros::NodeHandle nh_;
  std::string rod_eef_, surface_eef_;
  folding_algorithms::KalmanEstimator kalman_filter_;
  folding_algorithms::FoldingPoseController pose_controller_;
  folding_algorithms::AdaptiveController adaptive_velocity_controller_;
  std::shared_ptr<generic_control_toolbox::KDLManager> kdl_manager_;
  generic_control_toolbox::WrenchManager wrench_manager_;
  std::shared_ptr<folding_algorithms::ECTSController> ects_controller_;
};
#endif
