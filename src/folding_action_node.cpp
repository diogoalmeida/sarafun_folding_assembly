#include <ros/ros.h>
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
                                                                                FoldingControllerFeedback,
                                                                                FoldingControllerResult>
{
public:
  FoldingController() : ControllerTemplate<FoldingControllerAction,
                                           FoldingControllerFeedback,
                                           FoldingControllerResult>("nhanha") {}
  virtual ~FoldingController() {}

  sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, const ros::Duration &dt)
  {
    sensor_msgs::JointState ret;

    return ret;
  }
private:
  void goalCB() {}
  void preemptCB() {}
};


int main(int argc, char ** argv)
{
  folding_algorithms::KalmanEstimator kalman_filter;
  folding_algorithms::FoldingPoseController pose_controller;
  folding_algorithms::AdaptiveController adaptive_velocity_controller;
  generic_control_toolbox::KDLManager kdl_manager("blah");
  generic_control_toolbox::WrenchManager wrench_manager();
  folding_algorithms::ECTSController ects_controller("rod", "surface", std::shared_ptr<generic_control_toolbox::KDLManager>(&kdl_manager));
  FoldingController controller;
  return 0;
}
