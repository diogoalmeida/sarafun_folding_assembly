#include <folding_assembly_controller/folding_action_node.hpp>

using namespace folding_assembly_controller;

FoldingController::FoldingController(const std::string &action_name) : ControllerTemplate<FoldingControllerAction,
                                                                        FoldingControllerGoal,
                                                                        FoldingControllerFeedback,
                                                                        FoldingControllerResult>(action_name)
{
  nh_ = ros::NodeHandle("~");

  if (!init())
  {
    throw std::logic_error("Missing parameters for the folding controller");
  }
}

FoldingController::~FoldingController() {}

bool FoldingController::init()
{
  std::string base_link;
  if (!nh_.getParam("kinematic_chain_base_link", base_link))
  {
    ROS_ERROR("Missing kinematic_chain_base_link parameter");
    return false;
  }

  kdl_manager_.reset(new generic_control_toolbox::KDLManager(base_link));
}

sensor_msgs::JointState FoldingController::controlAlgorithm(const sensor_msgs::JointState &current_state, const ros::Duration &dt)
{
  sensor_msgs::JointState ret;

  return ret;
}

bool FoldingController::parseGoal(boost::shared_ptr<const FoldingControllerGoal> goal)
{
  try
  {
    ects_controller_.reset(new folding_algorithms::ECTSController(goal->rod_eef, goal->surface_eef, kdl_manager_));
  }
  catch(std::logic_error &e)
  {
    ROS_ERROR("Exception when initializing the ECTS controller: %s.", e.what());
    return false;
  }

  return true;
}

void FoldingController::resetController()
{

}

int main(int argc, char ** argv)
{
  folding_algorithms::KalmanEstimator kalman_filter;
  folding_algorithms::FoldingPoseController pose_controller;
  folding_algorithms::AdaptiveController adaptive_velocity_controller;
  generic_control_toolbox::KDLManager kdl_manager("blah");
  generic_control_toolbox::WrenchManager wrench_manager;
  folding_algorithms::ECTSController ects_controller("rod", "surface", std::shared_ptr<generic_control_toolbox::KDLManager>(&kdl_manager));
  FoldingController controller("action_name");
  return 0;
}
