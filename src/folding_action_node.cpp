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
  std::string base_link, rod_gripping_frame, surface_gripping_frame;
  if (!nh_.getParam("kinematic_chain_base_link", base_link))
  {
    ROS_ERROR("Missing kinematic_chain_base_link parameter");
    return false;
  }

  // Initialize arms and set gripping points.
  kdl_manager_.reset(new generic_control_toolbox::KDLManager(base_link));
}

sensor_msgs::JointState FoldingController::controlAlgorithm(const sensor_msgs::JointState &current_state, const ros::Duration &dt)
{
  sensor_msgs::JointState ret;
  Eigen::Vector3d pc_est;
  KDL::Frame rod_eef_pose, surface_eef_pose;

  return ret;
}

bool FoldingController::setArm(const generic_control_toolbox::ArmInfo &msg)
{
    if(!kdl_manager_->initializeArm(msg.kdl_eef_frame))
    {
      return false;
    }

    if (!kdl_manager_->setGrippingPoint(msg.kdl_eef_frame, msg.gripping_frame))
    {
      return false;
    }

    if (msg.has_ft_sensor)
    {
      if (!wrench_manager_.initializeWrenchComm(msg.kdl_eef_frame, msg.sensor_frame, msg.gripping_frame, msg.sensor_topic))
      {
        return false;
      }
    }
    else
    {
      ROS_WARN("End-effector %s has no F/T sensor.", msg.kdl_eef_frame.c_str());
    }

    return true;
}

bool FoldingController::parseGoal(boost::shared_ptr<const FoldingControllerGoal> goal)
{
  rod_eef_ = goal->rod_arm.kdl_eef_frame;
  surface_eef_ = goal->surface_arm.kdl_eef_frame;

  if (!goal->rod_arm.has_ft_sensor && !goal->surface_arm.has_ft_sensor)
  {
    ROS_ERROR("At least one arm must have a F/T sensor");
    return false;
  }

  try
  {
    ects_controller_.reset(new folding_algorithms::ECTSController(rod_eef_, surface_eef_, kdl_manager_));
  }
  catch(std::logic_error &e)
  {
    ROS_ERROR("Exception when initializing the ECTS controller: %s.", e.what());
    return false;
  }

  if (!setArm(goal->rod_arm))
  {
    return false;
  }

  if (!setArm(goal->surface_arm))
  {
    return false;
  }

  return true;
}

void FoldingController::resetController()
{

}

int main(int argc, char ** argv)
{
  FoldingController controller("action_name");
  return 0;
}
