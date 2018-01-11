#include <folding_assembly_controller/folding_controller.hpp>
#include <generic_control_toolbox/controller_action_node.hpp>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "/folding_controller");
  folding_assembly_controller::FoldingController controller("fold");
  generic_control_toolbox::ControllerActionNode action_node;

  action_node.runController(controller);
  return 0;
}
