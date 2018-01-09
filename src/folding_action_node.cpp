#include <folding_assembly_controller/folding_controller.hpp>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

sensor_msgs::JointState state_;
bool got_first_, is_disabled_;

void jointStatesCb(const sensor_msgs::JointState::ConstPtr &msg)
{
  ROS_INFO_ONCE("Joint state received!");
  state_ = *msg;
  got_first_ = true;
}

void disableCb(const std_msgs::Bool::ConstPtr &msg)
{
  if (msg->data == true)
  {
    ROS_WARN("Folding controller disabled");
    is_disabled_ = true;
  }
  else
  {
    ROS_WARN("Folding controller enabled");
    is_disabled_ = false;
  }
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "/folding_controller");
  ros::NodeHandle n("~");
  std::string action_name;

  if (!n.getParam("action_name", action_name))
  {
    action_name = "folding_action";
  }

  folding_assembly_controller::FoldingController controller(action_name);

  is_disabled_ = false;
  got_first_ = false;
  ros::Subscriber joint_state_sub = n.subscribe("/joint_states", 1000, jointStatesCb);
  ros::Subscriber disable_sub = n.subscribe("/folding/disable", 1000, disableCb);
  ros::Publisher state_pub = n.advertise<sensor_msgs::JointState>("/yumi/joint_command", 1000);
  ros::Rate loop_rate(100);
  ros::Time prev_time = ros::Time::now();
  sensor_msgs::JointState command;

  while(ros::ok())
  {
    if (got_first_ && !is_disabled_)
    {
      command = controller.updateControl(state_, ros::Time::now() - prev_time);
      state_pub.publish(command);
    }
    else
    {
      ROS_WARN_ONCE("No joint state received");
    }
    prev_time = ros::Time::now();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
