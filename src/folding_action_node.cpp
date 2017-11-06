#include <folding_assembly_controller/folding_controller.hpp>
#include <sensor_msgs/JointState.h>

sensor_msgs::JointState state;
bool got_first;

void jointStatesCb(const sensor_msgs::JointState::ConstPtr &msg)
{
  state = *msg;
  got_first = true;
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "/folding_controller");
  ros::NodeHandle n;
  folding_assembly_controller::FoldingController controller("action_name");

  ros::Subscriber joint_state_sub = n.subscribe("/joint_states", 1000, jointStatesCb);
  ros::Publisher state_pub = n.advertise<sensor_msgs::JointState>("/joint_command", 1000);
  ros::Rate loop_rate(250);
  ros::Time prev_time = ros::Time::now();
  sensor_msgs::JointState command;
  got_first = false;

  sleep(1.0);

  while(ros::ok())
  {
    if (got_first)
    {
      command = controller.controlAlgorithm(state, ros::Time::now() - prev_time);
      state_pub.publish(command);
    }
    else
    {
      ROS_WARN("No joint state received");
    }
    prev_time = ros::Time::now();
    loop_rate.sleep();
  }
  return 0;
}
