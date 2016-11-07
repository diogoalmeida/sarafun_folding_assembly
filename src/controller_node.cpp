#include <ros/ros.h>
#include <folding_assembly_controller/controller.hpp>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <folding_assembly_controller/FoldingAssemblyAction.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>

#include <sarafun_msgs/CompleteFeedback.h>

#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <urdf/model.h>

class FoldingAction
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<folding_assembly_controller::FoldingAssemblyAction> action_server_;
  std::string action_name_, rod_arm_, prefix_;
  std::string yumi_state_topic_, yumi_command_topic_;
  std::string left_tooltip_name_, right_tooltip_name_;

  folding_assembly_controller::FoldingAssemblyFeedback feedback_;
  folding_assembly_controller::FoldingAssemblyResult result_;

  ros::Publisher joint_publisher_;
  ros::Subscriber joint_subscriber_;

  // TF declarations (for visualisation purposes)
  tf::Transform p1_tf_, pc_tf_;

  foldingController controller_;
  KDL::ChainIkSolverVel_wdls *ikvel_;
  KDL::ChainFkSolverVel_recursive *fkvel_;
  KDL::ChainFkSolverPos_recursive *fkpos_;
  KDL::Chain chain_;
  KDL::JntArray joint_positions_;
  KDL::Tree tree_;
  urdf::Model model_;

  double control_frequency_, desired_contact_force_, eps_;

  /* Load the node parameters */
  bool loadParams()
  {
    if(!nh_.getParam("/folding_node/frequency", control_frequency_))
    {
      ROS_ERROR("%s could not retrive control frequency (/folding_node/frequency)!", action_name_.c_str());
      return false;
    }

    if(!nh_.getParam("/folding_node/wdls/epsilon", eps_))
    {
      ROS_WARN("%s could not retrive epsilon for damped least squares (/folding_node/wdls/epsilon)! Using default", action_name_.c_str());
      eps_ = 0.001;
    }

    if(!nh_.getParam("/folding_node/arm", rod_arm_))
    {
      ROS_WARN("%s could not retrive the rod arm (/folding_node/arm)! Using default", action_name_.c_str());
      rod_arm_ = std::string("right");
    }

    if(!nh_.getParam("/folding_node/yumi_state_topic", yumi_state_topic_))
    {
      ROS_ERROR("%s could not retrive yumi's state topic name (/folding_node/yumi_state_topic)!", action_name_.c_str());
      return false;
    }

    if(!nh_.getParam("/folding_node/yumi_command_topic", yumi_command_topic_))
    {
      ROS_ERROR("%s could not retrive yumi's command topic name (/folding_node/yumi_command_topic)!", action_name_.c_str());
      return false;
    }

    if(!nh_.getParam("/folding_node/right_tooltip_name", right_tooltip_name_))
    {
      ROS_ERROR("%s could not retrive yumi's right_tooltip_name (/folding_node/right_tooltip_name)!", action_name_.c_str());
      return false;
    }

    if(!nh_.getParam("/folding_node/left_tooltip_name", left_tooltip_name_))
    {
      ROS_ERROR("%s could not retrive yumi's left_tooltip_name (/folding_node/left_tooltip_name)!", action_name_.c_str());
      return false;
    }

    if(!model_.initParam("/robot_description")){
        ROS_ERROR("ERROR getting robot description");
        return false;
    }

    return true;
  }

public:

  FoldingAction(std::string name) :
    action_server_(nh_, name, boost::bind(&FoldingAction::executeCB, this, _1), false),
    action_name_(name)
  {
    if(!loadParams())
    {
      ros::shutdown();
      exit(0);
    }

    joint_positions_.resize(7);
    kdl_parser::treeFromUrdfModel(model_, tree_);

    if (rod_arm_ == "left")
    {
      tree_.getChain("base_link", left_tooltip_name_, chain_);
      ROS_INFO("NUM OF CHAIN LINKS: %d", chain_.getNrOfJoints());
      prefix_ = std::string("left_");
    }
    else
    {
      tree_.getChain("base_link", right_tooltip_name_, chain_);
      ROS_INFO("NUM OF CHAIN LINKS: %d", chain_.getNrOfJoints());
      prefix_ = std::string("right_");
    }

    ikvel_ = new KDL::ChainIkSolverVel_wdls(chain_, eps_);
    fkvel_ = new KDL::ChainFkSolverVel_recursive(chain_);
    fkpos_ = new KDL::ChainFkSolverPos_recursive(chain_);

    joint_publisher_ = nh_.advertise<sensor_msgs::JointState>(yumi_command_topic_, 1);
    joint_subscriber_ = nh_.subscribe(yumi_state_topic_, 1, &FoldingAction::jointStateCallback, this);

    ROS_INFO("%s is starting the action server...", action_name_.c_str());
    action_server_.start();
    ROS_INFO("%s started!", action_name_.c_str());
  }

  void jointStateCallback(const sarafun_msgs::CompleteFeedbackConstPtr &msg)
  {
    bool success = false;
    int count = 1;
    std::string joint_name;

    for (int i=0; i < msg->joint_state.position.size(); i++)
    {
      joint_name = prefix_ + std::string("joint_") + std::to_string(count);

      if(msg->joint_state.name[i] == joint_name)
      {
        joint_positions_(count - 1) = msg->joint_state.position[i];
        ROS_INFO("Getting joint position %d: %.6f", count, joint_positions_(count-1));
        count ++;

        if(count > 7)
        {
          success = true;
          break;
        }
      }
    }

    if(!success)
    {
      ROS_ERROR("%s FOUND %d JOINTS IN THE JOINT STATE CALLBACK!!", action_name_.c_str(), count);
    }
  }

  void getDebugOptions(const folding_assembly_controller::FoldingAssemblyGoalConstPtr &goal)
  {
    if(goal->estimate_type == goal->NO_ESTIMATE)
    {
      controller_.disableEstimate();
    }
    else
    {
      if(goal->estimate_type == goal->DIRECT_COMPUTATION)
      {
        controller_.enableDirectEstimate();
      }
      else
      {
        if(goal->estimate_type == goal->KALMAN_FILTER)
        {
          controller_.enableKF();
        }
      }
    }

    if(goal->force_control_type == goal->NO_FORCE_CONTROL)
    {
      controller_.disableForceControl();
    }
    else
    {
      if(goal->force_control_type == goal->CONTROL_ALONG_NORMAL)
      {
        controller_.normalForceControl();
      }
      else
      {
        if(goal->force_control_type == goal->CONTROL_ALONG_TANGENT)
        {
          controller_.tangentForceControl();
        }
        else
        {
          if(goal->force_control_type == goal->CONTROL_ALONG_ROD)
          {
            controller_.rodForceControl();
          }
          else
          {
            if(goal->force_control_type == goal->DEBUG_ROD_FORCE_CONTROL)
            {
              controller_.debugRodForceControl();
            }
          }
        }
      }
    }
  }

  void executeCB(const folding_assembly_controller::FoldingAssemblyGoalConstPtr &goal)
  {
    ros::Rate rate(control_frequency_);
    bool done = false, success = true;
    ros::Time begin_time = ros::Time::now();
    ros::Time begin_loop_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    double elapsed_time_sec, total_time_sec;
    Eigen::Vector3d v_out, w_out;
    Eigen::MatrixXd twist_eig(6, 1);
    Eigen::Matrix<double, 6, 1>  measured_twist_eig;
    double vd, wd, theta;
    Eigen::Vector3d contact_point, p1_eig;
    KDL::JntArray commanded_joint_velocities;
    KDL::JntArrayVel joint_velocities;
    KDL::Twist input_twist;
    KDL::FrameVel v1;
    KDL::Frame p1, base_link, contact_point_frame;
    static tf::TransformBroadcaster tf_broadcaster;

    geometry_msgs::Twist output_twist;
    geometry_msgs::Vector3 output_contact_point;

    ROS_INFO("%s received a goal!", action_name_.c_str());

    // Get desired linear and angular velocity
    vd = goal->desired_velocities.linear.x;
    wd = goal->desired_velocities.angular.z;
    desired_contact_force_ = goal->contact_force;

    // Get debug options
    getDebugOptions(goal);

    while(!done)
    {
      if(action_server_.isPreemptRequested() || !ros::ok())
      {
        ROS_WARN("%s was preempted!", action_name_.c_str());

        for (int i = 0; i < commanded_joint_velocities.columns(); i++)
        {
            commanded_joint_velocities(i) = 0;
        }

        publishJointState(commanded_joint_velocities);
        result_.elapsed_time = (ros::Time::now() - begin_time).toSec();
        action_server_.setPreempted(result_);
        success = false;
        break;
      }

      // get position of the end-effector
      fkpos_->JntToCart(joint_positions_, p1);
      // tf::vectorKDLToEigen(p1.p, p1_eig);

      // get velocity of the end-effector
      fkvel_->JntToCart(joint_velocities, v1);
      tf::twistKDLToEigen(v1.deriv(), measured_twist_eig);

      controller_.updateState(p1, measured_twist_eig);

      current_time = ros::Time::now();
      elapsed_time_sec = (current_time - begin_loop_time).toSec();
      controller_.control(vd, wd, desired_contact_force_, v_out, w_out, elapsed_time_sec);
      begin_loop_time = ros::Time::now();

      twist_eig << v_out, w_out;
      controller_.getEstimates(contact_point, theta, contact_point_frame);
      tf::twistEigenToMsg(twist_eig, output_twist);
      tf::vectorEigenToMsg(contact_point, output_contact_point);
      tf::twistEigenToKDL (twist_eig, input_twist);

      feedback_.commanded_velocities = output_twist;
      feedback_.contact_point_estimate = output_contact_point;
      feedback_.angle_estimate = theta;
      feedback_.elapsed_time = (ros::Time::now() - begin_time).toSec();
      action_server_.publishFeedback(feedback_);
      tf::poseKDLToTF(p1, p1_tf_);
      tf::poseKDLToTF(contact_point_frame, pc_tf_);
      tf_broadcaster.sendTransform(tf::StampedTransform(p1_tf_, ros::Time::now(), "base_link", "end_effector"));
      tf_broadcaster.sendTransform(tf::StampedTransform(pc_tf_, ros::Time::now(), "base_link", "contact_point"));

      // Convert twist to joint velocities
      // Need to get joint positions from yumi
      // joint_position = ...?
      ikvel_->CartToJnt(joint_positions_, input_twist, commanded_joint_velocities);
      publishJointState(commanded_joint_velocities);

      // Output joint velocities to yumi
      rate.sleep();
    }

    // Need to define what is a successful folding execution
    if(success)
    {
      result_.elapsed_time = (ros::Time::now() - begin_time).toSec();
      ROS_INFO("%s has succeeded!", action_name_.c_str());
      action_server_.setSucceeded(result_);
    }
  }

  void publishJointState(const KDL::JntArray &joint_velocities)
  {
    sensor_msgs::JointState joint_command;
    // left_joint_1 or right_joint_1, to 7. 1 is closer to base link
    joint_command.header.stamp = ros::Time::now();

    std::string other_arm = (prefix_=="left_")? "right_":"left_";


    for(int i=0; i < chain_.getNrOfJoints(); i++)
    {
      joint_command.name.push_back(prefix_ + std::string("joint_") + std::to_string(i+1));
      joint_command.position.push_back(0.0);
      // joint_command.velocity.push_back(joint_velocities.qdot(i));
      joint_command.velocity.push_back(joint_velocities(i));
      joint_command.effort.push_back(0.0);
    }

    for(int i=0; i < 7; i++)
    {
    //joint_command.name.push_back("hack");
      joint_command.name.push_back(other_arm+std::string("joint_")+std::to_string(i+1));
      joint_command.velocity.push_back(0.0);
      joint_command.position.push_back(0.0);
      joint_command.effort.push_back(0.0);
    }

    joint_publisher_.publish(joint_command);
  }
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "folding_assembly_node");

  FoldingAction folding_action(ros::this_node::getName());
  ROS_INFO("Folding assembly node started with name %s", ros::this_node::getName().c_str());
  ros::spin();
  return 0;
}
