#include <ros/ros.h>
#include <folding_assembly_controller/controller.hpp>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <folding_assembly_controller/FoldingAssemblyAction.h>

#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <urdf/model.h>


class FoldingAction
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<folding_assembly_controller::FoldingAssemblyAction> action_server_;
  std::string action_name_, rod_arm_;

  folding_assembly_controller::FoldingAssemblyFeedback feedback_;
  folding_assembly_controller::FoldingAssemblyResult result_;

  foldingController controller_;
  KDL::ChainIkSolverVel_wdls *ikvel_;
  KDL::Chain chain_;
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

    if(!model_.initParam("/robot_description")){
        ROS_ERROR("ERROR getting robot description");
        return false;
    }

    kdl_parser::treeFromUrdfModel(model_, tree_);

    if (rod_arm_ == "left")
    {
      tree_.getChain("base_link", "left_hand_base", chain_);
    }
    else
    {
      tree_.getChain("base_link", "right_hand_base", chain_);
    }

    ikvel_ = new KDL::ChainIkSolverVel_wdls(chain_, eps_);

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

    ROS_INFO("%s is starting the action server...", action_name_.c_str());
    action_server_.start();
    ROS_INFO("%s started!", action_name_.c_str());
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
    double vd, wd, theta;
    Eigen::Vector3d contact_point;
    KDL::JntArray joint_positions, commanded_joint_velocities;
    KDL::Twist input_twist;

    geometry_msgs::Twist output_twist;
    geometry_msgs::Vector3 output_contact_point;

    // Get desired linear and angular velocity
    vd = goal->desired_velocities.linear.x;
    wd = goal->desired_velocities.angular.z;
    desired_contact_force_ = goal->contact_force;

    // Get debug options
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
        }
      }
    }

    ROS_INFO("%s received a goal!", action_name_.c_str());

    while(!done)
    {
      if(action_server_.isPreemptRequested() || !ros::ok())
      {
        ROS_WARN("%s was preempted!", action_name_.c_str());
        result_.elapsed_time = (ros::Time::now() - begin_time).toSec();
        action_server_.setPreempted(result_);
        success = false;
        break;
      }

      current_time = ros::Time::now();
      elapsed_time_sec = (current_time - begin_loop_time).toSec();
      controller_.control(vd, wd, desired_contact_force_, v_out, w_out, elapsed_time_sec);
      begin_loop_time = ros::Time::now();

      twist_eig << v_out, w_out;
      controller_.getEstimates(contact_point, theta);
      tf::twistEigenToMsg(twist_eig, output_twist);
      tf::vectorEigenToMsg(contact_point, output_contact_point);
      tf::twistEigenToKDL (twist_eig, input_twist);

      feedback_.commanded_velocities = output_twist;
      feedback_.contact_point_estimate = output_contact_point;
      feedback_.angle_estimate = theta;
      feedback_.elapsed_time = (ros::Time::now() - begin_time).toSec();
      action_server_.publishFeedback(feedback_);

      // Convert twist to joint velocities
      // Need to get joint positions from yumi
      // joint_position = ...?
      ikvel_->CartToJnt(joint_positions, input_twist, commanded_joint_velocities);
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
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "folding_assembly_node");

  FoldingAction folding_action(ros::this_node::getName());
  ROS_INFO("Folding assembly node started with name %s", ros::this_node::getName().c_str());
  ros::spin();
  return 0;
}
