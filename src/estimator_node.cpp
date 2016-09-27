#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/WrenchStamped.h>
#include <folding_assembly_controller/EstimateAction.h>
#include <folding_assembly_controller/contact_point_estimator.hpp>

class EstimateAction
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<folding_assembly_controller::EstimateAction> action_server_;
  std::string action_name_, wrench_topic_name_;

  folding_assembly_controller::EstimateFeedback feedback_;
  folding_assembly_controller::EstimateResult result_;

  Eigen::Vector3d f2_, t2_;

  KFEstimator1 estimator_;
  ros::Subscriber wrench_sub_;

  double estimate_frequency_;
  double z_offset_; // offset between sensor frame and assembly part

  /* Load the node parameters */
  bool loadParams()
  {
    if(!nh_.getParam("/folding_node/frequency", estimate_frequency_))
    {
      ROS_ERROR("%s could not retrive estimation frequency (/folding_node/frequency)!", action_name_.c_str());
      return false;
    }

    if(!nh_.getParam("/config/ft_sensor_topic", wrench_topic_name_))
    {
      ROS_WARN("FT sensor topic name not defined! Will set to default (/config/ft_sensor_topic)");
      wrench_topic_name_ = std::string("/ft_sensor");
    }

    return true;
  }

public:

  EstimateAction(std::string name) :
    action_server_(nh_, name, boost::bind(&EstimateAction::executeCB, this, _1), false),
    action_name_(name)
  {
    if(!loadParams())
    {
      ros::shutdown();
      exit(0);
    }

    f2_ = Eigen::Vector3d::Zero();
    t2_ = Eigen::Vector3d::Zero();

    wrench_sub_ = nh_.subscribe(wrench_topic_name_, 1, &EstimateAction::wrenchCallback, this);

    ROS_INFO("%s is starting the action server...", action_name_.c_str());
    action_server_.start();
    ROS_INFO("%s started!", action_name_.c_str());
  }

  /*
    Updates the available wrench values
  */
  void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
    Eigen::Matrix<double, 6, 1> wrench_eigen;

    tf::wrenchMsgToEigen(msg->wrench, wrench_eigen);

    f2_ = wrench_eigen.block<3,1>(0,0);
    t2_ = wrench_eigen.block<3,1>(3,0);
  }


  void executeCB(const folding_assembly_controller::EstimateGoalConstPtr &goal)
  {
    ros::Rate rate(estimate_frequency_);
    bool done = false, success = true;
    ros::Time begin_time = ros::Time::now();
    ros::Time begin_loop_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    double elapsed_time_sec, total_time_sec;
    Eigen::Vector3d contact_point = Eigen::Vector3d::Zero();

    geometry_msgs::Vector3 output_contact_point;
    geometry_msgs::Vector3 output_contact_point_computed;

    estimator_.initialize(contact_point);

    ROS_INFO("%s received a goal!", action_name_.c_str());

    while(!done)
    {
      if(action_server_.isPreemptRequested() || !ros::ok())
      {
        ROS_WARN("%s was preempted!", action_name_.c_str());
        result_.elapsed_time = (ros::Time::now() - begin_time).toSec();
        action_server_.setPreempted(result_);
        success = false;
        f2_ = Eigen::Vector3d::Zero();
        t2_ = Eigen::Vector3d::Zero();
        break;
      }

      current_time = ros::Time::now();
      elapsed_time_sec = (current_time - begin_loop_time).toSec();
      contact_point = estimator_.estimate(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), f2_, t2_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), elapsed_time_sec);
      begin_loop_time = ros::Time::now();

      output_contact_point_computed.x = -t2_(1)/f2_(2);
      output_contact_point_computed.y = 0;
      output_contact_point_computed.z = 0;

      tf::vectorEigenToMsg(contact_point, output_contact_point);
      feedback_.contact_point_estimate = output_contact_point;
      feedback_.contact_point_computed = output_contact_point_computed;
      feedback_.elapsed_time = (ros::Time::now() - begin_time).toSec();
      action_server_.publishFeedback(feedback_);

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
  ros::init(argc, argv, "estimator_node");

  EstimateAction estimate_action(ros::this_node::getName());
  ROS_INFO("Estimator node started with name %s", ros::this_node::getName().c_str());
  ros::spin();
  return 0;
}
