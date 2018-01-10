#include <folding_assembly_controller/folding_controller.hpp>

namespace folding_assembly_controller
{
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
    std::string rod_gripping_frame, surface_gripping_frame;
    if (!nh_.getParam("kinematic_chain_base_link", base_frame_))
    {
      ROS_ERROR("Missing kinematic_chain_base_link parameter");
      return false;
    }

    // Initialize arms and set gripping points.
    kdl_manager_ = std::make_shared<generic_control_toolbox::KDLManager>(base_frame_);

    if (!setArm("rod_arm", rod_eef_))
    {
      return false;
    }

    if (!setArm("surface_arm", surface_eef_))
    {
      return false;
    }

    if (!nh_.getParam("contact_point_offset", contact_offset_))
    {
      ROS_ERROR("Missing contact_point_offset parameter");
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

    // Initialize markers
    marker_manager_.addMarkerGroup("estimates", "folding_markers/estimates");
    marker_manager_.addMarkerGroup("sticks", "folding_markers/sticks");
    marker_manager_.addMarkerGroup("pose_feedback", "folding_markers/pose_feedback");
    marker_manager_.addMarker("estimates", "translational_estimate", "folding_assembly", base_frame_, generic_control_toolbox::MarkerType::arrow);
    marker_manager_.setMarkerColor("estimates", "translational_estimate", 1, 0, 0);
    marker_manager_.addMarker("estimates", "rotational_estimate", "folding_assembly", base_frame_, generic_control_toolbox::MarkerType::arrow);
    marker_manager_.setMarkerColor("estimates", "rotational_estimate", 0, 1, 0);
    marker_manager_.addMarker("estimates", "contact_point_estimate", "folding_assembly", base_frame_, generic_control_toolbox::MarkerType::sphere);
    marker_manager_.setMarkerColor("estimates", "contact_point_estimate", 0, 0, 1);
    marker_manager_.addMarker("sticks", "r1", "folding_assembly", base_frame_, generic_control_toolbox::MarkerType::arrow);
    marker_manager_.setMarkerColor("sticks", "r1", 1, 0, 0);
    marker_manager_.addMarker("sticks", "r2", "folding_assembly", base_frame_, generic_control_toolbox::MarkerType::arrow);
    marker_manager_.setMarkerColor("sticks", "r2", 0, 1, 0);
    marker_manager_.addMarker("pose_feedback", "pose_target", "folding_assembly", base_frame_, generic_control_toolbox::MarkerType::arrow);
    marker_manager_.setMarkerColor("pose_feedback", "pose_target", 0, 0, 1);
    marker_manager_.addMarker("pose_feedback", "current_pose", "folding_assembly", base_frame_, generic_control_toolbox::MarkerType::arrow);
    marker_manager_.setMarkerColor("pose_feedback", "current_pose", 1, 0, 0);

    dynamic_reconfigure_server_.reset(new dynamic_reconfigure::Server<FoldingConfig>(ros::NodeHandle(ros::this_node::getName() + "/folding_config")));
    dynamic_reconfigure_callback_ = boost::bind(&FoldingController::reconfig, this, _1, _2);
    dynamic_reconfigure_server_->setCallback(dynamic_reconfigure_callback_);
    twist_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>(ros::this_node::getName() + "/adaptive_twist", 1);
    return true;
  }

  sensor_msgs::JointState FoldingController::controlAlgorithm(const sensor_msgs::JointState &current_state, const ros::Duration &dt)
  {
    sensor_msgs::JointState ret = current_state;
    KDL::Frame p1, p2;
    Eigen::Affine3d p1_eig, p2_eig, pc_est;

    kdl_manager_->getGrippingPoint(rod_eef_, current_state, p1);
    kdl_manager_->getGrippingPoint(surface_eef_, current_state, p2);
    tf::transformKDLToEigen(p1, p1_eig);
    tf::transformKDLToEigen(p2, p2_eig);

    KDL::Twist v1, v2;
    KDL::Wrench wrench_kdl;
    static tf::TransformBroadcaster br;
    tf::Transform wrench_transform;

    Eigen::Matrix<double, 6, 1> v1_eig, wrench2, wrench2_rotated;
    kdl_manager_->getGrippingTwist(rod_eef_, current_state, v1);
    wrench_manager_.wrenchAtGrippingPoint(surface_eef_, wrench2);
    tf::wrenchEigenToKDL(wrench2, wrench_kdl);
    wrench_kdl = p2.M*wrench_kdl;
    tf::wrenchKDLToEigen(wrench_kdl, wrench2_rotated); // wrench2_rotated is at the gripping point p2, expressed in the base frame coordinates
    tf::twistKDLToEigen(v1, v1_eig);
    wrench_transform.setOrigin(tf::Vector3(p2.p.x(), p2.p.y(), p2.p.z()));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    wrench_transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(wrench_transform, ros::Time::now(), base_frame_, "p2_rotated"));

    pc_est.linear() = p1_eig.linear();
    pc_est.translation() = kalman_filter_.estimate(p1_eig.translation(), v1_eig, p2_eig.translation(), wrench2_rotated, dt.toSec()); // The kalman filter estimates in the base frame, thus the wrench should be writen in that basis.
    // TEMP
    // pc_est.translation() = p1_eig.translation() + contact_offset_*p1_eig.matrix().block<3,1>(0, 2);
    // end TEMP
    marker_manager_.setMarkerPose("estimates", "contact_point_estimate", pc_est);

    Eigen::Vector3d t_est, k_est, n_est, r1, r2;
    Eigen::Matrix<double, 6, 1> relative_twist;
    double vd = 0, wd = 0;
    KDL::Frame sensor_to_base;
    KDL::Vector t_est_kdl, k_est_kdl;

    kdl_manager_->getSensorPoint(surface_eef_, current_state, sensor_to_base);
    adaptive_velocity_controller_.getEstimates(t_est, k_est);
    tf::vectorEigenToKDL(t_est, t_est_kdl);
    tf::vectorEigenToKDL(k_est, k_est_kdl);
    t_est_kdl = p2.M*t_est_kdl; // Convert motion estimates to base frame
    k_est_kdl = p2.M*k_est_kdl;
    tf::vectorKDLToEigen(t_est_kdl, t_est);
    tf::vectorKDLToEigen(k_est_kdl, k_est);
    n_est = t_est.cross(k_est);
    marker_manager_.setMarkerPoints("estimates", "translational_estimate", pc_est.translation(), pc_est.translation() + 0.1*t_est);
    marker_manager_.setMarkerPoints("estimates", "rotational_estimate", pc_est.translation(), pc_est.translation() + 0.1*k_est);
    r1 = pc_est.translation() - p1_eig.translation();
    r2 = pc_est.translation() - p2_eig.translation();
    marker_manager_.setMarkerPoints("sticks", "r1", p1_eig.translation(), pc_est.translation());
    marker_manager_.setMarkerPoints("sticks", "r2", p2_eig.translation(), pc_est.translation());

    if (pose_goal_)
    {
      double pc_proj, theta_proj;
      Eigen::Vector3d r2_y, pose_target_dir, target_point;
      pc_proj = r2.dot(t_est);
      r2_y = r2 - r2.dot(t_est)*t_est;
      theta_proj = atan2(-r1.dot(n_est), -r1.dot(t_est)); // want vector from contact to end-effector
      target_point = p2_eig.translation() + pc_goal_*t_est + r2_y;
      pose_target_dir = t_est*cos(thetac_goal_) + n_est*sin(thetac_goal_);
      ROS_DEBUG_STREAM("Theta proj: " << theta_proj);
      pose_controller_.computeControl(pc_proj, theta_proj, pc_goal_, thetac_goal_, vd, wd);
      ROS_DEBUG_STREAM("Wd: " << wd);
      marker_manager_.setMarkerPoints("pose_feedback", "pose_target", target_point, p1_eig.translation());
      marker_manager_.setMarkerPoints("pose_feedback", "current_pose", p2_eig.translation() + r2_y + pc_proj*t_est,  p1_eig.translation());
    }
    else
    {
      vd = vd_;
      wd = wd_;
    }

    KDL::Twist relative_twist_kdl;
    geometry_msgs::WrenchStamped twist_as_wrench;

    relative_twist = adaptive_velocity_controller_.control(wrench2, vd, wd, dt.toSec()); // twist expressed at the contact point, in p2 coordinates

    tf::twistEigenToKDL(relative_twist, relative_twist_kdl);
    relative_twist_kdl = p2.M*relative_twist_kdl;
    tf::twistKDLToEigen(relative_twist_kdl, relative_twist);

    twist_as_wrench.header.frame_id = "p2_rotated";
    twist_as_wrench.header.stamp = ros::Time::now();
    twist_as_wrench.wrench.force.x = relative_twist_kdl.vel.x();
    twist_as_wrench.wrench.force.y = relative_twist_kdl.vel.y();
    twist_as_wrench.wrench.force.z = relative_twist_kdl.vel.z();
    twist_as_wrench.wrench.torque.x = relative_twist_kdl.rot.x();
    twist_as_wrench.wrench.torque.y = relative_twist_kdl.rot.y();
    twist_as_wrench.wrench.torque.z = relative_twist_kdl.rot.z();
    twist_pub_.publish(twist_as_wrench);

    KDL::Frame eef1, eef2;
    Eigen::Affine3d eef1_eig, eef2_eig;

    kdl_manager_->getEefPose(rod_eef_, current_state, eef1);
    kdl_manager_->getEefPose(surface_eef_, current_state, eef2);
    tf::transformKDLToEigen(eef1, eef1_eig);
    tf::transformKDLToEigen(eef2, eef2_eig);

    Eigen::Matrix<double, 14, 1> qdot;
    // need to use virtual sticks up to the end-effector location, not the grasping point. TODO: Fix this
    qdot = ects_controller_->control(current_state, pc_est.translation() - eef1_eig.translation(), pc_est.translation() - eef2_eig.translation(), Eigen::Matrix<double, 6, 1>::Zero(), relative_twist);      kdl_manager_->getJointState(rod_eef_, qdot.block<7, 1>(0, 0), ret);
    kdl_manager_->getJointState(surface_eef_, qdot.block<7,1>(7, 0), ret);
    marker_manager_.publishMarkers();

    return ret;
  }

  bool FoldingController::setArm(const std::string &arm_name, std::string &eef_name)
  {
      generic_control_toolbox::ArmInfo info;

      if(!generic_control_toolbox::getArmInfo(arm_name, info))
      {
        return false;
      }

      eef_name = info.kdl_eef_frame;

      if(!generic_control_toolbox::setKDLManager(info, kdl_manager_))
      {
        return false;
      }

      if(!generic_control_toolbox::setWrenchManager(info, wrench_manager_))
      {
        return false;
      }

      return true;
  }

  bool FoldingController::parseGoal(boost::shared_ptr<const FoldingControllerGoal> goal)
  {
    Eigen::Vector3d t_init, k_init;
    adaptive_velocity_controller_.setReferenceForce(goal->adaptive_params.goal_force);
    t_init << sin(goal->adaptive_params.init_t_error), 0, cos(goal->adaptive_params.init_t_error);
    k_init << cos(goal->adaptive_params.init_k_error), sin(goal->adaptive_params.init_k_error), 0;
    adaptive_velocity_controller_.initEstimates(t_init, k_init);

    if (goal->use_pose_goal)
    {
      ROS_INFO("Using pose goal");
      pose_goal_ = true;
      pc_goal_ = goal->pose_goal.pd;
      thetac_goal_ = goal->pose_goal.thetad;
    }
    else
    {
      ROS_INFO("Using velocity goal");
      pose_goal_ = false;
      vd_ = goal->velocity_goal.vd;
      wd_ = goal->velocity_goal.wd;
    }

    KDL::Frame p2;
    Eigen::Affine3d p2_eig;
    kdl_manager_->getGrippingPoint(surface_eef_, lastState(sensor_msgs::JointState()), p2);
    tf::transformKDLToEigen(p2, p2_eig);
    kalman_filter_.initialize(p2_eig.translation());

    return true;
  }

  void FoldingController::reconfig(FoldingConfig &config, uint32_t level)
  {
    if (config.use_values)
    {
      pose_goal_ = config.pose_goal;
      adaptive_velocity_controller_.setReferenceForce(config.desired_contact_force);

      if (config.groups.ects.use_values_ects)
      {
        ects_controller_->setAlpha(config.groups.ects.alpha);
      }

      if (pose_goal_ && config.groups.pose_control.use_values_pose)
      {
        pc_goal_ = config.groups.pose_control.translational_offset;
        thetac_goal_ = config.groups.pose_control.angular_offset;
      }

      if (!pose_goal_ && config.groups.velocity_control.use_values_velocity)
      {
        vd_ = config.groups.velocity_control.translational_velocity;
        wd_ = config.groups.velocity_control.angular_velocity;
      }
    }
  }

  void FoldingController::resetController()
  {
    adaptive_velocity_controller_.reset();
  }
}
