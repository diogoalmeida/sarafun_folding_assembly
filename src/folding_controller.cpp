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

  bool FoldingController::checkAxis(const std::string &axis) const
  {
    if (axis != "x" && axis != "y" && axis != "z" && axis != "-x" && axis != "-y" && axis != "-z")
    {
      return false;
    }
    
    return true;
  }
  
  bool FoldingController::init()
  {
    std::string rod_gripping_frame, surface_gripping_frame;
    if (!nh_.getParam("kinematic_chain_base_link", base_frame_))
    {
      ROS_ERROR("Missing kinematic_chain_base_link parameter");
      return false;
    }

    if (!nh_.getParam("grasp/rotational_axis", rot_axis_))
    {
      ROS_WARN("Missing grasp/rotational_axis parameter, using default");
      rot_axis_ = "y";
    }

    if (!nh_.getParam("grasp/translational_axis", trans_axis_))
    {
      ROS_WARN("Missing grasp/translational_axis parameter, using default");
      trans_axis_ = "x";
    }
    
    if (!nh_.getParam("rod_arm/align_axis", p1_align_))
    {
      ROS_ERROR("Missing rod arm align axis");
      return false;
    }
    
    if (!nh_.getParam("surface_arm/align_axis", p2_align_))
    {
      ROS_ERROR("Missing surface arm align axis");
      return false;
    }
    
    if (!nh_.getParam("base_align", base_align_))
	{
	  ROS_ERROR("Missing base align axis");
	  return false;
	}
    
    if (!nh_.getParam("initial_wait_time", wait_time_))
    {
      ROS_WARN("Missing initial_wait_time, using default");
      wait_time_ = 5.0;
    }

    if (!checkAxis(rot_axis_))
    {
      ROS_ERROR_STREAM("Invalid rotational axis parameter: " << rot_axis_);
      return false;
    }

    if (!checkAxis(trans_axis_))
    {
      ROS_ERROR_STREAM("Invalid translational axis parameter: " << trans_axis_);
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

    relative_pose_controller_ = folding_algorithms::FoldingPoseController("relative_pose_controller");
    absolute_pose_controller_ = folding_algorithms::FoldingPoseController("absolute_pose_controller");

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
    marker_manager_.addMarker("estimates", "computed_p1", "folding_assembly", base_frame_, generic_control_toolbox::MarkerType::sphere);
    marker_manager_.setMarkerColor("estimates", "contact_point_estimate", 1, 1, 1);
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
    debug_twist_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>(ros::this_node::getName() + "/gripping_point_twist", 1);
    return true;
  }
  
  KDL::Vector FoldingController::getAxis(const KDL::Frame &pose, const std::string &axis) const
  {
    if (axis == "x")
    {
      return pose.M.UnitX();
    }
    
    if (axis == "-x")
    {
      return -pose.M.UnitX();
    }
    
    if (axis =="y")
    {
      return pose.M.UnitY();
    }
    
    if (axis == "-y")
    {
      return -pose.M.UnitY();
    }
    
    if (axis == "z")
    {
      return pose.M.UnitZ();
    }
    
    if (axis == "-z")
    {
      return -pose.M.UnitZ();
    }
    
    ROS_ERROR("Got invalid axis name, returning default");
    return pose.M.UnitX();
  }

  sensor_msgs::JointState FoldingController::controlAlgorithm(const sensor_msgs::JointState &current_state, const ros::Duration &dt)
  {
    sensor_msgs::JointState ret = current_state;
    KDL::Frame p1, p2;
    Eigen::Affine3d p1_eig, p2_eig, pc_est;
    bool compute_control = false;

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
    
    if ((ros::Time::now() - start_time_).toSec() > wait_time_)
    {
      compute_control = true;
    }

    pc_est.linear() = p1_eig.linear();
    if (!final_rotation_ && !block_rotation_) // For small angles we antecipate that the single contact point assumption is violated
    {
      pc_est.translation() = kalman_filter_.estimate(p1_eig.translation(), v1_eig, p2_eig.translation(), wrench2_rotated, dt.toSec()); // The kalman filter estimates in the base frame, thus the wrench should be writen in that basis.
    }
    else
    {
      ROS_INFO_ONCE("Entering final folding phase");
      block_rotation_ = true;
      pc_est.translation() = kalman_filter_.estimate(p1_eig.translation(), v1_eig, p2_eig.translation(), Eigen::Matrix<double, 6, 1>::Zero(), dt.toSec());
    }

    marker_manager_.setMarkerPose("estimates", "contact_point_estimate", pc_est);
    marker_manager_.setMarkerPose("estimates", "computed_p1", p1_eig);

    Eigen::Vector3d t_est, k_est, n_est, r1, r2, r1_in_c_frame;
    Eigen::Matrix<double, 6, 1> relative_twist;
    double vd = 0, wd = 0;
    KDL::Frame sensor_to_base;
    KDL::Vector t_est_kdl, k_est_kdl, r1_kdl;

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

    double pc_proj, theta_proj;
    Eigen::Vector3d r2_y, pose_target_dir, target_point, r2_plane, r1_plane;
    
    KDL::Vector align1, align2;
    Eigen::Vector3d align1_eig, align2_eig;
    double angle = 0.0;
    
    align1 = getAxis(p1, p1_align_);
// 	ROS_INFO_STREAM("P1 align axis: " << align1.x() << ", " << align1.y() << ", " << align1.z());
    align2 = getAxis(p2, p2_align_);
    tf::vectorKDLToEigen(align1, align1_eig);
    tf::vectorKDLToEigen(align2, align2_eig);
    align1_eig = (Eigen::Matrix3d::Identity() - k_est*k_est.transpose())*align1_eig; // project on rotation plane
    align2_eig = (Eigen::Matrix3d::Identity() - k_est*k_est.transpose())*align2_eig;
	
    if (compute_control && !block_rotation_)
    {
      if (pose_goal_)
      {
        relative_pose_controller_.computeControl(pc_proj, theta_proj, pc_goal_, thetac_goal_, vd, wd);
        feedback_.phase = "Pose regulation";
        angle = acos(align1_eig.dot(align2_eig));
        prev_theta_proj_ = angle;
        feedback_.current_angle = angle;
        if (fabs(angle) < angle_goal_threshold_)
        {
          final_rotation_ = true;
        }
      }
      else
      {
        vd = vd_;
        wd = wd_;
        feedback_.phase = "Velocity control";
      }  
    }

    KDL::Twist relative_twist_kdl;

    // TODO: Choose direction for force control
    tf::vectorEigenToKDL(r1, r1_kdl); // use r1 as direction for force control. Need to rotate to C-frame
    r1_kdl = p2.M.Inverse()*r1_kdl;
    tf::vectorKDLToEigen(r1_kdl, r1_in_c_frame);
    // tf::vectorEigenToKDL(p1_eig.translation(), r1_kdl);
    // r1_kdl = p2.M.Inverse()*r1_kdl;
    // tf::vectorKDLToEigen(r1_kdl, r1_in_c_frame);
    relative_twist = adaptive_velocity_controller_.control(wrench2, vd, wd, dt.toSec(), r1_in_c_frame.normalized()); // twist expressed at the contact point, in p2 coordinates

    tf::twistEigenToKDL(relative_twist, relative_twist_kdl);
    relative_twist_kdl = p2.M*relative_twist_kdl;
    tf::twistKDLToEigen(relative_twist_kdl, relative_twist);

    publishTwist(relative_twist_kdl, "p2_rotated", twist_pub_);

    KDL::Frame eef1, eef2;
    Eigen::Affine3d eef1_eig, eef2_eig;

    kdl_manager_->getEefPose(rod_eef_, current_state, eef1);
    kdl_manager_->getEefPose(surface_eef_, current_state, eef2);
    tf::transformKDLToEigen(eef1, eef1_eig);
    tf::transformKDLToEigen(eef2, eef2_eig);

    Eigen::Matrix<double, 14, 1> qdot;
    // need to use virtual sticks up to the end-effector location, not the grasping point. TODO: Fix this
    if (pose_goal_ && block_rotation_) // time to finish the action
    {
      Eigen::Matrix<double, 6, 1> absolute_twist;
      double pc_proj, theta_proj;
      KDL::Frame base;
      KDL::Vector align_base;
      Eigen::Vector3d p2_z, base_x, base_y, base_z, align_base_eig;
      base_x << 1, 0, 0;
      base_y << 0, 1, 0;
      base_z << 0, 0, 1;
      
      align_base = getAxis(base, base_align_);
      tf::vectorKDLToEigen(align_base, align_base_eig);
      align_base_eig = (Eigen::Matrix3d::Identity() - k_est*k_est.transpose())*align_base_eig;
      theta_proj = acos(align2_eig.dot(align_base_eig)); // want vector from contact to end-effector
      ROS_DEBUG_STREAM("Theta proj: " << theta_proj);
      feedback_.phase = "Final alignment";
      feedback_.current_angle = theta_proj;
      if (fabs(theta_proj) < angle_goal_threshold_)
      {
        action_server_->setSucceeded();
      }
      absolute_pose_controller_.computeControl(pc_proj, theta_proj, 0.0, 0.0, vd, wd);
      absolute_twist.block<3,1>(0, 0) = vd*base_y;
      absolute_twist.block<3,1>(3, 0) = wd*base_x;
      qdot = ects_controller_->control(current_state, pc_est.translation() - eef1_eig.translation(), pc_est.translation() - eef2_eig.translation(), absolute_twist, Eigen::Matrix<double, 6, 1>::Zero());
    }
    else
    {
      qdot = ects_controller_->control(current_state, pc_est.translation() - eef1_eig.translation(), pc_est.translation() - eef2_eig.translation(), Eigen::Matrix<double, 6, 1>::Zero(), relative_twist);
    }

    kdl_manager_->getJointState(rod_eef_, qdot.block<7, 1>(0, 0), ret);
    kdl_manager_->getJointState(surface_eef_, qdot.block<7,1>(7, 0), ret);
    marker_manager_.publishMarkers();

    action_server_->publishFeedback(feedback_);
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

    if (trans_axis_ == "x")
    {
      t_init << cos(goal->adaptive_params.init_t_error), 0, sin(goal->adaptive_params.init_t_error);
    }

    if (trans_axis_ == "-x")
    {
      t_init << -cos(goal->adaptive_params.init_t_error), 0, -sin(goal->adaptive_params.init_t_error);
    }

    if (trans_axis_ == "y")
    {
      t_init << 0, cos(goal->adaptive_params.init_t_error), sin(goal->adaptive_params.init_t_error);
    }

    if (trans_axis_ == "-y")
    {
      t_init << 0, -cos(goal->adaptive_params.init_t_error), -sin(goal->adaptive_params.init_t_error);
    }

    if (trans_axis_ == "z")
    {
      t_init << sin(goal->adaptive_params.init_t_error), 0, cos(goal->adaptive_params.init_t_error);
    }

    if (trans_axis_ == "-z")
    {
      t_init << -sin(goal->adaptive_params.init_t_error), 0, -cos(goal->adaptive_params.init_t_error);
    }

    if (rot_axis_ == "x")
    {
      k_init << cos(goal->adaptive_params.init_k_error), sin(goal->adaptive_params.init_k_error), 0;
    }

    if (rot_axis_ == "-x")
    {
      k_init << -cos(goal->adaptive_params.init_k_error), -sin(goal->adaptive_params.init_k_error), 0;
    }

    if (rot_axis_ == "y")
    {
      k_init << 0, cos(goal->adaptive_params.init_k_error), sin(goal->adaptive_params.init_k_error);
    }

    if (rot_axis_ == "-y")
    {
      k_init << 0, -cos(goal->adaptive_params.init_k_error), -sin(goal->adaptive_params.init_k_error);
    }

    if (rot_axis_ == "z")
    {
      k_init << 0, sin(goal->adaptive_params.init_k_error), cos(goal->adaptive_params.init_k_error);
    }

    if (rot_axis_ == "-z")
    {
      k_init << 0, -sin(goal->adaptive_params.init_k_error), -cos(goal->adaptive_params.init_k_error);
    }

    adaptive_velocity_controller_.initEstimates(t_init, k_init);
    prev_theta_proj_ = M_PI;
    block_rotation_ = false;

    if (goal->use_pose_goal)
    {
      ROS_INFO("Using pose goal");
      pose_goal_ = true;
      pc_goal_ = goal->pose_goal.pd;
      thetac_goal_ = goal->pose_goal.thetad;
      angle_goal_threshold_ = goal->pose_goal.angle_goal_threshold;
    }
    else
    {
      ROS_INFO("Using velocity goal");
      pose_goal_ = false;
      vd_ = goal->velocity_goal.vd;
      wd_ = goal->velocity_goal.wd;
    }

    theta_lim_ = goal->theta_lim;
    max_contact_force_ = goal->max_contact_force;

    KDL::Frame p1;
    Eigen::Affine3d p1_eig;
    kdl_manager_->getGrippingPoint(rod_eef_, lastState(sensor_msgs::JointState()), p1);
    tf::transformKDLToEigen(p1, p1_eig);
    kalman_filter_.initialize(p1_eig.translation() + contact_offset_*p1_eig.matrix().block<3,1>(0, 2));
	final_rotation_ = false;
    start_time_ = ros::Time::now();

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

  void FoldingController::publishTwist(const KDL::Twist &twist, const std::string &frame_id, ros::Publisher &pub)
  {
    geometry_msgs::WrenchStamped twist_as_wrench;

    twist_as_wrench.header.frame_id = frame_id;
    twist_as_wrench.header.stamp = ros::Time::now();
    twist_as_wrench.wrench.force.x = twist.vel.x();
    twist_as_wrench.wrench.force.y = twist.vel.y();
    twist_as_wrench.wrench.force.z = twist.vel.z();
    twist_as_wrench.wrench.torque.x = twist.rot.x();
    twist_as_wrench.wrench.torque.y = twist.rot.y();
    twist_as_wrench.wrench.torque.z = twist.rot.z();
    pub.publish(twist_as_wrench);
  }
}
