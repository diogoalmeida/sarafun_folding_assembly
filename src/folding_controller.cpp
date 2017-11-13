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
    std::string base_link, rod_gripping_frame, surface_gripping_frame;
    if (!nh_.getParam("kinematic_chain_base_link", base_link))
    {
      ROS_ERROR("Missing kinematic_chain_base_link parameter");
      return false;
    }

    has_init_ = false; // true if controlAlgorithm has been called after a new goal

    // Initialize arms and set gripping points.
    kdl_manager_ = std::make_shared<generic_control_toolbox::KDLManager>(base_link);

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
    marker_manager_.addMarker("estimates", "translational_estimate", "folding_assembly", base_link, generic_control_toolbox::MarkerType::arrow);
    marker_manager_.setMarkerColor("estimates", "translational_estimate", 1, 0, 0);
    marker_manager_.addMarker("estimates", "rotational_estimate", "folding_assembly", base_link, generic_control_toolbox::MarkerType::arrow);
    marker_manager_.setMarkerColor("estimates", "rotational_estimate", 0, 1, 0);
    marker_manager_.addMarker("estimates", "contact_point_estimate", "folding_assembly", base_link, generic_control_toolbox::MarkerType::sphere);
    marker_manager_.setMarkerColor("estimates", "contact_point_estimate", 0, 0, 1);
    marker_manager_.addMarker("sticks", "r1", "folding_assembly", base_link, generic_control_toolbox::MarkerType::arrow);
    marker_manager_.setMarkerColor("sticks", "r1", 1, 0, 0);
    marker_manager_.addMarker("sticks", "r2", "folding_assembly", base_link, generic_control_toolbox::MarkerType::arrow);
    marker_manager_.setMarkerColor("sticks", "r2", 0, 1, 0);
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
    Eigen::Matrix<double, 6, 1> v1_eig, wrench2;
    kdl_manager_->getGrippingTwist(rod_eef_, current_state, v1);
    wrench_manager_.wrenchAtGrippingPoint(surface_eef_, wrench2);
    tf::twistKDLToEigen(v1, v1_eig);

    pc_est.linear() = p1_eig.linear();
    pc_est.translation() = kalman_filter_.estimate(p1_eig.translation(), v1_eig, p2_eig.translation(), wrench2, dt.toSec());
    marker_manager_.setMarkerPose("estimates", "contact_point_estimate", pc_est);

    Eigen::Vector3d t_est, k_est, r1, r2;
    Eigen::Matrix<double, 6, 1> relative_twist;
    double pc_proj, theta_proj, vd = 0, wd = 0;
    KDL::Frame sensor_to_base;
    KDL::Vector t_est_kdl, k_est_kdl;

    kdl_manager_->getSensorPoint(surface_eef_, current_state, sensor_to_base);
    adaptive_velocity_controller_.getEstimates(t_est, k_est);
    tf::vectorEigenToKDL(t_est, t_est_kdl);
    tf::vectorEigenToKDL(k_est, k_est_kdl);
    t_est_kdl = sensor_to_base.M*t_est_kdl; // Convert motion estimates to base frame
    k_est_kdl = sensor_to_base.M*k_est_kdl;
    tf::vectorKDLToEigen(t_est_kdl, t_est);
    tf::vectorKDLToEigen(k_est_kdl, k_est);
    marker_manager_.setMarkerPoints("estimates", "translational_estimate", pc_est.translation(), pc_est.translation() + 0.1*t_est);
    marker_manager_.setMarkerPoints("estimates", "rotational_estimate", pc_est.translation(), pc_est.translation() + 0.1*k_est);
    r1 = pc_est.translation() - p1_eig.translation();
    r2 = pc_est.translation() - p2_eig.translation();
    marker_manager_.setMarkerPoints("sticks", "r1", p1_eig.translation(), pc_est.translation());
    marker_manager_.setMarkerPoints("sticks", "r2", p2_eig.translation(), pc_est.translation());

    if (pose_goal_)
    {
      pc_proj = (pc_est.translation() - p2_eig.translation()).dot(t_est);
      theta_proj = acos(r1.dot(t_est))/r1.norm();
      pose_controller_.computeControl(pc_proj, theta_proj, pc_goal_, thetac_goal_, vd, wd);
    }
    else
    {
      vd = vd_;
      wd = wd_;
    }

    KDL::Twist relative_twist_kdl;
    relative_twist = adaptive_velocity_controller_.control(wrench2, vd, wd, dt.toSec());
    tf::twistEigenToKDL(relative_twist, relative_twist_kdl);
    relative_twist_kdl = sensor_to_base.M*relative_twist_kdl;
    tf::twistKDLToEigen(relative_twist_kdl, relative_twist);

    Eigen::Matrix<double, 14, 1> qdot;
    qdot = ects_controller_->control(current_state, r1, r2, Eigen::Matrix<double, 6, 1>::Zero(), relative_twist);
    kdl_manager_->getJointState(rod_eef_, qdot.block<7, 1>(0, 0), ret);
    kdl_manager_->getJointState(surface_eef_, qdot.block<7,1>(7, 0), ret);
    marker_manager_.publishMarkers();

    return ret;
  }

  bool FoldingController::setArm(const std::string &arm_name, std::string &eef_name)
  {
      generic_control_toolbox::ArmInfo info;
      bool has_ft_sensor; // HACK: using the boolean in "ArmInfo" gives an rvalue assignment error that I do not understand

      if (!nh_.getParam(arm_name + "/kdl_eef_frame", info.kdl_eef_frame))
      {
        ROS_ERROR("Missing kinematic chain eef (%s/kdl_eef_frame)", arm_name.c_str());
        return false;
      }

      eef_name = info.kdl_eef_frame;

      if (!nh_.getParam(arm_name + "/gripping_frame", info.gripping_frame))
      {
        ROS_ERROR("Missing kinematic gripping_frame (%s/gripping_frame)", arm_name.c_str());
        return false;
      }

      if (!nh_.getParam(arm_name + "/has_ft_sensor", has_ft_sensor))
      {
        ROS_ERROR("Missing sensor info (%s/has_ft_sensor)", arm_name.c_str());
        return false;
      }

      if (!nh_.getParam(arm_name + "/sensor_frame", info.sensor_frame))
      {
        ROS_ERROR("Missing sensor info (%s/sensor_frame)", arm_name.c_str());
        return false;
      }

      if (!nh_.getParam(arm_name + "/sensor_topic", info.sensor_topic))
      {
        ROS_ERROR("Missing sensor info (%s/sensor_topic)", arm_name.c_str());
        return false;
      }

      if(!kdl_manager_->initializeArm(info.kdl_eef_frame))
      {
        return false;
      }

      if (!kdl_manager_->setGrippingPoint(info.kdl_eef_frame, info.gripping_frame))
      {
        return false;
      }

      if (has_ft_sensor)
      {
        if (!wrench_manager_.initializeWrenchComm(info.kdl_eef_frame, info.sensor_frame, info.gripping_frame, info.sensor_topic))
        {
          return false;
        }
      }
      else
      {
        ROS_WARN("End-effector %s has no F/T sensor.", info.kdl_eef_frame.c_str());
      }

      ROS_DEBUG("Successfully set up arm %s with eef_frame %s, gripping_frame %s sensor frame %s and sensor topic %s", arm_name.c_str(), info.kdl_eef_frame.c_str(), info.gripping_frame.c_str(), info.sensor_frame.c_str(), info.sensor_topic.c_str());

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

  void FoldingController::resetController()
  {
    has_init_ = false;
    adaptive_velocity_controller_.reset();
  }
}
