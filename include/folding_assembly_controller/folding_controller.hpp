#ifndef __FOLDING_CONTROLLER__
#define __FOLDING_CONTROLLER__

#include <ros/ros.h>
#include <stdexcept>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <generic_control_toolbox/marker_manager.hpp>
#include <generic_control_toolbox/kdl_manager.hpp>
#include <generic_control_toolbox/wrench_manager.hpp>
#include <generic_control_toolbox/controller_template.hpp>
#include <generic_control_toolbox/ArmInfo.h>
#include <folding_assembly_controller/ects.hpp>
#include <folding_assembly_controller/kalman_filter.hpp>
#include <folding_assembly_controller/pose_controller.hpp>
#include <folding_assembly_controller/FoldingControllerAction.h>
#include <folding_assembly_controller/AdaptiveController.h>
#include <folding_assembly_controller/PoseGoal.h>
#include <folding_assembly_controller/adaptive_velocity_controller.hpp>
#include <folding_assembly_controller/FoldingConfig.h>

namespace folding_assembly_controller
{
  class FoldingController : public generic_control_toolbox::ControllerTemplate<FoldingControllerAction,
                                                                                  FoldingControllerGoal,
                                                                                  FoldingControllerFeedback,
                                                                                  FoldingControllerResult>
  {
  public:
    FoldingController(const std::string &action_name);
    virtual ~FoldingController();

  private:
    bool parseGoal(boost::shared_ptr<const FoldingControllerGoal> goal);
    void resetController();

    sensor_msgs::JointState controlAlgorithm(const sensor_msgs::JointState &current_state, const ros::Duration &dt);
    /**
      Get the required parameters for initializing the folding controller
      elements.

      @return False if something goes wrong, true otherwise.
    **/
    bool init();

    /**
      Initializes the manager classes with the given arm info.

      @param arm_name The arm name in the parameter server.
      @param eef_name Arm's kinematic chain end-effector name.
      @param sensor_frame Arm's sensor frame name.
      @return False if something goes wrong, true otherwise.
    **/
    bool setArm(const std::string &arm_name, std::string &eef_name, std::string &sensor_frame);

    /**
      Allows dynamic reconfiguration of relevant parameters of the controlller.

      @param config Configuration object with parameters.
      @param level The reconfig level.
    **/
    void reconfig(FoldingConfig &config, uint32_t level);

    /**
      Publishes the given KDL twist using the provided publisher,
      as a WrenchStamped message.

      @param twist The KDL twist.
      @param frame_id Frame in which the twist is expressed.
      @param pub A publisher for WrenchStamped messages.
    **/
    void publishTwist(const KDL::Twist &twist, const std::string &frame_id, ros::Publisher &pub);

    ros::NodeHandle nh_;
    std::string rod_eef_, surface_eef_, base_frame_, rot_axis_, trans_axis_, rod_sensor_frame_, surface_sensor_frame_;
    folding_algorithms::KalmanEstimator kalman_filter_;
    folding_algorithms::FoldingPoseController pose_controller_;
    folding_algorithms::AdaptiveController adaptive_velocity_controller_;
    std::shared_ptr<generic_control_toolbox::KDLManager> kdl_manager_;
    generic_control_toolbox::WrenchManager wrench_manager_;
    generic_control_toolbox::MarkerManager marker_manager_;
    std::unique_ptr<folding_algorithms::ECTSController> ects_controller_;
    std::shared_ptr<dynamic_reconfigure::Server<FoldingConfig> > dynamic_reconfigure_server_;
    dynamic_reconfigure::Server<FoldingConfig>::CallbackType dynamic_reconfigure_callback_;
    double pc_goal_, thetac_goal_, vd_, wd_, contact_offset_;
    bool pose_goal_;
    KDL::Frame rod_sensor_to_gripping_point_, surface_sensor_to_gripping_point_;
    tf::TransformListener listener_;
    ros::Publisher twist_pub_;
  };
}
#endif
