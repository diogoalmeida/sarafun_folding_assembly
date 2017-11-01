#include <ros/ros.h>
#include <generic_control_toolbox/kdl_manager.hpp>
#include <generic_control_toolbox/wrench_manager.hpp>
#include <folding_assembly_controller/ects.hpp>
#include <folding_assembly_controller/kalman_filter.hpp>
#include <folding_assembly_controller/pose_controller.hpp>
#include <folding_assembly_controller/adaptive_velocity_controller.hpp>

int main(int argc, char ** argv)
{
  folding_algorithms::KalmanEstimator kalman_filter;
  folding_algorithms::FoldingPoseController pose_controller;
  folding_algorithms::AdaptiveController adaptive_velocity_controller;
  generic_control_toolbox::KDLManager kdl_manager("blah");
  generic_control_toolbox::WrenchManager wrench_manager();
  folding_algorithms::ECTSController ects_controller("rod", "surface", std::shared_ptr<generic_control_toolbox::KDLManager>(&kdl_manager));
  return 0;
}
