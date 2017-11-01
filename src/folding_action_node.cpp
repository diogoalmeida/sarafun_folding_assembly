#include <ros/ros.h>
#include <utils/kdl_manager.hpp>
#include <utils/wrench_manager.hpp>
#include <folding_assembly_controller/ects.hpp>
#include <folding_assembly_controller/kalman_filter.hpp>
#include <folding_assembly_controller/pose_controller.hpp>
#include <folding_assembly_controller/adaptive_velocity_controller.hpp>

int main(int argc, char ** argv)
{
  folding_algorithms::KalmanEstimator kalman_filter;
  folding_algorithms::FoldingPoseController pose_controller;
  folding_algorithms::AdaptiveController adaptive_velocity_controller;
  folding_utils::KDLManager kdl_manager("blah");
  folding_utils::WrenchManager wrench_manager();
  folding_algorithms::ECTSController ects_controller("rod", "surface", std::shared_ptr<folding_utils::KDLManager>(&kdl_manager));
  return 0;
}
