#include <ros/ros.h>
#include <folding_assembly_controller/ects.hpp>
#include <folding_assembly_controller/kalman_filter.hpp>
#include <folding_assembly_controller/pose_controller.hpp>
#include <folding_assembly_controller/adaptive_velocity_controller.hpp>

int main(int argc, char ** argv)
{
  // folding_algorithms::ECTSController ects_controller;
  folding_algorithms::KalmanEstimator kalman_filter;
  folding_algorithms::FoldingPoseController pose_controller;
  folding_algorithms::AdaptiveController adaptive_velocity_controller;
  return 0;
}
