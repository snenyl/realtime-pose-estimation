// Copyright 2022 Simon Erik Nylund.
// Author: snenyl

#include "PoseEstimation/PoseEstimation.h"

PoseEstimation pose_estimation_object;

int main() {
  pose_estimation_object.setup_pose_estimation();
  while (true) {
    pose_estimation_object.run_pose_estimation();
  }
}
