// Copyright 2022 Simon Erik Nylund.
// Author: snenyl

#include "PoseEstimation/PoseEstimation.h"

PoseEstimation pose_estimation_object;

void setup() {
  pose_estimation_object.setup_pose_estimation();
}

void run() {
  pose_estimation_object.run_pose_estimation();
}

int main() {
  setup();
  while (true) {
    run();
  }
}
