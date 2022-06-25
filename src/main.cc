// Copyright 2022 Simon Erik Nylund.
// Author: snenyl



#include "PoseEstimation.h"


PoseEstimation pose_estimation_object;


void setup(){
  pose_estimation_object.setup_pose_estimation();
};

void run(){
  pose_estimation_object.run_pose_estimation();
//  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
};

int main() {
  setup();
  while(true){
    run();
  }
  return 0;
}
