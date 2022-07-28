

#include "multi_kf_tracker.h"


int main(int argc, char **argv) {

  ros::init(argc, argv, "listener");

  MultiKfTracker node;

  node.run();

  return 0;
}