
#include "object_detection_ssd.h"


int main(int argc, char **argv) {

  ros::init(argc, argv, "sender");

  ObjectDetectionSSD node;

  node.run();

  return 0;
}
