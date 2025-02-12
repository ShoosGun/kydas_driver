#include "kydas_driver/kydas_driver.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "kydas_driver");

  KydasDriver right_driver{"/dev/ttyUSB0"};
  KydasDriver left_driver{"/dev/ttyUSB1"};

  if (right_driver.openComport()) {
    ROS_WARN("can't open comport of right driver, the driver might not be "
             "connected or the port with the permissions not set");
    return 0;
  }
  if (left_driver.openComport()) {
    ROS_WARN("can't open comport of left driver, the driver might not be "
             "connected or the port with the permissions not set");
    return 0;
  }
  ros::Rate rate{25}; // 25 hz is the optimal frequency to read/write to the
                      // driver without any information getting lost
  // Makes both drivers go at 1440 dps
  right_driver.speed_cmd =
      -1440.f * M_PI / 180; // assuming they are point outwards, we need to
                            // negate the right driver
  left_driver.speed_cmd = 1440.f * M_PI / 180;
  while (ros::ok()) {
    right_driver.update();
    left_driver.update();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
