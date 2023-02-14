#include "kydas_driver/kydas_driver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kydas_driver");
    
  KydasDriver right_driver{16};
  KydasDriver left_driver{17};

  if(right_driver.openComport())
  {
    ROS_WARN("can't open comport of right driver, the driver might not be connected or the port with the permissions not set");
    return 0;
  }
  if(left_driver.openComport())
  {
    ROS_WARN("can't open comport of left driver, the driver might not be connected or the port with the permissions not set");
    return 0;
  }
  right_driver.cmd
  ros::spin();

  return 0;
}