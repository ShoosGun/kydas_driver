#include "kydas_driver/kydas_driver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kydas_driver");
    
  KydasDriverNode node{};
  ros::Rate loop_rate(10);

  if(node.openComport())
  {
    ROS_WARN("can't open comport, the driver might not be connected or the port with the permissions not set");
    //return 0;
  }
  
  while (ros::ok())
  {
    node.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}