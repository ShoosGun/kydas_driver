#include "kydas_driver/kydas_driver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kydas_driver");
    
  KydasDriverNode node{};
  ros::Rate loop_rate(10);

  if(node.openComport())
  {
    ROS_INFO("Can not open comport");
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