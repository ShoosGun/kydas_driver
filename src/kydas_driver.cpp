#include "kydas_driver/kydas_driver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kydas_driver");
  
  int cport_nr;
  if (argc != 2){
    ROS_INFO("port not defined, using port 0 by default. If you want to set a unique port use: kydas_driver PORT");
    cport_nr = 0; /* /dev/ttyS0 (COM1 on windows) */
  }
  else{
    cport_nr = atoll(argv[1]);
  }

  char mode[] ={'8','N','1',0};
  KydasDriverNode node{cport_nr, 115200, mode};
  ros::Rate loop_rate(10);

  if(node.openComport())
  {
    ROS_INFO("Can not open comport");
    return 0;
  }
  
  while (ros::ok())
  {
    node.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}