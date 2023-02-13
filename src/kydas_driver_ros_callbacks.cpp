#include "kydas_driver/kydas_driver.h"

void KydasDriverNode::cmdSpeed(const kydas_driver::CmdSpeed::ConstPtr& cmd){
  int speed = cmd->speed * 180 / M_PI; //In DPS
  if(speed > 10000 || speed < - 10000){
    ROS_WARN("the speed must be bigger then -10000 DPS and less then 10000 DPS [%d DPS]",speed);
    return;
  }
  setSpeed(speed);
}