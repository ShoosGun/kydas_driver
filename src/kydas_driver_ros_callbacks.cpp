#include "kydas_driver/kydas_driver.h"

void KydasDriverNode::cmdSpeed(const kydas_driver::CmdSpeed::ConstPtr& cmd){
  int speed = cmd->speed;
  if(speed > 10000 || speed < - 10000){
    ROS_WARN("the speed must be bigger then -10000 and less then 10000 [%d]",speed);
    return;
  }
  setSpeed(speed);
}