#include "kydas_driver/kydas_driver.h"

bool KydasDriverNode::setSpeed(kydas_driver::SetSpeed::Request  &req,
              kydas_driver::SetSpeed::Response &res){
  int speed = req.speed;
  if(speed > 10000 || speed < - 10000){
    ROS_WARN("the speed must be bigger then -10000 and less then 10000 [%d]",speed);
    res.result = -1;
    return true;
  }
  
  setSpeed(speed);
  m_setSpeed = speed;
  res.result = 0;
  return true;
}