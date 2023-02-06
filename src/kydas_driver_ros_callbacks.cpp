#include "kydas_driver/kydas_driver.h"
bool KydasDriverNode::enableMotor(kydas_driver::EnableMotor::Request  &req,
                 kydas_driver::EnableMotor::Response &res){  
  int result = 0;
  if(!m_isEnabled){
      m_isEnabled = true;
      m_setSpeed = 0;
  }
  res.result = result;
  res.status = m_isEnabled;
  return true;
}

bool KydasDriverNode::disableMotor(kydas_driver::DisableMotor::Request  &req,
                  kydas_driver::DisableMotor::Response &res){  
  int result = 0;
  if(m_isEnabled){
      m_isEnabled = false;
      m_setSpeed = 0;
  }
  res.result = result;
  res.status = m_isEnabled;
  return true;
}

bool KydasDriverNode::setSpeed(kydas_driver::SetSpeed::Request  &req,
              kydas_driver::SetSpeed::Response &res){
  int speed = req.speed;
  if(speed > 10000 || speed < - 10000){
    ROS_WARN("the speed must be bigger then -10000 and less then 10000 [%d]",speed);
    res.result = -1;
    return true;
  }
  //else if(!m_isEnabled){
  //  ROS_WARN("the motor needs to be enabled first!");
  //  res.result = -1;
  //  return true;
  //}
  setSpeed(speed);
  m_setSpeed = speed;
  res.result = 0;
  return true;
}