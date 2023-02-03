#include "kydas_driver/kydas_driver.h"

bool KydasDriverNode::enableMotor(kydas_driver::EnableMotor::Request  &req,
                 kydas_driver::EnableMotor::Response &res){  
  int result = 0;
  if(!m_isEnabled){
    result = enableMotor();
    if(result != -1)
      m_isEnabled = true;
  }
  res.result = result;
  res.status = m_isEnabled;
  return true;
}

bool KydasDriverNode::disableMotor(kydas_driver::DisableMotor::Request  &req,
                  kydas_driver::DisableMotor::Response &res){  
  int result = 0;
  if(m_isEnabled){
    result = disableMotor();
    if(result != -1)
      m_isEnabled = false;
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
  else if(m_isEnabled){
    ROS_WARN("the motor needs to be enabled first!");
    res.result = -1;
    return true;
  }
  m_setSpeed = speed;
  m_setWorkingMode = (unsigned char)Control_Data::SpeedMode;
  res.result = 0;
  return true;
}

bool KydasDriverNode::setTorque(kydas_driver::SetTorque::Request  &req,
               kydas_driver::SetTorque::Response &res){ 
  int torque = req.torque;
  if(torque > 10000 || torque < - 10000){
    ROS_WARN("the torque must be bigger then -10000 and less then 10000 [%d]",torque);
    res.result = -1;
    return true;
  }
  else if(m_isEnabled){
    ROS_WARN("the motor needs to be enabled first!");
    res.result = -1;
    return true;
  }
  m_setTorque = torque;
  m_setWorkingMode = (unsigned char)Control_Data::TorqueMode;
  res.result = 0;
  return true;
}

bool KydasDriverNode::setPosition(kydas_driver::SetPosition::Request  &req,
                 kydas_driver::SetPosition::Response &res){ 
  int position = req.position;
  if(position > 4294967295 || position < -4294967295){
    ROS_WARN("the position must be bigger then -4294967295 and less then 4294967295 [%d]",position);
    res.result = -1;
    return true;
  }
  m_setPosition = position;
  m_setWorkingMode = (unsigned char)Control_Data::PositionMode;
  res.result = 0;
  return true;
}

bool KydasDriverNode::requestQueryData(kydas_driver::RequestQueryData::Request  &req,
                 kydas_driver::RequestQueryData::Response &res){ 
  unsigned char command = req.command;
  if(command >= (int)Query_Data::MAX_AMOUNT || command < 0){
    ROS_WARN("the comamand must be bigger or equal to 0 and less then %d [%d]",(int)Query_Data::MAX_AMOUNT, command);
    res.result = -1;
    return true;
  }
  res.result = requestQueryData(command);
  return true;
}
