#include "kydas_driver/kydas_driver.h"

bool KydasDriverNode::enableMotor(kydas_driver::EnableMotor::Request  &req,
                 kydas_driver::EnableMotor::Response &res){ 
  unsigned char enableCommand[]={CONTROL_HEADER,1,0,0,0,0,0,0};
  const int commandSize = 8;
  int result = 0;
  if(!m_isEnabled){
    result = RS232_SendBuf(m_cport_nr, enableCommand,commandSize);
    ROS_DEBUG("enabling motor");
    displayMessage(enableCommand, commandSize);
    m_isEnabled = true;
  }

  res.result = result;
  res.status = m_isEnabled;
  return true;
}

bool KydasDriverNode::disableMotor(kydas_driver::DisableMotor::Request  &req,
                  kydas_driver::DisableMotor::Response &res){ 
  unsigned char enableCommand[]={CONTROL_HEADER,0,0,0,0,0,0,0};
  const int commandSize = 8;
  int result = 0;
  if(m_isEnabled){
    result = RS232_SendBuf(m_cport_nr, enableCommand,commandSize);
    ROS_DEBUG("disabling motor");
    displayMessage(enableCommand, commandSize);
    m_isEnabled = false;
  }

  res.result = result;
  res.status = m_isEnabled;
  return true;
}

int KydasDriverNode::setMotorCommand(int value, unsigned char controlMode){
  char* valueInBytes = static_cast<char*>(static_cast<void*>(&value));
  unsigned char command[]={CONTROL_HEADER,1,0,0,0,0,0,0};
  const int commandSize = 8;
  
  //Placing the command mode
  //command[1] = controlMode; -- Precisamos verificar se realmente nao tem que modificar nada quando em modo serial
  //Placing the value
  command[4] = valueInBytes[3]; 
  command[5] = valueInBytes[2]; 
  command[6] = valueInBytes[1]; 
  command[7] = valueInBytes[0]; 
  int result = RS232_SendBuf(m_cport_nr, command,commandSize);
  ROS_DEBUG("seting motor value [%d] on mode [%d]",value, (int)controlMode);
  displayMessage(command, commandSize);
  return result;
}

bool KydasDriverNode::setSpeed(kydas_driver::SetSpeed::Request  &req,
              kydas_driver::SetSpeed::Response &res){
  int speed = req.speed;
  if(speed > 10000 || speed < - 10000){
    ROS_INFO("the speed must be bigger then -10000 and less then 10000 [%d]",speed);
    res.result = -1;
    return true;
  }
  m_setSpeed = speed;
  res.result = setMotorCommand(speed, (unsigned char)Control_Data::SpeedMode);
  return true;
}

bool KydasDriverNode::setTorque(kydas_driver::SetTorque::Request  &req,
               kydas_driver::SetTorque::Response &res){ 
  int torque = req.torque;
  if(torque > 10000 || torque < - 10000){
    ROS_INFO("the torque must be bigger then -10000 and less then 10000 [%d]",torque);
    res.result = -1;
    return true;
  }
  m_setTorque = torque;
  res.result = setMotorCommand(torque, (unsigned char)Control_Data::TorqueMode);
  return true;
}

bool KydasDriverNode::setPosition(kydas_driver::SetPosition::Request  &req,
                 kydas_driver::SetPosition::Response &res){ 
  int position = req.position;
  if(position > 4294967295 || position < -4294967295){
    ROS_INFO("the position must be bigger then -4294967295 and less then 4294967295 [%d]",position);
    res.result = -1;
    return true;
  }
  m_setPosition = position;
  res.result = setMotorCommand(position, (unsigned char)Control_Data::PositionMode);
  return true;
}

bool KydasDriverNode::requestQueryData(kydas_driver::RequestQueryData::Request  &req,
                 kydas_driver::RequestQueryData::Response &res){ 
  unsigned char command = req.command;
  if(command >= (int)Query_Data::MAX_AMOUNT || command < 0){
    ROS_INFO("the comamand must be bigger or equal to 0 and less then %d [%d]",(int)Query_Data::MAX_AMOUNT, command);
    res.result = -1;
    return true;
  }
  unsigned char queryCommand[]={QUERY_HEADER,0,0,0,0,0,0,0};
  const int commandSize = 8;
  queryCommand[1] = command;
  res.result = RS232_SendBuf(m_cport_nr, queryCommand,commandSize);
  ROS_DEBUG("requesting data [%d]", command);
  displayMessage(queryCommand, commandSize);
  return true;
}
