#include "kydas_driver/kydas_driver.h"


int KydasDriverNode::readQueryData(unsigned char* bytes, int currentPosition){
  if(bytes[currentPosition] != QUERY_HEADER){
    return 0; //Means we read nothing, because it isn't the right return data
  }
  Query_Data dataType = (Query_Data)bytes[currentPosition + 1];
  if(dataType == Query_Data::ControlStatus){
    //1.5 bytes value
    m_controlMode = bytes[currentPosition + 2] >> 4;//Gets the first 4 bits
    m_feedbackWay = bytes[currentPosition + 2] % 16;//Gets the second 4 bits
    m_workingMode = bytes[currentPosition + 3] >> 4;//Get the last 4 bits    
    
    kydas_driver::MotorControllerStatus msg;
    msg.header.stamp = ros::Time::now();
    msg.controlMode = m_controlMode;
    msg.feedbackWay = m_feedbackWay;
    msg.workingMode = m_workingMode;
    m_controllerStatus_pub.publish(msg);
    ROS_DEBUG("Controller Status:\n Control Mode [%d]\n Feedback Way [%d]\n Working Mode [%d]",
              m_controlMode, m_feedbackWay, m_workingMode);
  }
  else if(dataType == Query_Data::EletricalAngle){
    //Not on the datasheet, we need to ask them
  }
  else if(dataType == Query_Data::Speed){
    //2 bytes value
    m_currentSpeed = +bytes[currentPosition + 2] << 8
                + +bytes[currentPosition + 3];
    kydas_driver::MotorSpeed msg;
    msg.header.stamp = ros::Time::now();
    msg.speed = m_currentSpeed;
    m_speed_pub.publish(msg);
    ROS_DEBUG("Motor Speed [%d] RPM", m_currentSpeed);
  }
  else if(dataType == Query_Data::Current){
    //2 bytes value
    m_current = +bytes[currentPosition + 2] << 8
                + +bytes[currentPosition + 3];
    kydas_driver::MotorCurrent msg;
    msg.header.stamp = ros::Time::now();
    msg.current = m_current;
    m_current_pub.publish(msg);
    ROS_DEBUG("Motor Current [%d] A", m_current);
  }
  else if(dataType == Query_Data::RotorPosition){
    //2 bytes value
    m_rotorPosition = +bytes[currentPosition + 2] << 8
                + +bytes[currentPosition + 3];
    kydas_driver::MotorRotorPosition msg;
    msg.header.stamp = ros::Time::now();
    msg.rotorPosition = m_rotorPosition;
    m_rotorPosition_pub.publish(msg);
    ROS_DEBUG("Rotor Position [%d]", m_rotorPosition);
  }
  else if(dataType == Query_Data::Voltage){
    //1 byte value
    m_voltage = +bytes[currentPosition + 2];
    kydas_driver::MotorVoltage msg;
    msg.header.stamp = ros::Time::now();
    msg.voltage = m_voltage;
    m_voltage_pub.publish(msg);
    ROS_DEBUG("Voltage [%d] V", m_voltage);
  }
  else if(dataType == Query_Data::Temperature){
    //2 bytes value
    m_temperature = +bytes[currentPosition + 2] << 8
                + +bytes[currentPosition + 3];
    kydas_driver::MotorTemp msg;
    msg.header.stamp = ros::Time::now();
    msg.temp = m_temperature;
    m_temp_pub.publish(msg);
    ROS_DEBUG("Temperature [%d] C", m_temperature);
  }
  else if(dataType == Query_Data::FaultCode){
    //2 "bytes" value
    m_faultCode = +bytes[currentPosition + 2] << 8
                + +bytes[currentPosition + 3];
    kydas_driver::MotorFaultCode msg;
    msg.header.stamp = ros::Time::now();
    msg.faultCode = m_faultCode;
    m_faultCode_pub.publish(msg);
    displayFaultCode(m_faultCode);
  }
  else if(dataType == Query_Data::Position){
    m_currentPosition = 0;
  }
  else if(dataType == Query_Data::ProgramVersion){  
    m_programVersion = 0;
  }
  return 6; //Counting the header, the query data is always 6 bytes
}

int KydasDriverNode::readHeartbeatData(unsigned char* bytes, int currentPosition){
  if(bytes[currentPosition] != HEARTBEAT_HEADER){
    return 0; //Means we read nothing, because it isn't the right return data
  }
  int eletricalAngle = +bytes[currentPosition + 1] << 8
                + +bytes[currentPosition + 2];

  short faultCode = +bytes[currentPosition + 3] << 8
                + +bytes[currentPosition + 4];

  int temp = +bytes[currentPosition + 5];
  int voltage = +bytes[currentPosition + 6];

  int speed = +bytes[currentPosition + 7] << 8
                + +bytes[currentPosition + 8];  

  int position = +bytes[currentPosition + 9] << 24
                + +bytes[currentPosition + 10] << 16
                + +bytes[currentPosition + 11] << 8
                + +bytes[currentPosition + 12];

  //Publishing all this data
  kydas_driver::MotorEletricAngle eletricAngleMsg;
  eletricAngleMsg.header.stamp = ros::Time::now();
  eletricAngleMsg.eletricAngle = eletricalAngle;
  m_eletricAngle_pub.publish(eletricAngleMsg);
  ROS_DEBUG("Eletrical Angle [%d]",eletricalAngle);

  kydas_driver::MotorFaultCode faultCodeMsg;
  faultCodeMsg.header.stamp = ros::Time::now();
  faultCodeMsg.faultCode = faultCode;
  m_faultCode_pub.publish(faultCodeMsg);
  displayFaultCode(faultCode);

  kydas_driver::MotorTemp tempMsg;
  tempMsg.header.stamp = ros::Time::now();
  tempMsg.temp = temp;
  m_temp_pub.publish(tempMsg);
  ROS_DEBUG("Temperature [%d] C",temp);

  kydas_driver::MotorVoltage voltMsg;
  voltMsg.header.stamp = ros::Time::now();
  voltMsg.voltage = voltage;
  m_voltage_pub.publish(voltMsg);
  ROS_DEBUG("Voltage [%d] V",voltage);

  kydas_driver::MotorSpeed speedMsg;
  speedMsg.header.stamp = ros::Time::now();
  speedMsg.speed = speed;
  m_speed_pub.publish(speedMsg);
  ROS_DEBUG("Speed [%d] RPM",speed);
  
  kydas_driver::MotorPosition positionMsg;
  positionMsg.header.stamp = ros::Time::now();
  positionMsg.position = position;
  m_position_pub.publish(positionMsg);
  ROS_DEBUG("Position [%d] 10000/circle",position);

  return 13;
}