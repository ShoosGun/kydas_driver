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
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Controller Status:\n Control Mode [%d]\n Feedback Way [%d]\n Working Mode [%d]",
              m_controlMode, m_feedbackWay, m_workingMode);
  }
  else if(dataType == Query_Data::EletricalAngle){    
    //2 bytes value
    m_eletricalAngle = (short)(bytes[currentPosition + 2] << 8 | bytes[currentPosition + 3]);
    kydas_driver::MotorEletricAngle eletricAngleMsg;
    eletricAngleMsg.header.stamp = ros::Time::now();
    eletricAngleMsg.eletricAngle = m_eletricalAngle;
    m_eletricAngle_pub.publish(eletricAngleMsg);
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Eletrical Angle [%d]", m_eletricalAngle);
  }
  else if(dataType == Query_Data::Speed){
    //2 bytes value
    m_speed = (short)(bytes[currentPosition + 2] << 8 | bytes[currentPosition + 3]);
    kydas_driver::MotorSpeed msg;
    msg.header.stamp = ros::Time::now();
    msg.speed = (int)(m_speed / 0.15f);
    msg.rawSpeed = m_speed;
    m_speed_pub.publish(msg);
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Motor Speed [%d] DPS (%d)", (int)(m_speed / 0.15f), m_speed);
  }
  else if(dataType == Query_Data::Current){
    //2 bytes value
    m_current = (short)(bytes[currentPosition + 2] << 8 | bytes[currentPosition + 3]);
    kydas_driver::MotorCurrent msg;
    msg.header.stamp = ros::Time::now();
    msg.current = m_current;
    m_current_pub.publish(msg);
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Motor Current [%d] A", m_current);
  }
  else if(dataType == Query_Data::RotorPosition){
    //2 bytes value
    m_rotorPosition = (short)(bytes[currentPosition + 2] << 8 | bytes[currentPosition + 3]);
    kydas_driver::MotorRotorPosition msg;
    msg.header.stamp = ros::Time::now();
    msg.rotorPosition = m_rotorPosition;
    m_rotorPosition_pub.publish(msg);
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Rotor Position [%d]", m_rotorPosition);
  }
  else if(dataType == Query_Data::Voltage){
    //1 byte value
    m_voltage = (int)bytes[currentPosition + 2];
    kydas_driver::MotorVoltage msg;
    msg.header.stamp = ros::Time::now();
    msg.voltage = m_voltage;
    m_voltage_pub.publish(msg);
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Voltage [%d] V", m_voltage);
  }
  else if(dataType == Query_Data::Temperature){
    //2 bytes value
    m_temperature = (short)(bytes[currentPosition + 2] << 8 | bytes[currentPosition + 3]);
    sensor_msgs::Temperature msg;
    msg.header.stamp = ros::Time::now();
    msg.temperature = (float)m_temperature;
    m_temp_pub.publish(msg);
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Temperature [%d] C", m_temperature);
  }
  else if(dataType == Query_Data::FaultCode){
    //2 "bytes" value
    m_faultCode = (short)(bytes[currentPosition + 2] << 8 | bytes[currentPosition + 3]);
    kydas_driver::MotorFaultCode msg;
    msg.header.stamp = ros::Time::now();
    msg.faultCode = m_faultCode;
    m_faultCode_pub.publish(msg);

    std::string faultCodeStr = displayFaultCode(m_faultCode);
    const char* faultCodeCStr = faultCodeStr.c_str();
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Fault Code = [0b%s]", faultCodeCStr);
  }
  else if(dataType == Query_Data::Position){
    //4 bytes value
    m_position = (int)(bytes[currentPosition + 2] << 24 | bytes[currentPosition + 3] << 16 | bytes[currentPosition + 4] << 8 | bytes[currentPosition + 5]);
    kydas_driver::MotorPosition positionMsg;
    positionMsg.header.stamp = ros::Time::now();
    positionMsg.position = (m_position % 10000 * 360) / 10000;
    positionMsg.rawPosition = m_position;
    m_position_pub.publish(positionMsg);
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Position [%d] 10000/circle (%d) degrees", m_position, (m_position % 10000 * 360) / 10000); //Para conseguir em graus -> (m_position % 10000 * 360) / 10000
  }
  else if(dataType == Query_Data::ProgramVersion){  
    //4 bytes value
    m_programVersion = (int)(bytes[currentPosition + 2] << 24 | bytes[currentPosition + 3] << 16 | bytes[currentPosition + 4] << 8 | bytes[currentPosition + 5]);
    kydas_driver::MotorProgramVersion programVersionMsg;
    programVersionMsg.programVersion = m_programVersion;
    m_programVersion_pub.publish(programVersionMsg);
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Program Version [%d]", m_programVersion);
  }

  std::string s = displayMessage(&bytes[currentPosition], 6);
  const char* cstr = s.c_str();
  ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "message = [%s]", cstr);
  return 6; //Counting the header, the query data is always 6 bytes
}

int KydasDriverNode::readHeartbeatData(unsigned char* bytes, int currentPosition){
  if(bytes[currentPosition] != HEARTBEAT_HEADER){
    return 0; //Means we read nothing, because it isn't the right return data
  }
  m_eletricalAngle = (short)(bytes[currentPosition + 1] << 8 | bytes[currentPosition + 2]);

  m_faultCode = (short)(bytes[currentPosition + 3] << 8 | bytes[currentPosition + 4]);

  m_temperature = (int)bytes[currentPosition + 5];
  m_voltage = (int)bytes[currentPosition + 6];

  m_speed = ((short)(bytes[currentPosition + 7] << 8 | bytes[currentPosition + 8]));

  m_position = (int)(bytes[currentPosition + 9] << 24 | bytes[currentPosition + 10] << 16 | bytes[currentPosition + 11] << 8 | bytes[currentPosition + 12]);

  //Publishing all this data
  kydas_driver::MotorEletricAngle eletricAngleMsg;
  eletricAngleMsg.header.stamp = ros::Time::now();
  eletricAngleMsg.eletricAngle = m_eletricalAngle;
  m_eletricAngle_pub.publish(eletricAngleMsg);
  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW, "Eletrical Angle [%d]",m_eletricalAngle);

  kydas_driver::MotorFaultCode faultCodeMsg;
  faultCodeMsg.header.stamp = ros::Time::now();
  faultCodeMsg.faultCode = m_faultCode;
  m_faultCode_pub.publish(faultCodeMsg);
  
  std::string faultCodeStr = displayFaultCode(m_faultCode);
  const char* faultCodeCStr = faultCodeStr.c_str();
  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW, "Fault Code = [0b%s]", faultCodeCStr);

  sensor_msgs::Temperature tempMsg;
  tempMsg.header.stamp = ros::Time::now();
  tempMsg.temperature = (float)m_temperature;
  m_temp_pub.publish(tempMsg);
  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW, "Temperature [%d] C",m_temperature);

  kydas_driver::MotorVoltage voltMsg;
  voltMsg.header.stamp = ros::Time::now();
  voltMsg.voltage = m_voltage;
  m_voltage_pub.publish(voltMsg);
  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW, "Voltage [%d] V",m_voltage);

  kydas_driver::MotorSpeed speedMsg;
  speedMsg.header.stamp = ros::Time::now();
  speedMsg.speed = m_speed / 0.15f;
  speedMsg.rawSpeed = m_speed;
  m_speed_pub.publish(speedMsg);
  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW, "Speed [%d] DPS", m_speed);
  
  kydas_driver::MotorPosition positionMsg;
  positionMsg.header.stamp = ros::Time::now();
  positionMsg.position = (m_position % 10000 * 360) / 10000;
  positionMsg.rawPosition = m_position;
  m_position_pub.publish(positionMsg);
  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW, "Position [%d] 10000/circle (%d) degrees", m_position, (m_position % 10000 * 360) / 10000); //Para conseguir em graus -> (m_position % 10000 * 360) / 10000

  std::string s = displayMessage(&bytes[currentPosition], 13);
  const char* cstr = s.c_str();
  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW, "message = [%s]", cstr);

  return 13;
}