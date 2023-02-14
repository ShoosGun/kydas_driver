#include "kydas_driver/kydas_driver.h"


int KydasDriver::readQueryData(unsigned char* bytes, int currentPosition){
  if(bytes[currentPosition] != QUERY_HEADER){
    return 0; //Means we read nothing, because it isn't the right return data
  }
  Query_Data dataType = (Query_Data)bytes[currentPosition + 1];
  if(dataType == Query_Data::ControlStatus){
    //1.5 bytes value
    controlMode = bytes[currentPosition + 2] >> 4;//Gets the first 4 bits
    feedbackWay = bytes[currentPosition + 2] % 16;//Gets the second 4 bits
    workingMode = bytes[currentPosition + 3] >> 4;//Get the last 4 bits    
    
    kydas_driver::MotorControllerStatus msg;
    msg.header.stamp = ros::Time::now();
    msg.controlMode = controlMode;
    msg.feedbackWay = feedbackWay;
    msg.workingMode = workingMode;
    controllerStatus_pub.publish(msg);
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Controller Status:\n Control Mode [%d]\n Feedback Way [%d]\n Working Mode [%d]",
              controlMode, feedbackWay, workingMode);
  }
  else if(dataType == Query_Data::EletricalAngle){    
    //2 bytes value
    eletricalAngle = (short)(bytes[currentPosition + 2] << 8 | bytes[currentPosition + 3]);
    kydas_driver::MotorEletricAngle eletricAngleMsg;
    eletricAngleMsg.header.stamp = ros::Time::now();
    eletricAngleMsg.eletricAngle = eletricalAngle;
    eletricAngle_pub.publish(eletricAngleMsg);
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Eletrical Angle [%d]", eletricalAngle);
  }
  else if(dataType == Query_Data::Speed){
    //2 bytes value
    raw_speed = (short)(bytes[currentPosition + 2] << 8 | bytes[currentPosition + 3]);
    kydas_driver::MotorSpeed msg;
    msg.header.stamp = ros::Time::now();
    msg.speed = (raw_speed / 0.15f) * M_PI / 180.f;
    msg.rawSpeed = raw_speed;
    speed_pub.publish(msg);
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Motor Speed [%f] RPS (%d)", msg.speed, raw_speed);
  }
  else if(dataType == Query_Data::Current){
    //2 bytes value
    current = (short)(bytes[currentPosition + 2] << 8 | bytes[currentPosition + 3]);
    kydas_driver::MotorCurrent msg;
    msg.header.stamp = ros::Time::now();
    msg.current = current;
    current_pub.publish(msg);
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Motor Current [%d] A", current);
  }
  else if(dataType == Query_Data::RotorPosition){
    //2 bytes value
    rotorPosition = (short)(bytes[currentPosition + 2] << 8 | bytes[currentPosition + 3]);
    kydas_driver::MotorRotorPosition msg;
    msg.header.stamp = ros::Time::now();
    msg.rotorPosition = rotorPosition;
    rotorPosition_pub.publish(msg);
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Rotor Position [%d]", rotorPosition);
  }
  else if(dataType == Query_Data::Voltage){
    //1 byte value
    voltage = (int)bytes[currentPosition + 2];
    kydas_driver::MotorVoltage msg;
    msg.header.stamp = ros::Time::now();
    msg.voltage = voltage;
    voltage_pub.publish(msg);
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Voltage [%d] V", voltage);
  }
  else if(dataType == Query_Data::Temperature){
    //2 bytes value
    temperature = (short)(bytes[currentPosition + 2] << 8 | bytes[currentPosition + 3]);
    sensor_msgs::Temperature msg;
    msg.header.stamp = ros::Time::now();
    msg.temperature = (float)temperature;
    temp_pub.publish(msg);
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Temperature [%d] C", temperature);
  }
  else if(dataType == Query_Data::FaultCode){
    //2 "bytes" value
    faultCode = (short)(bytes[currentPosition + 2] << 8 | bytes[currentPosition + 3]);
    kydas_driver::MotorFaultCode msg;
    msg.header.stamp = ros::Time::now();
    msg.faultCode = faultCode;
    faultCode_pub.publish(msg);

    std::string faultCodeStr = displayFaultCode(faultCode);
    const char* faultCodeCStr = faultCodeStr.c_str();
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Fault Code = [0b%s]", faultCodeCStr);
  }
  else if(dataType == Query_Data::Position){
    //4 bytes value
    raw_position = (int)(bytes[currentPosition + 2] << 24 | bytes[currentPosition + 3] << 16 | bytes[currentPosition + 4] << 8 | bytes[currentPosition + 5]);
    kydas_driver::MotorPosition positionMsg;
    positionMsg.header.stamp = ros::Time::now();
    positionMsg.position = (raw_position  / 10000.f )* 2 * M_PI;
    positionMsg.rawPosition = raw_position;
    position_pub.publish(positionMsg);
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Position [%d] 10000/circle (%f) rad", raw_position, positionMsg.position);
  }
  else if(dataType == Query_Data::ProgramVersion){  
    //4 bytes value
    programVersion = (int)(bytes[currentPosition + 2] << 24 | bytes[currentPosition + 3] << 16 | bytes[currentPosition + 4] << 8 | bytes[currentPosition + 5]);
    kydas_driver::MotorProgramVersion programVersionMsg;
    programVersionMsg.programVersion = programVersion;
    programVersion_pub.publish(programVersionMsg);
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Program Version [%d]", programVersion);
  }

  std::string s = displayMessage(&bytes[currentPosition], 6);
  const char* cstr = s.c_str();
  ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "message = [%s]", cstr);
  return 6; //Counting the header, the query data is always 6 bytes
}

int KydasDriver::readHeartbeatData(unsigned char* bytes, int currentPosition){
  if(bytes[currentPosition] != HEARTBEAT_HEADER){
    return 0; //Means we read nothing, because it isn't the right return data
  }
  eletricalAngle = (short)(bytes[currentPosition + 1] << 8 | bytes[currentPosition + 2]);

  faultCode = (short)(bytes[currentPosition + 3] << 8 | bytes[currentPosition + 4]);

  temperature = (int)bytes[currentPosition + 5];
  voltage = (int)bytes[currentPosition + 6];

  raw_speed = ((short)(bytes[currentPosition + 7] << 8 | bytes[currentPosition + 8]));

  raw_position = (int)(bytes[currentPosition + 9] << 24 | bytes[currentPosition + 10] << 16 | bytes[currentPosition + 11] << 8 | bytes[currentPosition + 12]);

  //Publishing all this data
  kydas_driver::MotorEletricAngle eletricAngleMsg;
  eletricAngleMsg.header.stamp = ros::Time::now();
  eletricAngleMsg.eletricAngle = eletricalAngle;
  eletricAngle_pub.publish(eletricAngleMsg);
  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW, "Eletrical Angle [%d]",eletricalAngle);

  kydas_driver::MotorFaultCode faultCodeMsg;
  faultCodeMsg.header.stamp = ros::Time::now();
  faultCodeMsg.faultCode = faultCode;
  faultCode_pub.publish(faultCodeMsg);
  
  std::string faultCodeStr = displayFaultCode(faultCode);
  const char* faultCodeCStr = faultCodeStr.c_str();
  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW, "Fault Code = [0b%s]", faultCodeCStr);

  sensor_msgs::Temperature tempMsg;
  tempMsg.header.stamp = ros::Time::now();
  tempMsg.temperature = (float)temperature;
  temp_pub.publish(tempMsg);
  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW, "Temperature [%d] C",temperature);

  kydas_driver::MotorVoltage voltMsg;
  voltMsg.header.stamp = ros::Time::now();
  voltMsg.voltage = voltage;
  voltage_pub.publish(voltMsg);
  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW, "Voltage [%d] V",voltage);

  kydas_driver::MotorSpeed speedMsg;
  speedMsg.header.stamp = ros::Time::now();
  speedMsg.speed = (raw_speed / 0.15f) * M_PI / 180.f;
  speedMsg.rawSpeed = raw_speed;
  speed_pub.publish(speedMsg);
  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW, "Speed [%f] RPS", speedMsg.speed);
  
  kydas_driver::MotorPosition positionMsg;
  positionMsg.header.stamp = ros::Time::now();
  positionMsg.position = (raw_position / 10000.f) * 2 * M_PI;
  positionMsg.rawPosition = raw_position;
  position_pub.publish(positionMsg);
  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW, "Position [%d] 10000/circle (%f) rad", raw_position, positionMsg.position);

  std::string s = displayMessage(&bytes[currentPosition], 13);
  const char* cstr = s.c_str();
  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW, "message = [%s]", cstr);

  return 13;
}