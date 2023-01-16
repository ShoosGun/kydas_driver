#include "rs232.h"

#include "ros/ros.h"

#include "kydas_driver/DisableMotor.h"
#include "kydas_driver/EnableMotor.h"
#include "kydas_driver/SetPosition.h"
#include "kydas_driver/SetSpeed.h"
#include "kydas_driver/SetTorque.h"

#include "kydas_driver/RequestQueryData.h"

#include "kydas_driver/MotorControllerStatus.h"
#include "kydas_driver/MotorCurrent.h"
#include "kydas_driver/MotorEletricAngle.h"
#include "kydas_driver/MotorFaultCode.h"
#include "kydas_driver/MotorPosition.h"
#include "kydas_driver/MotorProgramVersion.h"
#include "kydas_driver/MotorRotorPosition.h"
#include "kydas_driver/MotorSpeed.h"
#include "kydas_driver/MotorTemp.h"
#include "kydas_driver/MotorVoltage.h"

#include <sstream>

//TODO colocar essas constantes e enums em um header separado
//e tbm colocar os headers das funcoes

const unsigned char CONTROL_HEADER = 0xE0;
const unsigned char QUERY_HEADER = 0xED;
const unsigned char HEARTBEAT_HEADER = 0xEE;

enum class Control_Data{
  SpeedMode = 1,
  TorqueMode,
  PositionMode
};

enum class Query_Data{
  ControlStatus = 0,
  EletricalAngle,
  Speed,
  Current,
  RotorPosition,
  Voltage,
  Temperature,
  FaultCode,
  Position,
  ProgramVersion,
  MAX_AMOUNT
};

//Serao apenas necessarios caso queiramos fazer esse codigo fazer a analise do controller status
/*
enum class ControlStatus_ControlMode{
  Analog = 1,
  CAN,
  RS232,
  RC
};

enum class ControlStatus_Feedback{
  Encoder = 1,
  Hall,
  AS5147,
  RotaryTransformer,
  Autosteering = 6,
  HallPlusEncoder,
  HallCloseLoop,
  HallPlusEncoderCloseLoop,
  Test,
  AbsoluteValueEncoder
};

enum class ControlStatus_WorkingMode{
  Speed = 1,
  Torque,
  AbsolutePosition,
  RelativePosition
};
*/

//TODO transformar tudo isso em uma classe, para que cport_nr seja uma variavel global apenas para esse codigo 
//(talvez nao seja necessario, nao tenho muita certeza, pois estou usando uma combinacao de c e cpp, estilo codigo de arduino)
int cport_nr;

ros::Publisher controllerStatus_pub;
ros::Publisher current_pub;
ros::Publisher eletricAngle_pub;
ros::Publisher faultCode_pub;
ros::Publisher position_pub;
ros::Publisher programVersion_pub;
ros::Publisher rotorPosition_pub;
ros::Publisher speed_pub;
ros::Publisher temp_pub;
ros::Publisher voltage_pub;

void displayMessage(const unsigned char* bytes, int n){
  std::stringstream ss("");
  
  ss << "0x" << std::hex;
  for (int i = 0; i < n; i++) 
      ss << ' ' << +bytes[i];
      
  const std::string tmp = ss.str();
  const char* cstr = tmp.c_str();
  ROS_INFO("message = [%s]", cstr);
}

void displayFaultCode(short faultCode){
  std::stringstream ss("");
  
  ss << "0x" << std::hex << faultCode;      
  const std::string tmp = ss.str();
  const char* cstr = tmp.c_str();
  ROS_INFO("fault code = [%s]", cstr);
}

bool enableMotor(kydas_driver::EnableMotor::Request  &req,
                 kydas_driver::EnableMotor::Response &res){ 
  unsigned char enableCommand[]={CONTROL_HEADER,1,0,0,0,0,0,0};
  const int commandSize = 8;
  int result = RS232_SendBuf(cport_nr, enableCommand,commandSize);
  res.result = result;
  ROS_INFO("enabling motor");
  displayMessage(enableCommand, commandSize);
  return true;
}

bool disableMotor(kydas_driver::DisableMotor::Request  &req,
                  kydas_driver::DisableMotor::Response &res){ 
   unsigned char enableCommand[]={CONTROL_HEADER,0,0,0,0,0,0,0};
   const int commandSize = 8;
   res.result = RS232_SendBuf(cport_nr, enableCommand,commandSize);
   ROS_INFO("disabling motor");
   displayMessage(enableCommand, commandSize);
   return true;
}

int setMotorCommand(int value, unsigned char controlMode){
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
  int result = RS232_SendBuf(cport_nr, command,commandSize);
  ROS_INFO("seting motor value [%d] on mode [%d]",value, (int)controlMode);
  displayMessage(command, commandSize);
  return result;
}

bool setSpeed(kydas_driver::SetSpeed::Request  &req,
              kydas_driver::SetSpeed::Response &res){
  int speed = req.speed;
  if(speed > 10000 || speed < - 10000){
    ROS_INFO("the speed must be bigger then -10000 and less then 10000 [%d]",speed);
    res.result = -1;
    return true;
  }
  res.result = setMotorCommand(speed, (unsigned char)Control_Data::SpeedMode);
  return true;
}

bool setTorque(kydas_driver::SetTorque::Request  &req,
               kydas_driver::SetTorque::Response &res){ 
  int torque = req.torque;
  if(torque > 10000 || torque < - 10000){
    ROS_INFO("the torque must be bigger then -10000 and less then 10000 [%d]",torque);
    res.result = -1;
    return true;
  }
  res.result = setMotorCommand(torque, (unsigned char)Control_Data::TorqueMode);
  return true;
}

bool setPosition(kydas_driver::SetPosition::Request  &req,
                 kydas_driver::SetPosition::Response &res){ 
  int position = req.position;
  if(position > 4294967295 || position < -4294967295){
    ROS_INFO("the position must be bigger then -4294967295 and less then 4294967295 [%d]",position);
    res.result = -1;
    return true;
  }
  res.result = setMotorCommand(position, (unsigned char)Control_Data::PositionMode);
  return true;
}

bool requestQueryData(kydas_driver::RequestQueryData::Request  &req,
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
  res.result = RS232_SendBuf(cport_nr, queryCommand,commandSize);
  ROS_INFO("requesting data [%d]", command);
  displayMessage(queryCommand, commandSize);
  return true;
}
//TODO colocar essas funcoes de read em um outro arquivo, pois elas sao muito grandes
int readQueryData(unsigned char* bytes, int currentPosition){
  if(bytes[currentPosition] != QUERY_HEADER){
    return 0; //Means we read nothing, because it isn't the right return data
  }
  Query_Data dataType = (Query_Data)bytes[currentPosition + 1];
  if(dataType == Query_Data::ControlStatus){
    //1.5 bytes value
    unsigned char controlMode = bytes[currentPosition + 2] >> 4;//Gets the first 4 bits
    unsigned char feedbackWay = bytes[currentPosition + 2] % 16;//Gets the second 4 bits
    unsigned char workingMode = bytes[currentPosition + 3] >> 4;//Get the last 4 bits    
    
    kydas_driver::MotorControllerStatus msg;
    msg.header.stamp = ros::Time::now();
    msg.controlMode = controlMode;
    msg.feedbackWay = feedbackWay;
    msg.workingMode = workingMode;
    controllerStatus_pub.publish(msg);
    ROS_INFO("Controller Status:\n Control Mode [%d]\n Feedback Way [%d]\n Working Mode [%d]",
              controlMode, feedbackWay, workingMode);
  }
  else if(dataType == Query_Data::EletricalAngle){
    //Not on the datasheet, we need to ask them
  }
  else if(dataType == Query_Data::Speed){
    //2 bytes value
    int speed = +bytes[currentPosition + 2] << 8
                + +bytes[currentPosition + 3];
    kydas_driver::MotorSpeed msg;
    msg.header.stamp = ros::Time::now();
    msg.speed = speed;
    speed_pub.publish(msg);
    ROS_INFO("Motor Speed [%d] RPM", speed);
  }
  else if(dataType == Query_Data::Current){
    //2 bytes value
    int current = +bytes[currentPosition + 2] << 8
                + +bytes[currentPosition + 3];
    kydas_driver::MotorCurrent msg;
    msg.header.stamp = ros::Time::now();
    msg.current = current;
    current_pub.publish(msg);
    ROS_INFO("Motor Current [%d] A", current);
  }
  else if(dataType == Query_Data::RotorPosition){
    //2 bytes value
    int rotorPosition = +bytes[currentPosition + 2] << 8
                + +bytes[currentPosition + 3];
    kydas_driver::MotorRotorPosition msg;
    msg.header.stamp = ros::Time::now();
    msg.rotorPosition = rotorPosition;
    rotorPosition_pub.publish(msg);
    ROS_INFO("Rotor Position [%d]", rotorPosition);
  }
  else if(dataType == Query_Data::Voltage){
    //1 byte value
    int voltage = +bytes[currentPosition + 2];
    kydas_driver::MotorVoltage msg;
    msg.header.stamp = ros::Time::now();
    msg.voltage = voltage;
    voltage_pub.publish(msg);
    ROS_INFO("Voltage [%d] V", voltage);
  }
  else if(dataType == Query_Data::Temperature){
    //2 bytes value
    int temp = +bytes[currentPosition + 2] << 8
                + +bytes[currentPosition + 3];
    kydas_driver::MotorTemp msg;
    msg.header.stamp = ros::Time::now();
    msg.temp = temp;
    temp_pub.publish(msg);
    ROS_INFO("Temperature [%d] C", temp);
  }
  else if(dataType == Query_Data::FaultCode){
    //2 "bytes" value
    short faultCode = +bytes[currentPosition + 2] << 8
                + +bytes[currentPosition + 3];
    kydas_driver::MotorFaultCode msg;
    msg.header.stamp = ros::Time::now();
    msg.faultCode = faultCode;
    faultCode_pub.publish(msg);
    displayFaultCode(faultCode);
  }
  else if(dataType == Query_Data::Position){
  }
  else if(dataType == Query_Data::ProgramVersion){  
  }
  return 6; //Counting the header, the query data is always 6 bytes
}

int readHeartbeatData(unsigned char* bytes, int currentPosition){
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
  eletricAngle_pub.publish(eletricAngleMsg);
  ROS_INFO("Eletrical Angle [%d]",eletricalAngle);

  kydas_driver::MotorFaultCode faultCodeMsg;
  faultCodeMsg.header.stamp = ros::Time::now();
  faultCodeMsg.faultCode = faultCode;
  faultCode_pub.publish(faultCodeMsg);
  displayFaultCode(faultCode);

  kydas_driver::MotorTemp tempMsg;
  tempMsg.header.stamp = ros::Time::now();
  tempMsg.temp = temp;
  temp_pub.publish(tempMsg);
  ROS_INFO("Temperature [%d] C",temp);

  kydas_driver::MotorVoltage voltMsg;
  voltMsg.header.stamp = ros::Time::now();
  voltMsg.voltage = voltage;
  voltage_pub.publish(voltMsg);
  ROS_INFO("Voltage [%d] V",voltage);

  kydas_driver::MotorSpeed speedMsg;
  speedMsg.header.stamp = ros::Time::now();
  speedMsg.speed = speed;
  speed_pub.publish(speedMsg);
  ROS_INFO("Speed [%d] RPM",speed);
  
  kydas_driver::MotorPosition positionMsg;
  positionMsg.header.stamp = ros::Time::now();
  positionMsg.position = position;
  position_pub.publish(positionMsg);
  ROS_INFO("Position [%d] 10000/circle",position);

  return 13;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kydas_driver");
  
  ros::NodeHandle node;
  
  //Publicar todas as informacoes do driver FEITO
  //eletric angle
  //speed
  //current
  //rotor mechanical position
  //voltage
  //temp
  //fault code
  //position
  //program version
  //controller status

  //Ter como servicoes acoes para controlar o driver e pegar informacoes (esses terao retorno de dados)
  //Controle:                  FEITO
  //speed mode                 FEITO
  //torque mode                FEITO
  //position mode              FEITO
  //Dados:                     Fazer apenas um servico em que se manda um int (ou char) e confere se ele é um pedido valido
  //                           FEITO
  //eletric angle
  //speed
  //current
  //rotor mechanical position
  //voltage
  //temp
  //fault code
  //position
  //program version
  //controller status
  //Duas opcoes:
  //1 - Esses comandos sao colocados em um query e chamados em ordem (preferivel, caso nao possamos enviar tudo de uma vez)
  //2 - Chamar imediatamente os comandos <- Isso sera o usado!

  //Setting Publishers
  controllerStatus_pub = node.advertise<kydas_driver::MotorControllerStatus>("controllerStatus", 1000);
  current_pub = node.advertise<kydas_driver::MotorCurrent>("current", 1000);
  eletricAngle_pub = node.advertise<kydas_driver::MotorEletricAngle>("eletricAngle", 1000);
  faultCode_pub = node.advertise<kydas_driver::MotorFaultCode>("faultCode", 1000);
  position_pub = node.advertise<kydas_driver::MotorPosition>("position", 1000);
  programVersion_pub = node.advertise<kydas_driver::MotorProgramVersion>("programVersion", 1000);
  rotorPosition_pub = node.advertise<kydas_driver::MotorRotorPosition>("rotorPosition", 1000);
  speed_pub = node.advertise<kydas_driver::MotorSpeed>("speed", 1000);
  temp_pub = node.advertise<kydas_driver::MotorTemp>("temp", 1000);
  voltage_pub = node.advertise<kydas_driver::MotorVoltage>("voltage", 1000);
  //Setting Services
  ros::ServiceServer enableMotorService = node.advertiseService("enable_motor", enableMotor);
  ros::ServiceServer disableMotorService = node.advertiseService("disable_motor", disableMotor);
  ros::ServiceServer setPositionService = node.advertiseService("set_position", setPosition);
  ros::ServiceServer setSpeedService = node.advertiseService("set_speed", setSpeed);
  ros::ServiceServer setTorqueService = node.advertiseService("set_torque", setTorque);
  ros::ServiceServer requestQueryDataService = node.advertiseService("request_query_data", requestQueryData);

  ros::Rate loop_rate(10);
   
  if (argc != 2){
    ROS_INFO("port not defined, using port 0 by default. If you want to set a unique port use: kydas_driver PORT");
    cport_nr = 0; /* /dev/ttyS0 (COM1 on windows) */
  }
  else{
    cport_nr = atoll(argv[1]);
  }

  int bdrate=115200;       /* 115200 baud */

  unsigned char buf[4096];

  char mode[]={'8','N','1',0};

  if(RS232_OpenComport(cport_nr, bdrate, mode, 0))
  {
    ROS_INFO("Can not open comport");
    //return 0;
  }

  while (ros::ok())
  {
    //Lendo do serial ---------------------
    int n = RS232_PollComport(cport_nr, buf, 4095);
    int i = 0;
    while(i < n){
      if(buf[i] == HEARTBEAT_HEADER){
        i += readHeartbeatData(buf, i);
      }
      else if(buf[i] == QUERY_HEADER){
        i += readQueryData(buf, i);
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}