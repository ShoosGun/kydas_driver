#ifndef KYDAS_DRIVER_H
#define KYDAS_DRIVER_H

#include "kydas_driver/rs232.h"

#include "ros/ros.h"

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "kydas_driver/MotorControllerStatus.h"
#include "kydas_driver/MotorCurrent.h"
#include "kydas_driver/MotorEletricAngle.h"
#include "kydas_driver/MotorFaultCode.h"
#include "kydas_driver/MotorPosition.h"
#include "kydas_driver/MotorProgramVersion.h"
#include "kydas_driver/MotorRotorPosition.h"
#include "kydas_driver/MotorSpeed.h"
#include "kydas_driver/MotorVoltage.h"

#include "kydas_driver/CmdSpeed.h"

#include "sensor_msgs/Temperature.h"

#include <sstream>
#include <string>
#include <array>
#include <queue>
#include <bitset>

const unsigned char CONTROL_HEADER = 0xE0;
const unsigned char QUERY_HEADER = 0xED;
const unsigned char HEARTBEAT_HEADER = 0xEE;

const char DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW []= "heartbeat_data_preview";
const char DEBUGGER_NAME_QUERY_DATA_PREVIEW []= "query_data_preview";
const char DEBUGGER_NAME_MESSAGE_RECEIVED [] = "message_received";
const char DEBUGGER_NAME_MESSAGE_SENT [] = "message_sent";
const char DEBUGGER_NAME_COMMAND_SENT [] = "command_sent";
const char DEBUGGER_NAME_DATA_REQUEST_SENT [] = "data_request_sent";

const int BUFFER_SIZE = 4096;

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

/*Serao apenas necessarios caso queiramos fazer esse codigo fazer a analise do controller status

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
std::string displayMessage(const unsigned char* bytes, int m_bufSize){
  std::stringstream ss("");
  
  ss << "0x" << std::hex;
  for (int i = 0; i < m_bufSize; i++) 
      ss << ' ' << +bytes[i];
      
  return ss.str();
}

std::string displayFaultCode(short faultCode){
  return std::bitset<8 * sizeof(faultCode)>(faultCode).to_string();
}

class KydasDriver{
  public:

    KydasDriver(int port = 16, int bdrate = 115200, float timeout_time = 2f);
    ~KydasDriver();

    int openComport();
    void update();
  private:
    //Dados para comunicacao serial
    int m_cport_nr;
    int m_bdrate;
    std::string m_mode;
    //Buffers e outros para receber dados do serial
    unsigned char * m_buf;
    int m_bufferMaxSize;
    int m_positionInBuf;
    int m_bufSize;
    unsigned char m_currentHeaderBeingRead;

    //Dados a serem enviados pela serial------
    bool m_isConnected; //Indica se o driver foi conectado
    double m_speed_cmd;
    //----------------------------------------

    //Dados do driver ------------------------
    int m_current; // Amps
    int m_rotorPosition; //
    int m_voltage; // Volts
    int m_temperature; // °C
    short m_faultCode; // vide manual

    int m_raw_speed; // rad/s
    double m_speed;
    int m_raw_position;
    double m_position; // rad
    
    int m_programVersion;

    int m_eletricalAngle;

    unsigned char m_controlMode;
    unsigned char m_feedbackWay;
    unsigned char m_workingMode;
        
    float m_timeoutTime;

    //Funcoes gerais de ler serial
    void readSerial();
    void readMessagesOnBuffer();
    //Funcoes para enviar comandos
    int m_currentCommandBeingSent;
    void sendSerial();

    void setSpeed(int value, unsigned char controlMode = 1);
    void requestQueryData(unsigned char command);
    int sendMessage(std::array<unsigned char> msg);
    //Funcoes para receber dados
    int readQueryData(unsigned char* bytes, int currentPosition);
    int readHeartbeatData(unsigned char* bytes, int currentPosition);
};
#endif