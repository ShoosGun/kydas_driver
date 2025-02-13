#ifndef KYDAS_DRIVER_H
#define KYDAS_DRIVER_H

#include "ros/ros.h"

#include <bitset>
#include <sstream>
#include <string>
#include <vector>

const unsigned char CONTROL_HEADER = 0xE0;
const unsigned char QUERY_HEADER = 0xED;
const unsigned char HEARTBEAT_HEADER = 0xEE;

const char DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW[] = "heartbeat_data_preview";
const char DEBUGGER_NAME_QUERY_DATA_PREVIEW[] = "query_data_preview";
const char DEBUGGER_NAME_MESSAGE_RECEIVED[] = "message_received";
const char DEBUGGER_NAME_MESSAGE_SENT[] = "message_sent";
const char DEBUGGER_NAME_COMMAND_SENT[] = "command_sent";
const char DEBUGGER_NAME_DATA_REQUEST_SENT[] = "data_request_sent";

const int BUFFER_SIZE = 4096;

enum class Control_Data { SpeedMode = 1, TorqueMode, PositionMode };

enum class Query_Data {
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

enum class ControlStatus_ControlMode { Analog = 1, CAN, RS232, RC };

enum class ControlStatus_Feedback {
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

enum class ControlStatus_WorkingMode {
  Speed = 1,
  Torque,
  AbsolutePosition,
  RelativePosition
};

std::string displayMessage(const unsigned char *bytes, int m_bufSize);

std::string displayFaultCode(short faultCode);

class KydasDriver {
public:
  KydasDriver(std::string port, float timeout_time = 2.f,
              double speed_filter_weight = 1.0,
              double position_filter_weight = 1.0, int speed_filter_size = 5,
              int position_filter_size = 5);
  ~KydasDriver();

  int openComport();
  void update();
  // Dados a serem enviados pela serial------
  bool isConnected; // Indica se o driver foi conectado
  double speed_cmd;
  //----------------------------------------

  // Dados do driver ------------------------
  int current;       // Amps
  int rotorPosition; //
  int voltage;       // Volts
  int temperature;   // °C
  short faultCode;   // vide manual

  int raw_speed; // rad/s
  double speed;
  double filtered_speed;
  int raw_position;
  double position; // rad
  double filtered_position;

  int programVersion;

  int eletricalAngle;

  unsigned char controlMode;
  unsigned char feedbackWay;
  unsigned char workingMode;

private:
  // Dados para comunicacao serial
  std::string m_port_name;
  int m_cport;
  int m_bdrate;
  // Buffers e outros para receber dados do serial
  unsigned char *m_buf;
  int m_bufferMaxSize;
  int m_positionInBuf;
  int m_bufSize;
  unsigned char m_currentHeaderBeingRead;

  float m_timeoutTime;

  int m_speed_filter_size;
  double m_speed_filter_weight;
  std::vector<double> m_speeds;
  int m_position_filter_size;
  double m_position_filter_weight;
  std::vector<double> m_positions;

  // Funcoes gerais de ler serial
  void readSerial();
  void readMessagesOnBuffer();
  // Funcoes para enviar comandos
  int m_currentCommandBeingSent;
  void sendSerial();

  void setSpeed(int value, unsigned char controlMode = 1);
  void requestQueryData(unsigned char command);
  int sendMessage(unsigned char *msg, int size);
  // Funcoes para receber dados
  int readQueryData(unsigned char *bytes, int currentPosition);
  int readHeartbeatData(unsigned char *bytes, int currentPosition);
  // Funcao para verificar se esta conectado
  void driverReponseCheck();
  ros::WallTime m_lastReceivedDataTimeFromDriver;
};
#endif
