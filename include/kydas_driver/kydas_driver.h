#ifndef KYDAS_DRIVER_H
#define KYDAS_DRIVER_H

#include "kydas_driver/rs232.h"

#include "ros/ros.h"

#include "kydas_driver/SetSpeed.h"

#include "kydas_driver/MotorControllerStatus.h"
#include "kydas_driver/MotorCurrent.h"
#include "kydas_driver/MotorEletricAngle.h"
#include "kydas_driver/MotorFaultCode.h"
#include "kydas_driver/MotorPosition.h"
#include "kydas_driver/MotorProgramVersion.h"
#include "kydas_driver/MotorRotorPosition.h"
#include "kydas_driver/MotorSpeed.h"
#include "kydas_driver/MotorVoltage.h"

#include "sensor_msgs/Temperature.h"

#include <sstream>
#include <string>
#include <vector>
#include <queue>

const unsigned char CONTROL_HEADER = 0xE0;
const unsigned char QUERY_HEADER = 0xED;
const unsigned char HEARTBEAT_HEADER = 0xEE;

const char DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW []= "heartbeat_data_preview";
const char DEBUGGER_NAME_QUERY_DATA_PREVIEW []= "query_data_preview";
const char DEBUGGER_NAME_MESSAGE_RECEIVED [] = "message_received";
const char DEBUGGER_NAME_MESSAGE_SENT [] = "message_sent";
const char DEBUGGER_NAME_COMMAND_SENT [] = "command_sent";

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
std::string displayMessage(const unsigned char* bytes, int n);
std::string displayFaultCode(short faultCode);

class KydasDriverNode{
  public:

    KydasDriverNode();
    ~KydasDriverNode();

    int openComport();

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
    //Para saber qual comando deve ser enviado
    int m_currentCommandBeingSent;
    std::queue<std::vector<unsigned char>> m_messagesToSend;

    //Dados a serem enviados pela serial------
    bool m_isConnected; //Indica se o driver foi conectado
    int m_setSpeed; // graus/s
    //----------------------------------------

    //Dados do driver ------------------------
    int m_current; // Amps
    int m_rotorPosition; //
    int m_voltage; // Volts
    int m_temperature; // °C
    short m_faultCode; // vide manual

    int m_speed; // graus/s
    int m_position;
    
    int m_programVersion;

    int m_eletricalAngle;

    unsigned char m_controlMode;
    unsigned char m_feedbackWay;
    unsigned char m_workingMode;
    //----------------------------------------
    //ROS-------------------------------------
    ros::NodeHandle m_nh;
    
    //Loop de enviar e ler dados
    ros::Timer m_loopTimer;
    float m_loop_rate;
    void loopCallback(const ros::TimerEvent&);
    //Loop de requesitar dados
    ros::Timer m_requestDataTimer;
    float m_request_data_rate;
    void requestDataLoopCallback(const ros::TimerEvent&);
    //Loop de verificar se o driver esta respondendo
    ros::Timer m_responseCheckTimer;
    ros::Time m_lastReceivedDataTimeFromDriver;
    float m_timeoutTime;
    float m_response_check_time;
    void driverReponseCheckCallback(const ros::TimerEvent&);

    //Publicadores
    ros::Publisher m_controllerStatus_pub;
    ros::Publisher m_current_pub;
    ros::Publisher m_eletricAngle_pub;
    ros::Publisher m_faultCode_pub;
    ros::Publisher m_position_pub;
    ros::Publisher m_programVersion_pub;
    ros::Publisher m_rotorPosition_pub;
    ros::Publisher m_speed_pub;
    ros::Publisher m_temp_pub;
    ros::Publisher m_voltage_pub;

    //Servicos
    ros::ServiceServer m_enableMotorService;
    ros::ServiceServer m_disableMotorService;
    ros::ServiceServer m_setSpeedService;

    //Funcoes gerais de ler serial
    void readSerial();
    void readMessagesOnBuffer();
    //Funcoes para enviar comandos
    void sendMotorCommand();
    void setSpeed(int value, unsigned char controlMode = 1);
    int enableMotor();
    int disableMotor();
    void requestQueryData(unsigned char command);
    int sendNextMessage();
    //Funcoes para receber dados
    int readQueryData(unsigned char* bytes, int currentPosition);
    int readHeartbeatData(unsigned char* bytes, int currentPosition);

    //Callbacks dos servicos
    bool setSpeed(kydas_driver::SetSpeed::Request  &req, kydas_driver::SetSpeed::Response &res);
};

#endif