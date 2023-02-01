#ifndef KYDAS_DRIVER_H
#define KYDAS_DRIVER_H

#include "kydas_driver/rs232.h"

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

const unsigned char CONTROL_HEADER = 0xE0;
const unsigned char QUERY_HEADER = 0xED;
const unsigned char HEARTBEAT_HEADER = 0xEE;

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
void displayMessage(const unsigned char* bytes, int n);
void displayFaultCode(short faultCode);

class KydasDriverNode
{
    public:

        KydasDriverNode(int cport_nr, int bdrate, char * mode);
        ~KydasDriverNode(){};

        void update();
        int openComport();

    private:
    int m_cport_nr;
    int m_bdrate;
    char * m_mode;

    unsigned char * m_buf;
    int m_bufferMaxSize;
    int m_positionInBuf;
    int m_bufSize;
    unsigned char m_currentHeaderBeingRead;


    ros::NodeHandle m_nh;

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

    ros::ServiceServer m_enableMotorService;
    ros::ServiceServer m_disableMotorService;
    ros::ServiceServer m_setPositionService;
    ros::ServiceServer m_setSpeedService;
    ros::ServiceServer m_setTorqueService;
    ros::ServiceServer m_requestQueryDataService;

    bool enableMotor(kydas_driver::EnableMotor::Request  &req, kydas_driver::EnableMotor::Response &res);
    bool disableMotor(kydas_driver::DisableMotor::Request  &req, kydas_driver::DisableMotor::Response &res);
    int setMotorCommand(int value, unsigned char controlMode);
    bool setSpeed(kydas_driver::SetSpeed::Request  &req, kydas_driver::SetSpeed::Response &res);
    bool setTorque(kydas_driver::SetTorque::Request  &req, kydas_driver::SetTorque::Response &res);
    bool setPosition(kydas_driver::SetPosition::Request  &req, kydas_driver::SetPosition::Response &res);
    bool requestQueryData(kydas_driver::RequestQueryData::Request  &req, kydas_driver::RequestQueryData::Response &res);
    int readQueryData(unsigned char* bytes, int currentPosition);
    int readHeartbeatData(unsigned char* bytes, int currentPosition);
  
};

#endif