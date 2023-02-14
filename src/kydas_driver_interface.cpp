#include "kydas_driver/kydas_driver.h"

KydasDriver::KydasDriver(): 
 m_nh{},
 m_positionInBuf{0}, m_bufSize{0}, m_currentHeaderBeingRead{0}, m_bufferMaxSize{BUFFER_SIZE},
 m_mode{"8N1"},
 m_isConnected{false},
 m_messagesToSend{}
{
  //Getting Params
  ros::NodeHandle nh_param("~");
  nh_param.param<int>("port", m_cport_nr, 16);
  nh_param.param<int>("bdrate", m_bdrate, 115200);
  nh_param.param<float>("loop_rate", m_loop_rate, 25);
  nh_param.param<float>("request_data_rate", m_request_data_rate, 8);
  nh_param.param<float>("response_check_time", m_response_check_time, 0.25f);
  nh_param.param<float>("timeout_time", m_timeoutTime, 2);
  

  //Creating buffer
  m_buf = new unsigned char[m_bufferMaxSize];

  //Creating timer to send values to driver
  m_loopTimer = m_nh.createTimer(ros::Rate(m_loop_rate) ,&KydasDriver::loopCallback, this); //Timer for general loop
  m_requestDataTimer = m_nh.createTimer(ros::Rate(m_request_data_rate), &KydasDriver::requestDataLoopCallback, this); //Timer for requesting data at a steady rate (to avoid connection loss)
  m_responseCheckTimer = m_nh.createTimer(ros::Duration(m_response_check_time), &KydasDriver::driverReponseCheckCallback, this); //Timer for checking if the driver is still connected

  //Setting Publishers
  m_controllerStatus_pub = m_nh.advertise<kydas_driver::MotorControllerStatus>("controllerStatus", 1000);
  m_current_pub = m_nh.advertise<kydas_driver::MotorCurrent>("current", 1000);
  m_eletricAngle_pub = m_nh.advertise<kydas_driver::MotorEletricAngle>("eletricAngle", 1000);
  m_faultCode_pub = m_nh.advertise<kydas_driver::MotorFaultCode>("faultCode", 1000);
  m_position_pub = m_nh.advertise<kydas_driver::MotorPosition>("position", 1000);
  m_programVersion_pub = m_nh.advertise<kydas_driver::MotorProgramVersion>("programVersion", 1000);
  m_rotorPosition_pub = m_nh.advertise<kydas_driver::MotorRotorPosition>("rotorPosition", 1000);
  m_speed_pub = m_nh.advertise<kydas_driver::MotorSpeed>("speed", 1000);
  m_temp_pub = m_nh.advertise<sensor_msgs::Temperature>("temperature", 1000);
  m_voltage_pub = m_nh.advertise<kydas_driver::MotorVoltage>("voltage", 1000);
  //Setting Services
  m_cmdSpeed_sub = m_nh.subscribe("cmd_speed", 1000, &KydasDriver::cmdSpeed, this);
}

KydasDriver::~KydasDriver(){
  delete[] m_buf;
}

int KydasDriver::openComport(){  
  return RS232_OpenComport(m_cport_nr, m_bdrate, m_mode.c_str(), 0);
}

void KydasDriver::readSerial(){
  if(m_currentHeaderBeingRead == 0){ //Nao temos mensagem pendente. ler normalmente
    m_bufSize = RS232_PollComport(m_cport_nr, m_buf, m_bufferMaxSize - 1);
    m_positionInBuf = 0;
  }
  else{ //Temos que ler a possivel parte faltante da mensagem
    //1 - remover o que ja lido (os i-nesimos primeiros bytes) e movimentar o que esta para frente para o inicio
    //2 - ler a partir de do fim dos que nao foram lidos
    //3 - voltar i = 0, e m_bufSize = tamanho total (nao lidos + recem lidos)
    //EXEMPLO: X - lido, + - nao lidos, % - recem lido, - - vazio
    //m_buf = XXXXX++ (i = 5)
    //1 - m_buf = -----++ -> m_buf = ++
    //2 - m_buf = ++%%%
    int amount_of_not_read = m_bufSize - m_positionInBuf;
    memcpy(m_buf, &m_buf[m_positionInBuf], amount_of_not_read);
    int amount_read = RS232_PollComport(m_cport_nr,&m_buf[m_positionInBuf + 1], m_bufferMaxSize - 1 - amount_of_not_read);
    m_positionInBuf = 0;
    m_bufSize = amount_of_not_read + amount_read;
  }
  std::string s = displayMessage(m_buf, m_bufSize);
  const char* cstr = s.c_str();
  ROS_DEBUG_NAMED(DEBUGGER_NAME_MESSAGE_RECEIVED, "message = [%s]", cstr);
}

void KydasDriver::readMessagesOnBuffer(){  
  bool receivedMessage = false;

  while(m_positionInBuf < m_bufSize){
    if(m_buf[m_positionInBuf] == HEARTBEAT_HEADER){
      receivedMessage = true;

      if(m_bufSize < 13){
        m_currentHeaderBeingRead = HEARTBEAT_HEADER;
        ROS_DEBUG("trying to read heartbeat next loop");
        break;
      }
      m_positionInBuf += readHeartbeatData(m_buf, m_positionInBuf);
      m_currentHeaderBeingRead = 0;
    }
    else if(m_buf[m_positionInBuf] == QUERY_HEADER){
      receivedMessage = true;

      if(m_bufSize < 6){
        m_currentHeaderBeingRead = QUERY_HEADER;
        ROS_DEBUG("trying to read query next loop");
        break;
      }
      m_positionInBuf += readQueryData(m_buf, m_positionInBuf);
      m_currentHeaderBeingRead = 0;
    }
    else{
      m_positionInBuf++;
    }
  }
  if(receivedMessage){
    if(!m_isConnected){
      m_isConnected = true;
      ROS_INFO("driver connected!");
    }
    m_lastReceivedDataTimeFromDriver = ros::Time::now();
  }
}

void KydasDriver::requestDataLoopCallback(const ros::TimerEvent&){
  if(!m_isConnected){
    return;
  }
  
  switch (m_currentCommandBeingSent)
  {
    case 0:
      requestQueryData((unsigned char)Query_Data::Speed);
      break;
    case 1:
      requestQueryData((unsigned char)Query_Data::Position);
      break;
  }
  m_currentCommandBeingSent = (m_currentCommandBeingSent + 1) % 2;
}

void KydasDriver::loopCallback(const ros::TimerEvent&)
{  
  sendNextMessage(); //Enviar a proxima mensagem na queue
  readSerial(); //Lendo do serial
  readMessagesOnBuffer(); //Interpretando a mensagem
}

void KydasDriver::driverReponseCheckCallback(const ros::TimerEvent&){
  if(m_isConnected){
    ros::Duration deltaTime = ros::Time::now() - m_lastReceivedDataTimeFromDriver;
    double delta = deltaTime.toSec();
    if(delta > m_timeoutTime){
      m_isConnected = false;
      ROS_WARN("driver timedout! [%f]", delta);
    }
  }
}