#include "kydas_driver/kydas_driver.h"

KydasDriverNode::KydasDriverNode(): 
 m_nh{"~"},
 m_positionInBuf{0}, m_bufSize{0}, m_currentHeaderBeingRead{0}, m_bufferMaxSize{BUFFER_SIZE},
 m_mode{"8N1"},
 m_isConnected{false}, m_isEnabled{false},
 m_messagesToSend{}
{
  //Getting Params
  m_nh.param<int>("port", m_cport_nr, 16);
  m_nh.param<int>("bdrate", m_bdrate, 115200);
  m_nh.param<int>("loop_rate", loop_rate, 50);
  m_nh.param<int>("command_rate", m_commandRate, 10);

  //Creating buffer
  m_buf = new unsigned char[m_bufferMaxSize];

  //Creating timer to send values to driver
  m_setValueTimer = m_nh.createTimer(ros::Duration(1.f / m_commandRate), &KydasDriverNode::sendMotorCommandLoopCallback, this);

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
  m_enableMotorService = m_nh.advertiseService("enable_motor", &KydasDriverNode::enableMotor, this);
  m_disableMotorService = m_nh.advertiseService("disable_motor",&KydasDriverNode::disableMotor, this);
  m_setSpeedService = m_nh.advertiseService("set_speed", &KydasDriverNode::setSpeed, this);
}

KydasDriverNode::~KydasDriverNode(){
  delete[] m_buf;
}

int KydasDriverNode::openComport(){
  
  int a = RS232_OpenComport(m_cport_nr, m_bdrate, m_mode.c_str(), 0);
  ROS_INFO("%d", a);
  return a;
}

void KydasDriverNode::readSerial(){
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

void KydasDriverNode::readMessagesOnBuffer(){  
  while(m_positionInBuf < m_bufSize){
    if(m_buf[m_positionInBuf] == HEARTBEAT_HEADER){

      if(!m_isConnected)
        m_isConnected = true;//Receber mensagem de Heartbeat significa que o driver está conectado
      
      if(m_bufSize < 13){
        m_currentHeaderBeingRead = HEARTBEAT_HEADER;
        ROS_DEBUG("trying to read heartbeat next loop");
        break;
      }
      m_positionInBuf += readHeartbeatData(m_buf, m_positionInBuf);
      m_currentHeaderBeingRead = 0;
    }
    else if(m_buf[m_positionInBuf] == QUERY_HEADER){
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
}

void KydasDriverNode::sendMotorCommandLoopCallback(const ros::TimerEvent&){
  switch (m_currentCommandBeingSent)
  {
    case 0:
    case 2:
      //if(m_isEnabled){
      //  setSpeed(m_setSpeed);
      //}
      break;
    case 1:
      requestQueryData((unsigned char)Query_Data::Speed);
      break;
    case 3:
      requestQueryData((unsigned char)Query_Data::Position);
      break;
  }
  m_currentCommandBeingSent = (m_currentCommandBeingSent + 1) % 4;
}

void KydasDriverNode::update()
{  
  readSerial(); //Lendo do serial
  readMessagesOnBuffer(); //Interpretando a mensagem
  sendNextMessage();
}