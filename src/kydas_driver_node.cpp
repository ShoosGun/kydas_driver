#include "kydas_driver/kydas_driver.h"
#include <bitset>

void displayMessage(const unsigned char* bytes, int m_bufSize){
  std::stringstream ss("");
  
  ss << "0x" << std::hex;
  for (int i = 0; i < m_bufSize; i++) 
      ss << ' ' << +bytes[i];
      
  const std::string tmp = ss.str();
  const char* cstr = tmp.c_str();
  ROS_DEBUG("message = [%s]", cstr);
}

void displayFaultCode(short faultCode){
  const std::string tmp = std::bitset<8 * sizeof(faultCode)>(faultCode).to_string();
  const char* cstr = tmp.c_str();
  ROS_DEBUG("fault code = [%s]", cstr);
}

KydasDriverNode::KydasDriverNode(): 
 m_nh{"~"},
 m_positionInBuf{0}, m_bufSize{0}, m_currentHeaderBeingRead{0}, m_bufferMaxSize{BUFFER_SIZE},
 m_mode{"8N1"},
 m_isConnected{false}, m_isEnabled{false}
{
  m_nh.param<int>("port", m_cport_nr, 0);
  m_nh.param<int>("bdrate", m_bdrate, 115200);

  //Setting Publishers
  m_controllerStatus_pub = m_nh.advertise<kydas_driver::MotorControllerStatus>("controllerStatus", 1000);
  m_current_pub = m_nh.advertise<kydas_driver::MotorCurrent>("current", 1000);
  m_eletricAngle_pub = m_nh.advertise<kydas_driver::MotorEletricAngle>("eletricAngle", 1000);
  m_faultCode_pub = m_nh.advertise<kydas_driver::MotorFaultCode>("faultCode", 1000);
  m_position_pub = m_nh.advertise<kydas_driver::MotorPosition>("position", 1000);
  m_programVersion_pub = m_nh.advertise<kydas_driver::MotorProgramVersion>("programVersion", 1000);
  m_rotorPosition_pub = m_nh.advertise<kydas_driver::MotorRotorPosition>("rotorPosition", 1000);
  m_speed_pub = m_nh.advertise<kydas_driver::MotorSpeed>("speed", 1000);
  m_temp_pub = m_nh.advertise<kydas_driver::MotorTemp>("temp", 1000);
  m_voltage_pub = m_nh.advertise<kydas_driver::MotorVoltage>("voltage", 1000);
  //Setting Services
  m_enableMotorService = m_nh.advertiseService("enable_motor", &KydasDriverNode::enableMotor, this);
  m_disableMotorService = m_nh.advertiseService("disable_motor",&KydasDriverNode::disableMotor, this);
  m_setPositionService = m_nh.advertiseService("set_position", &KydasDriverNode::setPosition, this);
  m_setSpeedService = m_nh.advertiseService("set_speed", &KydasDriverNode::setSpeed, this);
  m_setTorqueService = m_nh.advertiseService("set_torque", &KydasDriverNode::setTorque, this);
  m_requestQueryDataService = m_nh.advertiseService("request_query_data", &KydasDriverNode::requestQueryData, this);
}

int KydasDriverNode::openComport(){
  return RS232_OpenComport(m_cport_nr, m_bdrate, m_mode.c_str(), 0);
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

void KydasDriverNode::sendMotorCommand(){
  int value;
  switch((Control_Data)m_setWorkingMode){
    case Control_Data::SpeedMode:
      value = m_setSpeed;
      break;
    case Control_Data::TorqueMode:
      value = m_setTorque;
      break;
    case Control_Data::PositionMode:
      value = m_setPosition;
      break;
  }
  if(m_setWorkingMode != m_oldSetWorkingMode || value != m_oldSetValue){
    setMotorCommand(value, m_setWorkingMode);
    m_oldSetWorkingMode = m_setWorkingMode;
    m_oldSetValue = value;
  }
}

void KydasDriverNode::update()
{  
  readSerial(); //Lendo do serial
  readMessagesOnBuffer(); //Interpretando a mensagem
  sendMotorCommand(); //Enviando mensagem de comando caso exista necessidade
}