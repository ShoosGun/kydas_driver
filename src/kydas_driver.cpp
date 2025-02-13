#include "kydas_driver/kydas_driver.h"

KydasDriver::KydasDriver(int port, int bdrate, float timeout_time,
    double speed_filter_weight, double position_filter_weight,
    int speed_filter_size, int position_filter_size): 
 m_positionInBuf{0}, m_bufSize{0}, m_currentHeaderBeingRead{0}, m_bufferMaxSize{BUFFER_SIZE},
 m_mode{"8N1"},
 isConnected{false},
 m_cport_nr{port}, m_bdrate{bdrate}, m_timeoutTime{timeout_time},
 m_speed_filter_size{speed_filter_size}, m_position_filter_size{position_filter_size},
 m_speed_filter_weight{speed_filter_weight}, m_position_filter_weight{position_filter_weight},
 m_speeds{}, m_positions{}
{
  //Creating buffer
  m_buf = new unsigned char[m_bufferMaxSize];
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
    if(!isConnected){
      isConnected = true;
      ROS_WARN("driver connected!");
    }
    m_lastReceivedDataTimeFromDriver = ros::WallTime::now();
  }
}

void KydasDriver::sendSerial(){
  if(!isConnected){
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
    case 2:
      setSpeed(speed_cmd * 180 / M_PI); //command is in rps, but setSpeed works with dps
  }
  m_currentCommandBeingSent = (m_currentCommandBeingSent + 1) % 3;
}

void KydasDriver::update()
{  
  sendSerial(); //Enviar no serial
  readSerial(); //Lendo do serial
  readMessagesOnBuffer(); //Interpretando a mensagem
  driverReponseCheck(); //Checar se o driver está conectado
}

void KydasDriver::driverReponseCheck(){
  if(isConnected){
    ros::WallDuration deltaTime = ros::WallTime::now() - m_lastReceivedDataTimeFromDriver;
    double delta = deltaTime.toSec();
    if(delta > m_timeoutTime){
      isConnected = false;
      ROS_WARN("driver timedout! [%f]", delta);
    }
  }
}


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
