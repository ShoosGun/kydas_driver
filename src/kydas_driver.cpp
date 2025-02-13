#include "kydas_driver/kydas_driver.h"
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

KydasDriver::KydasDriver(std::string port, float timeout_time,
                         double speed_filter_weight,
                         double position_filter_weight, int speed_filter_size,
                         int position_filter_size)
    : m_positionInBuf{0}, m_bufSize{0}, m_currentHeaderBeingRead{0},
      m_bufferMaxSize{BUFFER_SIZE}, isConnected{false}, m_port_name{port},
      m_timeoutTime{timeout_time}, m_speed_filter_size{speed_filter_size},
      m_position_filter_size{position_filter_size},
      m_speed_filter_weight{speed_filter_weight},
      m_position_filter_weight{position_filter_weight}, m_speeds{},
      m_positions{} {
  // Creating buffer
  m_buf = new unsigned char[m_bufferMaxSize];
  // Retrieving FD of the serial port
}

KydasDriver::~KydasDriver() {
  delete[] m_buf;
  // Close and Free port
  close(m_cport);
}

int KydasDriver::openComport() {
  m_cport = open(m_port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (m_cport < 0)
    return -1;
  // See
  // https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
  // and https://www.man7.org/linux/man-pages/man3/termios.3.html
  struct termios tty;
  if (tcgetattr(m_cport, &tty) != 0)
    return -2;
  cfsetspeed(&tty, B115200);                  // Set Baudrate
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // Set so a char is 8 bits
  // Disable break processing and remove delays or anything else
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;     // no local mode
  tty.c_oflag = 0;     // no delays
  tty.c_cc[VMIN] = 0;  // no block on read (just returns 0)
  tty.c_cc[VTIME] = 5; // time out time (in deciseconds) 5 = 0.5s

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // disable xon/xoff control of io
  tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem controls and enable read
  tty.c_cflag &= ~(PARENB | PARODD); // Disables any kind of parity
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS; // Disables RTS/CTS flow control
  if (tcsetattr(m_cport, TCSANOW, &tty) != 0)
    return -3;
  return 0;
}

void KydasDriver::readSerial() {
  if (m_currentHeaderBeingRead ==
      0) { // Nao temos mensagem pendente. ler normalmente
    m_bufSize = read(m_cport, m_buf, m_bufferMaxSize);
    m_positionInBuf = 0;
  } else { // Temos que ler a possivel parte faltante da mensagem
    // 1 - remover o que ja lido (os i-nesimos primeiros bytes) e movimentar o
    // que esta para frente para o inicio 2 - ler a partir de do fim dos que nao
    // foram lidos 3 - voltar i = 0, e m_bufSize = tamanho total (nao lidos +
    // recem lidos) EXEMPLO: X - lido, + - nao lidos, % - recem lido, - - vazio
    // m_buf = XXXXX++ (i = 5)
    // 1 - m_buf = -----++ -> m_buf = ++
    // 2 - m_buf = ++%%%
    int amount_of_not_read = m_bufSize - m_positionInBuf;
    memcpy(m_buf, &m_buf[m_positionInBuf], amount_of_not_read);
    int amount_read = read(m_cport, &m_buf[m_positionInBuf + 1],
                           m_bufferMaxSize - amount_of_not_read);
    m_positionInBuf = 0;
    m_bufSize = amount_of_not_read + amount_read;
  }
  std::string s = displayMessage(m_buf, m_bufSize);
  const char *cstr = s.c_str();
  ROS_DEBUG_NAMED(DEBUGGER_NAME_MESSAGE_RECEIVED, "message = [%s]", cstr);
}

void KydasDriver::readMessagesOnBuffer() {
  bool receivedMessage = false;

  while (m_positionInBuf < m_bufSize) {
    if (m_buf[m_positionInBuf] == HEARTBEAT_HEADER) {
      receivedMessage = true;

      if (m_bufSize < 13) {
        m_currentHeaderBeingRead = HEARTBEAT_HEADER;
        ROS_DEBUG("trying to read heartbeat next loop");
        break;
      }
      m_positionInBuf += readHeartbeatData(m_buf, m_positionInBuf);
      m_currentHeaderBeingRead = 0;
    } else if (m_buf[m_positionInBuf] == QUERY_HEADER) {
      receivedMessage = true;

      if (m_bufSize < 6) {
        m_currentHeaderBeingRead = QUERY_HEADER;
        ROS_DEBUG("trying to read query next loop");
        break;
      }
      m_positionInBuf += readQueryData(m_buf, m_positionInBuf);
      m_currentHeaderBeingRead = 0;
    } else {
      m_positionInBuf++;
    }
  }
  if (receivedMessage) {
    if (!isConnected) {
      isConnected = true;
      ROS_WARN("driver connected!");
    }
    m_lastReceivedDataTimeFromDriver = ros::WallTime::now();
  }
}

void KydasDriver::sendSerial() {
  if (!isConnected) {
    return;
  }
  switch (m_currentCommandBeingSent) {
  case 0:
    requestQueryData((unsigned char)Query_Data::Speed);
    break;
  case 1:
    requestQueryData((unsigned char)Query_Data::Position);
    break;
  case 2:
    setSpeed(speed_cmd * 180 /
             M_PI); // command is in rps, but setSpeed works with dps
  }
  m_currentCommandBeingSent = (m_currentCommandBeingSent + 1) % 3;
}

void KydasDriver::update() {
  sendSerial();           // Enviar no serial
  readSerial();           // Lendo do serial
  readMessagesOnBuffer(); // Interpretando a mensagem
  driverReponseCheck();   // Checar se o driver está conectado
}

void KydasDriver::driverReponseCheck() {
  if (isConnected) {
    ros::WallDuration deltaTime =
        ros::WallTime::now() - m_lastReceivedDataTimeFromDriver;
    double delta = deltaTime.toSec();
    if (delta > m_timeoutTime) {
      isConnected = false;
      ROS_WARN("driver timedout! [%f]", delta);
    }
  }
}

std::string displayMessage(const unsigned char *bytes, int m_bufSize) {
  std::stringstream ss("");

  ss << "0x" << std::hex;
  for (int i = 0; i < m_bufSize; i++)
    ss << ' ' << +bytes[i];

  return ss.str();
}

std::string displayFaultCode(short faultCode) {
  return std::bitset<8 * sizeof(faultCode)>(faultCode).to_string();
}
