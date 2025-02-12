#include "kydas_driver/kydas_driver.h"

void KydasDriver::setSpeed(int value, unsigned char controlMode) {
  if (!isConnected) {
    ROS_WARN("can't set motor command: driver not connected");
    return;
  }

  if (controlMode == (unsigned char)ControlStatus_ControlMode::CAN) {
    ROS_WARN(
        "driver is on CAN mode, make sure the driver mode was correctly set");
  }

  char *valueInBytes = static_cast<char *>(static_cast<void *>(&value));
  unsigned char command[] = {CONTROL_HEADER, 0, 0, 0, 0, 0, 0, 0};

  // Placing the command mode
  if (value == 0) {
    command[1] = 0;
  } else {
    command[1] = controlMode;
  }
  // Placing the value
  command[4] = valueInBytes[3];
  command[5] = valueInBytes[2];
  command[6] = valueInBytes[1];
  command[7] = valueInBytes[0];
  sendMessage(command, 8);
  ROS_DEBUG_NAMED(DEBUGGER_NAME_COMMAND_SENT,
                  "seting motor value [%d] on mode [%d]", value,
                  (int)controlMode);
}

void KydasDriver::requestQueryData(unsigned char command) {
  if (!isConnected) {
    ROS_WARN("can't request query data: driver not connected");
    return;
  }
  unsigned char queryCommand[] = {QUERY_HEADER, 0, 0, 0, 0, 0, 0, 0};
  queryCommand[1] = command;

  sendMessage(queryCommand, 8);
  ROS_DEBUG_NAMED(DEBUGGER_NAME_DATA_REQUEST_SENT,
                  "requesting data of type [%d]", (int)command);
}

int KydasDriver::sendMessage(unsigned char *msg, int size) {
  if (size <= 0) {
    return 0;
  }
  int result = sp_nonblocking_write(m_cport, msg, size);
  result |= sp_drain(m_cport);
  std::string s = displayMessage(msg, size);
  const char *cstr = s.c_str();
  ROS_DEBUG_NAMED(DEBUGGER_NAME_MESSAGE_SENT, "message = [%s]", cstr);
  return result;
}
