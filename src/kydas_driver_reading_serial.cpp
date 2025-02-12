#include "kydas_driver/kydas_driver.h"
#include <cmath>
#include <numeric>
#include <vector>

// From
// https://www.geeksforgeeks.org/finding-median-of-unsorted-array-in-linear-time-using-c-stl/
double findMedian(std::vector<double> vector, int n) {
  // If size of the arr[] is even
  std::vector<double> vec{vector};
  if (n % 2 == 0) {

    // Applying nth_element
    // on n/2th index
    std::nth_element(vec.begin(), vec.begin() + n / 2, vec.end());

    // Applying nth_element
    // on (n-1)/2 th index
    std::nth_element(vec.begin(), vec.begin() + (n - 1) / 2, vec.end());

    // Find the average of value at
    // index N/2 and (N-1)/2
    return (double)(vec[(n - 1) / 2] + vec[n / 2]) / 2.0;
  }

  // If size of the arr[] is odd
  else {

    // Applying nth_element
    // on n/2
    std::nth_element(vec.begin(), vec.begin() + n / 2, vec.end());

    // Value at index (N/2)th
    // is the median
    return (double)vec[n / 2];
  }
}

int KydasDriver::readQueryData(unsigned char *bytes, int currentPosition) {
  if (bytes[currentPosition] != QUERY_HEADER) {
    return 0; // Means we read nothing, because it isn't the right return data
  }
  Query_Data dataType = (Query_Data)bytes[currentPosition + 1];
  if (dataType == Query_Data::ControlStatus) {
    // 1.5 bytes value
    controlMode = bytes[currentPosition + 2] >> 4; // Gets the first 4 bits
    feedbackWay = bytes[currentPosition + 2] % 16; // Gets the second 4 bits
    workingMode = bytes[currentPosition + 3] >> 4; // Get the last 4 bits

    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW,
                    "Controller Status:\n Control Mode [%d]\n Feedback Way "
                    "[%d]\n Working Mode [%d]",
                    controlMode, feedbackWay, workingMode);
  } else if (dataType == Query_Data::EletricalAngle) {
    // 2 bytes value
    eletricalAngle =
        (short)(bytes[currentPosition + 2] << 8 | bytes[currentPosition + 3]);

    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Eletrical Angle [%d]",
                    eletricalAngle);
  } else if (dataType == Query_Data::Speed) {
    // 2 bytes value
    raw_speed =
        (short)(bytes[currentPosition + 2] << 8 | bytes[currentPosition + 3]);
    speed = (raw_speed / 0.15f) * M_PI / 180.f;

    // filtering speed
    m_speeds.insert(m_speeds.begin(), speed);
    if (m_speeds.size() >= m_speed_filter_size) {
      filtered_speed =
          findMedian(m_speeds, m_speed_filter_size) * m_speed_filter_weight +
          speed * (1.0 - m_speed_filter_weight);
      m_speeds.pop_back();
    } else {
      filtered_speed = NAN;
    }

    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW,
                    "Motor Speed [%f] RPS (%d)", speed, raw_speed);
  } else if (dataType == Query_Data::Current) {
    // 2 bytes value
    current =
        (short)(bytes[currentPosition + 2] << 8 | bytes[currentPosition + 3]);

    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Motor Current [%d] A",
                    current);
  } else if (dataType == Query_Data::RotorPosition) {
    // 2 bytes value
    rotorPosition =
        (short)(bytes[currentPosition + 2] << 8 | bytes[currentPosition + 3]);

    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Rotor Position [%d]",
                    rotorPosition);
  } else if (dataType == Query_Data::Voltage) {
    // 1 byte value
    voltage = (int)bytes[currentPosition + 2];

    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Voltage [%d] V",
                    voltage);
  } else if (dataType == Query_Data::Temperature) {
    // 2 bytes value
    temperature =
        (short)(bytes[currentPosition + 2] << 8 | bytes[currentPosition + 3]);

    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Temperature [%d] C",
                    temperature);
  } else if (dataType == Query_Data::FaultCode) {
    // 2 "bytes" value
    faultCode =
        (short)(bytes[currentPosition + 2] << 8 | bytes[currentPosition + 3]);

    std::string faultCodeStr = displayFaultCode(faultCode);
    const char *faultCodeCStr = faultCodeStr.c_str();
    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Fault Code = [0b%s]",
                    faultCodeCStr);
  } else if (dataType == Query_Data::Position) {
    // 4 bytes value
    raw_position =
        (int)(bytes[currentPosition + 2] << 24 |
              bytes[currentPosition + 3] << 16 |
              bytes[currentPosition + 4] << 8 | bytes[currentPosition + 5]);
    position = (raw_position / 10000.f) * 2 * M_PI;

    // filtering position
    m_positions.insert(m_positions.begin(), position);
    if (m_positions.size() >= m_position_filter_size) {
      filtered_position = findMedian(m_positions, m_position_filter_size) *
                              m_position_filter_weight +
                          position * (1.0 - m_position_filter_weight);
      m_positions.pop_back();
    } else {
      filtered_position = NAN;
    }

    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW,
                    "Position [%d] 10000/circle (%f) rad", raw_position,
                    position);
  } else if (dataType == Query_Data::ProgramVersion) {
    // 4 bytes value
    programVersion =
        (int)(bytes[currentPosition + 2] << 24 |
              bytes[currentPosition + 3] << 16 |
              bytes[currentPosition + 4] << 8 | bytes[currentPosition + 5]);

    ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "Program Version [%d]",
                    programVersion);
  }

  std::string s = displayMessage(&bytes[currentPosition], 6);
  const char *cstr = s.c_str();
  ROS_DEBUG_NAMED(DEBUGGER_NAME_QUERY_DATA_PREVIEW, "message = [%s]", cstr);
  return 6; // Counting the header, the query data is always 6 bytes
}

int KydasDriver::readHeartbeatData(unsigned char *bytes, int currentPosition) {
  if (bytes[currentPosition] != HEARTBEAT_HEADER) {
    return 0; // Means we read nothing, because it isn't the right return data
  }
  eletricalAngle =
      (short)(bytes[currentPosition + 1] << 8 | bytes[currentPosition + 2]);

  faultCode =
      (short)(bytes[currentPosition + 3] << 8 | bytes[currentPosition + 4]);

  temperature = (int)bytes[currentPosition + 5];
  voltage = (int)bytes[currentPosition + 6];

  raw_speed =
      ((short)(bytes[currentPosition + 7] << 8 | bytes[currentPosition + 8]));
  speed = (raw_speed / 0.15f) * M_PI / 180.f;

  raw_position =
      (int)(bytes[currentPosition + 9] << 24 |
            bytes[currentPosition + 10] << 16 |
            bytes[currentPosition + 11] << 8 | bytes[currentPosition + 12]);
  position = (raw_position / 10000.f) * 2 * M_PI;

  // Publishing all this data
  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW, "Eletrical Angle [%d]",
                  eletricalAngle);

  std::string faultCodeStr = displayFaultCode(faultCode);
  const char *faultCodeCStr = faultCodeStr.c_str();
  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW, "Fault Code = [0b%s]",
                  faultCodeCStr);

  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW, "Temperature [%d] C",
                  temperature);

  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW, "Voltage [%d] V",
                  voltage);

  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW, "Speed [%f] RPS",
                  speed);

  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW,
                  "Position [%d] 10000/circle (%f) rad", raw_position,
                  position);

  std::string s = displayMessage(&bytes[currentPosition], 13);
  const char *cstr = s.c_str();
  ROS_DEBUG_NAMED(DEBUGGER_NAME_HEARTBEAT_DATA_PREVIEW, "message = [%s]", cstr);

  // filtering speed
  m_speeds.insert(m_speeds.begin(), speed);
  if (m_speeds.size() >= m_speed_filter_size) {
    filtered_speed =
        findMedian(m_speeds, m_speed_filter_size) * m_speed_filter_weight +
        speed * (1.0 - m_speed_filter_weight);
    m_speeds.pop_back();
  } else {
    filtered_speed = NAN;
  }

  // filtering position
  m_positions.insert(m_positions.begin(), position);
  if (m_positions.size() >= m_position_filter_size) {
    filtered_position = findMedian(m_positions, m_position_filter_size) *
                            m_position_filter_weight +
                        position * (1.0 - m_position_filter_weight);
    m_positions.pop_back();
  } else {
    filtered_position = NAN;
  }

  return 13;
}
