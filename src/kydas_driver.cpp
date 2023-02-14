#include "kydas_driver/kydas_driver.h"
#include <bitset>

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kydas_driver");
    
  KydasDriver node{};

  if(node.openComport())
  {
    ROS_WARN("can't open comport, the driver might not be connected or the port with the permissions not set");
    return 0;
  }
  
  ros::spin();

  return 0;
}