#include "hardware/serialport/serialport.hpp"
#include "tools/config/config.hpp"

#include <exception>
#include <iostream>
#include <string>

int main(int argc, char ** argv)
try
{
  // 配置文件
  const std::string path = "apps/infantry/config.yaml";
  const tools::config::Config config(path);

  // 串口通信
  hardware::serialport::SerialPort serial(config);
  if (serial.is_open())
  {
    std::cout << "serial port is open\n";
  }
  else
  {
    std::cout << "electronic control serial port is not connected, retry every 5s in background\n";
  }

  return 0;
}
catch (const std::exception & error)
{
  std::cerr << "infantry exception: " << error.what() << "\n";
  return 1;
}
