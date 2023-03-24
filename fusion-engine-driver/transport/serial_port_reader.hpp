#pragma once

#include <errno.h>    // Error integer and strerror() function
#include <fcntl.h>    // Contains file controls like O_RDWR
#include <termios.h>  // Contains POSIX terminal control definitions
#include <unistd.h>   // write(), read(), close()

#include <cstring>
#include <iostream>
#include <string>

class SerialPortReader {
 private:
  std::string _port;
  int _serialPort;
  termios tty;

 public:
  SerialPortReader(std::string const& port);
  ~SerialPortReader();

  uint8_t portRead(ssize_t n, uint8_t* buffer);
};

