#pragma once

#include <errno.h>    // Error integer and strerror() function
#include <fcntl.h>    // Contains file controls like O_RDWR
#include <termios.h>  // Contains POSIX terminal control definitions
#include <unistd.h>   // write(), read(), close()

#include <cstring>
#include <iostream>
#include <string>

class SerialPort {
 public:
  SerialPort() = default;

  bool Open(const char* device_path, uint32_t baud_rate);

  int Read(void* read_buffer, size_t read_size);

  int Write(const void* write_buffer, size_t write_size);

 private:
  using SerialFile = int;
  SerialFile serial_port_;
};