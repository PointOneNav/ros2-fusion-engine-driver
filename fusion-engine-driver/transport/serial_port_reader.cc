#include "serial_port_reader.hpp"

/******************************************************************************/
SerialPortReader::SerialPortReader(std::string const& port) : _port(port) {
  _serialPort = open(_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (_serialPort < 0) {
    throw std::runtime_error("Error opening serial port");
  }
  tcgetattr(_serialPort, &tty);
  cfmakeraw(&tty);
  tcsetattr(_serialPort, TCSANOW, &tty);
}

/******************************************************************************/
uint8_t SerialPortReader::portRead(ssize_t n, uint8_t* buffer) {
  return read(_serialPort, buffer, n);
}

/******************************************************************************/
SerialPortReader::~SerialPortReader() { close(_serialPort); }