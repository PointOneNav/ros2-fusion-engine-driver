#ifndef PORT8READER_HPP
#define PORT8READER_HPP

#include <iostream>
#include <string>
#include <cstring>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <unistd.h> // write(), read(), close()
#include <termios.h> // Contains POSIX terminal control definitions
#include <errno.h> // Error integer and strerror() function

class portReader
{
private:
    std::string _port;
    int _serialPort;
    termios tty;

public:
    portReader(std::string const& port) :
        _port(port)
    {
        _serialPort = open(_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (_serialPort < 0) {
            throw std::runtime_error("Error opening serial port");
        }
        tcgetattr(_serialPort, &tty);
        cfmakeraw(&tty);
        tcsetattr(_serialPort, TCSANOW, &tty);
    };

    uint8_t portRead(ssize_t n, uint8_t *buffer)
    {
        return read(_serialPort, buffer, n);
    }

    ~portReader()
    {
        close(_serialPort);
    };
};

#endif
