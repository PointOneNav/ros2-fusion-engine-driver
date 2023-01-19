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
    struct termios tty;

public:
    portReader(std::string const& port) :
        _port(port)
    {
        _serialPort = open(_port.c_str(), O_RDWR);
        if (_serialPort < 0) {
            throw std::runtime_error("Error opening serial port");
        }
        std::memset(&tty, 0, sizeof tty);
        if (tcgetattr(_serialPort, &tty) != 0) {
            throw std::runtime_error("Error getting serial port attributes");
        }

        tty.c_cflag = CS8 | CREAD | CLOCAL;
        tty.c_iflag = IGNPAR;
        tty.c_oflag = 0;
        tty.c_lflag = 0;
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 5;

        if (cfsetispeed(&tty, B460800) != 0 || cfsetospeed(&tty, B460800) != 0) {
            throw std::runtime_error("Error setting baud rate");
        }

        if (tcsetattr(_serialPort, TCSANOW, &tty) != 0) {
            throw std::runtime_error("Error setting serial port attributes");
        }

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
