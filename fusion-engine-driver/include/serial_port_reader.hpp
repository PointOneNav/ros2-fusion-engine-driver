#ifndef PORT8READER_HPP
#define PORT8READER_HPP

#include <iostream>
#include <string>
#include <cstring>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <unistd.h> // write(), read(), close()
#include <termios.h> // Contains POSIX terminal control definitions
#include <errno.h> // Error integer and strerror() function

class SerialPortReader
{
private:
    std::string _port;
    int _serialPort;
    termios tty;

public:
    SerialPortReader(std::string const& port);
    ~SerialPortReader();

    uint8_t portRead(ssize_t n, uint8_t *buffer);
};

#endif
