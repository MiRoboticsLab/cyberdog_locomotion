#ifndef SERIAL_H
#define SERIAL_H
#include <cstdint>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
class Serial {
public:
    Serial();
    ~Serial();

    int  OpenPort( const char* device );
    int  SetPara( int speed = 2, int databits = 8, int stopbits = 1, int parity = 0 );
    int  WriteData( const char* data, int datalength );
    int  ReadData( unsigned char* data, int datalength, bool block = true );
    void ClosePort();
    int  BaudRate( int baudrate );

    int fd() {
        return _fd;
    }

    operator bool() const {
        return ( _fd > 0 );
    }

private:
    int _fd;
};

#endif
