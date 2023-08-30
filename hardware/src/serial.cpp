#include <iomanip>
#include <iostream>

#include "serial.h"

Serial::Serial() : _fd( -1 ) {}

Serial::~Serial() {}

int Serial::BaudRate( int baudrate ) {
    switch ( baudrate ) {
    case 0:
        return ( B2400 );
    case 1:
        return ( B4800 );
    case 2:
        return ( B9600 );
    case 3:
        return ( B19200 );
    case 4:
        return ( B38400 );
#ifndef __APPLE__
    case 5:
        return ( B576000 );
    case 6:
        return ( B921600 );
#endif
    case 7:
        return ( B115200 );
    default:
        return ( B9600 );
    }
}

int Serial::SetPara( int speed, int databits, int stopbits, int parity ) {
    struct termios termios_new;
    bzero( &termios_new,
           sizeof( termios_new ) );  //等价于memset(&termios_new,sizeof(termios_new));
    // cfmakeraw(&termios_new);  //就是将终端设置为原始模式
    termios_new.c_cflag = BaudRate( speed );
    termios_new.c_cflag |= CLOCAL | CREAD;
    //  termios_new.c_iflag = IGNPAR | IGNBRK;

    termios_new.c_cflag &= ~CSIZE;
    switch ( databits ) {
    case 0:
        termios_new.c_cflag |= CS5;
        break;
    case 1:
        termios_new.c_cflag |= CS6;
        break;
    case 2:
        termios_new.c_cflag |= CS7;
        break;
    case 3:
        termios_new.c_cflag |= CS8;
        break;
    default:
        termios_new.c_cflag |= CS8;
        break;
    }

    switch ( parity ) {
    case 0:                              // as no parity
        termios_new.c_cflag &= ~PARENB;  // Clear parity enable
        //  termios_new.c_iflag &= ~INPCK; /* Enable parity checking */
        //  //add by fu
        break;
    case 1:
        termios_new.c_cflag |= PARENB;  // Enable parity
        termios_new.c_cflag &= ~PARODD;
        break;
    case 2:
        termios_new.c_cflag |= PARENB;
        termios_new.c_cflag |= ~PARODD;
        break;
    default:
        termios_new.c_cflag &= ~PARENB;  // Clear parity enable
        break;
    }
    switch ( stopbits )  // set Stop Bit
    {
    case 1:
        termios_new.c_cflag &= ~CSTOPB;
        break;
    case 2:
        termios_new.c_cflag |= CSTOPB;
        break;
    default:
        termios_new.c_cflag &= ~CSTOPB;
        break;
    }

    termios_new.c_cflag &= ~CRTSCTS;  // no flow control
    termios_new.c_lflag       = 0;    // no signaling chars, no echo
    termios_new.c_oflag       = 0;    // no remapping, no delays
    termios_new.c_cc[ VMIN ]  = 0;    // read doesn't block
    termios_new.c_cc[ VTIME ] = 1;    // 0.5 seconds read timeout

    termios_new.c_cflag |= CREAD | CLOCAL;                                                  // turn on READ & ignore ctrl lines
    termios_new.c_iflag &= ~( IXON | IXOFF | IXANY );                                       // turn off s/w flow ctrl
    termios_new.c_lflag &= ~( ICANON | ECHO | ECHOE | ISIG );                               // make raw
    termios_new.c_oflag &= ~OPOST;                                                          // make raw
    termios_new.c_iflag &= ~( ICRNL | IGNCR | INLCR | ISTRIP | PARMRK | BRKINT | IGNBRK );  // raw mode, no converting 0x0d to 0x0a

    tcflush( _fd, TCIFLUSH );  // 清除输入缓存
    tcflush( _fd, TCOFLUSH );  // 清除输出缓存
    tcflush( _fd,
             TCIFLUSH );  //    3、 MIN > 0 , TIME =0  READ 会等待,直到MIN字元可读
    return tcsetattr( _fd, TCSANOW,
                      &termios_new );  //    4、 MIN > 0 , TIME > 0
                                       //    每一格字元之间计时器即会被启动 READ
                                       //    会在读到MIN字元,传回值或
}

int Serial::WriteData( const char* data, int datalength ) {
    if ( _fd < 0 ) {
        return -1;
    }
    int len = 0, total_len = 0;  // modify8.
    for ( total_len = 0; total_len < datalength; ) {
        len = write( _fd, &data[ total_len ], datalength - total_len );
        // printf("WriteData fd = %d ,len =%d,data = %s\n",fd,len,data);
        if ( len > 0 ) {
            total_len += len;
        }
        else if ( len <= 0 ) {
            len = -1;
            break;
        }
    }
    return len;
}

int Serial::ReadData( unsigned char* data, int datalength, bool block ) {
    if ( _fd < 0 ) {
        return -1;
    }
    size_t read_count = 0;

    if ( block ) {
        while ( ( int )read_count < datalength ) {
            ssize_t read_result = read( _fd, data + read_count, datalength - read_count );
            if ( read_result < 0 ) {
                perror( "[ERROR] Reading serial port failed" );
                break;
            }
            read_count += read_result;
        }
        return datalength;
    }
    else {  // non block
        return read( _fd, data, datalength );
    }
}

void Serial::ClosePort() {
    struct termios termios_old;
    if ( _fd > 0 ) {
        tcsetattr( _fd, TCSADRAIN, &termios_old );
        ::close( _fd );
        _fd = -1;
    }
}

int Serial::OpenPort( const char* device ) {
    struct termios termios_old;
    std::cout << "****************begin open serial common****************" << std::endl;
    _fd = open( device, O_RDWR | O_NOCTTY );  // O_RDWR | O_NOCTTY | O_NDELAY   //O_NONBLOCK

    if ( _fd < 0 ) {
        return -1;
    }

    std::cout << "*****************open serial common success******************" << std::endl;

    tcgetattr( _fd, &termios_old );
    return _fd;
}
