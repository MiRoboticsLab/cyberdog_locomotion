#ifndef RT_SERIAL_H_
#define RT_SERIAL_H_

int  SetInterfaceAttribsCustomBaud( int fd, int speed, int parity, int port );
void InitSerialForSbus( int fd, int baud );

#endif  // RT_SERIAL_H_
