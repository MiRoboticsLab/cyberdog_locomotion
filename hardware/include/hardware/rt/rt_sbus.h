#ifndef RT_SBUS_H_
#define RT_SBUS_H_

#include "command_interface/rc_command.hpp"
#include <stdint.h>
#define RC_AT9s

// Name of SBUS serial port in simulator
#define K_SBUS_PORT_SIM "/dev/ttyUSB0"
// Name of SBUS serial port on the cyberdog2
#define K_SBUS_PORT_MC "/dev/ttyS4"

void UnpackSbusData( uint8_t sbus_data[], uint16_t* channels );

int ReadSbusData( int port, uint8_t* sbus_data );

int ReadSbusChannel( int channel );

int ReceiveSbus( int port, uint16_t* channels );

int InitSbus( int is_simulator );

enum TaranisSwitchState {
    kSwitchUp     = 0,
    kSwitchMiddle = 1,
    kSwitchDown   = 2,
};

struct Taranis_X7_data {
    TaranisSwitchState left_upper_switch, left_lower_left_switch, left_lower_right_switch, right_upper_switch, right_lower_left_switch, right_lower_right_switch;

    float left_stick[ 2 ];
    float right_stick[ 2 ];
    float knobs[ 2 ];
};

void UpdateTaranisX7( Taranis_X7_data* data );

void UpdateTaranisAt9s( RcCommand* data, uint16_t* channels );
#endif  // RT_SBUS_H_
