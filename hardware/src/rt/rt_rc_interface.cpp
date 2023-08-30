#include "utilities/edge_trigger.hpp"
#include "control_flags.hpp"
#include <pthread.h>
#include <rt/rt_rc_interface.h>
#include <rt/rt_sbus.h>
#include <stdio.h>
#include <string.h>  // memcpy

/* ------------------------- HANDLERS ------------------------- */

EdgeTrigger< int >                mode_edge_trigger( 0 );
EdgeTrigger< TaranisSwitchState > backflip_prep_edge_trigger( kSwitchUp );
EdgeTrigger< TaranisSwitchState > experiment_prep_edge_trigger( kSwitchUp );
TaranisSwitchState                initial_mode_go_switch = kSwitchDown;

RcCommand  data;
RcCommand* SbusPacketCompleteAt9s( uint16_t* channels ) {
    UpdateTaranisAt9s( &data, channels );
    return &data;
}

void* VMemcpy( void* dest, volatile void* src, size_t n ) {
    void* src_2 = ( void* )src;
    return memcpy( dest, src_2, n );
}

float Deadband( float command, float deadbandRegion, float minVal, float maxVal ) {
    if ( command < deadbandRegion && command > -deadbandRegion ) {
        return 0.0;
    }
    else {
        return ( command / ( 2 ) ) * ( maxVal - minVal );
    }
}
