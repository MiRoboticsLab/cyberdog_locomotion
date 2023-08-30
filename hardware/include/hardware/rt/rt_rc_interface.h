#ifndef RT_RC_INTERFACE_H_
#define RT_RC_INTERFACE_H_

#include <cstddef>

#include "c_types.h"
#include "command_interface/rc_command.hpp"

RcCommand* SbusPacketCompleteAt9s( uint16_t* channels );

void* VMemcpy( void* dest, volatile void* src, size_t n );

float Deadband( float command, float deadbandRegion, float minVal, float maxVal );

#endif  // RT_RC_INTERFACE_H_
