#ifndef MANAGER_HANDLE_IMU_SERIAL_CLIENT_HPP_
#define MANAGER_HANDLE_IMU_SERIAL_CLIENT_HPP_

#include <sys/types.h>
#include <vector>

void HandleIMUSerialClient( std::vector< pid_t >& childs_pid, const pid_t extprog_pid );

#endif  // MANAGER_HANDLE_IMU_SERIAL_CLIENT_HPP_
