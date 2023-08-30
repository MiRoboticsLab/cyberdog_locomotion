
#ifndef MANAGER_FORK_PROGRAM_HPP_
#define MANAGER_FORK_PROGRAM_HPP_

#include <algorithm>
#include <string>
#include <vector>

#include "libraries.h"

int ForkProgram( pid_t* pid, const char* name, const char* path, const char* log_path, char* const argv[] );

#endif  // MANAGER_FORK_PROGRAM_HPP_
