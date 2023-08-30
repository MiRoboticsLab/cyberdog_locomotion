#ifndef MANAGER_HANDLE_LEG_TEST_CLIENT_HPP_
#define MANAGER_HANDLE_LEG_TEST_CLIENT_HPP_

#include <algorithm>
#include <string>
#include <vector>

#include "libraries.h"

void HandleLegTestClient( const int connfd, const pid_t extprog_pid, const char* fileName = "" );

#endif  // MANAGER_HANDLE_LEG_TEST_CLIENT_HPP_
