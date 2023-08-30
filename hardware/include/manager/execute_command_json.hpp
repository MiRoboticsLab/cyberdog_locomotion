
#ifndef MANAGER_EXECUTE_COMMAND_JSON_HPP_
#define MANAGER_EXECUTE_COMMAND_JSON_HPP_

#include <string>

struct ExecuteCommandJson {
    std::string cmd;    /* json cmd for this test */
    std::string path;   /* test program location */
    std::string name;   /* test program name */
    std::string pc_cmd; /* cyberdog_control : 0-do nothing; 1-kill; 2-kill and restart later*/
};

#endif  // MANAGER_EXECUTE_COMMAND_JSON_HPP_
