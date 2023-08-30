
#ifndef MANAGER_PARSE_JSON_ON_EXECUTE_COMMAND_HPP_
#define MANAGER_PARSE_JSON_ON_EXECUTE_COMMAND_HPP_

#include <document.h>
#include <iostream>
#include <string>
#include <vector>
#include <writer.h>

#include "execute_command_json.hpp"

class ParseJsonOnExecuteCommand {
public:
    ParseJsonOnExecuteCommand();

    ~ParseJsonOnExecuteCommand();

    std::vector< ExecuteCommandJson > ParseJsonData( const char* strJson );

private:
    std::string ReadFile( const char* fileName );
};

#endif  // MANAGER_PARSE_JSON_ON_EXECUTE_COMMAND_HPP_
