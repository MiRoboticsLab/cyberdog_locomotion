
#ifndef MANAGER_PARSE_JSON_ON_FORK_PARA_CONF_HPP_
#define MANAGER_PARSE_JSON_ON_FORK_PARA_CONF_HPP_

#include <document.h>
#include <iostream>
#include <string>
#include <vector>
#include <writer.h>

#include "fork_para_conf_json.hpp"

class ParseJsonOnForkParaConf {
public:
    ParseJsonOnForkParaConf();

    ~ParseJsonOnForkParaConf();

    std::vector< ForkParaConfJson > ParseJsonData( const char* strJson );

private:
    std::string ReadFile( const char* fileName );

    bool GetParaValueByKey( ForkParaConfJson& jsonCong, int key, std::string jsonValue );
};

#endif  // MANAGER_PARSE_JSON_ON_FORK_PARA_CONF_HPP_
