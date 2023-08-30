
#ifndef MANAGER_FORKPARA_CONF_JSON_HPP_
#define MANAGER_FORKPARA_CONF_JSON_HPP_

#include <string>
#include <vector>

struct ForkParaConfJson {
    std::string                name;
    std::string                object_path;
    std::string                log_path;
    std::vector< std::string > para_values;
};

#endif  // MANAGER_FORKPARA_CONF_JSON_HPP_
