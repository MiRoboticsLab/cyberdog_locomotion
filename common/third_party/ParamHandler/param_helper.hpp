/*!
 * @file param_helper.hpp
 * @brief some helper function for parameter parser
 *
 *
 * Authored by JackeyHuo, 2021-02-07
 */

#ifndef PARAM_HELPER_H
#define PARAM_HELPER_H

#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

template < typename T > std::vector< T > loadVectorFromString( const std::string& str_buff ) {
    std::vector< T > vv;
    try {
        YAML::Node node = YAML::Load( str_buff );
        vv              = node.as< std::vector< T > >();
    }
    catch ( std::exception& e ) {
        // do nothing
    }
    return vv;
}

template < typename T > std::vector< std::vector< T > > loadMatFromString( const std::string& str_buff ) {
    std::vector< std::vector< T > > mm;
    try {
        YAML::Node node = YAML::Load( str_buff );
        mm              = node.as< std::vector< std::vector< T > > >();
    }
    catch ( std::exception& e ) {
        // do nothing
    }
    return mm;
}

#endif  // PARAM_HELPER_H
