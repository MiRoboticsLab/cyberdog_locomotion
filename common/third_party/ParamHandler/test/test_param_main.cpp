/*!
 * @file test_param_main.cpp
 * @brief
 *
 *
 * Authored by JackeyHuo, 2021-02-03
 */

#include "yaml-cpp/yaml.h"
#include <exception>
#include <iostream>

int main() {
    auto       dummy = YAML::Load( "[[1, 2, 3], [4, 5, 6], [7, 8, 9]]" );
    YAML::Node node  = dummy;
    std::cout << node.Type() << std::endl;
    for ( size_t i = 0; i < node.size(); i++ ) {
        std::cout << "the " << i << "th type: " << node[ i ].Type() << std::endl;
    }

    try {
        std::vector< std::vector< double > > vec_value = node.as< std::vector< std::vector< double > > >();
        for ( size_t i = 0; i < vec_value.size(); i++ ) {
            auto& line = vec_value[ i ];
            for ( size_t j = 0; j < line.size(); j++ ) {
                std::cout << line[ j ] << ", ";
            }
            std::cout << std::endl;
        }
    }
    catch ( std::exception& e ) {
        std::cout << e.what() << std::endl;
        std::cout << "Failed to cast matrix to vector" << std::endl;
    }

    return 0;
}
