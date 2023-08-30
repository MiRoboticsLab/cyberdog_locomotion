#ifndef MANAGER_UTILS_HPP_
#define MANAGER_UTILS_HPP_

#include <algorithm>
#include <iostream>
#include <vector>

template < typename T > inline bool RemoveVecElement( std::vector< T >& vec, T value ) {
    auto value_iter = std::find( vec.begin(), vec.end(), value );
    if ( value_iter != vec.end() ) {
        vec.erase( value_iter );
        return true;
    }
    else {
        std::cerr << "value: " << value << " does not exist in vec" << std::endl;
        return false;
    }
}

#endif  // MANAGER_UTILS_HPP_
