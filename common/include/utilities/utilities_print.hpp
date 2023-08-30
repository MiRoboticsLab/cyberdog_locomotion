#ifndef UTILITIES_PRINT_HPP_
#define UTILITIES_PRINT_HPP_

#include <iostream>
#include <stdarg.h>
#include <stdio.h>
#include <string>

#include "cpp_types.hpp"

/**
 * @brief Floating point value to string.
 *
 * @param input_data input data
 * @return std::string output string
 */
template < typename T > std::string PrettyPrintfString( T input_data ) {
    static int const buffer_length( 32 );
    static char      buffer[ buffer_length ];
    memset( buffer, 0, sizeof( buffer ) );
    snprintf( buffer, buffer_length - 1, "% 6.6f  ", input_data );
    std::string output_string( buffer );
    return output_string;
}

/**
 * @brief Operation function of print out
 *
 * @param input_data input data that is printed out
 * @param os operation ostream
 * @param title print title
 * @param prefix print prefix
 * @param fullline_print flag represents whether print out fullline by fullline, default is false
 * @param not_newline_print flag represents whether print out by newline, default is false
 */
template < typename T >
void PrettyPrint( DMat< T > const& input_data, std::ostream& os, std::string const& title, std::string const& prefix = "", bool fullline_print = false, bool not_newline_print = false ) {
    char const* newline( "\n" );
    if ( not_newline_print ) {
        newline = "";
    }
    if ( !title.empty() ) {
        os << title << newline;
    }
    if ( ( input_data.rows() <= 0 ) || ( input_data.cols() <= 0 ) ) {
        os << prefix << " (empty)" << newline;
    }
    else {
        if ( fullline_print ) {
            if ( !prefix.empty() )
                os << prefix;
            for ( int ir( 0 ); ir < input_data.rows(); ++ir ) {
                os << PrettyPrintfString( input_data.coeff( ir, 0 ) );
            }
            os << newline;
        }
        else {
            for ( int ir( 0 ); ir < input_data.rows(); ++ir ) {
                if ( !prefix.empty() )
                    os << prefix;
                for ( int ic( 0 ); ic < input_data.cols(); ++ic ) {
                    os << PrettyPrintfString( input_data.coeff( ir, ic ) );
                }
                os << newline;
            }
        }
    }
}

template < typename T > void PrettyPrint( Quat< T > const& input_data, std::ostream& os, std::string const& title, std::string const& prefix = "", bool not_newline_print = false ) {
    PrettyPrint( ( DMat< T > const& )input_data, os, title, prefix, true, not_newline_print );
}

template < typename T > void PrettyPrint( DVec< T > const& input_data, std::ostream& os, std::string const& title, std::string const& prefix = "", bool not_newline_print = false ) {
    PrettyPrint( ( DMat< T > const& )input_data, os, title, prefix, true, not_newline_print );
}

template < typename T > void PrettyPrint( D3Mat< T > const& input_data, std::ostream& os, std::string const& title, std::string const& prefix = "", bool not_newline_print = false ) {
    PrettyPrint( ( DMat< T > const& )input_data, os, title, prefix, false, not_newline_print );
}

template < typename T > void PrettyPrint( Mat3< T > const& input_data, std::ostream& os, std::string const& title, std::string const& prefix = "", bool not_newline_print = false ) {
    PrettyPrint( ( DMat< T > const& )input_data, os, title, prefix, false, not_newline_print );
}

template < typename T > void PrettyPrint( Mat6< T > const& input_data, std::ostream& os, std::string const& title, std::string const& prefix = "", bool not_newline_print = false ) {
    PrettyPrint( ( DMat< T > const& )input_data, os, title, prefix, false, not_newline_print );
}

template < typename T > void PrettyPrint( SVec< T > const& input_data, std::ostream& os, std::string const& title, std::string const& prefix = "", bool not_newline_print = false ) {
    PrettyPrint( ( DMat< T > const& )input_data, os, title, prefix, true, not_newline_print );
}

template < typename T > void PrettyPrint( Vec3< T > const& input_data, std::ostream& os, std::string const& title, std::string const& prefix = "", bool not_newline_print = false ) {
    PrettyPrint( ( DMat< T > const& )input_data, os, title, prefix, true, not_newline_print );
}

template < typename T > void PrettyPrint( Vec2< T > const& input_data, std::ostream& os, std::string const& title, std::string const& prefix = "", bool not_newline_print = false ) {
    PrettyPrint( ( DMat< T > const& )input_data, os, title, prefix, true, not_newline_print );
}

/**
 * @brief Operation function of print out
 *
 * @param input_data vector type
 * @param title print title
 */
template < typename T > void PrettyPrint( const std::vector< T >& input_data, const char* title ) {
    printf( "%s: ", title );
    for ( size_t i( 0 ); i < input_data.size(); ++i ) {
        printf( "% 6.4f, \t", input_data[ i ] );
    }
    printf( "\n" );
}

/**
 * @brief Operation function of print out
 *
 * @param input_data pointer type
 * @param title print title
 * @param size size of input data
 */
template < typename T > void PrettyPrint( const T* input_data, const char* title, size_t size ) {
    printf( "%s: ", title );
    for ( size_t i( 0 ); i < size; ++i ) {
        printf( "% 6.4f, \t", input_data[ i ] );
    }
    printf( "\n" );
}

enum class PrintColor { kDefault, kRed, kGreen, kYellow, kBlue, kMagenta, kCyan };

void PrintfColor( PrintColor color, const char* format, ... );
void FprintfColor( PrintColor color, FILE* stream, const char* format, ... );

#endif  // UTILITIES_PRINT_HPP_
