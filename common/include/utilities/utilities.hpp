#ifndef UTILITIES_HPP_
#define UTILITIES_HPP_

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <unordered_map>
#include <vector>

#include "cpp_types.hpp"

/**
 * @brief The file contains common utility functions
 *
 */

/********** Sort Utilities One: Numercal Comparision **********/

/**
 * @brief Judge if two floating point values are almost equal
 *
 * @param a first value
 * @param b second value
 * @param tol equality tolerance
 * @return true is equal
 * @return false is not equal
 */
template < typename T > bool IsNumbersEqual( T a, T b, T tol ) {
    return std::abs( a - b ) <= tol;
}

/**
 * @brief Judge if two vectors are equal, compares with "!=" operator.
 *
 * @param a first vector
 * @param b second vector
 * @return true
 * @return false
 */
template < typename T > bool IsVectorEqual( const std::vector< T >& a, const std::vector< T >& b ) {
    if ( a.size() != b.size() )
        return false;
    for ( size_t i = 0; i < a.size(); i++ ) {
        if ( a[ i ] != b[ i ] )
            return false;
    }
    return true;
}

/**
 * @brief Make input value within the certain range [min, max]
 *
 * @param target input value
 * @param min minimum of the range
 * @param max maxium of the range
 * @return T
 */
template < typename T > T WrapRange( const T& target, const T& min, const T& max ) {
    T result = target;
    assert( min <= max );
    if ( result < min ) {
        result = min;
    }
    if ( max < result ) {
        result = max;
    }
    return result;
}

/**
 * @brief Map the input value in (inputMin, inputMax) to (outputMin, outputMax) linearly
 *
 * @param x input value
 * @param inputMin range of input value
 * @param inputMax range of input value
 * @param outputMin range of output value
 * @param outputMax range of output value
 * @return T
 */
template < typename T > T MapToOtherRange( T x, T inputMin, T inputMax, T outputMin, T outputMax ) {
    return outputMin + ( x - inputMin ) * ( outputMax - outputMin ) / ( inputMax - inputMin );
}

/**
 * @brief Apply deadband
 *
 * @param x input value
 * @param range deadband (+/- range around 0)
 * @return T
 */
template < typename T > T ApplyDeadband( T x, T range ) {
    if ( x < range && x > -range )
        x = T( 0 );
    return x;
}

/**
 * @brief Apply deadband and Coerce the input value to be between min and max
 *
 * @param x input value
 * @param range deadband (+/- range around 0)
 * @param min minium value
 * @param max maxium value
 * @return T
 */
template < typename T > T ApplyDeadband( T x, T range, T min, T max ) {
    if ( x < range && x > -range )
        x = T( 0 );
    return std::min( std::max( x, min ), max );
}

/**
 * @brief Apply deadband to eigen matrix
 *
 * @param v input matrix
 * @param band deadband (+/- range around 0)
 */
template < typename T > void EigenApplyDeadband( Eigen::MatrixBase< T >& v, typename T::Scalar band ) {
    for ( size_t i = 0; i < T::RowsAtCompileTime; i++ ) {
        for ( size_t j = 0; j < T::ColsAtCompileTime; j++ ) {
            v( i, j ) = ApplyDeadband( v( i, j ), band );
        }
    }
}

/**
 * @brief Get the sign of a number
 *
 * @param val input value
 * @return int 1 for positive, 0 for 0, -1 for negative
 */
template < typename T > int MathSign( T val ) {
    return ( T( 0 ) < val ) - ( val < T( 0 ) );
}

/**
 * @brief Judge whether the unordered map contains the given element?
 *
 * @param set input map
 * @param key compared element
 * @return true
 * @return false
 */
template < typename T1, typename T2 > bool DoesMapContains( const std::unordered_map< T1, T2 >& set, T1 key ) {
    return set.find( key ) != set.end();
}

/**
 * @brief Judge whether the map contains the given element?
 *
 * @param set input map
 * @param key compared element
 * @return true
 * @return false
 */
template < typename T1, typename T2 > bool DoesMapContains( const std::map< T1, T2 >& set, T1 key ) {
    return set.find( key ) != set.end();
}

/**
 * @brief Check size of a 2D vector
 *
 * @param arr input data of 2D vector
 * @param rows check rows
 * @param cols check columns
 * @return true
 * @return false
 */
template < typename T > bool Check2DArraySize( const std::vector< std::vector< T > >& arr, int rows, int cols ) {
    if ( ( int )arr.size() != rows ) {
        return false;
    }
    for ( int i = 0; i < rows; i++ ) {
        if ( ( int )arr[ i ].size() != cols ) {
            return false;
        }
    }
    return true;
}

/********** Sort Utilities Two: Data Type Conversion **********/

/**
 * @brief Convert a floating point number to a string. Is preferable over
 * std::to_string because this uses scientific notation and won't truncate
 * small/large numbers.
 *
 * @param number
 * @return std::string
 */
template < typename T > std::string NumberToString( T number ) {
    static_assert( std::is_floating_point< T >::value, "numberToString must use a floating point type!" );
    char buffer[ 100 ];
    sprintf( buffer, "%g", number );
    return std::string( buffer );
}

/**
 * @brief Convert Eigen type to std::string.
 *
 * @param value input data of Eigen type
 * @return std::string
 */
template < typename T > std::string EigenToString( Eigen::MatrixBase< T >& value ) {
    std::stringstream ss;
    ss << value;
    return ss.str();
}

/**
 * @brief Convert boolean to string (true, false)
 *
 * @param b input data of bool type
 * @return std::string
 */
static inline std::string BoolToString( bool b ) {
    return std::string( b ? "true" : "false" );
}

/**
 * @brief Convert string to float or double.
 *
 * @param str input data of string type
 * @return T
 */
template < typename T > T StringToNumber( const std::string& str ) {
    static_assert( std::is_same< T, double >::value || std::is_same< T, float >::value, "StringToNumber only works for double/float" );

    if ( std::is_same< T, double >::value ) {
        return std::stod( str );
    }
    else if ( std::is_same< T, float >::value ) {
        return std::stof( str );
    }
}

/**
 * @brief Convert character array to float or double.
 *
 * @param str input data of character array
 * @return T
 */
template < typename T > T StringToNumber( const char* str ) {
    return StringToNumber< T >( std::string( str ) );
}

/**
 * @brief Convert string to Eigen matrix.
 *
 * @param str input data of string type
 * @return Eigen::Matrix< T, Rows, Cols >
 */
template < typename T, int Rows, int Cols > Eigen::Matrix< T, Rows, Cols > StringToEigen( const std::string& str ) {
    Eigen::Matrix< T, Rows, Cols > m;
    size_t                         pos = 0;

    // seek past whitespace
    while ( str.at( pos ) == ' ' )
        pos++;
    if ( str.at( pos ) == '[' ) {
        pos++;
    }
    else {
        throw std::runtime_error( "StringToEigen didn't find open bracket" );
    }
    // the user will write matrix in row major
    for ( int i = 0; i < Rows; i++ ) {
        for ( int j = 0; j < Cols; j++ ) {
            // seek past whitespace
            while ( str.at( pos ) == ' ' )
                pos++;
            size_t start = pos;

            // seek to end of first number
            while ( str.at( pos ) != ',' && str.at( pos ) != ']' )
                pos++;
            m( i, j ) = StringToNumber< T >( str.substr( start, pos - start ) );
            pos++;
        }
    }
    return m;
}

/**
 * @brief Print into a string as printf style.
 *
 * @param format input data of "string + variables"
 * @param args print variables
 * @return std::string
 */
template < typename... Args > std::string StringFormat( const std::string& format, Args... args ) {
    int size = snprintf( nullptr, 0, format.c_str(), args... ) + 1;  // Extra space for '\0'
    if ( size <= 0 ) {
        throw std::runtime_error( "Error during formatting." );
    }
    std::unique_ptr< char[] > buf( new char[ size ] );
    snprintf( buf.get(), size, format.c_str(), args... );
    return std::string( buf.get(), buf.get() + size - 1 );  // We don't want the '\0' inside
}

/********** Sort Utilities Three: Trajectory Interpolation **********/

/**
 * @brief Smooth interpolation with cosine
 *
 * @param ini initial point
 * @param end final point
 * @param moving_duration duration of the motion
 * @param curr_time current iteration
 * @return T return current point
 */
template < typename T > T SmoothInterpolation( T ini, T end, T moving_duration, T curr_time ) {
    if ( curr_time > moving_duration ) {
        return end;
    }
    return ( ini + ( end - ini ) * 0.5 * ( 1 - cos( curr_time / moving_duration * M_PI ) ) );
}

/**
 * @brief First derivative of the forward smooth interpolation.
 *
 * @param ini initial point
 * @param end final point
 * @param moving_duration duration of the motion
 * @param curr_time current iteration
 * @return T return current velocity
 */
template < typename T > T SmoothInterpolationFirstDerivative( T ini, T end, T moving_duration, T curr_time ) {
    if ( curr_time > moving_duration ) {
        return 0.0;
    }
    return ( ( end - ini ) * 0.5 * ( M_PI / moving_duration ) * sin( curr_time / moving_duration * M_PI ) );
}

/**
 * @brief Second derivative of the forward smooth interpolation.
 *
 * @param ini initial point
 * @param end final point
 * @param moving_duration duration of the motion
 * @param curr_time current iteration
 * @return T return current accelation
 */
template < typename T > T SmoothInterpolationSecondDerivative( T ini, T end, T moving_duration, T curr_time ) {
    if ( curr_time > moving_duration ) {
        return 0.0;
    }
    return ( ( end - ini ) * 0.5 * ( M_PI / moving_duration ) * ( M_PI / moving_duration ) * cos( curr_time / moving_duration * M_PI ) );
}

/********** Sort Utilities Four: Other Operation **********/

/**
 * @brief Fill an eigen type with random numbers from a random generator and uniform real distribution.
 *
 * @param v eigen type
 * @param gen random generator
 * @param dist uniform real distribution
 */
template < typename T > void FillEigenWithRandom( Eigen::MatrixBase< T >& v, std::mt19937& gen, std::uniform_real_distribution< typename T::Scalar >& dist ) {
    for ( size_t i = 0; i < T::RowsAtCompileTime; i++ ) {
        for ( size_t j = 0; j < T::ColsAtCompileTime; j++ ) {
            v( i, j ) = dist( gen );
        }
    }
}

/**
 * @brief Generate a random number following normal distribution
 *
 * @param mean average value
 * @param var variables
 * @return T
 */
template < typename T > T GenerateGaussianNoise( T mean, T var ) {
    static bool hasSpare = false;
    static T    rand1, rand2;

    if ( hasSpare ) {
        hasSpare = false;
        return mean + sqrt( var * rand1 ) * sin( rand2 );
    }
    hasSpare = true;

    rand1 = rand() / ( ( T )RAND_MAX );
    if ( rand1 < 1e-100 )
        rand1 = 1e-100;
    rand1 = -2 * log( rand1 );
    rand2 = rand() / ( ( T )RAND_MAX ) * M_PI * 2.;

    // printf("rand: %f, %f\n", rand1, rand2);
    return mean + sqrt( var * rand1 ) * cos( rand2 );
}

void        WriteStringToFile( const std::string& fileName, const std::string& fileData );
std::string GetCurrentTimeAndDate();
std::string GetConfigDirectoryPath();
std::string GetPreinstalledMotionPathForCyberDog();
std::string GetPreinstalledMotionPathForCyberDog2();
std::string GetUserGaitDefinePath();
std::string GetUserGaitParameterPath();
std::string GetNonperiodicGaitDefinePath();
std::string GetNonperiodicGaitParameterPath();
std::string GetJumpTrajectoryPath();
std::string GetJump3dTrajecotyPath();

#endif  // UTILITIES_HPP_
