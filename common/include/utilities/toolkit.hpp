#ifndef TOOLKIT_HPP_
#define TOOLKIT_HPP_

#include <assert.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

/**
 * @brief Get the LCM URL with desired TTL.
 *
 * @param ttl
 * @return std::string
 */
inline std::string GetLcmUrl( int64_t ttl ) {
    assert( ttl >= 0 && ttl <= 255 );
    return "udpm://239.255.76.67:7667?ttl=" + std::to_string( ttl );
}

/**
 * @brief Get the LCM URL with desired port and TTL.
 *
 * @param port
 * @param ttl
 * @return std::string
 */
inline std::string GetLcmUrlWithPort( int64_t port, int64_t ttl ) {
    assert( ttl >= 0 && ttl <= 255 );
    return "udpm://239.255.76.67:" + std::to_string( port ) + "?ttl=" + std::to_string( ttl );
}

/**
 * @brief resolve ERROR CODE in flag transformed from spine
 *
 * @param motor motor id
 * @param flag motor status input
 * @param error_string error information
 * @return true, no error detected
 * @return false, any error or warning detected
 */
inline bool ResolveErrorFlags( const int motor, const int flag, std::string& error_string ) {
    if ( !( flag & 0x00FFFFFF ) ) {
        return true;
    }  // no error

    bool res = true;

    // check if motor offline
    int is_offline = flag & 0x01;
    if ( is_offline ) {
        error_string += "motor " + std::to_string( motor + 1 ) + ": offline; ";
        res = ( res && false );
        return res;
    }

    // check if motor error reported
    std::vector< std::string > err_result = {
        "Coil Thermal-Shutdown (Over 100 Centigrade)",  // 0
        "Motor driver chip error",                      // 1
        "Motor bus under voltage (< 10V)",              // 2
        "Motor bus over voltage (> 30V)",               // 3
        "B phase sample overcurrent (> 78A)",           // 4
        "C phase sample overcurrent (> 78A)",           // 5
        "",                                             // 6
        "Encoder not calibrate",                        // 7
        "Overload error 1",                             // 8
        "Overload error 2",                             // 9
        "Overload error 3",                             // 10
        "Overload error 4",                             // 11
        "Overload error 5",                             // 12
        "Overload error 6",                             // 13
        "Overload error 7",                             // 14
        "Overload error 8",                             // 15
        "A phase sample overcurrent (> 78A)"            // 16
    };

    for ( int err = 0; err < 17; err++ ) {
        int shift_num = 1 + err;
        int is_error  = flag & ( 0x01 << shift_num );
        if ( is_error ) {
            error_string += ( "motor " + std::to_string( motor + 1 ) + " " + err_result[ err ] + "; " );
            if ( err != 7 ) {
                res = ( res && false );
            }
        }
    }

    return res;
}

/**
 * @brief resolve WARN CODE in flag transformed from spine
 *
 * @param motor motor id
 * @param flag motor status input
 * @param warn_string warning information
 * @param over_heat over heat or not
 * @return true
 * @return false
 */
inline bool ResolveWarnFlags( const int motor, const int flag, std::string& warn_string, bool& over_heat ) {
    if ( !( flag & 0x3F000000 ) ) {
        return true;
    }  // no warn

    bool res = true;

    std::vector< std::string > warn_result = {
        "Coil Thermal too high (Over 95 Centigrade)",  // 0
        "Hall sensor calibrate error",                 // 1
        "Torque calibration data illegal",             // 2
        "Torque calibration data zero offset overfit"  // 3
    };

    for ( int warn = 0; warn < 4; warn++ ) {
        int shift_num = 24 + warn;
        int is_warn   = flag & ( 0x01 << shift_num );
        if ( is_warn ) {
            warn_string += ( "motor " + std::to_string( motor + 1 ) + " " + warn_result[ warn ] + "; " );
            if ( warn == 0 ) {
                res = ( res && false );
            }
            if ( warn == 0 )
                over_heat = true;
        }
    }

    return res;
}

/**
 * @brief resolve WARN CODE in flag transformed from spine
 *
 * @param motor motor id
 * @param flag motor status input
 * @param warn_string warning information
 * @return true
 * @return false
 */
inline bool ResolveWarnFlags( const int motor, const int flag, std::string& warn_string ) {
    if ( !( flag & 0x3F000000 ) ) {
        return true;
    }  // no warn

    bool res = true;

    std::vector< std::string > warn_result = {
        "Coil Thermal too high (Over 95 Centigrade)",  // 0
        "Hall sensor calibrate error",                 // 1
        "Torque calibration data illegal",             // 2
        "Torque calibration data zero offset overfit"  // 3
    };

    for ( int warn = 0; warn < 4; warn++ ) {
        int shift_num = 24 + warn;
        int is_warn   = flag & ( 0x01 << shift_num );
        if ( is_warn ) {
            warn_string += ( "motor " + std::to_string( motor + 1 ) + " " + warn_result[ warn ] + "; " );
            if ( warn == 0 ) {
                res = ( res && false );
            }
        }
    }

    return res;
}
#endif  // TOOLKIT_HPP_
