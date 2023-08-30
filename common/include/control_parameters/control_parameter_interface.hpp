#ifndef CONTROL_PARAMETER_INTERFACE_HPP_
#define CONTROL_PARAMETER_INTERFACE_HPP_

#include <map>

#include "control_parameters/control_parameters.hpp"

/**
 * @brief Type of message to a control parameter collection.
 *
 */
enum class ControlParameterRequestKind {
    kGET_ROBOT_PARAM_BY_NAME,
    kSET_ROBOT_PARAM_BY_NAME,
    kGET_USER_PARAM_BY_NAME,
    kSET_USER_PARAM_BY_NAME,
    kGET_SPEED_CALIBRATE_PARAM_BY_NAME,
    kSET_SPEED_CALIBRATE_PARAM_BY_NAME,
    kGET_IMU_CALIBRATE_PARAM_BY_NAME,
    kSET_IMU_CALIBRATE_PARAM_BY_NAME
};

std::string ControlParameterRequestKindToString( ControlParameterRequestKind request );

/**
 * @brief Data sent to a control parameter collection to request a get/set of a value.
 *
 */
struct ControlParameterRequest {
    char                        name[ kCONTROL_PARAMETER_MAXIMUM_NAME_LENGTH ] = "";  // name of the parameter to set/get
    u64                         requestNumber                                  = UINT64_MAX;
    ControlParameterValue       value;
    ControlParameterValueKind   parameterKind;
    ControlParameterRequestKind requestKind;

    /**
     * @brief Convert to human-readable string
     *
     * @return std::string description of the request
     */
    std::string ToString() {
        std::string result = "Request(" + std::to_string( requestNumber ) + ") " + ControlParameterRequestKindToString( requestKind ) + " " + ControlParameterValueKindToString( parameterKind ) + " "
                             + std::string( name ) + " ";
        switch ( requestKind ) {
        case ControlParameterRequestKind::kGET_USER_PARAM_BY_NAME:
            result += "user is: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kSET_USER_PARAM_BY_NAME:
            result += "user to: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kGET_ROBOT_PARAM_BY_NAME:
            result += "robot is: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kSET_ROBOT_PARAM_BY_NAME:
            result += "robot to: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kGET_SPEED_CALIBRATE_PARAM_BY_NAME:
            result += "speed_calibrate is: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kSET_SPEED_CALIBRATE_PARAM_BY_NAME:
            result += "speed_calibrate to: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kGET_IMU_CALIBRATE_PARAM_BY_NAME:
            result += "imu_calibrate is: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kSET_IMU_CALIBRATE_PARAM_BY_NAME:
            result += "imu_calibrate to: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        default:
            return result + " unknown request type!";
        }
    }
};

/**
 * @brief Data sent from a control parameter collection in response to a request.
 *
 */
struct ControlParameterResponse {
    char                        name[ kCONTROL_PARAMETER_MAXIMUM_NAME_LENGTH ] = "";
    u64                         requestNumber                                  = UINT64_MAX;
    u64                         nParameters                                    = 0;
    ControlParameterValue       value;
    ControlParameterValueKind   parameterKind;
    ControlParameterRequestKind requestKind;

    /**
     * @brief Check if a response is a valid response to a given request.
     *
     * @param request
     * @return true
     * @return false
     */
    bool isResponseTo( ControlParameterRequest& request ) {
        return requestNumber == request.requestNumber && requestKind == request.requestKind && std::string( name ) == std::string( request.name );
    }

    /**
     * @brief Convert to human-readable string
     *
     * @return std::string description of the request
     */
    std::string ToString() {
        std::string result = "Response(" + std::to_string( requestNumber ) + ") " + ControlParameterRequestKindToString( requestKind ) + " " + ControlParameterValueKindToString( parameterKind ) + " "
                             + std::string( name ) + " ";

        switch ( requestKind ) {
        case ControlParameterRequestKind::kSET_USER_PARAM_BY_NAME:
            result += "user to: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kGET_USER_PARAM_BY_NAME:
            result += "user is: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kSET_ROBOT_PARAM_BY_NAME:
            result += "robot to: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kGET_ROBOT_PARAM_BY_NAME:
            result += "robot is: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kGET_SPEED_CALIBRATE_PARAM_BY_NAME:
            result += "speed_calibrate is: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kSET_SPEED_CALIBRATE_PARAM_BY_NAME:
            result += "speed_calibrate to: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kGET_IMU_CALIBRATE_PARAM_BY_NAME:
            result += "imu_calibrate is: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        case ControlParameterRequestKind::kSET_IMU_CALIBRATE_PARAM_BY_NAME:
            result += "imu_calibrate to: ";
            result += ControlParameterValueToString( value, parameterKind );
            return result;
        default:
            return result + " unknown request type!";
        }
    }
};

#endif  // CONTROL_PARAMETER_INTERFACE_HPP_
