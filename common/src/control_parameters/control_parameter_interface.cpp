#include "control_parameters/control_parameter_interface.hpp"

/**
 * @brief Convert control parameter request kind into a string
 *
 * @param request
 * @return std::string
 */
std::string ControlParameterRequestKindToString( ControlParameterRequestKind request ) {
    switch ( request ) {
    case ControlParameterRequestKind::kGET_USER_PARAM_BY_NAME:
        return "get user";
    case ControlParameterRequestKind::kSET_USER_PARAM_BY_NAME:
        return "set user";
    case ControlParameterRequestKind::kGET_ROBOT_PARAM_BY_NAME:
        return "get robot";
    case ControlParameterRequestKind::kSET_ROBOT_PARAM_BY_NAME:
        return "set robot";
    case ControlParameterRequestKind::kGET_SPEED_CALIBRATE_PARAM_BY_NAME:
        return "get speed_calibrate";
    case ControlParameterRequestKind::kSET_SPEED_CALIBRATE_PARAM_BY_NAME:
        return "set speed_calibrate";
    case ControlParameterRequestKind::kGET_IMU_CALIBRATE_PARAM_BY_NAME:
        return "get imu_calibrate";
    case ControlParameterRequestKind::kSET_IMU_CALIBRATE_PARAM_BY_NAME:
        return "set imu_calibrate";
    default:
        return "unknown";
    }
}
