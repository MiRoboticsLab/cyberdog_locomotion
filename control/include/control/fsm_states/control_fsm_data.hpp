#ifndef CONTROL_FSM_DATA_HPP_
#define CONTROL_FSM_DATA_HPP_

#include "controllers/leg_controller.hpp"
#include "controllers/state_estimator_container.hpp"
#include "dynamics/quadruped.hpp"
#include "command_interface/command_interface.hpp"
#include "robot_current_state.hpp"
#include "user_parameters.hpp"
#include "parameters/robot_parameters.hpp"

/**
 * @brief Contains all contol data
 *
 */
template < typename T > struct ControlFsmData {
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Quadruped< T >*               quadruped;
    StateEstimatorContainer< T >* state_estimator;
    LegController< T >*           leg_controller;
    const MotionControlCommand*   command = nullptr;
    RobotControlParameters*       control_parameters;
    UserParameters*               user_parameters;
    VisualizationData*            visualization_data;
    RobotCurrentState< T >*       robot_current_state;
};

template struct ControlFsmData< double >;
template struct ControlFsmData< float >;

#endif  // CONTROL_FSM_DATA_HPP_
