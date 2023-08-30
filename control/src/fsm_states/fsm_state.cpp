#include "fsm_states/fsm_state.hpp"

/**
 * @brief Constructor for the FSM State class.
 *
 * @param control_fsm_data holds all of the relevant control data
 * @param state_name_input the enumerated state name
 * @param state_string_input the string name of the current FSM state
 */
template < typename T >
FsmState< T >::FsmState( ControlFsmData< T >* control_fsm_data, FsmStateName state_name_input, std::string state_string_input )
    : data_( control_fsm_data ), state_name_( state_name_input ), state_string_( state_string_input ) {
    transition_data_.ResetTransitionDone();
    std::cout << "[FsmState] Initialized FSM state: " << state_string_input << std::endl;
}

/**
 * @brief Cartesian impedance control for a given leg.
 *
 * @param leg the leg number to control
 * @param q_des desired joint position
 * @param qd_des desired joint velocity
 */
template < typename T > void FsmState< T >::JointPdControl( int leg, Vec3< T > q_des, Vec3< T > qd_des ) {

    // kp_mat_ << 80, 0, 0, 0, 80, 0, 0, 0, 80;
    // kd_mat_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    kp_mat_ << 100, 0, 0, 0, 100, 0, 0, 0, 120;
    kd_mat_ << 2, 0, 0, 0, 2, 0, 0, 0, 2;

    data_->leg_controller->commands_[ leg ].kp_joint = kp_mat_;
    data_->leg_controller->commands_[ leg ].kd_joint = kd_mat_;

    data_->leg_controller->commands_[ leg ].q_des  = q_des;
    data_->leg_controller->commands_[ leg ].qd_des = qd_des;
}

/**
 * @brief Joint pure damper control for a given leg.
 *
 * @param leg the leg number to control
 * @param qd_des desired joint velocity
 */
template < typename T > void FsmState< T >::JointDamperControl( int leg, Vec3< T > qd_des ) {
    kd_mat_ << 3.5, 0, 0, 0, 3.5, 0, 0, 0, 3.5;
    data_->leg_controller->commands_[ leg ].qd_des   = qd_des;
    data_->leg_controller->commands_[ leg ].kd_joint = kd_mat_;
}

/**
 * @brief Joint safety control for a given leg.
 *
 * @param leg the leg number to control
 * @param qd_des desired joint velocity
 */
template < typename T > void FsmState< T >::JointSafetyControl( int leg, Vec3< T > qd_des ) {
    kp_mat_ << 0, 0, 0, 0, 0, 0, 0, 0, 10;
    kd_mat_ << 3.5, 0, 0, 0, 3.5, 0, 0, 0, 3.5;

    data_->leg_controller->commands_[ leg ].kp_joint = kp_mat_;
    data_->leg_controller->commands_[ leg ].kd_joint = kd_mat_;

    data_->leg_controller->commands_[ leg ].q_des  = Vec3< T >( 0, -1.4, 2.4 );
    data_->leg_controller->commands_[ leg ].qd_des = qd_des;
}

/**
 * @brief Cartesian impedance control for a given leg.
 *
 * @param leg the leg number to control
 * @param p_des desired foot position
 * @param v_des desired foot velocity
 * @param kp_cartesian P gains
 * @param kd_cartesian D gains
 */
template < typename T > void FsmState< T >::CartesianImpedanceControl( int leg, Vec3< T > p_des, Vec3< T > v_des, Vec3< double > kp_cartesian, Vec3< double > kd_cartesian ) {
    data_->leg_controller->commands_[ leg ].p_des = p_des;
    // Create the cartesian P gain matrix
    kp_mat_ << kp_cartesian[ 0 ], 0, 0, 0, kp_cartesian[ 1 ], 0, 0, 0, kp_cartesian[ 2 ];
    data_->leg_controller->commands_[ leg ].kp_cartesian = kp_mat_;

    data_->leg_controller->commands_[ leg ].v_des = v_des;
    // Create the cartesian D gain matrix
    kd_mat_ << kd_cartesian[ 0 ], 0, 0, 0, kd_cartesian[ 1 ], 0, 0, 0, kd_cartesian[ 2 ];
    data_->leg_controller->commands_[ leg ].kd_cartesian = kd_mat_;
}

/**
 * @brief turn on all safety checks
 *
 * @tparam T
 */
template < typename T > void FsmState< T >::TurnOnAllSafetyChecks() {
    // Pre controls safety checks
    check_safe_orientation_ = true;  // check roll and pitch
    check_robot_lifted_     = true;  // check robot lifted

    // Post control safety checks
    check_desired_foot_position_ = true;  // do not command footsetps too far
    check_feed_forward_force_    = true;  // do not command huge forces
    check_leg_singularity_       = true;  // do not let leg
}

/**
 * @brief turn off all safety checks
 *
 * @tparam T
 */
template < typename T > void FsmState< T >::TurnOffAllSafetyChecks() {
    // Pre controls safety checks
    check_safe_orientation_ = false;  // check roll and pitch
    check_robot_lifted_     = false;  // check robot lifted

    // Post control safety checks
    check_desired_foot_position_ = false;  // do not command footsetps too far
    check_feed_forward_force_    = false;  // do not command huge forces
    check_leg_singularity_       = false;  // do not let leg
}

// template class FsmState<double>;
template class FsmState< float >;
