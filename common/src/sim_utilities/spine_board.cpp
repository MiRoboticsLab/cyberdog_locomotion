#include <stdio.h>

#include "sim_utilities/spine_board.hpp"

/**
 *  @brief Spine board setup (per board)
 **/
void SpineBoard::Init( float side_sign, s32 board ) {
    this->board_num_ = board;
    this->side_sign_ = side_sign;
}

/**
 * @brief Reset all data_ for the board
 **/
void SpineBoard::ResetData() {
    if ( data_ == nullptr ) {
        printf( "[ERROR: SPINE board] reset_spine_board_data_ called when "
                "cyberdog2_lcm_spi_data_t* was null\n" );
        return;
    }

    data_->flags[ board_num_ ]   = 0;
    data_->qd_abad[ board_num_ ] = 0.f;
    data_->qd_hip[ board_num_ ]  = 0.f;
    data_->qd_knee[ board_num_ ] = 0.f;
    data_->q_abad[ board_num_ ]  = 0.f;
    data_->q_hip[ board_num_ ]   = 0.f;
    data_->q_knee[ board_num_ ]  = 0.f;
    data_->spi_driver_status     = 0;
}

/**
 *  @brief Reset all commands for the board
 **/
void SpineBoard::ResetCommand() {
    if ( cmd_ == nullptr ) {
        printf( "[ERROR: SPINE board] reset_spine_board_command called when "
                "cyberdog2_lcm_spi_command_t* was null\n" );
        return;
    }

    cmd_->flags[ board_num_ ]       = 0;
    cmd_->kd_abad[ board_num_ ]     = 0.f;
    cmd_->kd_hip[ board_num_ ]      = 0.f;
    cmd_->kd_knee[ board_num_ ]     = 0.f;
    cmd_->kp_abad[ board_num_ ]     = 0.f;
    cmd_->kp_hip[ board_num_ ]      = 0.f;
    cmd_->kp_knee[ board_num_ ]     = 0.f;
    cmd_->qd_des_abad[ board_num_ ] = 0.f;
    cmd_->qd_des_hip[ board_num_ ]  = 0.f;
    cmd_->qd_des_knee[ board_num_ ] = 0.f;
    cmd_->q_des_abad[ board_num_ ]  = 0.f;
    cmd_->q_des_hip[ board_num_ ]   = 0.f;
    cmd_->q_des_knee[ board_num_ ]  = 0.f;
    cmd_->tau_abad_ff[ board_num_ ] = 0.f;
    cmd_->tau_hip_ff[ board_num_ ]  = 0.f;
    cmd_->tau_hip_ff[ board_num_ ]  = 0.f;
    cmd_->tau_knee_ff[ board_num_ ] = 0.f;
}

/**
 * @brief Run spine board control
 **/
void SpineBoard::Run() {
    iter_counter_++;
    if ( cmd_ == nullptr || data_ == nullptr ) {
        printf( "[ERROR: SPINE board] run_spine_board_iteration called with null "
                "command or data_!\n" );
        torque_out_[ 0 ] = 0.f;
        torque_out_[ 1 ] = 0.f;
        torque_out_[ 2 ] = 0.f;
        return;
    }

    /// Check abad softstop ///
    if ( data_->q_abad[ board_num_ ] > q_limit_p_[ 0 ] ) {
        torque_out_[ 0 ] = kp_softstop_ * ( q_limit_p_[ 0 ] - data_->q_abad[ board_num_ ] ) - kd_softstop_ * ( data_->qd_abad[ board_num_ ] ) + cmd_->tau_abad_ff[ board_num_ ];
    }
    else if ( data_->q_abad[ board_num_ ] < q_limit_n_[ 0 ] ) {
        torque_out_[ 0 ] = kp_softstop_ * ( q_limit_n_[ 0 ] - data_->q_abad[ board_num_ ] ) - kd_softstop_ * ( data_->qd_abad[ board_num_ ] ) + cmd_->tau_abad_ff[ board_num_ ];
    }
    else {
        torque_out_[ 0 ] = cmd_->kp_abad[ board_num_ ] * ( cmd_->q_des_abad[ board_num_ ] - data_->q_abad[ board_num_ ] )
                           + cmd_->kd_abad[ board_num_ ] * ( cmd_->qd_des_abad[ board_num_ ] - data_->qd_abad[ board_num_ ] ) + cmd_->tau_abad_ff[ board_num_ ];
    }

    /// Check hip softstop ///
    if ( data_->q_hip[ board_num_ ] > q_limit_p_[ 1 ] ) {
        torque_out_[ 1 ] = kp_softstop_ * ( q_limit_p_[ 1 ] - data_->q_hip[ board_num_ ] ) - kd_softstop_ * ( data_->qd_hip[ board_num_ ] ) + cmd_->tau_hip_ff[ board_num_ ];
    }
    else if ( data_->q_hip[ board_num_ ] < q_limit_n_[ 1 ] ) {
        torque_out_[ 1 ] = kp_softstop_ * ( q_limit_n_[ 1 ] - data_->q_hip[ board_num_ ] ) - kd_softstop_ * ( data_->qd_hip[ board_num_ ] ) + cmd_->tau_hip_ff[ board_num_ ];
    }
    else {
        torque_out_[ 1 ] = cmd_->kp_hip[ board_num_ ] * ( cmd_->q_des_hip[ board_num_ ] - data_->q_hip[ board_num_ ] )
                           + cmd_->kd_hip[ board_num_ ] * ( cmd_->qd_des_hip[ board_num_ ] - data_->qd_hip[ board_num_ ] ) + cmd_->tau_hip_ff[ board_num_ ];
    }

    /// No knee softstop right now ///
    torque_out_[ 2 ] = cmd_->kp_knee[ board_num_ ] * ( cmd_->q_des_knee[ board_num_ ] - data_->q_knee[ board_num_ ] )
                       + cmd_->kd_knee[ board_num_ ] * ( cmd_->qd_des_knee[ board_num_ ] - data_->qd_knee[ board_num_ ] ) + cmd_->tau_knee_ff[ board_num_ ];

    const float* torque_limits = disabled_torque_;

    if ( cmd_->flags[ board_num_ ] & 0b1 ) {
        if ( cmd_->flags[ board_num_ ] & 0b10 )
            torque_limits = wimp_torque_;
        else
            torque_limits = max_torque_;
    }

    for ( int i = 0; i < 3; i++ ) {
        if ( torque_out_[ i ] > torque_limits[ i ] )
            torque_out_[ i ] = torque_limits[ i ];
        if ( torque_out_[ i ] < -torque_limits[ i ] )
            torque_out_[ i ] = -torque_limits[ i ];
    }
}