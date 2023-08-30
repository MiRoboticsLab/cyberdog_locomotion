#ifndef SPINE_BOARD_HPP_
#define SPINE_BOARD_HPP_

#include <cstring>

#include "c_types.h"

/**
 * @brief Command to spine board
 *
 */
struct SpiCommand {
    float q_des_abad[ 4 ];
    float q_des_hip[ 4 ];
    float q_des_knee[ 4 ];

    float qd_des_abad[ 4 ];
    float qd_des_hip[ 4 ];
    float qd_des_knee[ 4 ];

    float kp_abad[ 4 ];
    float kp_hip[ 4 ];
    float kp_knee[ 4 ];

    float kd_abad[ 4 ];
    float kd_hip[ 4 ];
    float kd_knee[ 4 ];

    float tau_abad_ff[ 4 ];
    float tau_hip_ff[ 4 ];
    float tau_knee_ff[ 4 ];

    int32_t flags[ 4 ];

    explicit SpiCommand() {
        memset( this, 0, sizeof( SpiCommand ) );
    }
};

/**
 * @brief Data from spine board
 *
 */
struct SpiData {
    explicit SpiData() {
        memset( this, 0, sizeof( SpiData ) );
    }

    float   q_abad[ 4 ];
    float   q_hip[ 4 ];
    float   q_knee[ 4 ];
    float   qd_abad[ 4 ];
    float   qd_hip[ 4 ];
    float   qd_knee[ 4 ];
    int32_t flags[ 12 ];
    int32_t spi_driver_status;

    float tau_abad[ 4 ];
    float tau_hip[ 4 ];
    float tau_knee[ 4 ];

    int16_t tmp_abad[ 4 ];
    int16_t tmp_hip[ 4 ];
    int16_t tmp_knee[ 4 ];

    uint8_t reserve[ 40 ];
};

/**
 * @brief Spine board control logic, used to simulate the SpineBoard.
 *
 */
class SpineBoard {
public:
    SpineBoard() {}
    void        Init( float side_sign, s32 board );
    void        Run();
    void        ResetData();
    void        ResetCommand();
    SpiCommand* cmd_  = nullptr;
    SpiData*    data_ = nullptr;
    float       torque_out_[ 3 ];

private:
    float       side_sign_;
    s32         board_num_;
    const float max_torque_[ 3 ]      = { 17.f, 24.f, 26.f };  // TODO CHECK WITH BEN
    const float wimp_torque_[ 3 ]     = { 6.f, 6.f, 6.f };     // TODO CHECK WITH BEN
    const float disabled_torque_[ 3 ] = { 0.f, 0.f, 0.f };
    const float q_limit_p_[ 3 ]       = { 1.5f, 5.0f, 0.f };
    const float q_limit_n_[ 3 ]       = { -1.5f, -5.0f, 0.f };
    const float kp_softstop_          = 100.f;
    const float kd_softstop_          = 0.4f;
    s32         iter_counter_         = 0;
};

#endif  // SPINE_BOARD_HPP_