#ifndef BALANCE_CONTROLLER_LSM_HPP_
#define BALANCE_CONTROLLER_LSM_HPP_

#ifndef EIGEN_NO_DEBUG
#define EIGEN_NO_DEBUG
#endif

#include <lcm/lcm-cpp.hpp>
#include <qpOASES.hpp>

#include "cpp_types.hpp"
#include "fsm_states/control_fsm_data.hpp"
#include "header/lcm_type/qp_controller_data_t.hpp"
#include "header/lcm_type/sim_command_t.hpp"

class BalanceControllerLSM {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BalanceControllerLSM( ControlFsmData< float >* data ) {
        cof_alpha_     = 0.01;
        cof_beta_      = 0;
        max_z_force_   = 500;
        foot_friction_ = 0.8;

        g_acc_ << 0, 0, 9.8;
        wb_com_ << 0, 0, 0;

        cof_s_ << 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            10.0;

        cof_w_dia_ << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        cof_q_dia_ << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        i_3_ << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0;

        cof_ax_all_ << 1.0, 0, -foot_friction_, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1.0, 0, -foot_friction_, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, -foot_friction_, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1.0,
            -foot_friction_, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, -foot_friction_, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1.0,
            0, -foot_friction_, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, -foot_friction_, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1.0, -foot_friction_, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, -1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, -foot_friction_, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1.0, 0, -foot_friction_, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, -foot_friction_, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, -1.0, -foot_friction_, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, -foot_friction_, 0,
            0, 0, 0, 0, 0, 0, 0, 0, -1.0, 0, -foot_friction_, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, -foot_friction_, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1.0, -foot_friction_, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1.0;

        cof_ux_all_ << 0.0, 0.0, 0.0, 0.0, max_z_force_, 0.0, 0.0, 0.0, 0.0, 0.0, max_z_force_, 0.0, 0.0, 0.0, 0.0, 0.0, max_z_force_, 0.0, 0.0, 0.0, 0.0, 0.0, max_z_force_, 0.0;
        f_prev_ << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;

        wb_inertia_ << data->user_parameters->mpc_body_inertia[ 0 ], 0, 0, 0, data->user_parameters->mpc_body_inertia[ 1 ], 0, 0, 0, data->user_parameters->mpc_body_inertia[ 2 ];
    }
    ~BalanceControllerLSM();

    void SolverLSM( ControlFsmData< float >* data, Vec3< float > aDes, Vec3< float > wdDes, Vec4< float > contactState );

    Mat6< float >    cof_s_;
    Vec12< float >   cof_w_dia_;
    Vec12< float >   cof_q_dia_;
    float            cof_alpha_;
    float            cof_beta_;
    Mat3< float >    i_3_;
    Mat2412< float > cof_ax_all_;
    Vec24< float >   cof_ux_all_;
    double           max_z_force_;
    Vec12< float >   f_prev_;
    Mat3< float >    wb_inertia_;
    Vec3< float >    wb_com_;
    Vec3< float >    g_acc_;
    float            foot_friction_;
};

#endif  // BALANCE_CONTROLLER_LSM_HPP_
