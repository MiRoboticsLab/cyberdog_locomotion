#include <iostream>

#include "balance_controller/balance_controller_lsm.hpp"

void BalanceControllerLSM::SolverLSM( ControlFsmData< float >* data, Vec3< float > aDes, Vec3< float > wdDes, Vec4< float > contactState ) {
    int foot_num = 0;
    for ( int i( 0 ); i < 4; i++ ) {
        if ( contactState[ i ] > 0 )
            foot_num++;
    }
    DMat< float > cof_a  = DMat< float >::Zero( 6, foot_num * 3 );
    DMat< float > cof_b  = DMat< float >::Zero( 6, 1 );
    DMat< float > cof_h  = DMat< float >::Zero( foot_num * 3, foot_num * 3 );
    DMat< float > cof_g  = DMat< float >::Zero( foot_num * 3, 1 );
    DMat< float > cof_ax = DMat< float >::Zero( foot_num * 6 + 2, foot_num * 3 );
    DMat< float > cof_ux = DMat< float >::Zero( foot_num * 6 + 2, 1 );
    DMat< float > cof_w  = DMat< float >::Zero( foot_num * 3, foot_num * 3 );
    DMat< float > cof_q  = DMat< float >::Zero( foot_num * 3, foot_num * 3 );
    DMat< float > f_p    = DMat< float >::Zero( foot_num * 3, 1 );

    Vec3< float > qp_result[ 4 ];
    for ( int i = 0; i < 4; i++ )
        qp_result[ i ].setZero();

    cof_b << data->user_parameters->mpc_body_mass * data->state_estimator->GetResult().world2body_rotation_matrix.transpose() * ( aDes + g_acc_ ),
        data->state_estimator->GetResult().world2body_rotation_matrix.transpose() * wb_inertia_ * wdDes;

    for ( int i = 0; i < foot_num; i++ ) {
        cof_a.block( 0, i * 3, 3, 3 ) << i_3_;
    }
    int           j = 0;
    Vec3< float > p_com_i;
    Vec3< float > pFoot[ 4 ];

    for ( int i = 0; i < 4; i++ )
        pFoot[ i ] = ( data->quadruped->GetHipLocation( i ) + data->leg_controller->datas_[ i ].p );

    for ( int i = 0; i < 4; i++ ) {
        p_com_i.setZero();
        if ( contactState[ i ] > 0 ) {
            p_com_i = data->state_estimator->GetResult().world2body_rotation_matrix.transpose() * ( pFoot[ i ] - wb_com_ ).eval();
            cof_a.block( 3, j * 3, 3, 3 ) << 0, -p_com_i( 2 ), p_com_i( 1 ), p_com_i( 2 ), 0, -p_com_i( 0 ), -p_com_i( 1 ), p_com_i( 0 ), 0;
            j++;
        }
    }
    j = 0;
    for ( int i = 0; i < 4; i++ ) {
        if ( contactState[ i ] > 0 ) {
            cof_w( j * 3, j * 3 )         = cof_w_dia_( i * 3 );
            cof_w( j * 3 + 1, j * 3 + 1 ) = cof_w_dia_( i * 3 + 1 );
            cof_w( j * 3 + 2, j * 3 + 2 ) = cof_w_dia_( i * 3 + 2 );

            cof_q( j * 3, j * 3 )         = cof_q_dia_( i * 3 );
            cof_q( j * 3 + 1, j * 3 + 1 ) = cof_q_dia_( i * 3 + 1 );
            cof_q( j * 3 + 2, j * 3 + 2 ) = cof_q_dia_( i * 3 + 2 );
            j++;
        }
    }
    j = 0;
    for ( int i = 0; i < 4; i++ ) {
        if ( contactState[ i ] > 0 ) {
            f_p( j * 3 )     = f_prev_( i * 3 );
            f_p( j * 3 + 1 ) = f_prev_( i * 3 + 1 );
            f_p( j * 3 + 2 ) = f_prev_( i * 3 + 2 );
            j++;
        }
    }

    cof_ax.block( 0, 0, foot_num * 6, foot_num * 3 ) << cof_ax_all_.block( 0, 0, foot_num * 6, foot_num * 3 );
    for ( int i = 0; i < foot_num; i++ ) {
        cof_ax( foot_num * 6, i * 3 + 2 )     = 1.0;
        cof_ax( foot_num * 6 + 1, i * 3 + 2 ) = -1.0;
    }

    cof_ux.block( 0, 0, foot_num * 6, 1 ) << cof_ux_all_.block( 0, 0, foot_num * 6, 1 );
    cof_ux( foot_num * 6, 0 )     = max_z_force_ * 4;
    cof_ux( foot_num * 6 + 1, 0 ) = 0;

    cof_h = ( 2 * ( cof_a.transpose() * cof_s_ * cof_a + cof_alpha_ * cof_w + cof_beta_ * cof_q ) ).eval();
    cof_h = ( ( cof_h.transpose() + cof_h ) / 2 ).eval();
    cof_g = ( ( -2.0 ) * ( cof_a.transpose() * cof_s_ * cof_b + cof_beta_ * cof_q * f_p ) ).eval();

    USING_NAMESPACE_QPOASES
    real_t* qpCOF_H  = new real_t[ foot_num * 3 * foot_num * 3 ];
    real_t* qpCOF_g  = new real_t[ foot_num * 3 ];
    real_t* qpCOF_AX = new real_t[ ( foot_num * 6 + 2 ) * foot_num * 3 ];
    real_t* qpCOF_UX = new real_t[ foot_num * 6 + 2 ];

    int nWSR = 100;

    for ( int i = 0; i < foot_num * 3; i++ ) {
        for ( int j = 0; j < foot_num * 3; j++ ) {
            *( qpCOF_H + i * foot_num * 3 + j ) = cof_h( i, j );
        }
    }

    for ( int i = 0; i < foot_num * 3; i++ ) {
        *( qpCOF_g + i ) = cof_g( i );
    }

    for ( int i = 0; i < ( foot_num * 6 + 2 ); i++ ) {
        for ( int j = 0; j < foot_num * 3; j++ ) {
            *( qpCOF_AX + i * ( foot_num * 3 ) + j ) = cof_ax( i, j );
        }
    }

    for ( int i = 0; i < foot_num * 6 + 2; i++ ) {
        *( qpCOF_UX + i ) = cof_ux( i );
    }

    QProblem qp_solver( foot_num * 3, foot_num * 6 + 2 );
    qp_solver.setPrintLevel( PL_NONE );
    qp_solver.init( qpCOF_H, qpCOF_g, qpCOF_AX, nullptr, nullptr, nullptr, qpCOF_UX, nWSR );

    real_t* qpResult = new real_t[ foot_num * 3 ];
    qp_solver.getPrimalSolution( qpResult );

    int k = 0;
    for ( int i = 0; i < 4; i++ ) {
        if ( contactState[ i ] > 0 ) {
            for ( int q = 0; q < 3; q++ ) {
                qp_result[ i ]( q ) = *( qpResult + k * 3 + q );
            }
            k++;
        }
    }

    for ( int i = 0; i < 4; i++ ) {
        data->leg_controller->commands_[ i ].force_feed_forward = -( data->state_estimator->GetResult().world2body_rotation_matrix * qp_result[ i ] );
        // std::cout<<i<<std::endl<<qp_result[i]<<std::endl;
    }

    for ( int i = 0; i < 4; i++ )
        for ( int q = 0; q < 3; q++ )
            f_prev_( i * 3 + q ) = qp_result[ i ]( q );
}
