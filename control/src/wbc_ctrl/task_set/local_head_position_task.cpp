#include "wbc_ctrl/task_set/local_head_position_task.hpp"
#include "dynamics/floating_base_model.hpp"
#include "dynamics/quadruped.hpp"
#include "math/orientation_tools.hpp"
#include "utilities/utilities_print.hpp"

/**
 * @brief Construct a new Local Head Pos Task<  T >:: Local Head Pos Task object
 *
 * @param robot floating dynamics model
 */
template < typename T > LocalHeadPosTask< T >::LocalHeadPosTask( const FloatingBaseModel< T >* robot ) : Task< T >( 3 ), robot_dynamics_( robot ) {
    TK::Jt_        = DMat< T >::Zero( TK::dim_task_, cyberdog2::kDimConfig );
    TK::JtDotQdot_ = DVec< T >::Zero( TK::dim_task_ );

    kp = DVec< T >::Constant( TK::dim_task_, 50. );
    kd = DVec< T >::Constant( TK::dim_task_, 3. );
}

/**
 * @brief Destroy the Local Head Pos Task<  T >:: Local Head Pos Task object
 *
 */
template < typename T > LocalHeadPosTask< T >::~LocalHeadPosTask() {}

/**
 * @brief Get desired accelation by offset between desired and real state
 *
 * @param pos_des desired position
 * @param vel_des desired velocity
 * @param acc_des desired accelation
 * @return true
 * @return false
 */
template < typename T > bool LocalHeadPosTask< T >::UpdateCommand( const void* pos_des, const DVec< T >& vel_des, const DVec< T >& acc_des ) {
    Vec3< T >* pos_cmd = ( Vec3< T >* )pos_des;
    Vec3< T >  link_pos, link_vel, local_pos, local_vel;
    link_pos = 0.5 * robot_dynamics_->pGC_[ linkID::kFrAbd ] + 0.5 * robot_dynamics_->pGC_[ linkID::kFlAbd ];
    link_vel = 0.5 * robot_dynamics_->vGC_[ linkID::kFrAbd ] + 0.5 * robot_dynamics_->vGC_[ linkID::kFlAbd ];

    local_pos = 0.5 * robot_dynamics_->pGC_[ linkID::kFr ] + 0.5 * robot_dynamics_->pGC_[ linkID::kFl ];
    local_vel = 0.5 * robot_dynamics_->vGC_[ linkID::kFr ] + 0.5 * robot_dynamics_->vGC_[ linkID::kFl ];

    // X, Y, Z
    for ( size_t i( 0 ); i < TK::dim_task_; ++i ) {
        TK::pos_err_[ i ] = ( ( *pos_cmd )[ i ] - ( link_pos[ i ] - local_pos[ i ] ) );
        TK::vel_des_[ i ] = vel_des[ i ];
        TK::acc_des_[ i ] = acc_des[ i ];

        TK::op_cmd_[ i ] = kp[ i ] * ( ( *pos_cmd )[ i ] - ( link_pos[ i ] - local_pos[ i ] ) ) + kd[ i ] * ( link_vel[ i ] - local_vel[ i ] ) + TK::acc_des_[ i ];
    }
    // printf("[Body Ori Pitch Yaw Task]\n");
    // PrettyPrint( TK::pos_err_, std::cout, "pos_err_" );
    // PrettyPrint( *ori_cmd, std::cout, "des_ori" );
    // PrettyPrint( link_ori, std::cout, "curr_ori" );
    // PrettyPrint( ori_err, std::cout, "quat_err" );

    // PrettyPrint( link_ori_inv, std::cout, "ori_inv" );
    // PrettyPrint( ori_err, std::cout, "ori_err" );
    // PrettyPrint( *ori_cmd, std::cout, "cmd" );
    // PrettyPrint( acc_des, std::cout, "acc_des" );
    // PrettyPrint( TK::Jt_, std::cout, "Jt" );

    return true;
}

/**
 * @brief Get task jacobian matrix, is Jt_
 *
 * @return true
 * @return false
 */
template < typename T > bool LocalHeadPosTask< T >::UpdateTaskJacobian() {
    TK::Jt_ = 0.5 * robot_dynamics_->Jc_[ linkID::kFrAbd ] + 0.5 * robot_dynamics_->Jc_[ linkID::kFlAbd ] - 0.5 * robot_dynamics_->Jc_[ linkID::kFr ] - 0.5 * robot_dynamics_->Jc_[ linkID::kFl ];

    return true;
}

/**
 * @brief Get task jacobian*qdot, is JtDotQdot_
 *
 * @return true
 * @return false
 */
template < typename T > bool LocalHeadPosTask< T >::UpdateTaskJDotQdot() {
    return true;
}

template class LocalHeadPosTask< double >;
template class LocalHeadPosTask< float >;
