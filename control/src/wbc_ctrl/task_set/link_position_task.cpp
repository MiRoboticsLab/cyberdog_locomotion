#include "wbc_ctrl/task_set/link_position_task.hpp"
#include "dynamics/floating_base_model.hpp"
#include "dynamics/quadruped.hpp"
#include "math/orientation_tools.hpp"
#include "utilities/utilities_print.hpp"

/**
 * @brief Construct a new Link Pos Task<  T >:: Link Pos Task object
 *
 * @param robot floating dynamics model
 * @param link_index link index of the model
 * @param virtual_depend represents the link does not link to the base
 */
template < typename T >
LinkPosTask< T >::LinkPosTask( const FloatingBaseModel< T >* robot, int link_index, bool virtual_depend )
    : Task< T >( 3 ), robot_sys_( robot ), link_index_( link_index ), virtual_depend_( virtual_depend ) {
    TK::Jt_        = DMat< T >::Zero( TK::dim_task_, cyberdog2::kDimConfig );
    TK::JtDotQdot_ = DVec< T >::Zero( TK::dim_task_ );

    kp = DVec< T >::Constant( TK::dim_task_, 100. );
    kd = DVec< T >::Constant( TK::dim_task_, 5. );
}

/**
 * @brief Destroy the Link Pos Task<  T >:: Link Pos Task object
 *
 */
template < typename T > LinkPosTask< T >::~LinkPosTask() {}

/**
 * @brief Get desired accelation by offset between desired and real state
 *
 * @param pos_des desired position of the link on global frame
 * @param vel_des desired velocity on global frame
 * @param acc_des desired accelation on global frame
 * @return true
 * @return false
 */
template < typename T > bool LinkPosTask< T >::UpdateCommand( const void* pos_des, const DVec< T >& vel_des, const DVec< T >& acc_des ) {
    Vec3< T >* pos_cmd = ( Vec3< T >* )pos_des;
    Vec3< T >  link_pos;

    link_pos = robot_sys_->pGC_[ link_index_ ];

    // X, Y, Z
    for ( int i( 0 ); i < 3; ++i ) {
        TK::pos_err_[ i ] = ( ( *pos_cmd )[ i ] - link_pos[ i ] );
        TK::vel_des_[ i ] = vel_des[ i ];
        TK::acc_des_[ i ] = acc_des[ i ];
    }

    // Op acceleration command
    for ( size_t i( 0 ); i < TK::dim_task_; ++i ) {
        TK::op_cmd_[ i ] = kp[ i ] * TK::pos_err_[ i ] + kd[ i ] * ( TK::vel_des_[ i ] - robot_sys_->vGC_[ link_index_ ][ i ] ) + TK::acc_des_[ i ];
    }

    // printf("[Link Pos Task]\n");
    // PrettyPrint( acc_des, std::cout, "acc_des" );
    // PrettyPrint( TK::pos_err_, std::cout, "pos_err_" );
    // PrettyPrint( *pos_cmd, std::cout, "pos cmd" );
    // PrettyPrint( robot_sys_->vGC_[ link_index_ ], std::cout, "velocity" );
    // PrettyPrint( TK::op_cmd_, std::cout, "op cmd" );
    // TK::op_cmd_.setZero();
    // PrettyPrint( TK::Jt_, std::cout, "Jt" );

    return true;
}

/**
 * @brief Get task jacobian matrix, is Jt_
 *
 * @return true
 * @return false
 */
template < typename T > bool LinkPosTask< T >::UpdateTaskJacobian() {
    TK::Jt_ = robot_sys_->Jc_[ link_index_ ];
    if ( !virtual_depend_ ) {
        TK::Jt_.block( 0, 0, 3, 6 ) = DMat< T >::Zero( 3, 6 );
    }
    return true;
}

/**
 * @brief Get task jacobian*qdot, is JtDotQdot_
 *
 * @return true
 * @return false
 */
template < typename T > bool LinkPosTask< T >::UpdateTaskJDotQdot() {
    TK::JtDotQdot_ = robot_sys_->Jcdqd_[ link_index_ ];
    return true;
}

template class LinkPosTask< double >;
template class LinkPosTask< float >;
