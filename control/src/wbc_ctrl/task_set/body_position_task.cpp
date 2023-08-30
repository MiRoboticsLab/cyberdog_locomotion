#include "wbc_ctrl/task_set/body_position_task.hpp"
#include "dynamics/floating_base_model.hpp"
#include "dynamics/quadruped.hpp"
#include "math/orientation_tools.hpp"
#include "utilities/utilities_print.hpp"

/**
 * @brief Construct a new Body Pos Task<  T >:: Body Pos Task object
 *
 * @param robot floating dynamics model
 */
template < typename T > BodyPosTask< T >::BodyPosTask( const FloatingBaseModel< T >* robot ) : Task< T >( 3 ), robot_dynamics_( robot ) {
    TK::Jt_ = DMat< T >::Zero( TK::dim_task_, cyberdog2::kDimConfig );
    TK::Jt_.block( 0, 3, 3, 3 ).setIdentity();
    TK::JtDotQdot_ = DVec< T >::Zero( TK::dim_task_ );

    kp = DVec< T >::Constant( TK::dim_task_, 100. );  // 50.);//
    kd = DVec< T >::Constant( TK::dim_task_, 10.0 );  // 1.0);//
}

/**
 * @brief Destroy the Body Pos Task<  T >:: Body Pos Task object
 *
 */
template < typename T > BodyPosTask< T >::~BodyPosTask() {}

/**
 * @brief Get desired accelation by offset between desired and real state
 *
 * @param pos_des desired body position represented on global frame
 * @param vel_des desired body velocity represented on global frame
 * @param acc_des desired body accelation represented on global frame
 * @return true
 * @return false
 */
template < typename T > bool BodyPosTask< T >::UpdateCommand( const void* pos_des, const DVec< T >& vel_des, const DVec< T >& acc_des ) {
    Vec3< T >* pos_cmd  = ( Vec3< T >* )pos_des;
    Vec3< T >  link_pos = robot_dynamics_->state_.body_position;

    Quat< T > quat = robot_dynamics_->state_.body_orientation;
    Mat3< T > Rot  = ori::QuaternionToRotationMatrix( quat );

    SVec< T > curr_vel = robot_dynamics_->state_.body_velocity;
    curr_vel.tail( 3 ) = Rot.transpose() * curr_vel.tail( 3 );

    // X, Y, Z
    for ( int i( 0 ); i < 3; ++i ) {
        TK::pos_err_[ i ] = ( ( *pos_cmd )[ i ] - link_pos[ i ] );
        TK::vel_des_[ i ] = vel_des[ i ];
        TK::acc_des_[ i ] = acc_des[ i ];

        TK::op_cmd_[ i ] = kp[ i ] * ( ( *pos_cmd )[ i ] - link_pos[ i ] ) + kd[ i ] * ( TK::vel_des_[ i ] - curr_vel[ i + 3 ] ) + TK::acc_des_[ i ];
    }
    // Quat<T> quat = robot_dynamics_->state_.body_orientation;
    // Mat3<T> Rot = ori::QuaternionToRotationMatrix(quat);
    // TK::pos_err_ = Rot * TK::pos_err_;
    // TK::vel_des_ = Rot * TK::vel_des_;
    // TK::acc_des_ = Rot * TK::acc_des_;

    // printf("[Body Pos Task]\n");
    // PrettyPrint( acc_des, std::cout, "acc_des" );
    // PrettyPrint( TK::pos_err_, std::cout, "pos_err_" );
    // PrettyPrint( curr_vel, std::cout, "curr_vel" );
    // PrettyPrint( *pos_cmd, std::cout, "pos cmd" );
    // PrettyPrint( TK::op_cmd_, std::cout, "op cmd" );
    // PrettyPrint( TK::vel_des_, std::cout, "vel des" );
    // PrettyPrint( TK::Jt_, std::cout, "Jt" );

    return true;
}

/**
 * @brief Get task jacobian matrix, is Jt_
 *
 * @return true
 * @return false
 */
template < typename T > bool BodyPosTask< T >::UpdateTaskJacobian() {
    Quat< T > quat              = robot_dynamics_->state_.body_orientation;
    Mat3< T > Rot               = ori::QuaternionToRotationMatrix( quat );
    TK::Jt_.block( 0, 3, 3, 3 ) = Rot.transpose();
    // PrettyPrint( TK::Jt_, std::cout, "Jt" );
    return true;
}

/**
 * @brief Get task jacobian*qdot, is JtDotQdot_
 *
 * @return true
 * @return false
 */
template < typename T > bool BodyPosTask< T >::UpdateTaskJDotQdot() {
    return true;
}

template class BodyPosTask< double >;
template class BodyPosTask< float >;
