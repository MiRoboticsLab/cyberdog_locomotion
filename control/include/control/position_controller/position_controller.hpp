#ifndef POSITION_CONTROLLER_HPP_
#define POSITION_CONTROLLER_HPP_

#ifndef EIGEN_NO_DEBUG
#define EIGEN_NO_DEBUG
#endif

#include <lcm/lcm-cpp.hpp>

#include "cpp_types.hpp"
#include "fsm_states/control_fsm_data.hpp"

class PositionController {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PositionController() {
        for ( int leg( 0 ); leg < 4; leg++ ) {
            joint_pcur_[ leg ].setZero();
            foot_in_world_p_cur_[ leg ].setZero();
            foot_in_robot_p_cur_[ leg ].setZero();
            joint_p_cmd_[ leg ].setZero();
            hip_location_[ leg ].setZero();
        }
        body_in_world_p_cur_.setZero();
        body_in_world_rpy_cur_.setZero();
        rbody_in_world_.setZero();
        bodyMove = true;
    }
    ~PositionController() {}

    Vec3< float > IKS( Vec3< float > footP, int leg );
    Vec4< float > FKS( Vec3< float > joint_q, int leg );

    void InitState( ControlFsmData< float >* data, Vec3< float > ( &q )[ 4 ] ) {
        body_in_world_p_cur_.block( 0, 0, 3, 1 ) = data->state_estimator->GetResult().position;
        body_in_world_p_cur_( 3 )                = 1.0f;
        body_in_world_rpy_cur_                   = data->state_estimator->GetResult().rpy;
        rbody_in_world_                          = data->state_estimator->GetResult().world2body_rotation_matrix;
        hip_knee_offset_                         = data->quadruped->abad_link_length_;
        hip_length_                              = data->quadruped->hip_link_length_;
        knee_length_                             = data->quadruped->knee_link_length_;
        for ( int leg( 0 ); leg < 4; leg++ ) {
            joint_pcur_[ leg ]          = data->leg_controller->datas_[ leg ].q;
            foot_in_robot_p_cur_[ leg ] = FKS( q[ leg ], leg );

            foot_in_robot_p_cur_[ leg ]( 3 )         = 1.0f;
            hip_location_[ leg ].block( 0, 0, 3, 1 ) = data->quadruped->GetHipLocation( leg );
            hip_location_[ leg ]( 3 )                = 1.0f;
            foot_in_world_p_cur_[ leg ]( 3 )         = 1.0f;
            foot_in_world_p_cur_[ leg ].block( 0, 0, 3, 1 ) =
                rbody_in_world_.transpose() * ( foot_in_robot_p_cur_[ leg ].block( 0, 0, 3, 1 ) + hip_location_[ leg ].block( 0, 0, 3, 1 ) ) + body_in_world_p_cur_.block( 0, 0, 3, 1 );
            joint_p_cmd_[ leg ] = q[ leg ];
            world2body_trans_[ leg ].setZero();
            world2body_trans_[ leg ].block( 0, 0, 3, 3 ) = rbody_in_world_;
            world2body_trans_[ leg ]( 3, 3 )             = 1.0f;
        }
    }

    void          TransBody( Vec6< float > body_cmd, Vec3< float > foot_cmd, Vec4< float > ctrl_point, Vec4< float > contact_state );
    Mat4< float > RBodyMat( float alpha, float beta, float gamma, float x, float y, float z );

    Vec3< float > joint_pcur_[ 4 ];
    Vec4< float > body_in_world_p_cur_;
    Vec3< float > body_in_world_rpy_cur_;
    Vec4< float > foot_in_world_p_cur_[ 4 ];
    Vec4< float > foot_in_robot_p_cur_[ 4 ];
    Vec3< float > joint_p_cmd_[ 4 ];
    Mat3< float > rbody_in_world_;
    Vec4< float > hip_location_[ 4 ];
    Mat4< float > world2body_trans_[ 4 ];
    bool          bodyMove;

private:
    float hip_knee_offset_;
    float hip_length_;
    float knee_length_;
};

#endif  // POSITION_CONTROLLER_HPP_
