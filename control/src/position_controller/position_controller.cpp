#include "position_controller/position_controller.hpp"

Vec4< float > PositionController::FKS( Vec3< float > joint_q, int leg ) {
    Vec4< float > footPosition;

    float x, y, z;
    float abad_hip_offset = 0.f;
    float alpha1          = M_PI / 2;

    if ( ( leg == 1 ) || ( leg == 3 ) )
        joint_q( 0 ) = -joint_q( 0 );

    x = hip_length_ * ( cos( joint_q( 0 ) ) * cos( joint_q( 1 ) ) - cos( alpha1 ) * sin( joint_q( 0 ) ) * sin( joint_q( 1 ) ) )
        + knee_length_
              * ( cos( joint_q( 2 ) ) * ( cos( joint_q( 0 ) ) * cos( joint_q( 1 ) ) - cos( alpha1 ) * sin( joint_q( 0 ) ) * sin( joint_q( 1 ) ) )
                  - sin( joint_q( 2 ) ) * ( cos( joint_q( 0 ) ) * sin( joint_q( 1 ) ) + cos( alpha1 ) * cos( joint_q( 1 ) ) * sin( joint_q( 0 ) ) ) )
        + abad_hip_offset * sin( alpha1 ) * sin( joint_q( 0 ) ) + hip_knee_offset_ * sin( alpha1 ) * sin( joint_q( 0 ) );
    y = hip_length_ * ( cos( joint_q( 1 ) ) * sin( joint_q( 0 ) ) + cos( alpha1 ) * cos( joint_q( 0 ) ) * sin( joint_q( 1 ) ) )
        + knee_length_
              * ( cos( joint_q( 2 ) ) * ( cos( joint_q( 1 ) ) * sin( joint_q( 0 ) ) + cos( alpha1 ) * cos( joint_q( 0 ) ) * sin( joint_q( 1 ) ) )
                  - sin( joint_q( 2 ) ) * ( sin( joint_q( 0 ) ) * sin( joint_q( 1 ) ) - cos( alpha1 ) * cos( joint_q( 0 ) ) * cos( joint_q( 1 ) ) ) )
        - abad_hip_offset * sin( alpha1 ) * cos( joint_q( 0 ) ) - hip_knee_offset_ * sin( alpha1 ) * cos( joint_q( 0 ) );
    z = abad_hip_offset * cos( alpha1 ) + hip_knee_offset_ * cos( alpha1 )
        + knee_length_ * ( sin( alpha1 ) * cos( joint_q( 1 ) ) * sin( joint_q( 2 ) ) + sin( alpha1 ) * cos( joint_q( 2 ) ) * sin( joint_q( 1 ) ) ) + hip_length_ * sin( alpha1 ) * sin( joint_q( 1 ) );

    footPosition( 0, 0 ) = z;
    footPosition( 1, 0 ) = y;
    if ( ( leg == 1 ) || ( leg == 3 ) )
        footPosition( 1, 0 ) = -y;
    footPosition( 2, 0 ) = -x;
    footPosition( 3, 0 ) = 1.0;

    return footPosition;
}

Vec3< float > PositionController::IKS( Vec3< float > footP, int leg ) {
    // 1 0
    // 3 2
    Vec3< float > qDes;
    float         sideSigns[ 4 ] = { -1, 1, -1, 1 };

    float l1       = hip_knee_offset_;
    float l2       = hip_length_;
    float l3       = knee_length_;
    float l4       = 0;
    float sideSign = sideSigns[ leg ];

    float  x = footP( 0 ), y = footP( 1 ), z = footP( 2 );
    double ss1 = sqrt( z * z + y * y );
    if ( sideSign * y >= 0 )
        qDes( 0 ) = sideSign * ( acos( ( z * z + ss1 * ss1 - y * y ) / 2 / fabs( z ) / ss1 ) + acos( ( l1 + l4 ) / ss1 ) - M_PI / 2 );
    else
        qDes( 0 ) = sideSign * ( -acos( ( z * z + ss1 * ss1 - y * y ) / 2 / fabs( z ) / ss1 ) + acos( ( l1 + l4 ) / ss1 ) - M_PI / 2 );
    double ss2 = sqrt( ss1 * ss1 - ( l1 + l4 ) * ( l1 + l4 ) + x * x );
    qDes( 2 )  = acos( ( l2 * l2 + l3 * l3 - ss2 * ss2 ) / 2 / l2 / l3 );
    qDes( 2 )  = M_PI - qDes( 2 );
    if ( z <= 0 )
        qDes( 1 ) = -acos( ( l2 * l2 + ss2 * ss2 - l3 * l3 ) / 2 / l2 / ss2 ) + asin( x / ss2 );
    else
        qDes( 1 ) = -acos( ( l2 * l2 + ss2 * ss2 - l3 * l3 ) / 2 / l2 / ss2 ) + ( M_PI - asin( x / ss2 ) );
    return qDes;
}

Mat4< float > PositionController::RBodyMat( float alpha, float beta, float gamma, float x, float y, float z ) {
    float         sa, sb, sy, ca, cb, cy;
    Mat4< float > temp_;
    sa = sin( alpha );
    sb = sin( beta );
    sy = sin( gamma );
    ca = cos( alpha );
    cb = cos( beta );
    cy = cos( gamma );
    temp_ << ca * cb, ca * sb * sy - sa * cy, ca * sb * cy + sa * sy, x, sa * cb, sa * sb * sy + ca * cy, sa * sb * cy - ca * sy, y, -sb, cb * sy, cb * cy, z, 0.f, 0.f, 0.f, 1.0f;
    return temp_;
}

void PositionController::TransBody( Vec6< float > body_cmd, Vec3< float > foot_cmd, Vec4< float > ctrl_point, Vec4< float > contact_state ) {
    Vec4< float > _temp;
    for ( int foot( 0 ); foot < 4; foot++ ) {
        if ( contact_state( foot ) > 0 ) {
            world2body_trans_[ foot ] = ( RBodyMat( body_cmd( 2 ), body_cmd( 1 ), body_cmd( 0 ), -body_cmd( 3 ), -body_cmd( 4 ), -body_cmd( 5 ) ) ).eval();
        }
        else {
            world2body_trans_[ foot ] = ( RBodyMat( 0.f, 0.f, 0.f, foot_cmd( 0 ), foot_cmd( 1 ), foot_cmd( 2 ) ) ).eval();
        }
        _temp                        = foot_in_robot_p_cur_[ foot ] + hip_location_[ foot ] + ctrl_point;
        _temp( 3 )                   = 1.0f;
        foot_in_robot_p_cur_[ foot ] = world2body_trans_[ foot ] * _temp - ctrl_point - hip_location_[ foot ];

        joint_p_cmd_[ foot ] = IKS( foot_in_robot_p_cur_[ foot ].block( 0, 0, 3, 1 ), foot );
    }
}
