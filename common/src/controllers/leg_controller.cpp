#include <fstream>

#include "controllers/leg_controller.hpp"

#define ABAD_MIN -0.823f
#define ABAD_MAX 0.779f
#define HIP_MIN_F -4.406f
#define HIP_MAX_F 1.266f
#define HIP_MIN_R -3.611f
#define HIP_MAX_R 2.051f
#define KNEE_MIN 0.524f
#define KNEE_MAX 2.491f
#define DELTA 0.05f
#define SOFT_KP 20.0f

/**
 * @brief Zero the leg command so the leg will not output torque.
 *
 */
template < typename T > void LegControllerCommand< T >::Zero() {
    tau_feed_forward   = Vec3< T >::Zero();
    force_feed_forward = Vec3< T >::Zero();
    q_des              = Vec3< T >::Zero();
    qd_des             = Vec3< T >::Zero();
    p_des              = Vec3< T >::Zero();
    v_des              = Vec3< T >::Zero();
    kp_cartesian       = Mat3< T >::Zero();
    kd_cartesian       = Mat3< T >::Zero();
    kp_joint           = Mat3< T >::Zero();
    kd_joint           = Mat3< T >::Zero();
}

/**
 * @brief Zero the leg data.
 *
 */
template < typename T > void LegControllerData< T >::Zero() {
    q            = Vec3< T >::Zero();
    qd           = Vec3< T >::Zero();
    p            = Vec3< T >::Zero();
    v            = Vec3< T >::Zero();
    jacobian     = Mat3< T >::Zero();
    tau_estimate = Vec3< T >::Zero();
}

/**
 * @brief Zero all leg commands. This should be run before any control code, so if
 *        the control code is confused and doesn't change the leg command, the legs
 *        won't remember the last command.
 *
 */
template < typename T > void LegController< T >::ZeroCommand() {
    for ( auto& cmd : commands_ ) {
        cmd.Zero();
    }
    legs_enabled_ = false;
}

/**
 * @brief Set the leg to edamp. This overwrites all command data and generates an
 *        emergency damp command using the given gain.
 *
 * @param robot robot type
 * @param gain motor kd
 */
template < typename T > void LegController< T >::EdampCommand( RobotType robot, T gain ) {
    ZeroCommand();
    for ( int leg = 0; leg < 4; leg++ ) {
        for ( int axis = 0; axis < 3; axis++ ) {
            commands_[ leg ].kd_joint( axis, axis ) = gain;
        }
    }
    ( void )robot;
}

/**
 * @brief Update the "leg data" from a SPIne board message.
 *
 * @param spi_data SPI data
 */
template < typename T > void LegController< T >::UpdateData( const SpiData* spi_data ) {
    for ( int leg = 0; leg < 4; leg++ ) {
        // q
        datas_[ leg ].q( 0 ) = spi_data->q_abad[ leg ];
        datas_[ leg ].q( 1 ) = spi_data->q_hip[ leg ];
        datas_[ leg ].q( 2 ) = spi_data->q_knee[ leg ];

        // qd
        datas_[ leg ].qd( 0 ) = spi_data->qd_abad[ leg ];
        datas_[ leg ].qd( 1 ) = spi_data->qd_hip[ leg ];
        datas_[ leg ].qd( 2 ) = spi_data->qd_knee[ leg ];

        // tau_actual
        datas_[ leg ].tau_actual( 0 ) = spi_data->tau_abad[ leg ];
        datas_[ leg ].tau_actual( 1 ) = spi_data->tau_hip[ leg ];
        datas_[ leg ].tau_actual( 2 ) = spi_data->tau_knee[ leg ];

        datas_[ leg ].jacobian = ComputeLegJacobian( quadruped_, leg, datas_[ leg ].q );
        datas_[ leg ].p        = ForwardKinematic( quadruped_, leg, datas_[ leg ].q );

        // v
        datas_[ leg ].v = datas_[ leg ].jacobian * datas_[ leg ].qd;

        Mat3< T > inv_jacobian_transpose = datas_[ leg ].jacobian.transpose().inverse();
        datas_[ leg ].foot_force_desired = inv_jacobian_transpose * commands_[ leg ].tau_feed_forward;
        datas_[ leg ].foot_force_actual  = inv_jacobian_transpose * datas_[ leg ].tau_actual;

        // tmp of motors
        datas_[ leg ].tmp_actual( 0 ) = 0.1 * spi_data->tmp_abad[ leg ];
        datas_[ leg ].tmp_actual( 1 ) = 0.1 * spi_data->tmp_hip[ leg ];
        datas_[ leg ].tmp_actual( 2 ) = 0.1 * spi_data->tmp_knee[ leg ];
    }
}

/**
 * @brief Update the "leg command" for the SPIne board message.
 *
 * @param spi_command SPI command
 */
template < typename T > void LegController< T >::UpdateCommand( SpiCommand* spi_command, int pos_limit_mode ) {
    static int iter = 0;
    for ( int leg = 0; leg < 4; leg++ ) {
        // tauFF
        Vec3< T > leg_torque = commands_[ leg ].tau_feed_forward;

        // forceFF
        Vec3< T > foot_force = commands_[ leg ].force_feed_forward;

        // cartesian PD
        foot_force += commands_[ leg ].kp_cartesian * ( commands_[ leg ].p_des - datas_[ leg ].p );
        foot_force += commands_[ leg ].kd_cartesian * ( commands_[ leg ].v_des - datas_[ leg ].v );

        // Torque
        leg_torque += datas_[ leg ].jacobian.transpose() * foot_force;
        // estimate torque
        datas_[ leg ].tau_estimate = leg_torque + commands_[ leg ].kp_joint * ( commands_[ leg ].q_des - datas_[ leg ].q ) + commands_[ leg ].kd_joint * ( commands_[ leg ].qd_des - datas_[ leg ].qd );

        q_abad_limit_[ leg ] = 0;
        q_hip_limit_[ leg ]  = 0;
        q_knee_limit_[ leg ] = 0;
        T diff_q             = 0;
        // abad soft limit，kp kd  set 0, add inverse Kd gains
        if ( datas_[ leg ].q( 0 ) < ( q_abad_lowerbound_ + DELTA ) ) {
            if ( datas_[ leg ].tau_estimate( 0 ) < 0 ) {
                if ( pos_limit_mode ) {
                    diff_q          = q_abad_lowerbound_ - datas_[ leg ].q( 0 );
                    leg_torque( 0 ) = diff_q > -0.0125 ? 8 : 0.5 * 0.0025 / ( diff_q * diff_q );
                }
                else
                    leg_torque( 0 ) = SOFT_KP * ( ( q_abad_lowerbound_ + DELTA ) - datas_[ leg ].q( 0 ) );
                commands_[ leg ].kp_joint( 0, 0 ) = 0;
                commands_[ leg ].kd_joint( 0, 0 ) = 0;
            }
            q_abad_limit_[ leg ] = -1;
        }
        if ( datas_[ leg ].q( 0 ) > ( q_abad_upperbound_ - DELTA ) ) {
            if ( datas_[ leg ].tau_estimate( 0 ) > 0 ) {
                if ( pos_limit_mode ) {
                    diff_q          = q_abad_upperbound_ - datas_[ leg ].q( 0 );
                    leg_torque( 0 ) = diff_q < 0.0125 ? -8 : -0.5 * 0.0025 / ( diff_q * diff_q );
                }
                else
                    leg_torque( 0 ) = SOFT_KP * ( ( q_abad_upperbound_ - DELTA ) - datas_[ leg ].q( 0 ) );
                commands_[ leg ].kp_joint( 0, 0 ) = 0;
                commands_[ leg ].kd_joint( 0, 0 ) = 0;
            }
            q_abad_limit_[ leg ] = 1;
        }
        // hip soft limit
        if ( leg == 0 || leg == 1 ) {
            if ( datas_[ leg ].q( 1 ) < ( q_fronthip_lowerbound_ + DELTA ) ) {
                if ( datas_[ leg ].tau_estimate( 1 ) < 0 ) {
                    if ( pos_limit_mode ) {
                        diff_q          = q_fronthip_lowerbound_ - datas_[ leg ].q( 1 );
                        leg_torque( 1 ) = diff_q > -0.0125 ? 8 : 0.5 * 0.0025 / ( diff_q * diff_q );
                    }
                    else
                        leg_torque( 1 ) = SOFT_KP * ( ( q_fronthip_lowerbound_ + DELTA ) - datas_[ leg ].q( 1 ) );
                    commands_[ leg ].kp_joint( 1, 1 ) = 0;
                    commands_[ leg ].kd_joint( 1, 1 ) = 0;
                }
                q_hip_limit_[ leg ] = -1;
            }
            if ( datas_[ leg ].q( 1 ) > ( q_fronthip_upperbound_ - DELTA ) ) {
                if ( datas_[ leg ].tau_estimate( 1 ) > 0 ) {
                    if ( pos_limit_mode ) {
                        diff_q          = q_fronthip_upperbound_ - datas_[ leg ].q( 1 );
                        leg_torque( 1 ) = diff_q < 0.0125 ? -8 : -0.5 * 0.0025 / ( diff_q * diff_q );
                    }
                    else
                        leg_torque( 1 ) = SOFT_KP * ( ( q_fronthip_upperbound_ - DELTA ) - datas_[ leg ].q( 1 ) );
                    commands_[ leg ].kp_joint( 1, 1 ) = 0;
                    commands_[ leg ].kd_joint( 1, 1 ) = 0;
                }
                q_hip_limit_[ leg ] = 1;
            }
        }
        else {
            if ( datas_[ leg ].q( 1 ) < ( q_rearhip_lowerbound_ + DELTA ) ) {
                if ( datas_[ leg ].tau_estimate( 1 ) < 0 ) {
                    if ( pos_limit_mode ) {
                        diff_q          = q_rearhip_lowerbound_ - datas_[ leg ].q( 1 );
                        leg_torque( 1 ) = diff_q > -0.0125 ? 8 : 0.5 * 0.0025 / ( diff_q * diff_q );
                    }
                    else
                        leg_torque( 1 ) = SOFT_KP * ( ( q_rearhip_lowerbound_ + DELTA ) - datas_[ leg ].q( 1 ) );

                    commands_[ leg ].kp_joint( 1, 1 ) = 0;
                    commands_[ leg ].kd_joint( 1, 1 ) = 0;
                }
                q_hip_limit_[ leg ] = -1;
            }
            if ( datas_[ leg ].q( 1 ) > ( q_rearhip_upperbound_ - DELTA ) ) {
                if ( datas_[ leg ].tau_estimate( 1 ) > 0 ) {
                    if ( pos_limit_mode ) {
                        diff_q          = q_rearhip_upperbound_ - datas_[ leg ].q( 1 );
                        leg_torque( 1 ) = diff_q < 0.0125 ? -8 : -0.5 * 0.0025 / ( diff_q * diff_q );
                    }
                    else
                        leg_torque( 1 ) = SOFT_KP * ( ( q_rearhip_upperbound_ - DELTA ) - datas_[ leg ].q( 1 ) );

                    commands_[ leg ].kp_joint( 1, 1 ) = 0;
                    commands_[ leg ].kd_joint( 1, 1 ) = 0;
                }
                q_hip_limit_[ leg ] = 1;
            }
        }
        // knee soft limit
        if ( datas_[ leg ].q( 2 ) < ( q_knee_lowerbound_ + DELTA ) ) {
            if ( datas_[ leg ].tau_estimate( 2 ) < -1 ) {
                if ( pos_limit_mode ) {
                    diff_q          = q_knee_lowerbound_ - datas_[ leg ].q( 2 );
                    leg_torque( 2 ) = diff_q > -0.0125 ? 8 : 0.5 * 0.0025 / ( diff_q * diff_q );
                }
                else
                    leg_torque( 2 ) = SOFT_KP * ( ( q_knee_lowerbound_ + DELTA ) - datas_[ leg ].q( 2 ) );

                commands_[ leg ].kp_joint( 2, 2 ) = 0;
                commands_[ leg ].kd_joint( 2, 2 ) = 0;
            }
            q_knee_limit_[ leg ] = -1;
        }
        if ( datas_[ leg ].q( 2 ) > ( q_knee_upperbound_ - DELTA ) ) {
            if ( datas_[ leg ].tau_estimate( 2 ) > 1 ) {
                if ( pos_limit_mode ) {
                    diff_q          = q_knee_upperbound_ - datas_[ leg ].q( 2 );
                    leg_torque( 2 ) = diff_q < 0.0125 ? -8 : -0.5 * 0.0025 / ( diff_q * diff_q );
                }
                else
                    leg_torque( 2 ) = SOFT_KP * ( ( q_knee_upperbound_ - DELTA ) - datas_[ leg ].q( 2 ) );
                commands_[ leg ].kp_joint( 2, 2 ) = 0;
                commands_[ leg ].kd_joint( 2, 2 ) = 0;
            }
            q_knee_limit_[ leg ] = 1;
        }

        // set tau
        spi_command->tau_abad_ff[ leg ] = leg_torque( 0 );
        spi_command->tau_hip_ff[ leg ]  = leg_torque( 1 );
        spi_command->tau_knee_ff[ leg ] = leg_torque( 2 );

        // set kd
        spi_command->kd_abad[ leg ] = commands_[ leg ].kd_joint( 0, 0 );
        spi_command->kd_hip[ leg ]  = commands_[ leg ].kd_joint( 1, 1 );
        spi_command->kd_knee[ leg ] = commands_[ leg ].kd_joint( 2, 2 );

        // set kp
        spi_command->kp_abad[ leg ] = commands_[ leg ].kp_joint( 0, 0 );
        spi_command->kp_hip[ leg ]  = commands_[ leg ].kp_joint( 1, 1 );
        spi_command->kp_knee[ leg ] = commands_[ leg ].kp_joint( 2, 2 );

        // set q_des
        spi_command->q_des_abad[ leg ] = commands_[ leg ].q_des( 0 );
        spi_command->q_des_hip[ leg ]  = commands_[ leg ].q_des( 1 );
        spi_command->q_des_knee[ leg ] = commands_[ leg ].q_des( 2 );

        // set qd_des
        spi_command->qd_des_abad[ leg ] = commands_[ leg ].qd_des( 0 );
        spi_command->qd_des_hip[ leg ]  = commands_[ leg ].qd_des( 1 );
        spi_command->qd_des_knee[ leg ] = commands_[ leg ].qd_des( 2 );

        spi_command->flags[ leg ] = legs_enabled_ ? 1 : 0;

        if ( clear_error_ ) {
            spi_command->flags[ leg ] = 2;
        }
    }
    iter++;
}

/**
 * @brief Set LCM debug data from leg commands_ and data.
 *
 * @param lcm_data lcm data
 * @param lcm_command lcm command
 */
template < typename T > void LegController< T >::SetLcm( leg_control_data_lcmt* lcm_data, leg_control_command_lcmt* lcm_command ) {
    for ( int leg = 0; leg < 4; leg++ ) {
        for ( int axis = 0; axis < 3; axis++ ) {
            int idx                        = leg * 3 + axis;
            lcm_data->q[ idx ]             = datas_[ leg ].q[ axis ];
            lcm_data->qd[ idx ]            = datas_[ leg ].qd[ axis ];
            lcm_data->p[ idx ]             = datas_[ leg ].p[ axis ];
            lcm_data->v[ idx ]             = datas_[ leg ].v[ axis ];
            lcm_data->tau_est[ idx ]       = datas_[ leg ].tau_estimate[ axis ];
            lcm_data->force_est[ idx ]     = datas_[ leg ].foot_force_actual[ axis ];
            lcm_data->force_desired[ idx ] = datas_[ leg ].foot_force_desired[ axis ];

            lcm_command->tau_ff[ idx ]       = commands_[ leg ].tau_feed_forward[ axis ];
            lcm_command->f_ff[ idx ]         = commands_[ leg ].force_feed_forward[ axis ];
            lcm_command->q_des[ idx ]        = commands_[ leg ].q_des[ axis ];
            lcm_command->qd_des[ idx ]       = commands_[ leg ].qd_des[ axis ];
            lcm_command->p_des[ idx ]        = commands_[ leg ].p_des[ axis ];
            lcm_command->v_des[ idx ]        = commands_[ leg ].v_des[ axis ];
            lcm_command->kp_cartesian[ idx ] = commands_[ leg ].kp_cartesian( axis, axis );
            lcm_command->kd_cartesian[ idx ] = commands_[ leg ].kd_cartesian( axis, axis );
            lcm_command->kp_joint[ idx ]     = commands_[ leg ].kp_joint( axis, axis );
            lcm_command->kd_joint[ idx ]     = commands_[ leg ].kd_joint( axis, axis );
        }
        lcm_data->q_abad_limit[ leg ] = q_abad_limit_[ leg ];
        lcm_data->q_hip_limit[ leg ]  = q_hip_limit_[ leg ];
        lcm_data->q_knee_limit[ leg ] = q_knee_limit_[ leg ];
    }
}

template struct LegControllerCommand< double >;
template struct LegControllerCommand< float >;

template struct LegControllerData< double >;
template struct LegControllerData< float >;

template class LegController< double >;
template class LegController< float >;

/**
 * @brief Compute the Jacobian of the foot.  This is done in the local leg coordinate system.
 *
 * @param quad quadruped robot
 * @param leg index of the leg (0-3)
 * @param q joint angles
 * @return the Jacobian matrix
 */
template < typename T > Mat3< T > ComputeLegJacobian( Quadruped< T >& quad, int leg, Vec3< T > q ) {
    Mat3< T > jacobian;

    T side_sign = quad.GetSideSign( leg );

    T l1 = quad.abad_link_length_;
    T l2 = quad.hip_link_length_;
    T l3 = quad.knee_link_length_;
    T l4 = quad.knee_link_y_offset_;

    T s1 = std::sin( q( 0 ) );
    T s2 = std::sin( q( 1 ) );
    T s3 = std::sin( q( 2 ) );

    T c1 = std::cos( q( 0 ) );
    T c2 = std::cos( q( 1 ) );
    T c3 = std::cos( q( 2 ) );

    T c23 = c2 * c3 - s2 * s3;
    T s23 = s2 * c3 + c2 * s3;

    jacobian( 0, 0 ) = 0;
    jacobian( 0, 1 ) = l3 * c23 + l2 * c2;
    jacobian( 0, 2 ) = l3 * c23;
    jacobian( 1, 0 ) = l3 * c1 * c23 + l2 * c1 * c2 - ( l1 + l4 ) * side_sign * s1;
    jacobian( 1, 1 ) = -l3 * s1 * s23 - l2 * s1 * s2;
    jacobian( 1, 2 ) = -l3 * s1 * s23;
    jacobian( 2, 0 ) = l3 * s1 * c23 + l2 * c2 * s1 + ( l1 + l4 ) * side_sign * c1;
    jacobian( 2, 1 ) = l3 * c1 * s23 + l2 * c1 * s2;
    jacobian( 2, 2 ) = l3 * c1 * s23;
    return jacobian;
}

template Mat3< double > ComputeLegJacobian< double >( Quadruped< double >& quad, int leg, Vec3< double > q );
template Mat3< float >  ComputeLegJacobian< float >( Quadruped< float >& quad, int leg, Vec3< float > q );

/**
 * @brief Compute the postion of the foot. This is done in the local leg coordinate system.
 *
 * @param quad quadruped robot
 * @param leg index of the leg (0-3)
 * @param q joint angles
 * @return the postion of the foot
 */
template < typename T > Vec3< T > ForwardKinematic( Quadruped< T >& quad, int leg, Vec3< T > q ) {
    Vec3< T > pos;

    T side_sign = quad.GetSideSign( leg );

    T l1 = quad.abad_link_length_;
    T l2 = quad.hip_link_length_;
    T l3 = quad.knee_link_length_;
    T l4 = quad.knee_link_y_offset_;

    T s1 = std::sin( q( 0 ) );
    T s2 = std::sin( q( 1 ) );
    T s3 = std::sin( q( 2 ) );

    T c1 = std::cos( q( 0 ) );
    T c2 = std::cos( q( 1 ) );
    T c3 = std::cos( q( 2 ) );

    T c23 = c2 * c3 - s2 * s3;
    T s23 = s2 * c3 + c2 * s3;

    pos( 0 ) = l3 * s23 + l2 * s2;
    pos( 1 ) = ( l1 + l4 ) * side_sign * c1 + l3 * ( s1 * c23 ) + l2 * c2 * s1;
    pos( 2 ) = ( l1 + l4 ) * side_sign * s1 - l3 * ( c1 * c23 ) - l2 * c1 * c2;
    return pos;
}

template Vec3< double > ForwardKinematic< double >( Quadruped< double >& quad, int leg, Vec3< double > q );
template Vec3< float >  ForwardKinematic< float >( Quadruped< float >& quad, int leg, Vec3< float > q );

/**
 * @brief Compute the joints of the foot by IKS. This is done in the local leg coordinate system.
 *
 * @param quad quadruped robot
 * @param leg index of the leg (0-3)
 * @param p end-effector position
 * @return joint angles
 */
template < typename T > Vec3< T > InverseKinematic( Quadruped< T >& quad, int leg, Vec3< T > p ) {
    static Vec3< T > q_des;
    static Vec3< T > results;

    T side_sign = quad.GetSideSign( leg );

    T l1 = quad.abad_link_length_;
    T l2 = quad.hip_link_length_;
    T l3 = quad.knee_link_length_;
    T l4 = quad.knee_link_y_offset_;

    T x = p( 0 );
    T y = p( 1 );
    T z = p( 2 );

    if ( fabs( y ) < 0.0001 )
        y = 0.0001;

    double ss1 = sqrt( z * z + y * y );

    if ( side_sign * y >= 0 ) {
        q_des( 0 ) = side_sign * ( acos( ( z * z + ss1 * ss1 - y * y ) / 2 / fabs( z ) / ss1 ) + acos( ( l1 + l4 ) / ss1 ) - M_PI / 2 );
    }
    else {
        q_des( 0 ) = side_sign * ( -acos( ( z * z + ss1 * ss1 - y * y ) / 2 / fabs( z ) / ss1 ) + acos( ( l1 + l4 ) / ss1 ) - M_PI / 2 );
    }

    double ss2 = sqrt( ss1 * ss1 - ( l1 + l4 ) * ( l1 + l4 ) + x * x );

    q_des( 2 ) = acos( ( l2 * l2 + l3 * l3 - ss2 * ss2 ) / 2 / l2 / l3 );
    q_des( 2 ) = M_PI - q_des( 2 );

    if ( z <= 0 ) {
        q_des( 1 ) = -acos( ( l2 * l2 + ss2 * ss2 - l3 * l3 ) / 2 / l2 / ss2 ) + asin( x / ss2 );
    }
    else {
        q_des( 1 ) = -acos( ( l2 * l2 + ss2 * ss2 - l3 * l3 ) / 2 / l2 / ss2 ) + ( M_PI - asin( x / ss2 ) );
    }
    for ( int i( 0 ); i < 3; i++ ) {
        if ( std::isnan( q_des( i ) ) ) {
            std::cout << "[LegController] leg " << leg << " joint " << i << " IK result is NaN !" << std::endl;
        }
        else {
            results( i ) = q_des( i );
        }
    }
    return results;
}

template Vec3< double > InverseKinematic< double >( Quadruped< double >& quad, int leg, Vec3< double > p );
template Vec3< float >  InverseKinematic< float >( Quadruped< float >& quad, int leg, Vec3< float > p );
