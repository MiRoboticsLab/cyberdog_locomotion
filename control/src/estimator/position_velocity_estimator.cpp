#include "estimator/position_velocity_estimator.hpp"

// #define USE_MODIFIED_RELATIVE_ODOM

/**
 * @brief Initialize the state estimator
 */
template < typename T > void LinearKFPositionVelocityEstimator< T >::Setup() {
    T dt = this->state_estimator_data_.parameters->controller_dt;
    xhat_.setZero();
    ps_.setZero();
    vs_.setZero();
    A_.setZero();
    A_.block( 0, 0, 3, 3 )   = Eigen::Matrix< T, 3, 3 >::Identity();
    A_.block( 0, 3, 3, 3 )   = dt * Eigen::Matrix< T, 3, 3 >::Identity();
    A_.block( 3, 3, 3, 3 )   = Eigen::Matrix< T, 3, 3 >::Identity();
    A_.block( 6, 6, 12, 12 ) = Eigen::Matrix< T, 12, 12 >::Identity();
    B_.setZero();
    B_.block( 3, 0, 3, 3 ) = dt * Eigen::Matrix< T, 3, 3 >::Identity();
    Eigen::Matrix< T, Eigen::Dynamic, Eigen::Dynamic > C_1( 3, 6 );
    C_1 << Eigen::Matrix< T, 3, 3 >::Identity(), Eigen::Matrix< T, 3, 3 >::Zero();
    Eigen::Matrix< T, Eigen::Dynamic, Eigen::Dynamic > C_2( 3, 6 );
    C_2 << Eigen::Matrix< T, 3, 3 >::Zero(), Eigen::Matrix< T, 3, 3 >::Identity();
    C_.setZero();
    C_.block( 0, 0, 3, 6 )   = C_1;
    C_.block( 3, 0, 3, 6 )   = C_1;
    C_.block( 6, 0, 3, 6 )   = C_1;
    C_.block( 9, 0, 3, 6 )   = C_1;
    C_.block( 0, 6, 12, 12 ) = T( -1 ) * Eigen::Matrix< T, 12, 12 >::Identity();
    C_.block( 12, 0, 3, 6 )  = C_2;
    C_.block( 15, 0, 3, 6 )  = C_2;
    C_.block( 18, 0, 3, 6 )  = C_2;
    C_.block( 21, 0, 3, 6 )  = C_2;
    C_( 27, 17 )             = T( 1 );
    C_( 26, 14 )             = T( 1 );
    C_( 25, 11 )             = T( 1 );
    C_( 24, 8 )              = T( 1 );
    P_.setIdentity();
    P_ = T( 100 ) * P_;
    Q0_.setIdentity();
    Q0_.block( 0, 0, 3, 3 )   = ( dt / 20.f ) * Eigen::Matrix< T, 3, 3 >::Identity();
    Q0_.block( 3, 3, 3, 3 )   = ( dt * 9.81f / 20.f ) * Eigen::Matrix< T, 3, 3 >::Identity();
    Q0_.block( 6, 6, 12, 12 ) = dt * Eigen::Matrix< T, 12, 12 >::Identity();
    R0_.setIdentity();
    cur_pos_.setZero();
    ini_pos_.setZero();
    for ( int i = 0; i < 4; ++i ) {
        dp_relative_previous_[ i ].setZero();
    }
}

template < typename T > LinearKFPositionVelocityEstimator< T >::LinearKFPositionVelocityEstimator() {}

/**
 * @brief Run state estimator
 */
template < typename T > void LinearKFPositionVelocityEstimator< T >::GetCurPosByLeg() {
    auto& quadruped = *this->state_estimator_data_.leg_controller_data->quadruped;

    Vec3< T > foot_pos_local[ 4 ];
    for ( int i = 0; i < 4; ++i ) {
        foot_pos_local[ i ] = quadruped.GetHipLocation( i ) + this->state_estimator_data_.leg_controller_data[ i ].p;
    }

    Vec3< T > foot_pos_world[ 4 ];
    for ( int i = 0; i < 4; ++i ) {
        foot_pos_world[ i ] = this->state_estimator_data_.result->world2body_rotation_matrix.transpose() * foot_pos_local[ i ];
    }

    cur_pos_ = -0.5 * ( 0.5 * ( foot_pos_world[ 0 ] + foot_pos_world[ 2 ] ) + 0.5 * ( foot_pos_world[ 1 ] + foot_pos_world[ 3 ] ) );
}
/**
 * @brief Run state estimator
 */
template < typename T > void LinearKFPositionVelocityEstimator< T >::Run() {

    if ( this->state_estimator_data_.result->contact( 0 ) == 0.5 && this->state_estimator_data_.result->contact( 1 ) == 0.5 && this->state_estimator_data_.result->contact( 2 ) == 0.5
         && this->state_estimator_data_.result->contact( 3 ) == 0.5 ) {
        if ( full_contact_iter_ > 1000 && !is_four_leg_stand_ ) {
            is_four_leg_stand_ = true;
            GetCurPosByLeg();
            ini_pos_      = this->state_estimator_data_.result->position - cur_pos_;
            ini_pos_[ 2 ] = cur_pos_[ 2 ];
        }
        else {
            full_contact_iter_++;
        }
    }
    else {
        full_contact_iter_ = 0;
        is_four_leg_stand_ = false;
    }

    T process_noise_pimu         = this->state_estimator_data_.parameters->imu_process_noise_position;
    T process_noise_vimu         = this->state_estimator_data_.parameters->imu_process_noise_velocity;
    T process_noise_pfoot        = this->state_estimator_data_.parameters->foot_process_noise_position;
    T sensor_noise_pimu_rel_foot = this->state_estimator_data_.parameters->foot_sensor_noise_position;
    T sensor_noise_vimu_rel_foot = this->state_estimator_data_.parameters->foot_sensor_noise_velocity;
    T sensor_noise_zfoot         = this->state_estimator_data_.parameters->foot_height_sensor_noise;

    Eigen::Matrix< T, 18, 18 > Q = Eigen::Matrix< T, 18, 18 >::Identity();
    Q.block( 0, 0, 3, 3 )        = Q0_.block( 0, 0, 3, 3 ) * process_noise_pimu;
    Q.block( 3, 3, 3, 3 )        = Q0_.block( 3, 3, 3, 3 ) * process_noise_vimu;
    Q.block( 6, 6, 12, 12 )      = Q0_.block( 6, 6, 12, 12 ) * process_noise_pfoot;

    Eigen::Matrix< T, 28, 28 > R = Eigen::Matrix< T, 28, 28 >::Identity();
    R.block( 0, 0, 12, 12 )      = R0_.block( 0, 0, 12, 12 ) * sensor_noise_pimu_rel_foot;
    R.block( 12, 12, 12, 12 )    = R0_.block( 12, 12, 12, 12 ) * sensor_noise_vimu_rel_foot;
    R.block( 24, 24, 4, 4 )      = R0_.block( 24, 24, 4, 4 ) * sensor_noise_zfoot;

    int qindex  = 0;
    int rindex1 = 0;
    int rindex2 = 0;
    int rindex3 = 0;

    if ( is_four_leg_stand_ ) {
        GetCurPosByLeg();
        xhat_.block( 0, 0, 3, 1 ) = ini_pos_ + cur_pos_;
        xhat_( 2 )                = cur_pos_( 2 );
    }

    Vec3< T > g( 0, 0, T( -9.81 ) );
    Mat3< T > Rbod = this->state_estimator_data_.result->world2body_rotation_matrix.transpose();

    // in old code, Rbod * se_acc + g
    Vec3< T > a = this->state_estimator_data_.result->acceleration_in_world_frame + g;
    //   std::cout << "A WORLD\n" << a << "\n";
    Vec4< T > pzs    = Vec4< T >::Zero();
    Vec4< T > trusts = Vec4< T >::Zero();
    Vec3< T > p0, v0;
    p0 << xhat_[ 0 ], xhat_[ 1 ], xhat_[ 2 ];
    v0 << xhat_[ 3 ], xhat_[ 4 ], xhat_[ 5 ];

    for ( int i = 0; i < 4; i++ ) {
        int             i1        = 3 * i;
        Quadruped< T >& quadruped = *( this->state_estimator_data_.leg_controller_data->quadruped );
        Vec3< T >       ph        = quadruped.GetHipLocation( i );  // hip positions relative to CoM
        Vec3< T >       p_rel     = ph + this->state_estimator_data_.leg_controller_data[ i ].p;

        Vec3< T > dp_rel = this->state_estimator_data_.leg_controller_data[ i ].v + this->state_estimator_data_.result->angular_velocity_in_body_frame.cross( p_rel );
        if ( is_four_leg_stand_ ) {
            dp_rel = 0.1 * dp_rel + 0.9 * dp_relative_previous_[ i ];
        }
        dp_relative_previous_[ i ] = dp_rel;
        if ( is_four_leg_stand_ ) {
            if ( dp_rel.norm() < 0.05 ) {
                dp_rel.setZero();
            }
        }
        Vec3< T > p_f = Rbod * p_rel;
        // Vec3<T> dp_f =
        //    Rbod *(this->state_estimator_data_.result->angular_velocity_in_body_frame.cross(p_rel) + dp_rel);

        qindex  = 6 + i1;
        rindex1 = i1;
        rindex2 = 12 + i1;
        rindex3 = 24 + i;

        T trust = T( 1 );
        T phase = fmin( this->state_estimator_data_.result->contact( i ), T( 1 ) );

#ifdef USE_MODIFIED_RELATIVE_ODOM
        Vec3< T > dp_f         = Rbod * dp_rel;
        T         trust_window = T( 0.03 / 0.15 );

        if ( phase < trust_window ) {
            trust = T( 0.0 );
        }
        else if ( phase > ( T( 1 ) - trust_window ) ) {
            trust = T( 0.0 );
        }

        T high_suspect_number( 100 );

        // printf("Trust %d: %.3f\n", i, trust);
        Q.block( qindex, qindex, 3, 3 )   = ( T( 1 ) + ( T( 1 ) - trust ) * high_suspect_number ) * Q.block( qindex, qindex, 3, 3 );
        R.block( rindex1, rindex1, 3, 3 ) = 1 * R.block( rindex1, rindex1, 3, 3 );
        R.block( rindex2, rindex2, 3, 3 ) = ( T( 1 ) + ( T( 1 ) - trust ) * high_suspect_number ) * R.block( rindex2, rindex2, 3, 3 );
        R( rindex3, rindex3 )             = ( T( 1 ) + ( T( 1 ) - trust ) * high_suspect_number ) * R( rindex3, rindex3 );

        trusts( i ) = trust;

        ps_.segment( i1, 3 ) = -p_f;
        vs_.segment( i1, 3 ) = ( 1.0f - trust ) * v0 + trust * ( -dp_f );
        pzs( i )             = ( 1.0f - trust ) * ( p0( 2 ) + p_f( 2 ) );
#else
        Mat3< T > Rwor         = this->state_estimator_data_.result->world2body_rotation_matrix;
        T         trust_window = T( 0.2 );

        if ( phase < trust_window ) {
            trust = phase / trust_window;
        }
        else if ( phase > ( T( 1 ) - trust_window ) ) {
            trust = ( T( 1 ) - phase ) / trust_window;
        }

        T trust_x        = T( 1 );
        T trust_window_x = T( 0.24 );  // trot
        if ( phase < trust_window_x ) {
            trust_x = phase / trust_window_x;
        }

        T trust_y        = T( 1 );
        T trust_window_y = T( 0.18 );  // trot
        if ( phase < trust_window_y ) {
            trust_y = phase / trust_window_y;
        }

        T high_suspect_number( 100 );

        Q( qindex, qindex )         = ( T( 1 ) + ( T( 1 ) - trust_x ) * high_suspect_number ) * Q( qindex, qindex );
        Q( qindex + 1, qindex + 1 ) = ( T( 1 ) + ( T( 1 ) - trust_y ) * high_suspect_number ) * Q( qindex + 1, qindex + 1 );
        Q( qindex + 2, qindex + 2 ) = ( T( 1 ) + ( T( 1 ) - trust ) * high_suspect_number ) * Q( qindex + 2, qindex + 2 );

        Vec3< T > Q_tmp             = Rbod * Vec3< T >( Q( qindex, qindex ), Q( qindex + 1, qindex + 1 ), Q( qindex + 2, qindex + 2 ) );
        Q( qindex, qindex )         = fabs( Q_tmp( 0 ) );
        Q( qindex + 1, qindex + 1 ) = fabs( Q_tmp( 1 ) );
        // Q(qindex + 2, qindex + 2) = Q_tmp(2);

        R.block( rindex1, rindex1, 3, 3 ) = 1 * R.block( rindex1, rindex1, 3, 3 );
        R( rindex2, rindex2 )             = ( T( 1 ) + ( T( 1 ) - trust_x ) * high_suspect_number ) * R( rindex2, rindex2 );
        R( rindex2 + 1, rindex2 + 1 )     = ( T( 1 ) + ( T( 1 ) - trust_y ) * high_suspect_number ) * R( rindex2 + 1, rindex2 + 1 );
        R( rindex2 + 2, rindex2 + 2 )     = ( T( 1 ) + ( T( 1 ) - trust ) * high_suspect_number ) * R( rindex2 + 2, rindex2 + 2 );

        Vec3< T > R_tmp               = Rbod * Vec3< T >( R( rindex2, rindex2 ), R( rindex2 + 1, rindex2 + 1 ), R( rindex2 + 2, rindex2 + 2 ) );
        R( rindex2, rindex2 )         = fabs( R_tmp( 0 ) );
        R( rindex2 + 1, rindex2 + 1 ) = fabs( R_tmp( 1 ) );
        // R(rindex2 + 2, rindex2 + 2) = R_tmp(2);

        R( rindex3, rindex3 ) = ( T( 1 ) + ( T( 1 ) - trust ) * high_suspect_number ) * R( rindex3, rindex3 );

        trusts( i ) = trust;

        ps_.segment( i1, 3 ) = -p_f;

        Vec3< T > v_bod = -Rwor * v0;
        vs_.segment( i1, 3 ) =
            -Rbod * Vec3< T >( ( 1.0f - trust_x ) * v_bod( 0 ) + trust_x * dp_rel( 0 ), ( 1.0f - trust_y ) * v_bod( 1 ) + trust_y * dp_rel( 1 ), ( 1.0f - trust ) * v_bod( 2 ) + trust * dp_rel( 2 ) );

        pzs( i ) = ( 1.0f - trust ) * ( p0( 2 ) + p_f( 2 ) );
#endif
    }

    Eigen::Matrix< T, 28, 1 > y;
    y << ps_, vs_, pzs;
    xhat_                             = A_ * xhat_ + B_ * a;
    Eigen::Matrix< T, 18, 18 > At     = A_.transpose();
    Eigen::Matrix< T, 18, 18 > Pm     = A_ * P_ * At + Q;  //
    Eigen::Matrix< T, 18, 28 > Ct     = C_.transpose();
    Eigen::Matrix< T, 28, 1 >  yModel = C_ * xhat_;
    Eigen::Matrix< T, 28, 1 >  ey     = y - yModel;
    Eigen::Matrix< T, 28, 28 > S      = C_ * Pm * Ct + R;

    // todo compute LU only once
    Eigen::Matrix< T, 28, 1 > S_ey = S.lu().solve( ey );  //
    xhat_ += Pm * Ct * S_ey;                              //

    Eigen::Matrix< T, 28, 18 > S_C = S.lu().solve( C_ );
    P_                             = ( Eigen::Matrix< T, 18, 18 >::Identity() - Pm * Ct * S_C ) * Pm;  //

    Eigen::Matrix< T, 18, 18 > Pt = P_.transpose();
    P_                            = ( P_ + Pt ) / T( 2 );

    if ( P_.block( 0, 0, 2, 2 ).determinant() > T( 0.000001 ) ) {
        P_.block( 0, 2, 2, 16 ).setZero();
        P_.block( 2, 0, 16, 2 ).setZero();
        P_.block( 0, 0, 2, 2 ) /= T( 10 );
    }

    this->state_estimator_data_.result->position                = xhat_.block( 0, 0, 3, 1 );
    this->state_estimator_data_.result->velocity_in_world_frame = xhat_.block( 3, 0, 3, 1 );
    this->state_estimator_data_.result->velocity_in_body_frame  = this->state_estimator_data_.result->world2body_rotation_matrix * this->state_estimator_data_.result->velocity_in_world_frame;
}

template class LinearKFPositionVelocityEstimator< float >;
template class LinearKFPositionVelocityEstimator< double >;

/**
 * @brief Run cheater estimator to copy cheater state into state estimate
 */
template < typename T > void CheaterPositionVelocityEstimator< T >::Run() {
    this->state_estimator_data_.result->position = this->state_estimator_data_.cheater_state->position.template cast< T >();
    this->state_estimator_data_.result->velocity_in_world_frame =
        this->state_estimator_data_.result->world2body_rotation_matrix.transpose().template cast< T >() * this->state_estimator_data_.cheater_state->velocity_in_body_frame.template cast< T >();
    this->state_estimator_data_.result->velocity_in_body_frame = this->state_estimator_data_.cheater_state->velocity_in_body_frame.template cast< T >();
}

template class CheaterPositionVelocityEstimator< float >;
template class CheaterPositionVelocityEstimator< double >;
