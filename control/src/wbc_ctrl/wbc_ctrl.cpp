#include "wbc_ctrl/wbc_ctrl.hpp"
#include "utilities/timer.hpp"
#include "utilities/toolkit.hpp"

template < typename T >
WbcCtrl< T >::WbcCtrl( FloatingBaseModel< T > model )
    : full_config_( cyberdog2::kNumActJoint + 7 ), tau_ff_( cyberdog2::kNumActJoint ), des_jpos_( cyberdog2::kNumActJoint ), des_jvel_( cyberdog2::kNumActJoint ), wbc_lcm_( GetLcmUrl( 255 ) ) {
    iter_ = 0;
    full_config_.setZero();

    model_   = model;
    kin_wbc_ = new KinWbc< T >( cyberdog2::kDimConfig );

    wbic_      = new WBIC< T >( cyberdog2::kDimConfig, &( contact_list_ ), &( task_list_ ) );
    wbic_data_ = new WbicExtraData< T >();

    wbic_data_->body_pose_weight_      = DVec< T >::Constant( 6, 0.1 );
    wbic_data_->reaction_force_weight_ = DVec< T >::Constant( 12, 1. );
    wbic_data_->abad_joint_limit_      = model.abad_joint_torque_limit_;
    wbic_data_->hip_joint_limit_       = model.hip_joint_torque_limit_;
    wbic_data_->knee_joint_limit_      = model.knee_joint_torque_limit_;

    kp_joint_.resize( cyberdog2::kNumLegJoint, 5. );
    kd_joint_.resize( cyberdog2::kNumLegJoint, 1.5 );

    state_.q  = DVec< T >::Zero( cyberdog2::kNumActJoint );
    state_.qd = DVec< T >::Zero( cyberdog2::kNumActJoint );
}

template < typename T > WbcCtrl< T >::~WbcCtrl() {
    delete kin_wbc_;
    delete wbic_;
    delete wbic_data_;

    typename std::vector< Task< T >* >::iterator iter = task_list_.begin();
    while ( iter < task_list_.end() ) {
        delete ( *iter );
        ++iter;
    }
    task_list_.clear();

    typename std::vector< ContactSpec< T >* >::iterator iter2 = contact_list_.begin();
    while ( iter2 < contact_list_.end() ) {
        delete ( *iter2 );
        ++iter2;
    }
    contact_list_.clear();
}

template < typename T > void WbcCtrl< T >::ComputeWbc() {
    // TEST
    kin_wbc_->FindConfiguration( full_config_, task_list_, contact_list_, des_jpos_, des_jvel_ );

    // WBIC
    wbic_->UpdateSetting( A_, Ainv_, coriolis_, grav_ );
    wbic_->MakeTorque( tau_ff_, wbic_data_ );
}

template < typename T > void WbcCtrl< T >::Run( void* input, ControlFsmData< T >& data, const bool& use_absolute_dom ) {
    ++iter_;

    // Update Model
    UpdateModel( data.state_estimator->GetResult(), data.leg_controller->datas_, use_absolute_dom );

    // Task & Contact Update
    ContactTaskUpdate( input, data );

    // WBC Computation
    ComputeWbc();

    // TEST
    // T dt(0.002);
    // for(size_t i(0); i<12; ++i){
    // des_jpos_[i] = state_.q[i] + state_.qd[i] * dt + 0.5 * wbic_data_->qddot_[i+6] * dt * dt;
    // des_jvel_[i] = state_.qd[i] + wbic_data_->qddot_[i+6]*dt;
    //}

    // ContactTaskUpdateTEST(input, data);
    // ComputeWbc();
    // END of TEST

    // Update Leg Command
    UpdateLegCmd( data );

    // LCM publish
    LcmPublishData( data );
}

template < typename T > void WbcCtrl< T >::RunVirtual( void* input, ControlFsmData< T >& data ) {
    ++iter_;

    // Update Model
    UpdateModel( data.state_estimator->GetResult(), data.leg_controller->datas_ );

    // Task & Contact Update
    ContactTaskUpdate( input, data );

    // WBC Computation
    ComputeWbc();

    // TEST
    // T dt(0.002);
    // for(size_t i(0); i<12; ++i){
    // des_jpos_[i] = state_.q[i] + state_.qd[i] * dt + 0.5 * wbic_data_->qddot_[i+6] * dt * dt;
    // des_jvel_[i] = state_.qd[i] + wbic_data_->qddot_[i+6]*dt;
    //}

    // ContactTaskUpdateTEST(input, data);
    // ComputeWbc();
    // END of TEST

    // Update Leg Command
    // UpdateLegCmd(data);

    // LCM publish
    LcmPublishData( data );
}

template < typename T > void WbcCtrl< T >::UpdateLegCmd( ControlFsmData< T >& data ) {
    LegControllerCommand< T >* cmd = data.leg_controller->commands_;
    // Vec4<T> contact = data.state_estimator->GetResult().contact;

    for ( size_t leg( 0 ); leg < cyberdog2::kNumLeg; ++leg ) {
        cmd[ leg ].Zero();
        for ( size_t jidx( 0 ); jidx < cyberdog2::kNumLegJoint; ++jidx ) {
            cmd[ leg ].tau_feed_forward[ jidx ] = tau_ff_[ cyberdog2::kNumLegJoint * leg + jidx ];
            cmd[ leg ].q_des[ jidx ]            = des_jpos_[ cyberdog2::kNumLegJoint * leg + jidx ];
            cmd[ leg ].qd_des[ jidx ]           = des_jvel_[ cyberdog2::kNumLegJoint * leg + jidx ];

            cmd[ leg ].kp_joint( jidx, jidx ) = kp_joint_[ jidx ];
            cmd[ leg ].kd_joint( jidx, jidx ) = kd_joint_[ jidx ];
        }
    }

    // Knee joint non flip barrier
    T knee_pos_limit = data.quadruped->knee_lower_bound_ + 0.2;

    for ( size_t leg( 0 ); leg < 4; ++leg ) {
        if ( cmd[ leg ].q_des[ 2 ] < knee_pos_limit ) {
            cmd[ leg ].q_des[ 2 ] = knee_pos_limit;
        }
        if ( data.leg_controller->datas_[ leg ].q[ 2 ] < knee_pos_limit ) {
            T knee_pos                       = data.leg_controller->datas_[ leg ].q[ 2 ];
            cmd[ leg ].tau_feed_forward[ 2 ] = 2. / ( knee_pos * knee_pos + 0.1 );
        }
    }
}

template < typename T > void WbcCtrl< T >::UpdateModel( const StateEstimatorResult< T >& state_est, const LegControllerData< T >* leg_data, const bool& use_absolute_dom ) {
    if ( use_absolute_dom ) {
        state_.body_position = state_est.absolute_position;
    }
    else {
        state_.body_position = state_est.position;
    }
    state_.body_orientation = state_est.orientation;
    state_.rpy              = state_est.rpy;
    for ( size_t i( 0 ); i < 3; ++i ) {
        if ( use_absolute_dom ) {
            state_.body_velocity[ i + 3 ] = state_est.absolute_velocity_in_body_frame[ i ];
        }
        else {
            state_.body_velocity[ i + 3 ] = state_est.velocity_in_body_frame[ i ];
        }
        state_.body_velocity[ i ] = state_est.angular_velocity_in_body_frame[ i ];

        for ( size_t leg( 0 ); leg < 4; ++leg ) {
            state_.q[ 3 * leg + i ]  = leg_data[ leg ].q[ i ];
            state_.qd[ 3 * leg + i ] = leg_data[ leg ].qd[ i ];

            full_config_[ 3 * leg + i + 6 ] = state_.q[ 3 * leg + i ];
        }
    }
    model_.SetState( state_ );

    model_.ContactJacobians();
    model_.MassMatrix();
    model_.GeneralizedGravityForce();
    model_.GeneralizedCoriolisForce();

    A_        = model_.GetMassMatrix();
    grav_     = model_.GetGravityForce();
    coriolis_ = model_.GetCoriolisForce();
    Ainv_     = A_.inverse();
}

template class WbcCtrl< float >;
template class WbcCtrl< double >;
