#include "wbc_ctrl/locomotion_ctrl/locomotion_ctrl.hpp"
#include "wbc_ctrl/contact_set/single_contact.hpp"
#include "wbc_ctrl/task_set/body_orientation_task.hpp"
#include "wbc_ctrl/task_set/body_position_task.hpp"
#include "wbc_ctrl/task_set/link_position_task.hpp"

template < typename T > LocomotionCtrl< T >::LocomotionCtrl( FloatingBaseModel< T > model ) : WbcCtrl< T >( model ) {
    body_pos_task_ = new BodyPosTask< T >( &( WBCtrl::model_ ) );
    body_ori_task_ = new BodyOriTask< T >( &( WBCtrl::model_ ) );

    foot_contact_[ 0 ] = new SingleContact< T >( &( WBCtrl::model_ ), linkID::kFr );
    foot_contact_[ 1 ] = new SingleContact< T >( &( WBCtrl::model_ ), linkID::kFl );
    foot_contact_[ 2 ] = new SingleContact< T >( &( WBCtrl::model_ ), linkID::kHr );
    foot_contact_[ 3 ] = new SingleContact< T >( &( WBCtrl::model_ ), linkID::kHl );

    foot_task_[ 0 ] = new LinkPosTask< T >( &( WBCtrl::model_ ), linkID::kFr );
    foot_task_[ 1 ] = new LinkPosTask< T >( &( WBCtrl::model_ ), linkID::kFl );
    foot_task_[ 2 ] = new LinkPosTask< T >( &( WBCtrl::model_ ), linkID::kHr );
    foot_task_[ 3 ] = new LinkPosTask< T >( &( WBCtrl::model_ ), linkID::kHl );

    kp_body_.setConstant( 60.0 );
    kd_body_.setConstant( 6.0 );
    kp_ori_.setConstant( 60.0 );
    kd_ori_.setConstant( 6.0 );
    kp_foot_.setConstant( 300.0 );
    kd_foot_.setConstant( 36.0 );
    kp_joint_.setConstant( 3.0 );
    kd_joint_.setConstant( 1.0 );
}

template < typename T > LocomotionCtrl< T >::~LocomotionCtrl() {
    delete body_pos_task_;
    delete body_ori_task_;

    for ( size_t i( 0 ); i < 4; ++i ) {
        delete foot_contact_[ i ];
        delete foot_task_[ i ];
    }
}

template < typename T > void LocomotionCtrl< T >::SetFriction( const T& mu ) {
    for ( size_t leg( 0 ); leg < 4; ++leg ) {
        foot_contact_[ leg ]->SetFriction( mu );
    }
}

template < typename T >
void LocomotionCtrl< T >::SetTaskPD( const Vec3< T >& kp_body, const Vec3< T >& kd_body, const Vec3< T >& kp_ori, const Vec3< T >& kd_ori, const Vec3< T >& kp_foot, const Vec3< T >& kd_foot,
                                     const Vec3< T >& kp_joint, const Vec3< T >& kd_joint ) {
    kp_body_  = kp_body;
    kd_body_  = kd_body;
    kp_ori_   = kp_ori;
    kd_ori_   = kd_ori;
    kp_foot_  = kp_foot;
    kd_foot_  = kd_foot;
    kp_joint_ = kp_joint;
    kd_joint_ = kd_joint;
    
    task_pd_set_outside_ = true;
}

template < typename T > void LocomotionCtrl< T >::ContactTaskUpdate( void* input, ControlFsmData< T >& data ) {
    input_data_ = static_cast< LocomotionCtrlData< T >* >( input );

    if ( !task_pd_set_outside_ ) {
        ParameterSetup( data.user_parameters ); // set with default value
    }
    else {
        ParameterSetup();                      // set with customized value
    }
    
    // Wash out the previous setup
    CleanUp();

    quat_des_ = ori::RpyToQuat( input_data_->body_rpy_des );

    Vec3< T > zero_vec3;
    zero_vec3.setZero();
    body_ori_task_->UpdateTask( &quat_des_, input_data_->body_omg_des, zero_vec3 );
    body_pos_task_->UpdateTask( &( input_data_->body_pos_des ), input_data_->body_vel_des, input_data_->body_acc_des );

    WBCtrl::task_list_.push_back( body_ori_task_ );
    WBCtrl::task_list_.push_back( body_pos_task_ );

    for ( size_t leg( 0 ); leg < 4; ++leg ) {
        if ( input_data_->contact_state[ leg ] > 0. ) {  // Contact
            foot_contact_[ leg ]->SetDesiredReactionForce( ( DVec< T > )( input_data_->reaction_force_des[ leg ] ) );
            foot_contact_[ leg ]->UpdateContactSpec();
            WBCtrl::contact_list_.push_back( foot_contact_[ leg ] );
        }
        else {  // No Contact (swing)
            foot_task_[ leg ]->UpdateTask( &( input_data_->foot_pos_des[ leg ] ), input_data_->foot_vel_des[ leg ], input_data_->foot_acc_des[ leg ] );
            // zero_vec3);
            WBCtrl::task_list_.push_back( foot_task_[ leg ] );
        }
    }
}

template < typename T > void LocomotionCtrl< T >::ContactTaskUpdateTEST( void* input, ControlFsmData< T >& data ) {
    ( void )data;
    input_data_ = static_cast< LocomotionCtrlData< T >* >( input );

    for ( size_t i( 0 ); i < 3; ++i ) {
        ( ( BodyPosTask< T >* )body_pos_task_ )->kp[ i ] = 10.;
        ( ( BodyPosTask< T >* )body_pos_task_ )->kd[ i ] = 3.;

        ( ( BodyOriTask< T >* )body_ori_task_ )->kp[ i ] = 10.;
        ( ( BodyOriTask< T >* )body_ori_task_ )->kd[ i ] = 3.;

        for ( size_t j( 0 ); j < 4; ++j ) {
            ( ( LinkPosTask< T >* )foot_task_[ j ] )->kp[ i ] = 70;
            ( ( LinkPosTask< T >* )foot_task_[ j ] )->kd[ i ] = 3.;
        }
    }
    // Wash out the previous setup
    CleanUp();

    quat_des_ = ori::RpyToQuat( input_data_->body_rpy_des );

    Vec3< T > zero_vec3;
    zero_vec3.setZero();
    body_ori_task_->UpdateTask( &quat_des_, input_data_->body_omg_des, zero_vec3 );
    body_pos_task_->UpdateTask( &( input_data_->body_pos_des ), input_data_->body_vel_des, input_data_->body_acc_des );

    WBCtrl::task_list_.push_back( body_ori_task_ );
    WBCtrl::task_list_.push_back( body_pos_task_ );

    for ( size_t leg( 0 ); leg < 4; ++leg ) {
        if ( input_data_->contact_state[ leg ] > 0. ) {  // Contact
            foot_contact_[ leg ]->SetDesiredReactionForce( ( DVec< T > )( input_data_->reaction_force_des[ leg ] ) );
            foot_contact_[ leg ]->UpdateContactSpec();
            WBCtrl::contact_list_.push_back( foot_contact_[ leg ] );
        }
        else {  // No Contact (swing)
            foot_task_[ leg ]->UpdateTask( &( input_data_->foot_pos_des[ leg ] ), input_data_->foot_vel_des[ leg ], input_data_->foot_acc_des[ leg ] );
            // zero_vec3);
            WBCtrl::task_list_.push_back( foot_task_[ leg ] );
        }
    }
}

template < typename T > void LocomotionCtrl< T >::ParameterSetup( const UserParameters* param ) {

    for ( size_t i( 0 ); i < 3; ++i ) {
        ( ( BodyPosTask< T >* )body_pos_task_ )->kp[ i ] = param->wbc_body_kp[ i ];
        ( ( BodyPosTask< T >* )body_pos_task_ )->kd[ i ] = param->wbc_body_kd[ i ];

        ( ( BodyOriTask< T >* )body_ori_task_ )->kp[ i ] = param->wbc_orient_kp[ i ];
        ( ( BodyOriTask< T >* )body_ori_task_ )->kd[ i ] = param->wbc_orient_kd[ i ];

        for ( size_t j( 0 ); j < 4; ++j ) {
            ( ( LinkPosTask< T >* )foot_task_[ j ] )->kp[ i ] = param->wbc_foot_kp[ i ];
            ( ( LinkPosTask< T >* )foot_task_[ j ] )->kd[ i ] = param->wbc_foot_kd[ i ];
        }

        WBCtrl::kp_joint_[ i ] = param->wbc_joint_kp[ i ];
        WBCtrl::kd_joint_[ i ] = param->wbc_joint_kd[ i ];
    }
}

template < typename T > void LocomotionCtrl< T >::ParameterSetup() {
    for ( size_t i( 0 ); i < 3; ++i ) {
        ( ( BodyPosTask< T >* )body_pos_task_ )->kp[ i ] = kp_body_[ i ];
        ( ( BodyPosTask< T >* )body_pos_task_ )->kd[ i ] = kd_body_[ i ];

        ( ( BodyOriTask< T >* )body_ori_task_ )->kp[ i ] = kp_ori_[ i ];
        ( ( BodyOriTask< T >* )body_ori_task_ )->kd[ i ] = kd_ori_[ i ];

        for ( size_t j( 0 ); j < 4; ++j ) {
            ( ( LinkPosTask< T >* )foot_task_[ j ] )->kp[ i ] = kp_foot_[ i ];
            ( ( LinkPosTask< T >* )foot_task_[ j ] )->kd[ i ] = kd_foot_[ i ];
        }

        WBCtrl::kp_joint_[ i ] = kp_joint_[ i ];
        WBCtrl::kd_joint_[ i ] = kd_joint_[ i ];
    }

    // // DEBUG
    // std::cout << "kp_foot_: " << kp_foot_.transpose() << std::endl;
    // std::cout << "kd_foot_: " << kd_foot_.transpose() << std::endl;
}

template < typename T > void LocomotionCtrl< T >::CleanUp() {
    WBCtrl::contact_list_.clear();
    WBCtrl::task_list_.clear();
}

template < typename T > void LocomotionCtrl< T >::LcmPublishData( ControlFsmData< T >& data ) {
    int iter( 0 );
    for ( size_t leg( 0 ); leg < 4; ++leg ) {
        reaction_force_result_[ leg ].setZero();

        if ( input_data_->contact_state[ leg ] > 0. ) {
            for ( size_t i( 0 ); i < 3; ++i ) {
                reaction_force_result_[ leg ][ i ] = WBCtrl::wbic_data_->reaction_force_[ 3 * iter + i ];
            }
            ++iter;
        }

        if ( input_data_->contact_state[ leg ] > 0. ) {  // Contact
            WBCtrl::wbc_data_lcm_.contact_est[ leg ] = 1;
        }
        else {
            WBCtrl::wbc_data_lcm_.contact_est[ leg ] = 0;
        }
    }

    for ( size_t i( 0 ); i < 3; ++i ) {
        WBCtrl::wbc_data_lcm_.foot_pos[ i ] = WBCtrl::model_.pGC_[ linkID::kFr ][ i ];
        WBCtrl::wbc_data_lcm_.foot_vel[ i ] = WBCtrl::model_.vGC_[ linkID::kFr ][ i ];

        WBCtrl::wbc_data_lcm_.foot_pos[ i + 3 ] = WBCtrl::model_.pGC_[ linkID::kFl ][ i ];
        WBCtrl::wbc_data_lcm_.foot_vel[ i + 3 ] = WBCtrl::model_.vGC_[ linkID::kFl ][ i ];

        WBCtrl::wbc_data_lcm_.foot_pos[ i + 6 ] = WBCtrl::model_.pGC_[ linkID::kHr ][ i ];
        WBCtrl::wbc_data_lcm_.foot_vel[ i + 6 ] = WBCtrl::model_.vGC_[ linkID::kHr ][ i ];

        WBCtrl::wbc_data_lcm_.foot_pos[ i + 9 ] = WBCtrl::model_.pGC_[ linkID::kHl ][ i ];
        WBCtrl::wbc_data_lcm_.foot_vel[ i + 9 ] = WBCtrl::model_.vGC_[ linkID::kHl ][ i ];

        for ( size_t leg( 0 ); leg < 4; ++leg ) {
            WBCtrl::wbc_data_lcm_.Fr_des[ 3 * leg + i ] = input_data_->reaction_force_des[ leg ][ i ];
            WBCtrl::wbc_data_lcm_.Fr[ 3 * leg + i ]     = reaction_force_result_[ leg ][ i ];

            WBCtrl::wbc_data_lcm_.foot_pos_cmd[ 3 * leg + i ] = input_data_->foot_pos_des[ leg ][ i ];
            WBCtrl::wbc_data_lcm_.foot_vel_cmd[ 3 * leg + i ] = input_data_->foot_vel_des[ leg ][ i ];
            WBCtrl::wbc_data_lcm_.foot_acc_cmd[ 3 * leg + i ] = input_data_->foot_acc_des[ leg ][ i ];

            WBCtrl::wbc_data_lcm_.jpos_cmd[ 3 * leg + i ] = WBCtrl::des_jpos_[ 3 * leg + i ];
            WBCtrl::wbc_data_lcm_.jvel_cmd[ 3 * leg + i ] = WBCtrl::des_jvel_[ 3 * leg + i ];

            WBCtrl::wbc_data_lcm_.jpos[ 3 * leg + i ] = WBCtrl::state_.q[ 3 * leg + i ];
            WBCtrl::wbc_data_lcm_.jvel[ 3 * leg + i ] = WBCtrl::state_.qd[ 3 * leg + i ];
        }

        WBCtrl::wbc_data_lcm_.body_pos_cmd[ i ]     = input_data_->body_pos_des[ i ];
        WBCtrl::wbc_data_lcm_.body_vel_cmd[ i ]     = input_data_->body_vel_des[ i ];
        WBCtrl::wbc_data_lcm_.body_ori_cmd[ i ]     = quat_des_[ i ];  //input_data_->body_rpy_des[ i ]
        WBCtrl::wbc_data_lcm_.body_ang_vel_cmd[ i ] = input_data_->body_omg_des[ i ];

        Quat< T > quat            = WBCtrl::state_.body_orientation;
        Mat3< T > Rot             = ori::QuaternionToRotationMatrix( quat );
        Vec3< T > global_body_vel = Rot.transpose() * WBCtrl::state_.body_velocity.tail( 3 );

        WBCtrl::wbc_data_lcm_.body_pos[ i ]     = WBCtrl::state_.body_position[ i ];
        WBCtrl::wbc_data_lcm_.body_vel[ i ]     = global_body_vel[ i ];
        WBCtrl::wbc_data_lcm_.body_ori[ i ]     = WBCtrl::state_.body_orientation[ i ];  // WBCtrl::state_.rpy[ i ];
        WBCtrl::wbc_data_lcm_.body_ang_vel[ i ] = WBCtrl::state_.body_velocity[ i ];
    }
    WBCtrl::wbc_data_lcm_.body_ori_cmd[ 3 ] = quat_des_[ 3 ];
    WBCtrl::wbc_data_lcm_.body_ori[ 3 ]     = WBCtrl::state_.body_orientation[ 3 ];
    // TODO: delete lcm publish , for independent thread to publish
    if ( data.control_parameters->lcm_debug_switch == 1 )
        WBCtrl::wbc_lcm_.publish( "wbc_lcm_data", &( WBCtrl::wbc_data_lcm_ ) );
}

template class LocomotionCtrl< float >;
template class LocomotionCtrl< double >;
