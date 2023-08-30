#include <inttypes.h>

#include <Eigen/LU>
#include <Eigen/SVD>

#include "dynamics/quadruped.hpp"
#include "utilities/timer.hpp"
#include "wbc/wbic/wbic.hpp"

template < typename T >
WBIC< T >::WBIC( size_t num_qdot, const std::vector< ContactSpec< T >* >* contact_list, const std::vector< Task< T >* >* task_list ) : WBC< T >( num_qdot ), dim_floating_( 6 ) {
    contact_list_ = contact_list;
    task_list_    = task_list;

    eye_ = DMat< T >::Identity( WB::num_qdot_, WB::num_qdot_ );
}

template < typename T > void WBIC< T >::MakeTorque( DVec< T >& cmd, void* extra_input ) {
    if ( !WB::update_setting_ ) {
        printf( "[Wanning] WBIC setting is not done\n" );
    }
    if ( extra_input )
        data_ = static_cast< WbicExtraData< T >* >( extra_input );

    // resize G_, g0_, CE_, ce0_, CI_, ci0_
    SetOptimizationSize();
    SetCost();

    DVec< T > qddot_pre;
    DMat< T > JcBar;
    DMat< T > Npre;

    if ( dim_rf_ > 0 ) {
        // Contact Setting
        ContactBuilding();

        // Set inequality constraints
        SetInEqualityConstraint();
        WB::WeightedInverse( Jc_, WB::Ainv_, JcBar );
        qddot_pre = JcBar * ( -JcDotQdot_ );
        Npre      = eye_ - JcBar * Jc_;
        // PrettyPrint( JcBar, std::cout, "JcBar" );
        // PrettyPrint( JcDotQdot_, std::cout, "JcDotQdot" );
        // PrettyPrint( qddot_pre, std::cout, "qddot 1" );
    }
    else {
        qddot_pre = DVec< T >::Zero( WB::num_qdot_ );
        Npre      = eye_;
    }

    // Task
    Task< T >* task;
    DMat< T >  Jt, JtBar, JtPre;
    DVec< T >  JtDotQdot, xddot;

    for ( size_t i( 0 ); i < ( *task_list_ ).size(); ++i ) {
        task = ( *task_list_ )[ i ];

        task->GetTaskJacobian( Jt );
        task->GetTaskJacobianDotQdot( JtDotQdot );
        task->GetCommand( xddot );

        JtPre = Jt * Npre;
        WB::WeightedInverse( JtPre, WB::Ainv_, JtBar );

        qddot_pre += JtBar * ( xddot - JtDotQdot - Jt * qddot_pre );
        Npre = Npre * ( eye_ - JtBar * JtPre );

        // PrettyPrint( xddot, std::cout, "xddot" );
        // PrettyPrint( JtDotQdot, std::cout, "JtDotQdot" );
        // PrettyPrint( qddot_pre, std::cout, "qddot 2" );
        // PrettyPrint( Jt, std::cout, "Jt" );
        // PrettyPrint( JtPre, std::cout, "JtPre" );
        // PrettyPrint( JtBar, std::cout, "JtBar" );
    }

    // Set equality constraints
    SetEqualityConstraint( qddot_pre );

    // printf("G_:\n");
    // std::cout<<G_<<std::endl;
    // printf("g0_:\n");
    // std::cout<<g0_<<std::endl;

    // Optimization
    // Timer timer;
    T f = solve_quadprog( G_, g0_, CE_, ce0_, CI_, ci0_, z_ );
    // std::cout<<"\n wbic old time: "<<timer.GetElapsedMilliseconds()<<std::endl;
    ( void )f;

    // PrettyPrint( qddot_pre, std::cout, "qddot_cmd" );
    for ( size_t i( 0 ); i < dim_floating_; ++i )
        qddot_pre[ i ] += z_[ i ];

    for ( size_t i = 0; i < cyberdog2::kDimConfig; i++ ) {
        if ( fabs( qddot_pre[ i ] ) > 99999. ) {
            std::cout << "qddot_pre = DVec<T>::Zero(WB::num_qdot_);  !!!!!!!!!!!!" << std::endl << qddot_pre << std::endl;
            qddot_pre = DVec< T >::Zero( WB::num_qdot_ );
        }
    }

    GetSolution( qddot_pre, cmd );

    data_->opt_result_ = DVec< T >( dim_opt_ );
    for ( size_t i( 0 ); i < dim_opt_; ++i ) {
        data_->opt_result_[ i ] = z_[ i ];
    }

    // std::cout << "f: " << f << std::endl;
    // std::cout << "x: " << z_ << std::endl;

    // DVec<T> check_eq = dyn_CE_ * data_->opt_result_ + dyn_ce0_;
    // PrettyPrint( check_eq, std::cout, "equality constr" );
    // std::cout << "cmd: " << cmd << std::endl;
    // PrettyPrint( qddot_pre, std::cout, "qddot_pre" );
    // PrettyPrint( JcN, std::cout, "JcN" );
    // PrettyPrint( Nci_, std::cout, "Nci" );
    // DVec< T > eq_check = dyn_CE * data_->opt_result_;
    // PrettyPrint( dyn_ce0, std::cout, "dyn ce0_" );
    // PrettyPrint( eq_check, std::cout, "eq_check" );

    // PrettyPrint( Jt, std::cout, "Jt" );
    // PrettyPrint( JtDotQdot, std::cout, "Jtdotqdot" );
    // PrettyPrint( xddot, std::cout, "xddot" );

    // printf("CE_:\n");
    // std::cout<<CE_<<std::endl;
    // printf("ce0_:\n");
    // std::cout<<ce0_<<std::endl;

    // printf("CI_:\n");
    // std::cout<<CI_<<std::endl;
    // printf("ci0_:\n");
    // std::cout<<ci0_<<std::endl;
}

template < typename T > void WBIC< T >::SetEqualityConstraint( const DVec< T >& qddot ) {
    if ( dim_rf_ > 0 ) {
        dyn_CE_.block( 0, 0, dim_eq_cstr_, dim_floating_ )       = WB::A_.block( 0, 0, dim_floating_, dim_floating_ );
        dyn_CE_.block( 0, dim_floating_, dim_eq_cstr_, dim_rf_ ) = -WB::Sv_ * Jc_.transpose();
        dyn_ce0_                                                 = -WB::Sv_ * ( WB::A_ * qddot + WB::cori_ + WB::grav_ - Jc_.transpose() * force_des_ );
    }
    else {
        dyn_CE_.block( 0, 0, dim_eq_cstr_, dim_floating_ ) = WB::A_.block( 0, 0, dim_floating_, dim_floating_ );
        dyn_ce0_                                           = -WB::Sv_ * ( WB::A_ * qddot + WB::cori_ + WB::grav_ );
    }

    for ( size_t i( 0 ); i < dim_eq_cstr_; ++i ) {
        for ( size_t j( 0 ); j < dim_opt_; ++j ) {
            CE_[ j ][ i ] = dyn_CE_( i, j );
        }
        ce0_[ i ] = -dyn_ce0_[ i ];
    }
    // PrettyPrint( dyn_CE_, std::cout, "WBIC: CE_" );
    // PrettyPrint( dyn_ce0_, std::cout, "WBIC: ce0_" );
}

template < typename T > void WBIC< T >::SetInEqualityConstraint() {
    dyn_CI_.block( 0, dim_floating_, dim_uf_, dim_rf_ ) = Uf_;
    dyn_ci0_                                            = Uf_ieq_vec_ - Uf_ * force_des_;

    for ( size_t i( 0 ); i < dim_uf_; ++i ) {
        for ( size_t j( 0 ); j < dim_opt_; ++j ) {
            CI_[ j ][ i ] = dyn_CI_( i, j );
        }
        ci0_[ i ] = -dyn_ci0_[ i ];
    }
    // PrettyPrint( dyn_CI_, std::cout, "WBIC: CI_" );
    // PrettyPrint( dyn_ci0_, std::cout, "WBIC: ci0_" );
}

template < typename T > void WBIC< T >::ContactBuilding() {
    DMat< T > Uf;
    DVec< T > Uf_ieq_vec;
    // Initial
    DMat< T > Jc;
    DVec< T > JcDotQdot;
    size_t    dim_accumul_rf, dim_accumul_uf;
    ( *contact_list_ )[ 0 ]->GetContactJacobian( Jc );
    ( *contact_list_ )[ 0 ]->GetJcDotQdot( JcDotQdot );
    ( *contact_list_ )[ 0 ]->GetReactionForceConstraintMatrix( Uf );
    ( *contact_list_ )[ 0 ]->GetReactionForceConstraintVector( Uf_ieq_vec );

    dim_accumul_rf = ( *contact_list_ )[ 0 ]->GetDimension();
    dim_accumul_uf = ( *contact_list_ )[ 0 ]->GetReactionForceDimension();

    Jc_.block( 0, 0, dim_accumul_rf, WB::num_qdot_ )  = Jc;
    JcDotQdot_.head( dim_accumul_rf )                 = JcDotQdot;
    Uf_.block( 0, 0, dim_accumul_uf, dim_accumul_rf ) = Uf;
    Uf_ieq_vec_.head( dim_accumul_uf )                = Uf_ieq_vec;
    force_des_.head( dim_accumul_rf )                 = ( *contact_list_ )[ 0 ]->GetDesiredReactionForce();

    size_t dim_new_rf, dim_new_uf;

    for ( size_t i( 1 ); i < ( *contact_list_ ).size(); ++i ) {
        ( *contact_list_ )[ i ]->GetContactJacobian( Jc );
        ( *contact_list_ )[ i ]->GetJcDotQdot( JcDotQdot );

        dim_new_rf = ( *contact_list_ )[ i ]->GetDimension();
        dim_new_uf = ( *contact_list_ )[ i ]->GetReactionForceDimension();

        // Jc append
        Jc_.block( dim_accumul_rf, 0, dim_new_rf, WB::num_qdot_ ) = Jc;

        // JcDotQdot append
        JcDotQdot_.segment( dim_accumul_rf, dim_new_rf ) = JcDotQdot;

        // Uf
        ( *contact_list_ )[ i ]->GetReactionForceConstraintMatrix( Uf );
        Uf_.block( dim_accumul_uf, dim_accumul_rf, dim_new_uf, dim_new_rf ) = Uf;

        // Uf inequality vector
        ( *contact_list_ )[ i ]->GetReactionForceConstraintVector( Uf_ieq_vec );
        Uf_ieq_vec_.segment( dim_accumul_uf, dim_new_uf ) = Uf_ieq_vec;

        // Fr desired
        force_des_.segment( dim_accumul_rf, dim_new_rf ) = ( *contact_list_ )[ i ]->GetDesiredReactionForce();
        dim_accumul_rf += dim_new_rf;
        dim_accumul_uf += dim_new_uf;
    }
    // PrettyPrint( force_des_, std::cout, "[WBIC] Fr des" );
    // PrettyPrint( Jc_, std::cout, "[WBIC] Jc" );
    // PrettyPrint( JcDotQdot_, std::cout, "[WBIC] JcDot Qdot" );
    // PrettyPrint( Uf_, std::cout, "[WBIC] Uf" );
}

template < typename T > void WBIC< T >::GetSolution( const DVec< T >& qddot, DVec< T >& cmd ) {
    DVec< T > tot_tau;
    if ( dim_rf_ > 0 ) {
        data_->reaction_force_ = DVec< T >( dim_rf_ );
        // get Reaction forces
        for ( size_t i( 0 ); i < dim_rf_; ++i )
            data_->reaction_force_[ i ] = z_[ i + dim_floating_ ] + force_des_[ i ];
        tot_tau = WB::A_ * qddot + WB::cori_ + WB::grav_ - Jc_.transpose() * data_->reaction_force_;
    }
    else {
        tot_tau = WB::A_ * qddot + WB::cori_ + WB::grav_;
    }
    data_->qddot_ = qddot;

    cmd = tot_tau.tail( WB::num_act_joint_ ) * 1.0;

    static std::pair< bool, int > limit_flag_num[ 12 ];
    static float                  sum_over_torque[ 12 ] = { 0 };
    for ( int i = 0; i < 12; i += 3 ) {
        if ( fabs( cmd[ i ] ) > data_->abad_joint_limit_ ) {
            // printf( "[WBIC] WBC torque num %d\tout the limitation: %f.2f ", i, cmd[ i ] );
            // cmd[ i ] = cmd[ i ] / fabs( cmd[ i ] ) * data_->abad_joint_limit_;
            // printf( "  edit to %.2f\n", cmd[ i ] );
            // if ( !limit_flag_num[ i ].first )
            //     printf( "[WBIC] WBC torque id %d\tstart out the limitation\n", i );
            limit_flag_num[ i ].first = true;
            limit_flag_num[ i ].second++;
            sum_over_torque[ i ] += cmd[ i ];
            cmd[ i ] = cmd[ i ] / fabs( cmd[ i ] ) * data_->abad_joint_limit_;
        }
        else {
            if ( limit_flag_num[ i ].first )
                printf( "[WBIC] WBC torque id %d\tstop out the limitation, over iter is %d, average torque is %f.2f\n", i, limit_flag_num[ i ].second,
                        1.0 * sum_over_torque[ i ] / limit_flag_num[ i ].second );
            limit_flag_num[ i ].first  = false;
            limit_flag_num[ i ].second = 0;
            sum_over_torque[ i ]       = 0.0;
        }
    }
    for ( int i = 1; i < 12; i += 3 ) {
        if ( fabs( cmd[ i ] ) > data_->hip_joint_limit_ ) {
            // printf( "[WBIC] WBC torque num %d\tout the limitation: %.2f ", i, cmd[ i ] );
            // cmd[ i ] = cmd[ i ] / fabs( cmd[ i ] ) * data_->hip_joint_limit_;
            // printf( "  edit to %.2f\n", cmd[ i ] );
            // if ( !limit_flag_num[ i ].first )
            //     printf( "[WBIC] WBC torque id %d\tstart out the limitation\n", i );
            limit_flag_num[ i ].first = true;
            limit_flag_num[ i ].second++;
            sum_over_torque[ i ] += cmd[ i ];
            cmd[ i ] = cmd[ i ] / fabs( cmd[ i ] ) * data_->hip_joint_limit_;
        }
        else {
            if ( limit_flag_num[ i ].first )
                printf( "[WBIC] WBC torque id %d\tstop out the limitation, over iter is %d, average torque is %f.2f\n", i, limit_flag_num[ i ].second,
                        1.0 * sum_over_torque[ i ] / limit_flag_num[ i ].second );
            limit_flag_num[ i ].first  = false;
            limit_flag_num[ i ].second = 0;
            sum_over_torque[ i ]       = 0.0;
        }
    }
    for ( int i = 2; i < 12; i += 3 ) {
        if ( fabs( cmd[ i ] ) > data_->knee_joint_limit_ ) {
            // printf( "[WBIC] WBC torque num %d\tout the limitation: %.2f ", i, cmd[ i ] );
            // cmd[ i ] = cmd[ i ] / fabs( cmd[ i ] ) * data_->knee_joint_limit_;
            // printf( "  edit to %.2f\n", cmd[ i ] );
            // if ( !limit_flag_num[ i ].first )
            //     printf( "[WBIC] WBC torque id %d\tstart out the limitation\n", i );
            limit_flag_num[ i ].first = true;
            limit_flag_num[ i ].second++;
            sum_over_torque[ i ] += cmd[ i ];
            cmd[ i ] = cmd[ i ] / fabs( cmd[ i ] ) * data_->knee_joint_limit_;
        }
        else {
            if ( limit_flag_num[ i ].first )
                printf( "[WBIC] WBC torque id %d\tstop out the limitation, over iter is %d, average torque is %f.2f\n", i, limit_flag_num[ i ].second,
                        1.0 * sum_over_torque[ i ] / limit_flag_num[ i ].second );
            limit_flag_num[ i ].first  = false;
            limit_flag_num[ i ].second = 0;
            sum_over_torque[ i ]       = 0.0;
        }
    }

    // Torque check
    // DVec<T> delta_tau = DVec<T>::Zero(WB::num_qdot_);
    // for(size_t i(0); i<dim_floating_; ++i) delta_tau[i] = z_[i];
    // PrettyPrint( tot_tau, std::cout, "tot tau original" );
    // tot_tau += delta_tau;
    // PrettyPrint( tot_tau, std::cout, "tot tau result" );
    // PrettyPrint( qddot, std::cout, "qddot" );
    // PrettyPrint( data_->reaction_force_, std::cout, "Fr" );
    // PrettyPrint( force_des_, std::cout, "Fr des" );
}

template < typename T > void WBIC< T >::SetCost() {
    // Set Cost
    size_t idx_offset( 0 );
    for ( size_t i( 0 ); i < dim_floating_; ++i ) {
        G_[ i + idx_offset ][ i + idx_offset ] = data_->body_pose_weight_[ i ];
    }
    idx_offset += dim_floating_;
    for ( size_t i( 0 ); i < dim_rf_; ++i ) {
        G_[ i + idx_offset ][ i + idx_offset ] = data_->reaction_force_weight_[ i ];
    }
    // PrettyPrint( data_->body_pose_weight_, std::cout, "W floating" );
    // PrettyPrint( data_->reaction_force_weight_, std::cout, "W rf" );
}

template < typename T > void WBIC< T >::UpdateSetting( const DMat< T >& A, const DMat< T >& Ainv, const DVec< T >& cori, const DVec< T >& grav, void* extra_setting ) {
    WB::A_              = A;
    WB::Ainv_           = Ainv;
    WB::cori_           = cori;
    WB::grav_           = grav;
    WB::update_setting_ = true;

    ( void )extra_setting;
}

template < typename T > void WBIC< T >::SetOptimizationSize() {
    // TODO : to opt

    // Dimension
    dim_rf_ = 0;
    dim_uf_ = 0;  // Dimension of inequality constraint
    for ( size_t i( 0 ); i < ( *contact_list_ ).size(); ++i ) {
        dim_rf_ += ( *contact_list_ )[ i ]->GetDimension();
        dim_uf_ += ( *contact_list_ )[ i ]->GetReactionForceDimension();
    }

    dim_opt_     = dim_floating_ + dim_rf_;
    dim_eq_cstr_ = dim_floating_;

    // Matrix Setting
    G_.resize( 0., dim_opt_, dim_opt_ );
    g0_.resize( 0., dim_opt_ );
    CE_.resize( 0., dim_opt_, dim_eq_cstr_ );
    ce0_.resize( 0., dim_eq_cstr_ );

    // Eigen Matrix Setting
    dyn_CE_  = DMat< T >::Zero( dim_eq_cstr_, dim_opt_ );
    dyn_ce0_ = DVec< T >( dim_eq_cstr_ );
    if ( dim_rf_ > 0 ) {
        CI_.resize( 0., dim_opt_, dim_uf_ );
        ci0_.resize( 0., dim_uf_ );
        dyn_CI_  = DMat< T >::Zero( dim_uf_, dim_opt_ );
        dyn_ci0_ = DVec< T >( dim_uf_ );

        Jc_        = DMat< T >( dim_rf_, WB::num_qdot_ );
        JcDotQdot_ = DVec< T >( dim_rf_ );
        force_des_ = DVec< T >( dim_rf_ );

        Uf_ = DMat< T >( dim_uf_, dim_rf_ );
        Uf_.setZero();
        Uf_ieq_vec_ = DVec< T >( dim_uf_ );
    }
    else {
        CI_.resize( 0., dim_opt_, 1 );
        ci0_.resize( 0., 1 );
    }
}

template class WBIC< double >;
template class WBIC< float >;
