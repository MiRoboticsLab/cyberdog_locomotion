#include "wbc/wbic/kin_wbc.hpp"
#include "utilities/pseudoInverse.hpp"

template < typename T > KinWbc< T >::KinWbc( size_t num_qdot ) : threshold_( 0.001 ), num_qdot_( num_qdot ), num_act_joint_( num_qdot - 6 ) {
    inertia_matrix_ = DMat< T >::Identity( num_qdot_, num_qdot_ );
}

template < typename T >
bool KinWbc< T >::FindConfiguration( const DVec< T >& curr_config, const std::vector< Task< T >* >& task_list, const std::vector< ContactSpec< T >* >& contact_list, DVec< T >& jpos_cmd,
                                     DVec< T >& jvel_cmd ) {

    // Contact Jacobian Setup
    DMat< T > Nc( num_qdot_, num_qdot_ );
    Nc.setIdentity();
    if ( contact_list.size() > 0 ) {
        DMat< T > Jc, Jc_i;
        contact_list[ 0 ]->GetContactJacobian( Jc );
        size_t num_rows = Jc.rows();

        for ( size_t i( 1 ); i < contact_list.size(); ++i ) {
            contact_list[ i ]->GetContactJacobian( Jc_i );
            size_t num_new_rows = Jc_i.rows();
            Jc.conservativeResize( num_rows + num_new_rows, num_qdot_ );
            Jc.block( num_rows, 0, num_new_rows, num_qdot_ ) = Jc_i;
            num_rows += num_new_rows;
        }

        // Projection Matrix
        BuildProjectionMatrix( Jc, Nc );
    }

    // First Task
    DVec< T > delta_q, qdot;
    DMat< T > Jt, JtPre, JtPre_pinv, N_nx, N_pre;

    Task< T >* task = task_list[ 0 ];
    task->GetTaskJacobian( Jt );
    JtPre = Jt * Nc;
    PseudoInverseWrapper( JtPre, JtPre_pinv );

    delta_q = JtPre_pinv * ( task->GetPosError() );
    qdot    = JtPre_pinv * ( task->GetDesVel() );

    DVec< T > prev_delta_q = delta_q;
    DVec< T > prev_qdot    = qdot;

    BuildProjectionMatrix( JtPre, JtPre_pinv, N_nx );
    N_pre = Nc * N_nx;

    for ( size_t i( 1 ); i < task_list.size(); ++i ) {
        task = task_list[ i ];

        task->GetTaskJacobian( Jt );
        JtPre = Jt * N_pre;

        PseudoInverseWrapper( JtPre, JtPre_pinv );
        delta_q = prev_delta_q + JtPre_pinv * ( task->GetPosError() - Jt * prev_delta_q );
        qdot    = prev_qdot + JtPre_pinv * ( task->GetDesVel() - Jt * prev_qdot );

        // For the next task
        BuildProjectionMatrix( JtPre, JtPre_pinv, N_nx );
        N_pre *= N_nx;
        prev_delta_q = delta_q;
        prev_qdot    = qdot;
    }
    for ( size_t i( 0 ); i < num_act_joint_; ++i ) {
        jpos_cmd[ i ] = curr_config[ i + 6 ] + delta_q[ i + 6 ];
        jvel_cmd[ i ] = qdot[ i + 6 ];
    }
    return true;
}

template < typename T > void KinWbc< T >::BuildProjectionMatrix( const DMat< T >& jacobian, DMat< T >& project_matrix ) {
    DMat< T > pseudo_inverse_jacobian;
    PseudoInverseWrapper( jacobian, pseudo_inverse_jacobian );
    project_matrix = inertia_matrix_ - pseudo_inverse_jacobian * jacobian;
}

template < typename T > void KinWbc< T >::BuildProjectionMatrix( const DMat< T >& jacobian, const DMat< T >& pseudo_inverse_jacobian, DMat< T >& project_matrix ) {
    project_matrix = inertia_matrix_ - pseudo_inverse_jacobian * jacobian;
}

template < typename T > void KinWbc< T >::PseudoInverseWrapper( const DMat< T > jacobian, DMat< T >& inverse_jacobian ) {
    PseudoInverse( jacobian, threshold_, inverse_jacobian );
}

template class KinWbc< float >;
template class KinWbc< double >;