#include "wbc_ctrl/contact_set/single_contact.hpp"
#include "utilities/utilities_print.hpp"

template < typename T > SingleContact< T >::SingleContact( const FloatingBaseModel< T >* robot, int pt ) : ContactSpec< T >( 3 ), max_fz_( 1500. ), contact_pt_( pt ), u_dim_( 6 ) {
    Contact::idx_Fz_    = 2;
    robot_sys_          = robot;
    Contact::Jc_        = DMat< T >( Contact::dim_contact_, cyberdog2::kDimConfig );
    Contact::JcDotQdot_ = DVec< T >::Zero( Contact::dim_contact_ );
    Contact::Uf_        = DMat< T >::Zero( u_dim_, Contact::dim_contact_ );

    T mu( 0.4 );

    Contact::Uf_( 0, 2 ) = 1.;

    Contact::Uf_( 1, 0 ) = 1.;
    Contact::Uf_( 1, 2 ) = mu;
    Contact::Uf_( 2, 0 ) = -1.;
    Contact::Uf_( 2, 2 ) = mu;

    Contact::Uf_( 3, 1 ) = 1.;
    Contact::Uf_( 3, 2 ) = mu;
    Contact::Uf_( 4, 1 ) = -1.;
    Contact::Uf_( 4, 2 ) = mu;

    // Upper bound of normal force
    Contact::Uf_( 5, 2 ) = -1.;
}

template < typename T > SingleContact< T >::~SingleContact() {}

template < typename T > bool SingleContact< T >::UpdateJc() {
    Contact::Jc_ = robot_sys_->Jc_[ contact_pt_ ];

    // Quat<T> quat = robot_sys_->state_.body_orientation;
    // Mat3<T> Rot = ori::QuaternionToRotationMatrix(quat);
    // Contact::Jc_.block(0,3, 3,3) = Rot*Contact::Jc_.block(0,3,3,3);

    // Contact::Jc_.block(0,0, 3,3) = Rot.transpose()*Contact::Jc_.block(0,0,3,3);
    // PrettyPrint( Rot, std::cout, "body ori" );
    // PrettyPrint( Contact::Jc_, std::cout, "Jc" );
    return true;
}

template < typename T > bool SingleContact< T >::UpdateJcDotQdot() {
    Contact::JcDotQdot_ = robot_sys_->Jcdqd_[ contact_pt_ ];
    // PrettyPrint( Contact::JcDotQdot_, std::cout, "JcDotQdot" );
    return true;
}

template < typename T > bool SingleContact< T >::UpdateUf() {
    return true;
}

template < typename T > bool SingleContact< T >::UpdateInequalityVector() {
    Contact::ieq_vec_      = DVec< T >::Zero( u_dim_ );
    Contact::ieq_vec_[ 5 ] = -max_fz_;
    return true;
}

template class SingleContact< double >;
template class SingleContact< float >;
