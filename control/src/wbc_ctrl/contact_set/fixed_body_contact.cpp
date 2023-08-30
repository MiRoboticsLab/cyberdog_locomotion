#include "wbc_ctrl/contact_set/fixed_body_contact.hpp"
#include "dynamics/quadruped.hpp"

template < typename T > FixedBodyContact< T >::FixedBodyContact() : ContactSpec< T >( 6 ) {
    Contact::Jc_ = DMat< T >::Zero( Contact::dim_contact_, cyberdog2::kDimConfig );

    for ( size_t i( 0 ); i < Contact::dim_contact_; ++i )
        Contact::Jc_( i, i ) = 1.;
    Contact::Uf_      = DMat< T >::Zero( 1, Contact::dim_contact_ );
    Contact::ieq_vec_ = DVec< T >::Zero( 1 );
}

template < typename T > FixedBodyContact< T >::~FixedBodyContact() {}

template < typename T > bool FixedBodyContact< T >::UpdateJc() {
    return true;
}

template < typename T > bool FixedBodyContact< T >::UpdateJcDotQdot() {
    Contact::JcDotQdot_ = DVec< T >::Zero( Contact::dim_contact_ );
    return true;
}

template < typename T > bool FixedBodyContact< T >::UpdateUf() {
    return true;
}

template < typename T > bool FixedBodyContact< T >::UpdateInequalityVector() {
    return true;
}

template class FixedBodyContact< double >;
template class FixedBodyContact< float >;
