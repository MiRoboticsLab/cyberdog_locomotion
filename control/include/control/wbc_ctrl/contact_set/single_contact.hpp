#ifndef SINGLE_CONTACT_HPP_
#define SINGLE_CONTACT_HPP_

#include "dynamics/floating_base_model.hpp"
#include "dynamics/quadruped.hpp"
#include "wbc/contact_spec.hpp"

template < typename T > class SingleContact : public ContactSpec< T > {
public:
    SingleContact( const FloatingBaseModel< T >* robot, int contact_pt );
    virtual ~SingleContact();

    void setMaxFz( T max_fz ) {
        max_fz_ = max_fz;
    }

    void SetFriction( const T& mu ) {
        Contact::Uf_( 1, 2 ) = mu;
        Contact::Uf_( 2, 2 ) = mu;
        Contact::Uf_( 3, 2 ) = mu;
        Contact::Uf_( 4, 2 ) = mu;
    }

protected:
    T   max_fz_;
    int contact_pt_;
    int u_dim_;

    virtual bool UpdateJc();
    virtual bool UpdateJcDotQdot();
    virtual bool UpdateUf();
    virtual bool UpdateInequalityVector();

    const FloatingBaseModel< T >* robot_sys_;
};

#endif  // SINGLE_CONTACT_HPP_
