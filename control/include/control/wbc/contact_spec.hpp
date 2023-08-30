#ifndef CONTACT_SPEC_HPP_
#define CONTACT_SPEC_HPP_

#include "cpp_types.hpp"

#define Contact ContactSpec< T >

/**
 * @brief set up contact constraint for whole body controller
 * 
 */
template < typename T > class ContactSpec {
public:
    ContactSpec( size_t dim ) : dim_contact_( dim ), set_contact_flag_( false ) {
        idx_Fz_ = dim - 1;  // because normally (tau_x,y,z , linear_x,y,z)
        force_des_ = DVec< T >::Zero( dim );
    }
    virtual ~ContactSpec() {}

    size_t GetDimension() const {
        return dim_contact_;
    }

    size_t GetReactionForceDimension() const {
        return Uf_.rows();
    }

    size_t GetFzIndex() const {
        return idx_Fz_;
    }

    void GetContactJacobian( DMat< T >& Jc ) {
        Jc = Jc_;
    }

    void GetJcDotQdot( DVec< T >& JcDotQdot ) {
        JcDotQdot = JcDotQdot_;
    }

    void UnsetContact() {
        set_contact_flag_ = false;
    }

    void GetReactionForceConstraintMatrix( DMat< T >& Uf ) {
        Uf = Uf_;
    }

    void GetReactionForceConstraintVector( DVec< T >& ieq_vec ) {
        ieq_vec = ieq_vec_;
    }

    const DVec< T >& GetDesiredReactionForce() {
        return force_des_;
    }

    void SetDesiredReactionForce( const DVec< T >& reaction_force_des ) {
        force_des_ = reaction_force_des;
    }

    virtual void SetFriction( const T& mu ) {
        ( void )( mu );
    }

    bool UpdateContactSpec() {
        UpdateJc();
        UpdateJcDotQdot();
        UpdateUf();
        UpdateInequalityVector();
        set_contact_flag_ = true;
        return true;
    }

protected:
    virtual bool UpdateJc()               = 0;
    virtual bool UpdateJcDotQdot()        = 0;
    virtual bool UpdateUf()               = 0;
    virtual bool UpdateInequalityVector() = 0;

    int       idx_Fz_;
    DMat< T > Uf_;
    DVec< T > ieq_vec_;
    DVec< T > force_des_;

    DMat< T > Jc_;
    DVec< T > JcDotQdot_;
    size_t    dim_contact_;
    bool      set_contact_flag_;
};
#endif  // CONTACT_SPEC_HPP_