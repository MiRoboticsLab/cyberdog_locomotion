#ifndef FIXED_BODY_CONTACT_HPP_
#define FIXED_BODY_CONTACT_HPP_

#include "wbc/contact_spec.hpp"

template < typename T > class FixedBodyContact : public ContactSpec< T > {
public:
    FixedBodyContact();
    virtual ~FixedBodyContact();

protected:
    virtual bool UpdateJc();
    virtual bool UpdateJcDotQdot();
    virtual bool UpdateUf();
    virtual bool UpdateInequalityVector();
};

#endif  // FIXED_BODY_CONTACT_HPP_