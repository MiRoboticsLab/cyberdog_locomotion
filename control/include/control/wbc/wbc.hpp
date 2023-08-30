#ifndef WBC_HPP_
#define WBC_HPP_

#include <vector>

#include "contact_spec.hpp"
#include "task.hpp"
#include "utilities/pseudoInverse.hpp"
#include "cpp_types.hpp"

#define WB WBC< T >

/**
 * @brief whole body controller
 * 
 * Assume first 6 (or 3 in 2D case) joints are for the representation of
 * a floating base.
 */
template < typename T > class WBC {
public:
    WBC( size_t num_qdot ) : num_act_joint_( num_qdot - 6 ), num_qdot_( num_qdot ) {
        Sa_ = DMat< T >::Zero( num_act_joint_, num_qdot_ );
        Sv_ = DMat< T >::Zero( 6, num_qdot_ );

        Sa_.block( 0, 6, num_act_joint_, num_act_joint_ ).setIdentity();
        Sv_.block( 0, 0, 6, 6 ).setIdentity();
    }
    virtual ~WBC() {}

    virtual void UpdateSetting( const DMat< T >& A, const DMat< T >& Ainv, const DVec< T >& cori, const DVec< T >& grav, void* extra_setting = NULL ) = 0;

    virtual void MakeTorque( DVec< T >& cmd, void* extra_input = NULL ) = 0;

protected:
    // full rank fat matrix only
    void WeightedInverse( const DMat< T >& jacobian, const DMat< T >& inverse_weight, DMat< T >& inverse_jacobian, double threshold = 0.0001 ) {
        DMat< T > lambda( jacobian * inverse_weight * jacobian.transpose() );
        DMat< T > lambda_inv;
        PseudoInverse( lambda, threshold, lambda_inv );
        inverse_jacobian = inverse_weight * jacobian.transpose() * lambda_inv;
    }

    size_t num_act_joint_;
    size_t num_qdot_;

    DMat< T > Sa_;  // Actuated joint
    DMat< T > Sv_;  // Virtual joint

    DMat< T > A_;
    DMat< T > Ainv_;
    DVec< T > cori_;
    DVec< T > grav_;

    bool update_setting_;
};

#endif  // WBC_HPP_