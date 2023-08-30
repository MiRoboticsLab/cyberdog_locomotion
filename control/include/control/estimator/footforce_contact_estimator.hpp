#ifndef FOOTFORCE_CONTACT_ESTIMATOR_HPP_
#define FOOTFORCE_CONTACT_ESTIMATOR_HPP_

#include "controllers/state_estimator_container.hpp"

/**
 * @brief An estimator considering both swing-phase and foot-force which returns the estimated contact state.
 */
template < typename T > class FootForceContactEstimator : public GenericEstimator< T > {
public:
    /**
     * @brief Run the contact estimator
     */
    virtual void Run();

    /**
     * @brief Set up the contact estimator
     */
    virtual void Setup();

private:
    Vec4< T > last_foot_force_;
    bool      firstcontact_fourleg_[ 4 ];
};

#endif  // FOOTFORCE_CONTACT_ESTIMATOR_HPP_
