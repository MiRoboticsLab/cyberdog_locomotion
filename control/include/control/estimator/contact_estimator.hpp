#ifndef CONTACT_ESTIMATOR_HPP_
#define CONTACT_ESTIMATOR_HPP_

#include "controllers/state_estimator_container.hpp"

/**
 * @brief A pass-through algorithm which passes the phase estimation to the state
 * estimator.
 */
template < typename T > class ContactEstimator : public GenericEstimator< T > {
public:
    /**
     * @brief Set the estimated contact by copying the exptected contact state into the
     * estimated contact state
     */
    virtual void Run() {
        this->state_estimator_data_.result->contact = *this->state_estimator_data_.contact_phase;
    }

    /**
     * @brief Set up the contact estimator
     */
    virtual void Setup() {}
};

#endif  // CONTACT_ESTIMATOR_HPP_
