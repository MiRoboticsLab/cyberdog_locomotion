#ifndef NEW_BAT_ESTIMATOR_HPP_
#define NEW_BAT_ESTIMATOR_HPP_

#include "controllers/state_estimator_container.hpp"

/**
 * @brief Compared to BatEstimator, NewBatEstimator adds batt_soc.
 */
template < typename T > class NewBatEstimator : public GenericEstimator< T > {
public:
    /**
     * @brief battVolt:0~100
     */
    virtual void Run() {
        this->state_estimator_data_.result->is_battery_low = ( *this->state_estimator_data_.bms_status & 0x02 ) != 0;
        this->state_estimator_data_.result->is_charging    = ( *this->state_estimator_data_.bms_status & 0x01 ) != 0;
        this->state_estimator_data_.result->battery_soc    = *this->state_estimator_data_.battery_soc;
    }
    virtual void Setup() {}
};

#endif  // NEW_BAT_ESTIMATOR_HPP_
