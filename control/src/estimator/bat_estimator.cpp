
#include "estimator/bat_estimator.hpp"

/**
 * @brief Initialize the state estimator
 *
 */
template < typename T > void BatEstimator< T >::Setup() {
    bms_lcm_.subscribe( "bms_data", &BatEstimator::HandleBmsLcm, this );
    iter_ = 0;
}

template < typename T > BatEstimator< T >::BatEstimator() : bms_lcm_( GetLcmUrlWithPort( 7672, 255 ) ) {}

/**
 * @brief Run state estimator
 */
template < typename T > void BatEstimator< T >::Run() {
    iter_++;
    bms_lcm_.handleTimeout( 0 );
    this->state_estimator_data_.result->is_battery_low = low_bat_;
    this->state_estimator_data_.result->is_charging    = is_charging_;
}

template < typename T > void BatEstimator< T >::HandleBmsLcm( const lcm::ReceiveBuffer* buf, const std::string& channel, const bms_response_lcmt* msg ) {
    ( void )buf;
    if ( channel == "bms_data" ) {
        if ( msg->status & 0x02 ) {
            low_bat_ = true;
        }
        else {
            low_bat_ = false;
        }
        if ( msg->status & 0x01 ) {
            is_charging_ = true;
        }
        else {
            is_charging_ = false;
        }
    }
}

template class BatEstimator< float >;
template class BatEstimator< double >;

/**
 * @brief Run cheater estimator to copy cheater state into state estimate
 */
template < typename T > void CheaterBatEstimator< T >::Run() {
    this->state_estimator_data_.result->is_battery_low = false;
}

template class CheaterBatEstimator< float >;
template class CheaterBatEstimator< double >;
