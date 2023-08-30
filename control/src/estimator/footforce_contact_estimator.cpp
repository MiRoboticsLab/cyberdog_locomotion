#include "estimator/footforce_contact_estimator.hpp"

template < typename T > void FootForceContactEstimator< T >::Setup() {
    for ( size_t i = 0; i < 4; i++ ) {
        last_foot_force_[ i ]      = this->state_estimator_data_.leg_controller_data[ i ].foot_force_actual[ 2 ];
        firstcontact_fourleg_[ i ] = false;
    }
}

template < typename T > void FootForceContactEstimator< T >::Run() {

    Vec4< T > swing_phase   = *this->state_estimator_data_.swing_phase;
    Vec4< T > contact_phase = *this->state_estimator_data_.contact_phase;
    for ( size_t i = 0; i < 4; i++ ) {
        float current_foot_force = this->state_estimator_data_.leg_controller_data[ i ].foot_force_actual[ 2 ];
        float u_footforce = -40, delta_footforce = 35;
        float probability_footforce = 0.5 + 0.5 * std::erff( ( current_foot_force + u_footforce ) / ( std::sqrt( 2 ) * delta_footforce ) );
        float current_foot_force_dt = ( current_foot_force - last_foot_force_[ i ] ) / 0.002;
        float u_footforce_dt = -8000, delta_footforce_dt = 1400;
        float probability_footforce_dt = 0.5 + 0.5 * std::erff( ( -current_foot_force_dt + u_footforce_dt ) / ( std::sqrt( 2 ) * delta_footforce_dt ) );

        last_foot_force_[ i ] = current_foot_force;
        if ( swing_phase[ i ] > 0.6 && probability_footforce_dt + probability_footforce > 0.5 ) {
            if ( !firstcontact_fourleg_[ i ] ) {
                this->state_estimator_data_.result->footforce_contact[ i ] = 1.0;

                firstcontact_fourleg_[ i ] = true;
            }
        }
        else if ( contact_phase[ i ] > std::numeric_limits< float >::epsilon() ) {
            this->state_estimator_data_.result->footforce_contact[ i ] = 1.0;

            firstcontact_fourleg_[ i ] = false;
        }
        else {
            this->state_estimator_data_.result->footforce_contact[ i ] = 0.0;
        }
    }
    // for ( size_t i = 0; i < 4; i++ ) {
    //     std::cout << i << ":" << this->state_estimator_data_.result->footforce_contact[ i ] << std::endl;
    // }
}

template class FootForceContactEstimator< float >;
template class FootForceContactEstimator< double >;
