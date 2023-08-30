#ifndef STATE_ESTIMATOR_CONTAINER_HPP_
#define STATE_ESTIMATOR_CONTAINER_HPP_

#include "controllers/leg_controller.hpp"
#include "header/lcm_type/state_estimator_lcmt.hpp"
#include "parameters/robot_parameters.hpp"
#include "sim_utilities/imu_types.hpp"
#include "sim_utilities/visualization_data.hpp"

/**
 * @brief Result of state estimation
 *
 */
template < typename T > struct StateEstimatorResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vec3< T >   position;
    Quat< T >   orientation;
    Vec3< T >   rpy;
    Vec3< T >   velocity_in_body_frame;
    Vec3< T >   angular_velocity_in_body_frame;
    Vec3< T >   acceleration_in_body_frame;
    RotMat< T > world2body_rotation_matrix;
    Vec4< T >   contact;
    Vec4< T >   footforce_contact;

    Vec3< T > velocity_in_world_frame;
    Vec3< T > angular_velocity_in_world_frame;
    Vec3< T > acceleration_in_world_frame;
    Vec3< T > remoter_velocity;
    Vec3< T > terrain_coefficient;
    Mat3< T > terrain_rotation_matrix;

    bool   is_battery_low;
    bool   is_charging;
    int8_t battery_soc;

    T         height;
    Vec3< T > absolute_position;
    Vec3< T > absolute_velocity_in_body_frame;
    Vec3< T > absolute_velocity_in_world_frame;

    explicit StateEstimatorResult() {
        memset( ( void* )this, 0, sizeof( StateEstimatorResult< T > ) );
    }

    void setLcm( state_estimator_lcmt& lcm_data ) {
        for ( int i = 0; i < 3; i++ ) {
            lcm_data.p[ i ]          = position[ i ];
            lcm_data.vWorld[ i ]     = velocity_in_world_frame[ i ];
            lcm_data.vBody[ i ]      = velocity_in_body_frame[ i ];
            lcm_data.p_abs[ i ]      = absolute_position[ i ];
            lcm_data.vWorld_abs[ i ] = absolute_velocity_in_world_frame[ i ];
            lcm_data.vBody_abs[ i ]  = absolute_velocity_in_body_frame[ i ];
            lcm_data.rpy[ i ]        = rpy[ i ];
            lcm_data.omegaBody[ i ]  = angular_velocity_in_body_frame[ i ];
            lcm_data.omegaWorld[ i ] = angular_velocity_in_world_frame[ i ];
            lcm_data.vRemoter[ i ]   = remoter_velocity[ i ];
            lcm_data.aBody[ i ]      = acceleration_in_body_frame[ i ];
            lcm_data.aWorld[ i ]     = acceleration_in_world_frame[ i ];
        }

        for ( int i = 0; i < 4; i++ ) {
            lcm_data.quat[ i ]            = orientation[ i ];
            lcm_data.contactEstimate[ i ] = contact[ i ];
        }
    }
};

/**
 * @brief Inputs for state estimation.
 *
 * If robot code needs to inform the state estimator of something,
 * it should be added here. (You should also add a setter method to
 * StateEstimatorContainer)
 */
template < typename T > struct StateEstimatorData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StateEstimatorResult< T >* result;  // where to write the output to
    VectorNavData*             vector_nav_data;
    CheaterState< double >*    cheater_state;
    LegControllerData< T >*    leg_controller_data;
    Vec4< T >*                 contact_phase;
    Vec4< T >*                 swing_phase;
    RobotControlParameters*    parameters;
    Vec3< T >*                 ori_cali_gain;
    Vec3< T >*                 ori_cali_offset;
    int8_t*                    bms_status;
    int8_t*                    battery_soc;
};

/**
 * @brief All Estimators should inherit from this class
 *
 */
template < typename T > class GenericEstimator {
public:
    virtual void Run()   = 0;
    virtual void Setup() = 0;

    void SetData( StateEstimatorData< T > data ) {
        state_estimator_data_ = data;
    }

    virtual ~GenericEstimator() = default;
    StateEstimatorData< T > state_estimator_data_;
};

/**
 * @brief Main State Estimator Class
 * Contains all GenericEstimators, and can run them
 * Also updates visualizations
 *
 */
template < typename T > class StateEstimatorContainer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct a new state estimator container
     */
    StateEstimatorContainer( CheaterState< double >* cheater_state, VectorNavData* vector_nav_data, LegControllerData< T >* leg_controller_data, StateEstimatorResult< T >* state_estimate,
                             RobotControlParameters* parameters ) {
        data_.cheater_state       = cheater_state;
        data_.vector_nav_data     = vector_nav_data;
        data_.leg_controller_data = leg_controller_data;
        data_.result              = state_estimate;
        phase_                    = Vec4< T >::Zero();
        swing_phase_              = Vec4< T >::Zero();
        data_.contact_phase       = &phase_;
        data_.swing_phase         = &swing_phase_;
        data_.parameters          = parameters;
        ori_cali_gain_            = Vec3< T >::Zero();
        data_.ori_cali_gain       = &ori_cali_gain_;
        ori_cali_offset_          = Vec3< T >::Zero();
        data_.ori_cali_offset     = &ori_cali_offset_;
        bms_status_               = 0;
        data_.bms_status          = &bms_status_;
        battery_soc_              = 0;
        data_.battery_soc         = &battery_soc_;
    }

    /**
     * @brief Run all estimators
     */
    void Run( Cyberdog2Visualization* visualization = nullptr ) {
        for ( auto estimator : estimators_ ) {
            estimator->Run();
        }
        if ( visualization ) {
            visualization->quat = data_.result->orientation.template cast< float >();
            visualization->p    = data_.result->position.template cast< float >();
            // todo contact!
        }
    }

    /**
     * @brief Get the Result
     *
     * @return const StateEstimatorResult< T >&
     */
    const StateEstimatorResult< T >& GetResult() {
        return *data_.result;
    }

    /**
     * @brief Get the Result Handle
     *
     * @return StateEstimatorResult< T >*
     */
    StateEstimatorResult< T >* getResultHandle() {
        return data_.result;
    }

    /**
     * @brief Set the Remoter Velocity Result
     *
     * @param rs
     */
    void SetRemoterVelocityResult( Vec3< T > rs ) {
        data_.result->remoter_velocity = rs;
        //      printf("remoterVelocity:%.2f\t%.2f\t%.2f\n",rs(0),rs(1),rs(2));
    }

    /**
     * @brief Set the contact phase
     *
     */
    void SetContactPhase( Vec4< T >& phase ) {
        *data_.contact_phase = phase;
    }

    /**
     * @brief Set the swing phase
     *
     */
    void SetSwingPhase( Vec4< T >& phase ) {
        *data_.swing_phase = phase;
    }

    /**
     * @brief Set the orientation calibration gain based on velocity
     *
     */
    void SetOriCaliGain( Vec3< T >& gain ) {
        *data_.ori_cali_gain = gain;
    }

    /**
     * @brief Set the orientation calibration offset
     *
     */
    void SetOriCaliOffset( Vec3< T >& offset ) {
        *data_.ori_cali_offset = offset;
    }

    /**
     * @brief Set bms status
     *
     */
    void SetBmsStatus( int8_t* status ) {
        *data_.bms_status = ( status == nullptr ? 0 : *status );
    }

    /**
     * @brief Set battery SoC
     *
     */
    void SetBattSoc( int8_t* soc ) {
        *data_.battery_soc = ( soc == nullptr ? 100 : *soc );
    }

    /**
     * @brief Add an estimator of the given type
     *
     * @tparam EstimatorToAdd
     */
    template < typename EstimatorToAdd > void AddEstimator() {
        auto* estimator = new EstimatorToAdd();
        estimator->SetData( data_ );
        estimator->Setup();
        estimators_.push_back( estimator );
    }

    /**
     * @brief Remove all estimators of a given type
     *
     * @tparam EstimatorToRemove
     */
    template < typename EstimatorToRemove > void RemoveEstimator() {
        int removed_counter = 0;
        estimators_.erase( std::remove_if( estimators_.begin(), estimators_.end(),
                                           [ &removed_counter ]( GenericEstimator< T >* e ) {
                                               if ( dynamic_cast< EstimatorToRemove* >( e ) ) {
                                                   delete e;
                                                   removed_counter++;
                                                   return true;
                                               }
                                               else {
                                                   return false;
                                               }
                                           } ),
                           estimators_.end() );
    }

    /**
     * @brief Remove all estimators
     *
     */
    void RemoveAllEstimators() {
        for ( auto estimator : estimators_ ) {
            delete estimator;
        }
        estimators_.clear();
    }

    ~StateEstimatorContainer() {
        for ( auto estimator : estimators_ ) {
            delete estimator;
        }
    }

private:
    StateEstimatorData< T >               data_;
    std::vector< GenericEstimator< T >* > estimators_;
    Vec4< T >                             phase_;
    Vec4< T >                             swing_phase_;
    Vec3< T >                             ori_cali_gain_;
    Vec3< T >                             ori_cali_offset_;
    int8_t                                bms_status_;
    int8_t                                battery_soc_;
};

#endif  // STATE_ESTIMATOR_CONTAINER_HPP_
