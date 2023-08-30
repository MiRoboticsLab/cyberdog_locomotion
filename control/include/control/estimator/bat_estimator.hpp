#ifndef BAT_ESTIMATOR_HPP_
#define BAT_ESTIMATOR_HPP_

#include "lcm/lcm-cpp.hpp"

#include "controllers/state_estimator_container.hpp"
#include "header/lcm_type/bms_response_lcmt.hpp"
#include "utilities/toolkit.hpp"

/**
 * @brief Battery Estimation Algorithms
 *
 */
template < typename T > class BatEstimator : public GenericEstimator< T > {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BatEstimator();
    virtual void Run();
    virtual void Setup();

    void HandleBmsLcm( const lcm::ReceiveBuffer* buf, const std::string& channel, const bms_response_lcmt* msg );

private:
    lcm::LCM bms_lcm_;
    bool     low_bat_     = false;
    bool     is_charging_ = false;
    uint32_t iter_        = 0;
};

/**
 * @brief "Cheater" battery info which will return the fake battery info
 *  when running in simulation.
 *
 */
template < typename T > class CheaterBatEstimator : public GenericEstimator< T > {
public:
    virtual void Run();
    virtual void Setup() {}
};

#endif  // BAT_ESTIMATOR_HPP_
