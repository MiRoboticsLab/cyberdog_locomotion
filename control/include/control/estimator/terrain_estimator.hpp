#ifndef TERRAIN_ESTIMATOR_HPP_
#define TERRAIN_ESTIMATOR_HPP_

#include "controllers/state_estimator_container.hpp"

/**
 * @brief Use foot pos in global frame as input to estimate terrain equation in global frame.
 *
 */
template < typename T > class TerrainEstimator : public GenericEstimator< T > {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TerrainEstimator();
    virtual void Run();
    virtual void Setup();

private:
    Eigen::Matrix< float, Eigen::Dynamic, Eigen::Dynamic > W_plane_;
    Eigen::Matrix< float, Eigen::Dynamic, Eigen::Dynamic > W_plane_inv_;
    Eigen::Matrix< float, 4, 1 >                           z_feet_;
    Eigen::Matrix< float, 3, 1 >                           a_plane_;

    Eigen::Matrix< float, 3, 3 > R_plane_;
};

#endif  // TERRAIN_ESTIMATOR_HPP_