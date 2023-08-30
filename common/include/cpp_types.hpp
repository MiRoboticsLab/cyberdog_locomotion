#ifndef CPP_TYPES_HPP_
#define CPP_TYPES_HPP_

// TODO: WE MUST COMMIT IT
#if defined( __APPLE__ )
#undef __ARM_NEON__
#undef __ARM_NEON
#endif

#include <vector>

#include <Eigen/Dense>

#include "c_types.h"

/**
 * @brief Common types that are only valid in C++
 *
 * This file contains types which are only used in C++ code. This includes
 * Eigen types, template types, aliases, ...
 */
// Rotation Matrix
template < typename T > using RotMat = typename Eigen::Matrix< T, 3, 3 >;

// 2x1 Vector
template < typename T > using Vec2 = typename Eigen::Matrix< T, 2, 1 >;

// 3x1 Vector
template < typename T > using Vec3 = typename Eigen::Matrix< T, 3, 1 >;

// 4x1 Vector
template < typename T > using Vec4 = typename Eigen::Matrix< T, 4, 1 >;

// 6x1 Vector
template < typename T > using Vec6 = Eigen::Matrix< T, 6, 1 >;

// 8x1 Vector
template < typename T > using Vec8 = Eigen::Matrix< T, 8, 1 >;

// 9x1 Vector
template < typename T > using Vec9 = Eigen::Matrix< T, 9, 1 >;

// 10x1 Vector
template < typename T > using Vec10 = Eigen::Matrix< T, 10, 1 >;

// 12x1 Vector
template < typename T > using Vec12 = Eigen::Matrix< T, 12, 1 >;

// 18x1 Vector
template < typename T > using Vec18 = Eigen::Matrix< T, 18, 1 >;

// 24x1 vector
template < typename T > using Vec24 = Eigen::Matrix< T, 24, 1 >;

// 28x1 vector
template < typename T > using Vec28 = Eigen::Matrix< T, 28, 1 >;

template < typename T, int Rows > using VecX = Eigen::Matrix< T, Rows, 1 >;

// 3x3 Matrix
template < typename T > using Mat3 = typename Eigen::Matrix< T, 3, 3 >;

// 4x1 Vector
template < typename T > using Quat = typename Eigen::Matrix< T, 4, 1 >;

// Spatial Vector (6x1, all subspaces)
template < typename T > using SVec = typename Eigen::Matrix< T, 6, 1 >;

// Spatial Transform (6x6)
template < typename T > using SXform = typename Eigen::Matrix< T, 6, 6 >;

// 6x6 Matrix
template < typename T > using Mat6 = typename Eigen::Matrix< T, 6, 6 >;

// 12x12 Matrix
template < typename T > using Mat12 = typename Eigen::Matrix< T, 12, 12 >;

// 18x18 Matrix
template < typename T > using Mat18 = Eigen::Matrix< T, 18, 18 >;

// 24x12 Matrix
template < typename T > using Mat2412 = Eigen::Matrix< T, 24, 12 >;

// 28x28 Matrix
template < typename T > using Mat28 = Eigen::Matrix< T, 28, 28 >;

// 3x4 Matrix
template < typename T > using Mat34 = Eigen::Matrix< T, 3, 4 >;

// 3x4 Matrix
template < typename T > using Mat23 = Eigen::Matrix< T, 2, 3 >;

// 4x4 Matrix
template < typename T > using Mat4 = typename Eigen::Matrix< T, 4, 4 >;

// 10x1 Vector
template < typename T > using MassProperties = typename Eigen::Matrix< T, 10, 1 >;

// Dynamically sized vector
template < typename T > using DVec = typename Eigen::Matrix< T, Eigen::Dynamic, 1 >;

// Dynamically sized matrix
template < typename T > using DMat = typename Eigen::Matrix< T, Eigen::Dynamic, Eigen::Dynamic >;

// Dynamically sized matrix with spatial vector columns
template < typename T > using D6Mat = typename Eigen::Matrix< T, 6, Eigen::Dynamic >;

// Dynamically sized matrix with cartesian vector columns
template < typename T > using D3Mat = typename Eigen::Matrix< T, 3, Eigen::Dynamic >;

// std::vector (a list) of Eigen things
template < typename T > using vectorAligned = typename std::vector< T, Eigen::aligned_allocator< T > >;

enum class RobotType { CYBERDOG, CYBERDOG2 };

enum class RobotAppearanceType { CURVED, ANGULAR };

// #define FOR_COLORED_SKIN_BOT
/**
 * @brief the configuration parameters of main function, like robot type,
 * run in simulation or on real robot, control parameters loaded from file
 * or set online
 *
 */
struct MasterConfig {
    RobotType robot;
    bool      simulated      = false;
    bool      load_from_file = false;
};

#endif  // CPP_TYPES_HPP_
