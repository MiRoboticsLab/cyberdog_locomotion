#ifndef ORIENTATION_TOOLS_HPP_
#define ORIENTATION_TOOLS_HPP_

#include <cmath>
#include <iostream>
#include <type_traits>

#include "math/math_utilities.hpp"
#include "cpp_types.hpp"

namespace ori {

static constexpr double quaternion_derivative_stabilization = 0.1;

enum class CoordinateAxis { X, Y, Z };

/**
 * @brief Convert radians to degrees.
 *
 * @param rad angle in radians to be converted
 * @return angle in degrees
 */
template < typename T > T Rad2Deg( T rad ) {
    static_assert( std::is_floating_point< T >::value, "must use floating point value" );
    return rad * T( 180 ) / T( M_PI );
}

/**
 * @brief Convert degrees to radians.
 *
 * @param deg angle in degrees to be converted
 * @return angle in radians
 */
template < typename T > T Deg2Rad( T deg ) {
    static_assert( std::is_floating_point< T >::value, "must use floating point value" );
    return deg * T( M_PI ) / T( 180 );
}

/**
 * @brief Compute rotation matrix for coordinate transformation. Note that
 *        CoordinateRotation(CoordinateAxis:X, .1) * v will rotate v by -.1 radians -
 *        this transforms into a frame rotated by .1 radians!.
 *
 * @param axis the coordinate axis to rotate around (X, Y, or Z)
 * @param theta the rotation angle in radians
 * @return the 3x3 rotation matrix
 */
template < typename T > Mat3< T > CoordinateRotation( CoordinateAxis axis, T theta ) {
    static_assert( std::is_floating_point< T >::value, "must use floating point value" );
    T s = std::sin( theta );
    T c = std::cos( theta );

    Mat3< T > rot_mat;

    if ( axis == CoordinateAxis::X ) {
        rot_mat << 1, 0, 0, 0, c, s, 0, -s, c;
    }
    else if ( axis == CoordinateAxis::Y ) {
        rot_mat << c, 0, -s, 0, 1, 0, s, 0, c;
    }
    else if ( axis == CoordinateAxis::Z ) {
        rot_mat << c, s, 0, -s, c, 0, 0, 0, 1;
    }

    return rot_mat;
}

/**
 * @brief Compute the cross-product matrix of a 3D vector.
 *
 * @param v the input vector
 * @return the 3x3 cross-product matrix of the input vector
 */
template < typename T > Mat3< typename T::Scalar > CrossMatrix( const Eigen::MatrixBase< T >& v ) {
    static_assert( T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3, "must have 3x1 vector" );
    Mat3< typename T::Scalar > m;
    m << 0, -v( 2 ), v( 1 ), v( 2 ), 0, -v( 0 ), -v( 1 ), v( 0 ), 0;
    return m;
}

/**
 * @brief Go from rpy to rotation matrix.
 *
 * @param v the input vector of rpy angles
 * @return the 3x3 rotation matrix
 */
template < typename T > Mat3< typename T::Scalar > RpyToRotMat( const Eigen::MatrixBase< T >& v ) {
    static_assert( T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3, "must have 3x1 vector" );
    Mat3< typename T::Scalar > m = CoordinateRotation( CoordinateAxis::X, v[ 0 ] ) * CoordinateRotation( CoordinateAxis::Y, v[ 1 ] ) * CoordinateRotation( CoordinateAxis::Z, v[ 2 ] );
    return m;
}

/**
 * @brief Convert a 3x1 vector to a skew-symmetric 3x3 matri.
 *
 * @param v the input vector
 * @return the skew-symmetric matrix
 */
template < typename T > Mat3< typename T::Scalar > VectorToSkewMat( const Eigen::MatrixBase< T >& v ) {
    static_assert( T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3, "Must have 3x1 matrix" );
    Mat3< typename T::Scalar > m;
    m << 0, -v[ 2 ], v[ 1 ], v[ 2 ], 0, -v[ 0 ], -v[ 1 ], v[ 0 ], 0;
    return m;
}

/**
 * @brief Put the skew-symmetric component of 3x3 matrix m into a 3x1 vector.
 *
 * @param m the input 3x3 skew-symmetric matrix
 * @return the equivalent 3D vector
 */
template < typename T > Vec3< typename T::Scalar > MatToSkewVec( const Eigen::MatrixBase< T >& m ) {
    static_assert( T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3, "Must have 3x3 matrix" );
    return 0.5 * Vec3< typename T::Scalar >( m( 2, 1 ) - m( 1, 2 ), m( 0, 2 ) - m( 2, 0 ), ( m( 1, 0 ) - m( 0, 1 ) ) );
}

/**
 * @brief Convert a coordinate transformation matrix to an orientation quaternion.
 *
 * @param r1 input rotation matrix
 * @return the resulting quaternion
 */
template < typename T > Quat< typename T::Scalar > RotationMatrixToQuaternion( const Eigen::MatrixBase< T >& r1 ) {
    static_assert( T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3, "Must have 3x3 matrix" );
    Quat< typename T::Scalar > q;
    Mat3< typename T::Scalar > r  = r1.transpose();
    typename T::Scalar         tr = r.trace();
    if ( tr > 0.0 ) {
        typename T::Scalar s = sqrt( tr + 1.0 ) * 2.0;
        q( 0 )               = 0.25 * s;
        q( 1 )               = ( r( 2, 1 ) - r( 1, 2 ) ) / s;
        q( 2 )               = ( r( 0, 2 ) - r( 2, 0 ) ) / s;
        q( 3 )               = ( r( 1, 0 ) - r( 0, 1 ) ) / s;
    }
    else if ( ( r( 0, 0 ) > r( 1, 1 ) ) && ( r( 0, 0 ) > r( 2, 2 ) ) ) {
        typename T::Scalar s = sqrt( 1.0 + r( 0, 0 ) - r( 1, 1 ) - r( 2, 2 ) ) * 2.0;
        q( 0 )               = ( r( 2, 1 ) - r( 1, 2 ) ) / s;
        q( 1 )               = 0.25 * s;
        q( 2 )               = ( r( 0, 1 ) + r( 1, 0 ) ) / s;
        q( 3 )               = ( r( 0, 2 ) + r( 2, 0 ) ) / s;
    }
    else if ( r( 1, 1 ) > r( 2, 2 ) ) {
        typename T::Scalar s = sqrt( 1.0 + r( 1, 1 ) - r( 0, 0 ) - r( 2, 2 ) ) * 2.0;
        q( 0 )               = ( r( 0, 2 ) - r( 2, 0 ) ) / s;
        q( 1 )               = ( r( 0, 1 ) + r( 1, 0 ) ) / s;
        q( 2 )               = 0.25 * s;
        q( 3 )               = ( r( 1, 2 ) + r( 2, 1 ) ) / s;
    }
    else {
        typename T::Scalar s = sqrt( 1.0 + r( 2, 2 ) - r( 0, 0 ) - r( 1, 1 ) ) * 2.0;
        q( 0 )               = ( r( 1, 0 ) - r( 0, 1 ) ) / s;
        q( 1 )               = ( r( 0, 2 ) + r( 2, 0 ) ) / s;
        q( 2 )               = ( r( 1, 2 ) + r( 2, 1 ) ) / s;
        q( 3 )               = 0.25 * s;
    }
    return q;
}

/**
 * @brief Convert a quaternion to a rotation matrix. This matrix represents a
 *        coordinate transformation into the frame which has the orientation specified
 *        by the quaternion.
 *
 * @param q the quaternion to convert
 * @return the 3x3 rotation matrix
 */
template < typename T > Mat3< typename T::Scalar > QuaternionToRotationMatrix( const Eigen::MatrixBase< T >& q ) {
    static_assert( T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4, "Must have 4x1 quat" );
    typename T::Scalar e0 = q( 0 );
    typename T::Scalar e1 = q( 1 );
    typename T::Scalar e2 = q( 2 );
    typename T::Scalar e3 = q( 3 );

    Mat3< typename T::Scalar > rot_mat;

    rot_mat << 1 - 2 * ( e2 * e2 + e3 * e3 ), 2 * ( e1 * e2 - e0 * e3 ), 2 * ( e1 * e3 + e0 * e2 ), 2 * ( e1 * e2 + e0 * e3 ), 1 - 2 * ( e1 * e1 + e3 * e3 ), 2 * ( e2 * e3 - e0 * e1 ),
        2 * ( e1 * e3 - e0 * e2 ), 2 * ( e2 * e3 + e0 * e1 ), 1 - 2 * ( e1 * e1 + e2 * e2 );
    rot_mat.transposeInPlace();
    return rot_mat;
}

/**
 * @brief Convert a quaternion to RPY. Uses ZYX order (yaw-pitch-roll), but returns
 *        angles in (roll, pitch, yaw).
 *
 * @param q the input quaternion
 * @return the RPY angles in a 3D vector
 */
template < typename T > Vec3< typename T::Scalar > QuatToRPY( const Eigen::MatrixBase< T >& q ) {
    static_assert( T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4, "Must have 4x1 quat" );
    Vec3< typename T::Scalar > rpy;
    typename T::Scalar         as = std::min( -2. * ( q[ 1 ] * q[ 3 ] - q[ 0 ] * q[ 2 ] ), .99999 );
    rpy( 2 )                      = std::atan2( 2 * ( q[ 1 ] * q[ 2 ] + q[ 0 ] * q[ 3 ] ), Square( q[ 0 ] ) + Square( q[ 1 ] ) - Square( q[ 2 ] ) - Square( q[ 3 ] ) );
    rpy( 1 )                      = std::asin( as );
    rpy( 0 )                      = std::atan2( 2 * ( q[ 2 ] * q[ 3 ] + q[ 0 ] * q[ 1 ] ), Square( q[ 0 ] ) - Square( q[ 1 ] ) - Square( q[ 2 ] ) + Square( q[ 3 ] ) );
    return rpy;
}

/**
 * @brief Convert RPY to a quaternion.
 *
 * @param rpy the roll, pitch, and yaw angles as a 3x1 vector
 * @return the quaternion representing the rotation
 */
template < typename T > Quat< typename T::Scalar > RpyToQuat( const Eigen::MatrixBase< T >& rpy ) {
    static_assert( T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3, "Must have 3x1 vec" );
    Mat3< typename T::Scalar > rot_mat = RpyToRotMat( rpy );
    Quat< typename T::Scalar > q       = RotationMatrixToQuaternion( rot_mat );
    return q;
}

/**
 * @brief Convert a quaternion to SO(3).
 *
 * @param q the input quaternion
 * @return the 3x1 SO(3) rotation vector
 */
template < typename T > Vec3< typename T::Scalar > QuatToso3( const Eigen::MatrixBase< T >& q ) {
    static_assert( T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4, "Must have 4x1 quat" );
    Vec3< typename T::Scalar > so3;
    typename T::Scalar         theta = 2. * std::acos( q[ 0 ] );
    so3[ 0 ]                         = theta * q[ 1 ] / std::sin( theta / 2. );
    so3[ 1 ]                         = theta * q[ 2 ] / std::sin( theta / 2. );
    so3[ 2 ]                         = theta * q[ 3 ] / std::sin( theta / 2. );
    return so3;
}

/**
 * @brief Convert a rotation matrix to RPY.
 *
 * @param rot_mat the 3x3 rotation matrix
 * @return the rpy angles in radians
 */
template < typename T > Vec3< typename T::Scalar > RotationMatrixToRPY( const Eigen::MatrixBase< T >& rot_mat ) {
    static_assert( T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3, "Must have 3x3 matrix" );
    Quat< typename T::Scalar > q   = RotationMatrixToQuaternion( rot_mat );
    Vec3< typename T::Scalar > rpy = QuatToRPY( q );
    return rpy;
}

/**
 * @brief Quaternion derivative calculation, like rqd(q, omega) in MATLAB.
 *        the omega is expressed in body frame.
 *
 * @param q the quaternion to differentiate
 * @param omega the angular velocity vector
 * @return the derivative of the quaternion
 */
template < typename T, typename T2 > Quat< typename T::Scalar > QuatDerivative( const Eigen::MatrixBase< T >& q, const Eigen::MatrixBase< T2 >& omega ) {
    static_assert( T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4, "Must have 4x1 quat" );
    static_assert( T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3, "Must have 3x1 omega" );
    // first case in rqd
    Mat4< typename T::Scalar > quat_deriv;
    quat_deriv << q[ 0 ], -q[ 1 ], -q[ 2 ], -q[ 3 ], q[ 1 ], q[ 0 ], -q[ 3 ], q[ 2 ], q[ 2 ], q[ 3 ], q[ 0 ], -q[ 1 ], q[ 3 ], -q[ 2 ], q[ 1 ], q[ 0 ];

    Quat< typename T::Scalar > qq( quaternion_derivative_stabilization * omega.norm() * ( 1 - q.norm() ), omega[ 0 ], omega[ 1 ], omega[ 2 ] );
    Quat< typename T::Scalar > dq = 0.5 * quat_deriv * qq;
    return dq;
}

/**
 * @brief Compute the product of two quaternions.
 *
 * @param q1 the first quaternion
 * @param q2 the second quaternion
 * @return the product of the two quaternions
 */
template < typename T > Quat< typename T::Scalar > QuatProduct( const Eigen::MatrixBase< T >& q1, const Eigen::MatrixBase< T >& q2 ) {
    typename T::Scalar         r1 = q1[ 0 ];
    typename T::Scalar         r2 = q2[ 0 ];
    Vec3< typename T::Scalar > v1( q1[ 1 ], q1[ 2 ], q1[ 3 ] );
    Vec3< typename T::Scalar > v2( q2[ 1 ], q2[ 2 ], q2[ 3 ] );

    typename T::Scalar         r = r1 * r2 - v1.dot( v2 );
    Vec3< typename T::Scalar > v = r1 * v2 + r2 * v1 + v1.cross( v2 );
    Quat< typename T::Scalar > q( r, v[ 0 ], v[ 1 ], v[ 2 ] );
    return q;
}

/**
 * @brief Integrate a quaternion using the given angular velocity and time step.
 *
 * @param quat the input quaternion
 * @param omega the angular velocity vector
 * @param dt the time step
 * @return the new quaternion
 */
template < typename T, typename T2, typename T3 > Quat< typename T::Scalar > IntegrateQuat( const Eigen::MatrixBase< T >& quat, const Eigen::MatrixBase< T2 >& omega, T3 dt ) {
    static_assert( T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4, "Must have 4x1 quat" );
    static_assert( T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3, "Must have 3x1 omega" );
    Vec3< typename T::Scalar > axis;
    typename T::Scalar         ang = omega.norm();
    if ( ang > 0 ) {
        axis = omega / ang;
    }
    else {
        axis = Vec3< typename T::Scalar >( 1, 0, 0 );
    }

    ang *= dt;
    Vec3< typename T::Scalar > ee = std::sin( ang / 2 ) * axis;
    Quat< typename T::Scalar > quat_d( std::cos( ang / 2 ), ee[ 0 ], ee[ 1 ], ee[ 2 ] );

    Quat< typename T::Scalar > quat_new = QuatProduct( quat_d, quat );
    quat_new                            = quat_new / quat_new.norm();
    return quat_new;
}

/**
 * @brief Integrate a quaternion using an implicit Euler method.
 *
 * @param quat the initial quaternion
 * @param omega the angular velocity vector
 * @param dt the time step
 * @return the integrated quaternion
 */
template < typename T, typename T2, typename T3 > Quat< typename T::Scalar > IntegrateQuatImplicit( const Eigen::MatrixBase< T >& quat, const Eigen::MatrixBase< T2 >& omega, T3 dt ) {
    static_assert( T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4, "Must have 4x1 quat" );
    static_assert( T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3, "Must have 3x1 omega" );
    Vec3< typename T::Scalar > axis;
    typename T::Scalar         ang = omega.norm();
    if ( ang > 0 ) {
        axis = omega / ang;
    }
    else {
        axis = Vec3< typename T::Scalar >( 1, 0, 0 );
    }

    ang *= dt;
    Vec3< typename T::Scalar > ee = std::sin( ang / 2 ) * axis;
    Quat< typename T::Scalar > quat_d( std::cos( ang / 2 ), ee[ 0 ], ee[ 1 ], ee[ 2 ] );

    Quat< typename T::Scalar > quat_new = QuatProduct( quat, quat_d );
    quat_new                            = quat_new / quat_new.norm();
    return quat_new;
}

/**
 * @brief Convert a quaternion to a 3D rotation vector.
 *
 * @param quat the quaternion to convert
 * @param so3 the output 3D rotation vector in the axis-angle representation
 */
template < typename T > void QuaternionToso3( const Quat< T > quat, Vec3< T >& so3 ) {
    so3[ 0 ] = quat[ 1 ];
    so3[ 1 ] = quat[ 2 ];
    so3[ 2 ] = quat[ 3 ];

    T theta = 2.0 * asin( sqrt( so3[ 0 ] * so3[ 0 ] + so3[ 1 ] * so3[ 1 ] + so3[ 2 ] * so3[ 2 ] ) );

    if ( fabs( theta ) < 0.0000001 ) {
        so3.setZero();
        return;
    }
    so3 /= sin( theta / 2.0 );
    so3 *= theta;
}

/**
 * @brief Convert a 3D rotation vector to a quaternion.
 *
 * @param so3 the 3D rotation vector
 * @return the quaternion that represents the same rotation as the input vector
 */
template < typename T > Quat< T > So3ToQuat( Vec3< T >& so3 ) {
    Quat< T > quat;

    T theta = sqrt( so3[ 0 ] * so3[ 0 ] + so3[ 1 ] * so3[ 1 ] + so3[ 2 ] * so3[ 2 ] );

    if ( fabs( theta ) < 1.e-6 ) {
        quat.setZero();
        quat[ 0 ] = 1.;
        return quat;
    }
    quat[ 0 ] = cos( theta / 2. );
    quat[ 1 ] = so3[ 0 ] / theta * sin( theta / 2. );
    quat[ 2 ] = so3[ 1 ] / theta * sin( theta / 2. );
    quat[ 3 ] = so3[ 2 ] / theta * sin( theta / 2. );
    return quat;
}
}  // namespace ori

#endif  // ORIENTATION_TOOLS_HPP_
