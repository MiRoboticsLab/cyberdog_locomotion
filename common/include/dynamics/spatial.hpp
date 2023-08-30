#ifndef SPATIAL_HPP_
#define SPATIAL_HPP_

#include <cmath>
#include <iostream>
#include <type_traits>

#include "math/orientation_tools.hpp"

namespace spatial {
using namespace ori;
enum class JointType { kPrismatic, kRevolute, kFloatingBase, kNothing };

/**
 * @brief Calculate the spatial coordinate transform from A to B where B is rotate by
 *        theta about axis.
 *
 * @param axis the coordinate axis around which to rotate
 * @param theta the rotation angle, in radians
 * @return an SXform<T> object representing the spatial rotation transform
 */
template < typename T > SXform< T > SpatialRotation( CoordinateAxis axis, T theta ) {
    static_assert( std::is_floating_point< T >::value, "must use floating point value" );
    RotMat< T > R                          = CoordinateRotation( axis, theta );
    SXform< T > X                          = SXform< T >::Zero();
    X.template topLeftCorner< 3, 3 >()     = R;
    X.template bottomRightCorner< 3, 3 >() = R;
    return X;
}

/**
 * @brief Calculate the spatial coordinate transform from A to B where B is rotate by
 *        theta about axis.
 *
 * @param axis the coordinate axis around which to rotate
 * @param theta the rotation angle, in radians
 * @param X the transformation matrix to be rotated
 */
template < typename T > void SpatialRotation( CoordinateAxis axis, T theta, SXform< T >& X ) {
    static_assert( std::is_floating_point< T >::value, "must use floating point value" );
    RotMat< T > R = CoordinateRotation( axis, theta );
    X.setZero();
    X.template topLeftCorner< 3, 3 >()     = R;
    X.template bottomRightCorner< 3, 3 >() = R;
    return;
}

/**
 * @brief Compute the spatial motion cross product matrix.
 *        Prefer MotionCrossProduct when possible.
 *
 * @param v input vector, must have 6 rows and 1 column
 * @return the resulting 6x6 matrix
 */
template < typename T > auto MotionCrossMatrix( const Eigen::MatrixBase< T >& v ) {
    static_assert( T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6, "Must have 6x1 vector" );
    Mat6< typename T::Scalar > m;
    m << 0, -v( 2 ), v( 1 ), 0, 0, 0, v( 2 ), 0, -v( 0 ), 0, 0, 0, -v( 1 ), v( 0 ), 0, 0, 0, 0,

        0, -v( 5 ), v( 4 ), 0, -v( 2 ), v( 1 ), v( 5 ), 0, -v( 3 ), v( 2 ), 0, -v( 0 ), -v( 4 ), v( 3 ), 0, -v( 1 ), v( 0 ), 0;
    return m;
}

/**
 * @brief Compute spatial force cross product matrix.
 *        Prefer ForceCrossProduct when possible.
 *
 * @param v the 3D vector input
 * @return the resulting 6x6 matrix
 */
template < typename T > auto ForceCrossMatrix( const Eigen::MatrixBase< T >& v ) {
    Mat6< typename T::Scalar > f;
    f << 0, -v( 2 ), v( 1 ), 0, -v( 5 ), v( 4 ), v( 2 ), 0, -v( 0 ), v( 5 ), 0, -v( 3 ), -v( 1 ), v( 0 ), 0, -v( 4 ), v( 3 ), 0, 0, 0, 0, 0, -v( 2 ), v( 1 ), 0, 0, 0, v( 2 ), 0, -v( 0 ), 0, 0, 0,
        -v( 1 ), v( 0 ), 0;
    return f;
}

/**
 * @brief Compute spatial motion cross product.
 *        Faster than the matrix multiplication version.
 *
 * @param a the first vector
 * @param b the second vector
 * @return the result of the cross product as a 6x1 vector
 */
template < typename T > auto MotionCrossProduct( const Eigen::MatrixBase< T >& a, const Eigen::MatrixBase< T >& b ) {
    static_assert( T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6, "Must have 6x1 vector" );
    SVec< typename T::Scalar > mv;
    mv << a( 1 ) * b( 2 ) - a( 2 ) * b( 1 ), a( 2 ) * b( 0 ) - a( 0 ) * b( 2 ), a( 0 ) * b( 1 ) - a( 1 ) * b( 0 ), a( 1 ) * b( 5 ) - a( 2 ) * b( 4 ) + a( 4 ) * b( 2 ) - a( 5 ) * b( 1 ),
        a( 2 ) * b( 3 ) - a( 0 ) * b( 5 ) - a( 3 ) * b( 2 ) + a( 5 ) * b( 0 ), a( 0 ) * b( 4 ) - a( 1 ) * b( 3 ) + a( 3 ) * b( 1 ) - a( 4 ) * b( 0 );
    return mv;
}

/**
 * @brief Compute spatial force cross product.
 *        Faster than the matrix multiplication version.
 *
 * @param a the first vector
 * @param b the second vector
 * @return the result of the cross product as a 6x1 vector
 */
template < typename T > auto ForceCrossProduct( const Eigen::MatrixBase< T >& a, const Eigen::MatrixBase< T >& b ) {
    static_assert( T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6, "Must have 6x1 vector" );
    SVec< typename T::Scalar > mv;
    mv << b( 2 ) * a( 1 ) - b( 1 ) * a( 2 ) - b( 4 ) * a( 5 ) + b( 5 ) * a( 4 ), b( 0 ) * a( 2 ) - b( 2 ) * a( 0 ) + b( 3 ) * a( 5 ) - b( 5 ) * a( 3 ),
        b( 1 ) * a( 0 ) - b( 0 ) * a( 1 ) - b( 3 ) * a( 4 ) + b( 4 ) * a( 3 ), b( 5 ) * a( 1 ) - b( 4 ) * a( 2 ), b( 3 ) * a( 2 ) - b( 5 ) * a( 0 ), b( 4 ) * a( 0 ) - b( 3 ) * a( 1 );
    return mv;
}

/**
 * @brief Convert a spatial transform to a homogeneous coordinate transformation.
 *
 * @param X the input matrix
 * @return a 4x4 matrix representing the homogeneous transformation
 */
template < typename T > auto SxformToHomogeneous( const Eigen::MatrixBase< T >& X ) {
    static_assert( T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6, "Must have 6x6 matrix" );
    Mat4< typename T::Scalar >   H      = Mat4< typename T::Scalar >::Zero();
    RotMat< typename T::Scalar > R      = X.template topLeftCorner< 3, 3 >();
    Mat3< typename T::Scalar >   skewR  = X.template bottomLeftCorner< 3, 3 >();
    H.template topLeftCorner< 3, 3 >()  = R;
    H.template topRightCorner< 3, 1 >() = MatToSkewVec( skewR * R.transpose() );
    H( 3, 3 )                           = 1;
    return H;
}

/**
 * @brief Convert a homogeneous coordinate transformation to a spatial one.
 *
 * @param H the input matrix
 * @return the resulting special Euclidean transformation matrix in 6x6 matrix form
 */
template < typename T > auto HomogeneousToSXform( const Eigen::MatrixBase< T >& H ) {
    static_assert( T::ColsAtCompileTime == 4 && T::RowsAtCompileTime == 4, "Must have 4x4 matrix" );
    Mat3< typename T::Scalar > R           = H.template topLeftCorner< 3, 3 >();
    Vec3< typename T::Scalar > translate   = H.template topRightCorner< 3, 1 >();
    Mat6< typename T::Scalar > X           = Mat6< typename T::Scalar >::Zero();
    X.template topLeftCorner< 3, 3 >()     = R;
    X.template bottomLeftCorner< 3, 3 >()  = VectorToSkewMat( translate ) * R;
    X.template bottomRightCorner< 3, 3 >() = R;
    return X;
}

/**
 * @brief Create spatial coordinate transformation from rotation and translation.
 *
 * @param R a 3x3 rotation matrix
 * @param r a 3x1 translation vector
 * @return a 4x4 homogeneous transformation matrix
 */
template < typename T, typename T2 > auto CreateSXform( const Eigen::MatrixBase< T >& R, const Eigen::MatrixBase< T2 >& r ) {
    static_assert( T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3, "Must have 3x3 matrix" );
    static_assert( T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3, "Must have 3x1 matrix" );
    Mat6< typename T::Scalar > X           = Mat6< typename T::Scalar >::Zero();
    X.template topLeftCorner< 3, 3 >()     = R;
    X.template bottomRightCorner< 3, 3 >() = R;
    X.template bottomLeftCorner< 3, 3 >()  = -R * VectorToSkewMat( r );
    return X;
}

/**
 * @brief Create spatial coordinate transformation from rotation and translation.
 *
 * @param R a 3x3 rotation matrix
 * @param r a 3x1 translation vecto
 * @param X a 4x4 transformation matrix (SE(3))
 */
template < typename T, typename T2 > void CreateSXform( const Eigen::MatrixBase< T >& R, const Eigen::MatrixBase< T2 >& r, Eigen::MatrixBase< T >& X ) {
    static_assert( T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3, "Must have 3x3 matrix" );
    static_assert( T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3, "Must have 3x1 matrix" );
    X.setZero();
    X.template topLeftCorner< 3, 3 >()     = R;
    X.template bottomRightCorner< 3, 3 >() = R;
    X.template bottomLeftCorner< 3, 3 >()  = -R * VectorToSkewMat( r );
    return;
}

/**
 * @brief Get rotation matrix from spatial transformation.
 *
 * @param X the homogeneous transformation matrix
 * @return the rotation matrix
 */
template < typename T > auto RotationFromSXform( const Eigen::MatrixBase< T >& X ) {
    static_assert( T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6, "Must have 6x6 matrix" );
    RotMat< typename T::Scalar > R = X.template topLeftCorner< 3, 3 >();
    return R;
}

/**
 * @brief Get translation vector from spatial transformation.
 *
 * @param X the input matrix that represents the SE(3) transformation
 * @return the translation vector
 */
template < typename T > auto TranslationFromSXform( const Eigen::MatrixBase< T >& X ) {
    static_assert( T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6, "Must have 6x6 matrix" );
    RotMat< typename T::Scalar > R = RotationFromSXform( X );
    Vec3< typename T::Scalar >   r = -MatToSkewVec( R.transpose() * X.template bottomLeftCorner< 3, 3 >() );
    return r;
}

/**
 * @brief Invert a spatial transformation (much faster than matrix inverse).
 *
 * @param X the input matrix representing the spatial transform
 * @return the inverse of the spatial transform as a SXform
 */
template < typename T > auto InvertSXform( const Eigen::MatrixBase< T >& X ) {
    static_assert( T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6, "Must have 6x6 matrix" );
    RotMat< typename T::Scalar > R    = RotationFromSXform( X );
    Vec3< typename T::Scalar >   r    = -MatToSkewVec( R.transpose() * X.template bottomLeftCorner< 3, 3 >() );
    SXform< typename T::Scalar > Xinv = CreateSXform( R.transpose(), -R * r );
    return Xinv;
}

/**
 * @brief Compute joint motion subspace vector.
 *
 * @param joint the type of joint to create the motion subspace for
 * @param axis the coordinate axis to create the motion subspace for
 * @return a sparse matrix representing the motion subspace
 */
template < typename T > SVec< T > JointMotionSubspace( JointType joint, CoordinateAxis axis ) {
    Vec3< T > v( 0, 0, 0 );
    SVec< T > phi = SVec< T >::Zero();
    if ( axis == CoordinateAxis::X )
        v( 0 ) = 1;
    else if ( axis == CoordinateAxis::Y )
        v( 1 ) = 1;
    else
        v( 2 ) = 1;

    if ( joint == JointType::kPrismatic )
        phi.template bottomLeftCorner< 3, 1 >() = v;
    else if ( joint == JointType::kRevolute )
        phi.template topLeftCorner< 3, 1 >() = v;
    else
        throw std::runtime_error( "Unknown motion subspace" );

    return phi;
}

/**
 * @brief Compute joint transformation.
 *
 * @param joint the type of joint (Revolute or Prismatic)
 * @param axis the axis of rotation or translation
 * @param q the joint angle or displacement
 * @return the 6x6 transformation matrix
 */
template < typename T > Mat6< T > JointXform( JointType joint, CoordinateAxis axis, T q ) {
    Mat6< T > X = Mat6< T >::Zero();
    if ( joint == JointType::kRevolute ) {
        X = SpatialRotation( axis, q );
    }
    else if ( joint == JointType::kPrismatic ) {
        Vec3< T > v( 0, 0, 0 );
        if ( axis == CoordinateAxis::X )
            v( 0 ) = q;
        else if ( axis == CoordinateAxis::Y )
            v( 1 ) = q;
        else if ( axis == CoordinateAxis::Z )
            v( 2 ) = q;

        X = CreateSXform( RotMat< T >::Identity(), v );
    }
    else {
        throw std::runtime_error( "Unknown joint xform\n" );
    }
    return X;
}

/**
 * @brief Compute joint transformation.
 *
 * @param joint the type of joint (Revolute or Prismatic)
 * @param axis the axis of rotation or translation
 * @param q the joint angle or displacement
 * @param X the matrix storing the transformed joint
 */
template < typename T > void JointXform( JointType joint, CoordinateAxis axis, T q, Mat6< T >& X ) {
    X.setZero();
    if ( joint == JointType::kRevolute ) {
        SpatialRotation( axis, q, X );
    }
    else if ( joint == JointType::kPrismatic ) {
        Vec3< T > v( 0, 0 );
        if ( axis == CoordinateAxis::X )
            v( 0 ) = q;
        else if ( axis == CoordinateAxis::Y )
            v( 1 ) = q;
        else if ( axis == CoordinateAxis::Z )
            v( 2 ) = q;

        CreateSXform( RotMat< T >::Identity(), v, X );
    }
    else {
        throw std::runtime_error( "Unknown joint xform\n" );
    }
    return;
}

/**
 * @brief Construct the rotational inertia of a uniform density box with a given mass.
 *
 * @param mass mass of the box
 * @param dims dimensions of the box
 * @return Mat3< typename T::Scalar >
 */
template < typename T > Mat3< typename T::Scalar > RotInertiaOfBox( typename T::Scalar mass, const Eigen::MatrixBase< T >& dims ) {
    static_assert( T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3, "Must have 3x1 vector" );
    Mat3< typename T::Scalar > I = Mat3< typename T::Scalar >::Identity() * dims.norm() * dims.norm();
    for ( int i = 0; i < 3; i++ )
        I( i, i ) -= dims( i ) * dims( i );
    I = I * mass / 12;
    return I;
}

/**
 * @brief Convert from spatial velocity to linear velocity.
 *        Uses spatial velocity at the given point.
 *
 * @param v the input spatial velocity vector
 * @param x the input point in space
 * @return the linear velocity at the given point
 */
template < typename T, typename T2 > auto SpatialToLinearVelocity( const Eigen::MatrixBase< T >& v, const Eigen::MatrixBase< T2 >& x ) {
    static_assert( T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6, "Must have 6x1 vector" );
    static_assert( T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3, "Must have 3x1 vector" );
    Vec3< typename T::Scalar > vsAng   = v.template topLeftCorner< 3, 1 >();
    Vec3< typename T::Scalar > vsLin   = v.template bottomLeftCorner< 3, 1 >();
    Vec3< typename T::Scalar > vLinear = vsLin + vsAng.cross( x );
    return vLinear;
}

/**
 * @brief Convert from spatial velocity to angular velocity.
 *
 * @param v the input spatial velocity vector
 * @return the angular velocity at the given point
 */
template < typename T > auto SpatialToAngularVelocity( const Eigen::MatrixBase< T >& v ) {
    static_assert( T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6, "Must have 6x1 vector" );
    Vec3< typename T::Scalar > vsAng = v.template topLeftCorner< 3, 1 >();
    return vsAng;
}

/**
 * @brief Compute the classical lienear accleeration of a frame given its spatial
 *        acceleration and velocity.
 *
 * @param a the matrix containing the spatial acceleration vector
 * @param v the matrix containing the velocity vector
 * @return the linear acceleration vector in the non-inertial frame of reference
 */
template < typename T, typename T2 > auto SpatialToLinearAcceleration( const Eigen::MatrixBase< T >& a, const Eigen::MatrixBase< T2 >& v ) {
    static_assert( T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6, "Must have 6x1 vector" );
    static_assert( T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 6, "Must have 6x1 vector" );

    Vec3< typename T::Scalar > acc;
    // classical accleration = spatial linear acc + omega x v
    acc = a.template tail< 3 >() + v.template head< 3 >().cross( v.template tail< 3 >() );
    return acc;
}

/**
 * @brief Compute the classical lienear acceleration of a frame given its spatial
 *        acceleration and velocity.
 *
 * @param a the matrix containing the spatial acceleration vector
 * @param v the matrix containing the velocity vector
 * @param x the matrix containing the position vector
 * @return the linear acceleration in the given reference frame
 */
template < typename T, typename T2, typename T3 > auto SpatialToLinearAcceleration( const Eigen::MatrixBase< T >& a, const Eigen::MatrixBase< T2 >& v, const Eigen::MatrixBase< T3 >& x ) {
    static_assert( T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6, "Must have 6x1 vector" );
    static_assert( T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 6, "Must have 6x1 vector" );
    static_assert( T3::ColsAtCompileTime == 1 && T3::RowsAtCompileTime == 3, "Must have 3x1 vector" );

    Vec3< typename T::Scalar > alin_x = SpatialToLinearVelocity( a, x );
    Vec3< typename T::Scalar > vlin_x = SpatialToLinearVelocity( v, x );

    // classical accleration = spatial linear acc + omega x v
    Vec3< typename T::Scalar > acc = alin_x + v.template head< 3 >().cross( vlin_x );
    return acc;
}

/**
 * @brief Apply spatial transformation to a point.
 *
 * @param X the input transformation matrix of size 6x6
 * @param p the input point matrix of size 3x1
 * @return the resulting transformed point matrix of size 3x1
 */
template < typename T, typename T2 > auto SXFormPoint( const Eigen::MatrixBase< T >& X, const Eigen::MatrixBase< T2 >& p ) {
    static_assert( T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6, "Must have 6x6 vector" );
    static_assert( T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3, "Must have 3x1 vector" );

    Mat3< typename T::Scalar > R  = RotationFromSXform( X );
    Vec3< typename T::Scalar > r  = TranslationFromSXform( X );
    Vec3< typename T::Scalar > Xp = R * ( p - r );
    return Xp;
}

/**
 * @brief Convert a force at a point to a spatial force.
 *
 * @param f force
 * @param p point
 * @return the resulting spatial force vector
 */
template < typename T, typename T2 > auto ForceToSpatialForce( const Eigen::MatrixBase< T >& f, const Eigen::MatrixBase< T2 >& p ) {
    static_assert( T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3, "Must have 3x1 vector" );
    static_assert( T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3, "Must have 3x1 vector" );
    SVec< typename T::Scalar > fs;
    fs.template topLeftCorner< 3, 1 >()    = p.cross( f );
    fs.template bottomLeftCorner< 3, 1 >() = f;
    return fs;
}

}  // namespace spatial

#endif  // SPATIAL_HPP_
