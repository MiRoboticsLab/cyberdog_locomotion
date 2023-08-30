#ifndef SPATIAL_INERTIA_HPP_
#define SPATIAL_INERTIA_HPP_

#include <cmath>
#include <iostream>
#include <type_traits>

#include "dynamics/spatial.hpp"
#include "math/orientation_tools.hpp"

using namespace ori;
using namespace spatial;

/**
 * @brief Representation of Rigid Body Inertia as a 6x6 Spatial Inertia Tensor
 *
 */
template < typename T > class SpatialInertia {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct spatial inertia from mass, center of mass, and 3x3 rotational inertia.
     *
     * @param mass mass of the object
     * @param com center of mass
     * @param inertia 3x3 rotational inertia
     */
    SpatialInertia( T mass, const Vec3< T >& com, const Mat3< T >& inertia ) {
        Mat3< T > c_skew                              = VectorToSkewMat( com );
        inertia_.template topLeftCorner< 3, 3 >()     = inertia + mass * c_skew * c_skew.transpose();
        inertia_.template topRightCorner< 3, 3 >()    = mass * c_skew;
        inertia_.template bottomLeftCorner< 3, 3 >()  = mass * c_skew.transpose();
        inertia_.template bottomRightCorner< 3, 3 >() = mass * Mat3< T >::Identity();
    }

    /**
     * @brief Construct spatial inertia from 6x6 matrix.
     *
     * @param inertia 3x3 rotational inertia
     */
    explicit SpatialInertia( const Mat6< T >& inertia ) {
        inertia_ = inertia;
    }

    /**
     * @brief If no argument is given, zero.
     *
     */
    SpatialInertia() {
        inertia_ = Mat6< T >::Zero();
    }

    /**
     * @brief Construct spatial inertia from mass property vector.
     *
     * @param a mass properties of the object
     */
    explicit SpatialInertia( const MassProperties< T >& a ) {
        inertia_( 0, 0 )                              = a( 4 );
        inertia_( 0, 1 )                              = a( 9 );
        inertia_( 0, 2 )                              = a( 8 );
        inertia_( 1, 0 )                              = a( 9 );
        inertia_( 1, 1 )                              = a( 5 );
        inertia_( 1, 2 )                              = a( 7 );
        inertia_( 2, 0 )                              = a( 8 );
        inertia_( 2, 1 )                              = a( 7 );
        inertia_( 2, 2 )                              = a( 6 );
        Mat3< T > c_skew                              = VectorToSkewMat( Vec3< T >( a( 1 ), a( 2 ), a( 3 ) ) );
        inertia_.template topRightCorner< 3, 3 >()    = c_skew;
        inertia_.template bottomLeftCorner< 3, 3 >()  = c_skew.transpose();
        inertia_.template bottomRightCorner< 3, 3 >() = a( 0 ) * Mat3< T >::Identity();
    }

    /**
     * @brief Construct spatial inertia from pseudo-inertia. This is described in
     *        Linear Matrix Inequalities for Physically Consistent Inertial Parameter
     *        Identification: A Statistical Perspective on the Mass Distribution, by
     *        Wensing, Kim, Slotine.
     *
     * @param P the homogeneous transformation matrix
     */
    explicit SpatialInertia( const Mat4< T >& P ) {
        Mat6< T > I;
        T         m                            = P( 3, 3 );
        Vec3< T > h                            = P.template topRightCorner< 3, 1 >();
        Mat3< T > E                            = P.template topLeftCorner< 3, 3 >();
        Mat3< T > Ibar                         = E.trace() * Mat3< T >::Identity() - E;
        I.template topLeftCorner< 3, 3 >()     = Ibar;
        I.template topRightCorner< 3, 3 >()    = VectorToSkewMat( h );
        I.template bottomLeftCorner< 3, 3 >()  = VectorToSkewMat( h ).transpose();
        I.template bottomRightCorner< 3, 3 >() = m * Mat3< T >::Identity();
        inertia_                               = I;
    }

    /**
     * @brief Convert spatial inertia to mass property vector.
     *
     * @return MassProperties vector representation of the SpatialInertia object
     */
    MassProperties< T > AsMassPropertyVector() {
        MassProperties< T > a;
        Vec3< T >           h = MatToSkewVec( inertia_.template topRightCorner< 3, 3 >() );
        a << inertia_( 5, 5 ), h( 0 ), h( 1 ), h( 2 ), inertia_( 0, 0 ), inertia_( 1, 1 ), inertia_( 2, 2 ), inertia_( 2, 1 ), inertia_( 2, 0 ), inertia_( 1, 0 );
        return a;
    }

    /**
     * @brief Get 6x6 spatial inertia.
     *
     * @return spatial inertia
     */
    const Mat6< T >& GetMatrix() const {
        return inertia_;
    }

    /**
     * @brief Set spatial inertia.
     *
     * @param mat the matrix to set the spatial inertia tensor to
     */
    void SetMatrix( const Mat6< T >& mat ) {
        inertia_ = mat;
    }

    /**
     * @brief Adds the given matrix to the current spatial inertia tensor.
     *
     * @param mat the matrix to add to the spatial inertia tensor
     */
    void AddMatrix( const Mat6< T >& mat ) {
        inertia_ += mat;
    }

    /**
     * @brief Get mass
     *
     * @return mass
     */
    T GetMass() {
        return inertia_( 5, 5 );
    }

    /**
     * @brief Get center of mass location.
     *
     * @return center of mass location
     */
    Vec3< T > GetCOM() {
        T         m       = GetMass();
        Mat3< T > mc_skew = inertia_.template topRightCorner< 3, 3 >();
        Vec3< T > com     = MatToSkewVec( mc_skew ) / m;
        return com;
    }

    /**
     * @brief Get 3x3 rotational inertia.
     *
     * @return rotational inertia
     */
    Mat3< T > GetInertiaTensor() {
        T         m       = GetMass();
        Mat3< T > mc_skew = inertia_.template topRightCorner< 3, 3 >();
        Mat3< T > I_rot   = inertia_.template topLeftCorner< 3, 3 >() - mc_skew * mc_skew.transpose() / m;
        return I_rot;
    }

    /**
     * @brief Convert to 4x4 pseudo-inertia matrix.  This is described in
     *        Linear Matrix Inequalities for Physically Consistent Inertial Parameter
     *        Identification: A Statistical Perspective on the Mass Distribution, by
     *        Wensing, Kim, Slotine
     *
     * @return pseudo-inertia matrix
     */
    Mat4< T > GetPseudoInertia() {
        Vec3< T > h    = MatToSkewVec( inertia_.template topRightCorner< 3, 3 >() );
        Mat3< T > Ibar = inertia_.template topLeftCorner< 3, 3 >();
        T         m    = inertia_( 5, 5 );
        Mat4< T > P;
        P.template topLeftCorner< 3, 3 >()    = 0.5 * Ibar.trace() * Mat3< T >::Identity() - Ibar;
        P.template topRightCorner< 3, 1 >()   = h;
        P.template bottomLeftCorner< 1, 3 >() = h.transpose();
        P( 3, 3 )                             = m;
        return P;
    }

    /**
     * @brief Flip inertia matrix around an axis. This isn't efficient, but it works!
     *
     * @param axis the axis to flip the tensor along
     * @return the new spatial inertia tensor after flipping
     */
    SpatialInertia FlipAlongAxis( CoordinateAxis axis ) {
        Mat4< T > P = GetPseudoInertia();
        Mat4< T > X = Mat4< T >::Identity();
        if ( axis == CoordinateAxis::X )
            X( 0, 0 ) = -1;
        else if ( axis == CoordinateAxis::Y )
            X( 1, 1 ) = -1;
        else if ( axis == CoordinateAxis::Z )
            X( 2, 2 ) = -1;
        P = X * P * X;
        return SpatialInertia( P );
    }

private:
    Mat6< T > inertia_;
};

#endif  // SPATIAL_INERTIA_HPP_
