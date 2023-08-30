#ifndef INTERPOLATION_HPP_
#define INTERPOLATION_HPP_

#include <assert.h>
#include <cmath>
#include <type_traits>

/**
 * @brief Utility functions to interpolate between two points, contain linear cycloid bezier and polynomial.
 *
 */
namespace Interpolate {

/**
 * @brief Linear interpolation between y0 and yf. x is between 0 and 1.
 *
 * @param y0 initial point
 * @param yf final point
 * @param x current iteration/time
 * @return y_t
 */
template < typename y_t, typename x_t > y_t Lerp( y_t y0, y_t yf, x_t x ) {
    static_assert( std::is_floating_point< x_t >::value, "must use floating point value" );
    assert( x >= 0 && x <= 1 );
    return y0 + ( yf - y0 ) * x;
}

/**
 * @brief Cyclodial curve interpolation between y0 and yf. x is between 0 and 1.
 *
 * @param y0 initial point
 * @param yf final point
 * @param x current iteration/time
 * @return y_t
 */
template < typename y_t, typename x_t > y_t Cycloid( y_t y0, y_t yf, x_t x ) {
    static_assert( std::is_floating_point< x_t >::value, "must use floating point value" );
    assert( x >= 0 && x <= 1 );
    y_t y_diff    = yf - y0;
    x_t cyclodial = x - sin( 2 * M_PI * x ) / ( 2 * M_PI );
    return y0 + cyclodial * y_diff;
}

/**
 * @brief First derivative of cyclodial curve interpolation between y0 and yf. x is between 0 and 1.
 *
 * @param y0 initial point
 * @param yf final point
 * @param x current iteration/time
 * @return y_t
 */
template < typename y_t, typename x_t > y_t CycloidFirstDerivative( y_t y0, y_t yf, x_t x ) {
    static_assert( std::is_floating_point< x_t >::value, "must use floating point value" );
    assert( x >= 0 && x <= 1 );
    y_t y_diff    = yf - y0;
    x_t cyclodial = 1 - cos( 2 * M_PI * x );
    return cyclodial * y_diff;
}

/**
 * @brief Second derivative of cyclodial curve interpolation between y0 and yf. x is between 0 and 1.
 *
 * @param y0 initial point
 * @param yf final point
 * @param x current iteration/time
 * @return y_t
 */
template < typename y_t, typename x_t > y_t CycloidSecondDerivative( y_t y0, y_t yf, x_t x ) {
    static_assert( std::is_floating_point< x_t >::value, "must use floating point value" );
    assert( x >= 0 && x <= 1 );
    y_t y_diff    = yf - y0;
    x_t cyclodial = 2 * M_PI * sin( 2 * M_PI * x );
    return cyclodial * y_diff;
}

/**
 * @brief Cubic bezier interpolation between y0 and yf. x is between 0 and 1.
 *
 * @param y0 initial point
 * @param yf final point
 * @param x current iteration/time
 * @return y_t
 */
template < typename y_t, typename x_t > y_t CubicBezier( y_t y0, y_t yf, x_t x ) {
    static_assert( std::is_floating_point< x_t >::value, "must use floating point value" );
    assert( x >= 0 && x <= 1 );
    y_t y_diff = yf - y0;
    x_t bezier = x * x * x + x_t( 3 ) * ( x * x * ( x_t( 1 ) - x ) );
    return y0 + bezier * y_diff;
}

/**
 * @brief First order of cubic bezier interpolation between y0 and yf. x is between 0 and 1
 *
 * @param y0 initial point
 * @param yf final point
 * @param x current iteration/time
 * @return y_t
 */
template < typename y_t, typename x_t > y_t CubicBezierFirstDerivative( y_t y0, y_t yf, x_t x ) {
    static_assert( std::is_floating_point< x_t >::value, "must use floating point value" );
    assert( x >= 0 && x <= 1 );
    y_t y_diff = yf - y0;
    x_t bezier = x_t( 6 ) * x * ( x_t( 1 ) - x );
    return bezier * y_diff;
}

/**
 * @brief Second order of cubic bezier interpolation between y0 and yf. x is between 0 and 1.
 *
 * @param y0 initial point
 * @param yf final point
 * @param x current iteration/time
 * @return y_t
 */
template < typename y_t, typename x_t > y_t CubicBezierSecondDerivative( y_t y0, y_t yf, x_t x ) {
    static_assert( std::is_floating_point< x_t >::value, "must use floating point value" );
    assert( x >= 0 && x <= 1 );
    y_t y_diff = yf - y0;
    x_t bezier = x_t( 6 ) - x_t( 12 ) * x;
    return bezier * y_diff;
}

/**
 * @brief Cubic bezier interpolation of four control points.
 *
 * @param y0 initial control point
 * @param y1 second control point
 * @param y2 third control point
 * @param y3 fourth contol point
 * @param x current iteration/time
 * @return y_t
 */
template < typename y_t, typename x_t > y_t CubicBezierFourCtrlPoint( y_t y0, y_t y1, y_t y2, y_t y3, x_t x ) {
    static_assert( std::is_floating_point< x_t >::value, "must use floating point value" );
    assert( x >= 0 && x <= 1 );
    y_t y00 = y0;
    y_t yf  = y3;
    // y_t bezier = x * x * x*y3 + x_t(3) * (x * x * (x_t(1) - x))*y2 + x_t(3)*x*(x_t(1)-x)*(x_t(1)-x)*y1 + (x_t(1)-x)*(x_t(1)-x)*(x_t(1)-x)*y0;
    y_t bezier = pow( 1 - x, 5 ) * y00 + 5 * ( x * pow( 1 - x, 4 ) ) * y0 + 10 * ( pow( x, 2 ) * pow( 1 - x, 3 ) ) * y1 + 10 * ( pow( x, 3 ) * pow( 1 - x, 2 ) ) * y2
                 + 5 * ( pow( x, 4 ) * pow( 1 - x, 1 ) ) * y3 + pow( x, 5 ) * yf;
    return bezier;
}

/**
 * @brief First derivatice of cubic bezier interpolation of four control points.
 *
 * @param y0 initial control point
 * @param y1 second control point
 * @param y2 third control point
 * @param y3 fourth contol point
 * @param x current iteration/time
 * @return y_t
 */
template < typename y_t, typename x_t > y_t CubicBezierFourCtrlPointFirstDerivative( y_t y0, y_t y1, y_t y2, y_t y3, x_t x ) {
    static_assert( std::is_floating_point< x_t >::value, "must use floating point value" );
    assert( x >= 0 && x <= 1 );
    y_t y00 = y0;
    y_t yf  = y3;
    // y_t bezier = 3* x * x*y3 + (x_t(6)* x  - x_t(9)*x*x)*y2 + (x_t(3)-x_t(12)*x+x_t(9)*x*x)*y1 + (-x_t(3)+x_t(6)*x-x_t(3)*x*x)*y0;
    y_t bezier = -5 * pow( 1 - x, 4 ) * y00 + 5 * ( 1 - 5 * x ) * pow( 1 - x, 3 ) * y0 + 10 * x * pow( 1 - x, 2 ) * ( 2 - 5 * x ) * y1 + 10 * ( pow( x, 2 ) * ( 1 - x ) * ( 3 - 5 * x ) ) * y2
                 + 5 * pow( x, 3 ) * ( 4 - 5 * x ) * y3 + 5 * pow( x, 4 ) * yf;
    return bezier;
}

/**
 * @brief Second derivative of cubic bezier interpolation of four control points.
 *
 * @param y0 initial control point
 * @param y1 second control point
 * @param y2 third control point
 * @param y3 fourth contol point
 * @param x current iteration/time
 * @return y_t
 */
template < typename y_t, typename x_t > y_t CubicBezierFourCtrlPointSecondDerivative( y_t y0, y_t y1, y_t y2, y_t y3, x_t x ) {
    static_assert( std::is_floating_point< x_t >::value, "must use floating point value" );
    assert( x >= 0 && x <= 1 );
    y_t y00 = y0;
    y_t yf  = y3;
    // y_t bezier = 6* x*y3 + (x_t(6)  - x_t(18)*x)*y2 + (-x_t(12)+x_t(18)*x)*y1 + (x_t(6)-x_t(6)*x)*y0;
    y_t bezier = 20 * pow( 1 - x, 3 ) * y00 + 20 * pow( 1 - x, 2 ) * ( -2 + 5 * x ) * y0 + 10 * ( -2 * ( 1 - x ) * ( 2 * x - 5 * x * x ) + pow( 1 - x, 2 ) * ( 2 - 10 * x ) ) * y1
                 + 10 * ( x * ( 2 - 3 * x ) * ( 3 - 5 * x ) - 5 * pow( x, 2 ) * ( 1 - x ) ) * y2 + 5 * ( 3 * pow( x, 2 ) * ( 4 - 5 * x ) - 5 * pow( x, 3 ) ) * y3 + 20 * pow( x, 3 ) * yf;
    return bezier;
}

/**
 * @brief Quintic(five-order) polynomial interpolation between y0 and y1. x is between 0 and 1.
 *
 * @param y0 initial point
 * @param y1 final point
 * @param x current iteration/time
 * @return y_t
 */
template < typename y_t, typename x_t > y_t QuinticPolynomial( y_t y0, y_t y1, x_t x ) {
    static_assert( std::is_floating_point< x_t >::value, "must use floating point value" );
    assert( x >= 0 && x <= 1 );
    y_t k0  = y0;
    y_t k1  = 0 * y0;
    y_t k2  = 0 * y0;
    y_t k3  = 10 * ( y1 - y0 );
    y_t k4  = -15 * ( y1 - y0 );
    y_t k5  = 6 * ( y1 - y0 );
    y_t res = k0 + k1 * x + k2 * pow( x, 2 ) + k3 * pow( x, 3 ) + k4 * pow( x, 4 ) + k5 * pow( x, 5 );
    return res;
}

/**
 * @brief First derivative of quintic(five-order) polynomial interpolation between y0 and y1. x is between 0 and 1.
 *
 * @param y0 initial point
 * @param yf final point
 * @param x current iteration/time
 * @return y_t
 */
template < typename y_t, typename x_t > y_t QuinticPolynomialFirstDerivative( y_t y0, y_t yf, x_t x ) {
    static_assert( std::is_floating_point< x_t >::value, "must use floating point value" );
    assert( x >= 0 && x <= 1 );
    y_t k1  = 0 * y0;
    y_t k2  = 0 * y0;
    y_t k3  = 10 * ( yf - y0 );
    y_t k4  = -15 * ( yf - y0 );
    y_t k5  = 6 * ( yf - y0 );
    y_t res = k1 + 2 * k2 * x + 3 * k3 * pow( x, 2 ) + 4 * k4 * pow( x, 3 ) + 5 * k5 * pow( x, 4 );
    return res;
}

/**
 * @brief Second derivative of quintic(five-order) polynomial interpolation between y0 and yf. x is between 0 and 1.
 *
 * @param y0 initial point
 * @param yf final point
 * @param x current iteration/time
 * @return y_t
 */
template < typename y_t, typename x_t > y_t QuinticPolynomialSecondDerivative( y_t y0, y_t yf, x_t x ) {
    static_assert( std::is_floating_point< x_t >::value, "must use floating point value" );
    assert( x >= 0 && x <= 1 );
    y_t k2  = 0 * y0;
    y_t k3  = 10 * ( yf - y0 );
    y_t k4  = -15 * ( yf - y0 );
    y_t k5  = 6 * ( yf - y0 );
    y_t res = 2 * k2 + 6 * k3 * x + 12 * k4 * pow( x, 2 ) + 20 * k5 * pow( x, 3 );
    return res;
}

/**
 * @brief Septic(seven-order) polynomial interpolation between y0 and yf. x is between 0 and 1.
 *
 * @param y0 initial point
 * @param ym middle point
 * @param yf final point
 * @param x current iteration/time
 * @return y_t
 */
template < typename y_t, typename x_t > y_t SepticPolynomial( y_t y0, y_t ym, y_t yf, x_t x ) {
    static_assert( std::is_floating_point< x_t >::value, "must use floating point value" );
    assert( x >= 0 && x <= 1 );
    y_t k0  = y0;
    y_t k1  = 0 * y0;
    y_t k2  = 0 * y0;
    y_t k3  = 64 * ( ym - y0 ) + 38 * ( yf - y0 );
    y_t k4  = -192 * ( ym - y0 ) - 219 * ( yf - y0 );
    y_t k5  = 192 * ( ym - y0 ) + 450 * ( yf - y0 );
    y_t k6  = -64 * ( ym - y0 ) - 388 * ( yf - y0 );
    y_t k7  = 120 * ( yf - y0 );
    y_t res = k0 + k1 * x + k2 * pow( x, 2 ) + k3 * pow( x, 3 ) + k4 * pow( x, 4 ) + k5 * pow( x, 5 ) + k6 * pow( x, 6 ) + k7 * pow( x, 7 );
    return res;
}

/**
 * @brief First derivative of septic(seven-order) polynomial interpolation between y0 and yf. x is between 0 and 1.
 *
 * @param y0 initial point
 * @param ym middle point
 * @param yf final point
 * @param x current iteration/time
 * @return y_t
 */
template < typename y_t, typename x_t > y_t SepticPolynomialFirstDerivative( y_t y0, y_t ym, y_t yf, x_t x ) {
    static_assert( std::is_floating_point< x_t >::value, "must use floating point value" );
    assert( x >= 0 && x <= 1 );
    y_t k1  = 0 * y0;
    y_t k2  = 0 * y0;
    y_t k3  = 64 * ( ym - y0 ) + 38 * ( yf - y0 );
    y_t k4  = -192 * ( ym - y0 ) - 219 * ( yf - y0 );
    y_t k5  = 192 * ( ym - y0 ) + 450 * ( yf - y0 );
    y_t k6  = -64 * ( ym - y0 ) - 388 * ( yf - y0 );
    y_t k7  = 120 * ( yf - y0 );
    y_t res = k1 + 2 * k2 * x + 3 * k3 * pow( x, 2 ) + 4 * k4 * pow( x, 3 ) + 5 * k5 * pow( x, 4 ) + 6 * k6 * pow( x, 5 ) + 7 * k7 * pow( x, 6 );
    return res;
}

/**
 * @brief Second derivative of septic(seven-order) polynomial interpolation between y0 and yf. x is between 0 and 1.
 *
 * @param y0 initial point
 * @param ym middle point
 * @param yf final point
 * @param x current iteration/time
 * @return y_t
 */
template < typename y_t, typename x_t > y_t SepticPolynomialSecondDerivative( y_t y0, y_t ym, y_t yf, x_t x ) {
    static_assert( std::is_floating_point< x_t >::value, "must use floating point value" );
    assert( x >= 0 && x <= 1 );
    y_t k2  = 0 * y0;
    y_t k3  = 64 * ( ym - y0 ) + 38 * ( yf - y0 );
    y_t k4  = -192 * ( ym - y0 ) - 219 * ( yf - y0 );
    y_t k5  = 192 * ( ym - y0 ) + 450 * ( yf - y0 );
    y_t k6  = -64 * ( ym - y0 ) - 388 * ( yf - y0 );
    y_t k7  = 120 * ( yf - y0 );
    y_t res = 2 * k2 + 6 * k3 * x + 12 * k4 * pow( x, 2 ) + 20 * k5 * pow( x, 3 ) + 30 * k6 * pow( x, 4 ) + 42 * k7 * pow( x, 5 );
    return res;
}

}  // namespace Interpolate

#endif  // INTERPOLATION_HPP_
