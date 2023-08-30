#ifndef MATH_UTILITIES_HPP_
#define MATH_UTILITIES_HPP_

#include <Eigen/Dense>

/**
 * @brief Square a number.
 *
 * @param a number
 * @return the square of the input number
 */
template < typename T > T Square( T a ) {
    return a * a;
}

/**
 * @brief Check if two eigen matrices are approximately equal.
 *
 * @param a the first matrix to compare
 * @param b the second matrix to compare
 * @param threshold the allowed threshold of error
 * @return true if the matrices a and b are approximately equal within the given threshold
 * @return false if the matrices a and b are not approximately equal within the given threshold
 */
template < typename T, typename T2 > bool AlmostEqual( const Eigen::MatrixBase< T >& a, const Eigen::MatrixBase< T >& b, T2 threshold ) {
    long rows = T::RowsAtCompileTime;
    long cols = T::ColsAtCompileTime;

    if ( T::RowsAtCompileTime == Eigen::Dynamic || T::ColsAtCompileTime == Eigen::Dynamic ) {
        assert( a.rows() == b.rows() );
        assert( a.cols() == b.cols() );
        rows = a.rows();
        cols = a.cols();
    }

    for ( long i = 0; i < rows; i++ ) {
        for ( long j = 0; j < cols; j++ ) {
            T2 error = std::abs( a( i, j ) - b( i, j ) );
            if ( error >= threshold )
                return false;
        }
    }
    return true;
}

#endif  // MATH_UTILITIES_HPP_
