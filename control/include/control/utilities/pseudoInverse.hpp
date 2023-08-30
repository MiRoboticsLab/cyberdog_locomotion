#ifndef PSEUDOINVERSE_HPP_
#define PSEUDOINVERSE_HPP_

#include <stdio.h>

#include <Eigen/LU>
#include <Eigen/SVD>

/**
 * @brief Compute the pseudo inverse of a matrix by svd.
 *
 * @param input_matrix input matrix
 * @param sigmaThreshold threshold for singular values being zero
 * @param inverse_matrix output matrix
 */
template < typename T > void PseudoInverse( DMat< T > const& input_matrix, double sigmaThreshold, DMat< T >& inverse_matrix ) {
    if ( ( 1 == input_matrix.rows() ) && ( 1 == input_matrix.cols() ) ) {
        inverse_matrix.resize( 1, 1 );
        if ( input_matrix.coeff( 0, 0 ) > sigmaThreshold ) {
            inverse_matrix.coeffRef( 0, 0 ) = 1.0 / input_matrix.coeff( 0, 0 );
        }
        else {
            inverse_matrix.coeffRef( 0, 0 ) = 0.0;
        }
        return;
    }

    Eigen::JacobiSVD< DMat< T > > svd( input_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV );
    // not sure if we need to svd.sort()... probably not
    int const num_rows( svd.singularValues().rows() );
    DMat< T > inverse_singular_value;
    inverse_singular_value = DMat< T >::Zero( num_rows, num_rows );
    for ( int ii( 0 ); ii < num_rows; ++ii ) {
        if ( svd.singularValues().coeff( ii ) > sigmaThreshold ) {
            inverse_singular_value.coeffRef( ii, ii ) = 1.0 / svd.singularValues().coeff( ii );
        }
        else {
            // inverse_singular_value.coeffRef(ii, ii) = 1.0/ sigmaThreshold;
            // printf("sigular value is too small: %f\n",
            // svd.singularValues().coeff(ii));
        }
    }
    inverse_matrix = svd.matrixV() * inverse_singular_value * svd.matrixU().transpose();
}

#endif  // PSEUDOINVERSE_HPP_