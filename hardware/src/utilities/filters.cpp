#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "utilities/filters.hpp"

template < typename T > MovingAverageFilter< T >::MovingAverageFilter( int num_data ) : num_data_( num_data ), idx_( 0 ), sum_( 0.0 ) {
    buffer_ = new T[ num_data_ ];
    memset( ( void* )buffer_, 0.0, sizeof( T ) * num_data_ );
}

template < typename T > void MovingAverageFilter< T >::Input( T input_value ) {
    sum_ -= buffer_[ idx_ ];
    sum_ += input_value;
    buffer_[ idx_ ] = input_value;
    ++idx_;
    idx_ %= num_data_;
}

template < typename T > T MovingAverageFilter< T >::Output() {
    return sum_ / num_data_;
}

template < typename T > void MovingAverageFilter< T >::SetDataNum( int num_data ) {
    Clear();
    buffer_ = new T[ num_data ];
    memset( ( void* )buffer_, 0.0, sizeof( T ) * num_data );
}

template < typename T > int MovingAverageFilter< T >::GetDataNum( void ) {
    return num_data_;
}

template < typename T > void MovingAverageFilter< T >::Clear( void ) {
    sum_ = 0.0;
    memset( ( void* )buffer_, 0.0, sizeof( T ) * num_data_ );
}

template < typename T > MovingAverageFilter< T >::~MovingAverageFilter() {
    delete[] buffer_;
}

template class MovingAverageFilter< double >;
template class MovingAverageFilter< float >;

/*============================================================================*/

template < typename T > ButterWorthFilter< T >::ButterWorthFilter( int num_sample, T dt, T cutoff_frequency ) {
    sample_num_  = num_sample;
    dt_         = dt;
    cutoff_freq_ = cutoff_frequency;

    buffer_ = new T[ num_sample ];
    memset( ( void* )buffer_, 0, sizeof( T ) * num_sample );

    cur_idx_ = 0;
}

template < typename T > ButterWorthFilter< T >::~ButterWorthFilter( void ) {
    delete[] buffer_;
}

template < typename T > void ButterWorthFilter< T >::Input( T input_value ) {
    int j;
    T   sqrt_2 = sqrt( 2 );
    T   value  = 0;
    for ( j = sample_num_ - 2; j >= 0; j-- ) {
        buffer_[ j + 1 ] = buffer_[ j ];
    }

    buffer_[ 0 ] = input_value;
    for ( j = 0; j < sample_num_; j++ ) {
        T t = ( T )j * dt_;
        value += sqrt_2 / cutoff_freq_ * buffer_[ j ] * exp( -1. / sqrt_2 * t ) * sin( cutoff_freq_ / sqrt_2 * t ) * dt_;
        //		value += sqrt_2 * exp(-1./sqrt_2*t) * sin(1./sqrt_2*t ) * dt_;
    }
    value_ = value;
}

template < typename T > T ButterWorthFilter< T >::Output( void ) {
    return value_;
}

template < typename T > void ButterWorthFilter< T >::SetTimeStep( T dt ) {
    dt_ = dt;
}

template < typename T > void ButterWorthFilter< T >::SetSampleNum( int num_sample ) {
    sample_num_ = num_sample;
}

template < typename T > void ButterWorthFilter< T >::SetCutoffFrequency( T cutoff_frequency ) {
    cutoff_freq_ = cutoff_frequency;
}

template < typename T > void ButterWorthFilter< T >::Clear( void ) {
    for ( int i( 0 ); i < sample_num_; ++i ) {
        buffer_[ i ] = 0.0;
    }
}

template class ButterWorthFilter< double >;
template class ButterWorthFilter< float >;

/*============================================================================*/

template < typename T > DigitalLpFilter< T >::DigitalLpFilter( T w_c, T t_s ) {
    lpf_in_prev_[ 0 ] = lpf_in_prev_[ 1 ] = 0;
    lpf_out_prev_[ 0 ] = lpf_out_prev_[ 1 ] = 0;
    lpf_in1_ = 0, lpf_in2_ = 0, lpf_in3_ = 0, lpf_out1_ = 0, lpf_out2_ = 0;
    wc_ = w_c;
    ts_ = t_s;
    Update();
}

template < typename T > void DigitalLpFilter< T >::Update( void ) {
    float den = 2500 * ts_ * ts_ * wc_ * wc_ + 7071 * ts_ * wc_ + 10000;

    lpf_in1_  = 2500 * ts_ * ts_ * wc_ * wc_ / den;
    lpf_in2_  = 5000 * ts_ * ts_ * wc_ * wc_ / den;
    lpf_in3_  = 2500 * ts_ * ts_ * wc_ * wc_ / den;
    lpf_out1_ = -( 5000 * ts_ * ts_ * wc_ * wc_ - 20000 ) / den;
    lpf_out2_ = -( 2500 * ts_ * ts_ * wc_ * wc_ - 7071 * ts_ * wc_ + 10000 ) / den;
}

template < typename T > DigitalLpFilter< T >::~DigitalLpFilter( void ) {}

template < typename T > void DigitalLpFilter< T >::Input( T lpf_in ) {
    lpf_out_ = lpf_in1_ * lpf_in + lpf_in2_ * lpf_in_prev_[ 0 ] + lpf_in3_ * lpf_in_prev_[ 1 ] +  // input component
              lpf_out1_ * lpf_out_prev_[ 0 ] + lpf_out2_ * lpf_out_prev_[ 1 ];                  // output component
    lpf_in_prev_[ 1 ]  = lpf_in_prev_[ 0 ];
    lpf_in_prev_[ 0 ]  = lpf_in;
    lpf_out_prev_[ 1 ] = lpf_out_prev_[ 0 ];
    lpf_out_prev_[ 0 ] = lpf_out_;
}

template < typename T > T DigitalLpFilter< T >::Output( void ) {
    return lpf_out_;
}

template < typename T > void DigitalLpFilter< T >::SetTs( T t_s ) {
    ts_ = t_s;
    Update();
}

template < typename T > void DigitalLpFilter< T >::SetWc( T w_c ) {
    wc_ = w_c;
    Update();
}

template < typename T > void DigitalLpFilter< T >::Clear( void ) {
    lpf_in_prev_[ 1 ]  = 0;
    lpf_in_prev_[ 0 ]  = 0;
    lpf_out_prev_[ 1 ] = 0;
    lpf_out_prev_[ 0 ] = 0;
}

template class DigitalLpFilter< double >;
template class DigitalLpFilter< float >;

/*============================================================================*/

template < typename T > DerivLpFilter< T >::DerivLpFilter( T w_c, T t_s ) {
    lpf_in_prev_[ 0 ]  = 0;
    lpf_in_prev_[ 1 ]  = 0;
    lpf_out_prev_[ 0 ] = 0;
    lpf_out_prev_[ 1 ] = 0;
    lpf_in1_           = 0;
    lpf_in2_           = 0;
    lpf_in3_           = 0;
    lpf_out1_          = 0;
    lpf_out2_          = 0;
    T a               = 1.4142;
    T den             = 4 + 2 * a * w_c * t_s + t_s * t_s * w_c * w_c;

    lpf_in1_  = 2 * t_s * w_c * w_c / den;
    lpf_in2_  = 0;
    lpf_in3_  = -2. * t_s * w_c * w_c / den;
    lpf_out1_ = -1. * ( -8 + t_s * t_s * w_c * w_c * 2 ) / den;
    lpf_out2_ = -1. * ( 4 - 2 * a * w_c * t_s + t_s * t_s * w_c * w_c ) / den;
    lpf_out_  = 0.0;
    Clear();
}

template < typename T > DerivLpFilter< T >::~DerivLpFilter( void ) {}

template < typename T > void DerivLpFilter< T >::Input( T lpf_in ) {
    // static int i(0);
    lpf_out_ = lpf_in1_ * lpf_in + lpf_in2_ * lpf_in_prev_[ 0 ] + lpf_in3_ * lpf_in_prev_[ 1 ] +  // input component
              lpf_out1_ * lpf_out_prev_[ 0 ] + lpf_out2_ * lpf_out_prev_[ 1 ];                  // output component

    // printf("%i th filter (%f): %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",i,
    //        lpf_out_,
    //        lpf_in1_, lpf_in, lpf_in2_,
    //        lpf_in_prev_[0], lpf_in3_, lpf_in_prev_[1],
    //        lpf_out1_, lpf_out_prev_[0], lpf_out2_, lpf_out_prev_[1]);

    // if(lpf_out_>100){
    //     exit(0);
    // }

    lpf_in_prev_[ 1 ]  = lpf_in_prev_[ 0 ];
    lpf_in_prev_[ 0 ]  = lpf_in;
    lpf_out_prev_[ 1 ] = lpf_out_prev_[ 0 ];
    lpf_out_prev_[ 0 ] = lpf_out_;
    // ++i;
}

template < typename T > T DerivLpFilter< T >::Output( void ) {
    return lpf_out_;
}

template < typename T > void DerivLpFilter< T >::Clear( void ) {
    lpf_in_prev_[ 1 ]  = 0;
    lpf_in_prev_[ 0 ]  = 0;
    lpf_out_prev_[ 1 ] = 0;
    lpf_out_prev_[ 0 ] = 0;
}

template class DerivLpFilter< double >;
template class DerivLpFilter< float >;

/*============================================================================*/

template < typename T > Ff01Filter< T >::Ff01Filter( float t_s, float w_c ) {
    lpf_in_prev_[ 0 ] = lpf_in_prev_[ 1 ] = 0;
    lpf_out_prev_[ 0 ] = lpf_out_prev_[ 1 ] = 0;
    lpf_in1_ = 0, lpf_in2_ = 0, lpf_in3_ = 0, lpf_out1_ = 0, lpf_out2_ = 0;
    T a   = 1.4142;
    T den = 4 + 2 * a * w_c * t_s + t_s * t_s * w_c * w_c;
    T J   = 0.00008;
    T B   = 0.0002;

    lpf_in1_  = B * t_s * t_s * w_c * w_c + 2 * J * t_s * w_c * w_c;
    lpf_in2_  = 2 * B * t_s * t_s * w_c * w_c;
    lpf_in3_  = B * t_s * t_s * w_c * w_c - 2 * J * t_s * w_c * w_c;
    lpf_out1_ = -1. * ( -8 + t_s * t_s * w_c * w_c * 2 ) / den;
    lpf_out2_ = -1. * ( 4 - 2 * a * w_c * t_s + t_s * t_s * w_c * w_c ) / den;
}

template < typename T > Ff01Filter< T >::~Ff01Filter( void ) {}

template < typename T > void Ff01Filter< T >::Input( T lpf_in ) {
    lpf_out_ = lpf_in1_ * lpf_in + lpf_in2_ * lpf_in_prev_[ 0 ] + lpf_in3_ * lpf_in_prev_[ 1 ] +  // input component
              lpf_out1_ * lpf_out_prev_[ 0 ] + lpf_out2_ * lpf_out_prev_[ 1 ];                  // output component
    lpf_in_prev_[ 1 ]  = lpf_in_prev_[ 0 ];
    lpf_in_prev_[ 0 ]  = lpf_in;
    lpf_out_prev_[ 1 ] = lpf_out_prev_[ 0 ];
    lpf_out_prev_[ 0 ] = lpf_out_;
}

template < typename T > T Ff01Filter< T >::Output( void ) {
    return lpf_out_;
}

template < typename T > void Ff01Filter< T >::Clear( void ) {
    lpf_in_prev_[ 1 ]  = 0;
    lpf_in_prev_[ 0 ]  = 0;
    lpf_out_prev_[ 1 ] = 0;
    lpf_out_prev_[ 0 ] = 0;
}

template class Ff01Filter< float >;
template class Ff01Filter< double >;

/*============================================================================*/

template < typename T > Ff02Filter< T >::Ff02Filter( float t_s, float w_c ) {
    T J = 0.003216;

    lpf_in_prev_[ 0 ] = lpf_in_prev_[ 1 ] = 0;
    lpf_out_prev_[ 0 ] = lpf_out_prev_[ 1 ] = 0;
    lpf_in1_ = 0, lpf_in2_ = 0, lpf_in3_ = 0, lpf_out1_ = 0, lpf_out2_ = 0;

    T a   = 1.4142;
    T den = 4 + 2 * a * w_c * t_s + t_s * t_s * w_c * w_c;

    lpf_in1_  = J * 2 * t_s * w_c * w_c / den;
    lpf_in2_  = 0;
    lpf_in3_  = -2. * J * t_s * w_c * w_c / den;
    lpf_out1_ = -1. * ( -8 + t_s * t_s * w_c * w_c * 2 ) / den;
    lpf_out2_ = -1. * ( 4 - 2 * a * w_c * t_s + t_s * t_s * w_c * w_c ) / den;

    Clear();
}

template < typename T > Ff02Filter< T >::~Ff02Filter( void ) {}

template < typename T > void Ff02Filter< T >::Input( T lpf_in ) {
    lpf_out_ = lpf_in1_ * lpf_in + lpf_in2_ * lpf_in_prev_[ 0 ] + lpf_in3_ * lpf_in_prev_[ 1 ] +  // input component
              lpf_out1_ * lpf_out_prev_[ 0 ] + lpf_out2_ * lpf_out_prev_[ 1 ];                  // output component
    lpf_in_prev_[ 0 ]  = lpf_in;
    lpf_in_prev_[ 1 ]  = lpf_in_prev_[ 0 ];
    lpf_out_prev_[ 0 ] = lpf_out_;
    lpf_out_prev_[ 1 ] = lpf_out_prev_[ 0 ];
}

template < typename T > T Ff02Filter< T >::Output( void ) {
    return lpf_out_;
}

template < typename T > void Ff02Filter< T >::Clear( void ) {
    lpf_in_prev_[ 1 ]  = 0;
    lpf_in_prev_[ 0 ]  = 0;
    lpf_out_prev_[ 1 ] = 0;
    lpf_out_prev_[ 0 ] = 0;
}

template class Ff02Filter< float >;
template class Ff02Filter< double >;

/*============================================================================*/

template < typename T > AverageFilter< T >::AverageFilter( T dt, T t_const, T limit ) : dt_( dt ), t_const_( t_const ), limit_( limit ) {
    est_value_ = 0.;
}

template < typename T > AverageFilter< T >::~AverageFilter() {
    est_value_ = 0;
}

template < typename T > void AverageFilter< T >::Clear() {
    est_value_ = 0.;
}

template < typename T > void AverageFilter< T >::Input( T input ) {
    T update_value = input - est_value_;
    if ( fabs( update_value ) > limit_ ) {
        update_value = 0.;
    }
    est_value_ += ( dt_ / ( dt_ + t_const_ ) ) * update_value;
}

template < typename T > T AverageFilter< T >::Output() {
    return est_value_;
}

template class AverageFilter< float >;
template class AverageFilter< double >;
