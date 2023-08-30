#include "trajectory/body_swing_trajectory.hpp"
#include "utilities/interpolation.hpp"

/**
 * @brief Compute body swing trajectory with a bezier curve
 *
 * @param phase How far along we are in one period (0 to 1)
 * @param swingTime How long the period should take (seconds)
 */
template < typename T > void BodySwingTrajectory< T >::ComputeSwingTrajectoryBezier( T phase, T swingTime ) {
    if ( phase > 1.0 )
        phase = phase - 1.0;
    if ( phase <= T( 1.0 ) ) {
        T the_phase   = phase / 1.0;
        T the_time    = swingTime;
        position_     = Interpolate::CubicBezier< Vec6< T > >( initial_position_, final_position_, the_phase );
        velocity_     = Interpolate::CubicBezierFirstDerivative< Vec6< T > >( initial_position_, final_position_, the_phase ) / the_time;
        acceleration_ = Interpolate::CubicBezierSecondDerivative< Vec6< T > >( initial_position_, final_position_, the_phase ) / ( the_time * the_time );
    }
    else {
        Vec6< T > zeros_vec6;
        zeros_vec6.setZero();
        position_     = final_position_;
        velocity_     = zeros_vec6;
        acceleration_ = zeros_vec6;
    }
}

/**
 * @brief Compute body swing trajectory with a bezier curve for gait "walking"
 *
 * @param phase How far along we are in one period (0 to 1)
 * @param swingTime How long the period should take (seconds)
 */
template < typename T > void BodySwingTrajectory< T >::ComputeSwingTrajectoryBezierWalking( T phase, T swingTime ) {
    float port             = 0.5;
    float shift_body_phase = 1 / ( 4 * ( 1 + 1 / port ) );
    float swing_legs_phase = shift_body_phase / port;
    float shift_body_time  = swingTime / ( 4 * ( 1 + 1 / port ) );
    float swing_legs_time  = shift_body_time / port;
    if ( phase > 1.0 )
        phase = phase - 1.0;
    /*********************** shift body and then swing legs****************/
    if ( phase <= T( shift_body_phase ) ) {
        // shift body
        T the_phase         = phase / shift_body_phase;
        T the_time          = shift_body_time;
        mid_position1_      = initial_position_;
        mid_position2_      = initial_position_;
        mid_position2_[ 4 ] = initial_position_[ 4 ] + stability_;
        mid_position1_[ 3 ] = ( final_position_[ 3 ] - initial_position_[ 3 ] ) * shift_body_phase + initial_position_[ 3 ];
        position_           = Interpolate::CubicBezier< Vec6< T > >( mid_position1_, mid_position2_, the_phase );
        velocity_           = Interpolate::CubicBezierFirstDerivative< Vec6< T > >( mid_position1_, mid_position2_, the_phase ) / the_time;
        acceleration_       = Interpolate::CubicBezierSecondDerivative< Vec6< T > >( mid_position1_, mid_position2_, the_phase ) / ( the_time * the_time );
    }
    else if ( phase <= T( shift_body_phase + 2 * swing_legs_phase ) ) {
        // swing right legs
        T the_phase         = ( phase - shift_body_phase ) / 2 / swing_legs_phase;
        T the_time          = 2 * swing_legs_time;
        mid_position1_      = initial_position_;
        mid_position2_      = initial_position_;
        mid_position1_[ 4 ] = initial_position_[ 4 ] + stability_;
        mid_position2_[ 4 ] = initial_position_[ 4 ] + stability_;
        mid_position1_[ 3 ] = ( final_position_[ 3 ] - initial_position_[ 3 ] ) * shift_body_phase + initial_position_[ 3 ];
        mid_position2_[ 3 ] = ( final_position_[ 3 ] - initial_position_[ 3 ] ) * ( shift_body_phase + 2 * swing_legs_phase ) + initial_position_[ 3 ];
        position_           = Interpolate::CubicBezier< Vec6< T > >( mid_position1_, mid_position2_, the_phase );
        velocity_           = Interpolate::CubicBezierFirstDerivative< Vec6< T > >( mid_position1_, mid_position2_, the_phase ) / the_time;
        acceleration_       = Interpolate::CubicBezierSecondDerivative< Vec6< T > >( mid_position1_, mid_position2_, the_phase ) / ( the_time * the_time );
    }
    else if ( phase <= T( 3 * shift_body_phase + 2 * swing_legs_phase ) ) {
        // shift body
        T the_phase         = ( phase - ( shift_body_phase + 2 * swing_legs_phase ) ) / 2 / shift_body_phase;
        T the_time          = 2 * shift_body_time;
        mid_position1_      = initial_position_;
        mid_position1_[ 4 ] = initial_position_[ 4 ] + stability_;
        mid_position2_      = initial_position_;
        mid_position2_[ 4 ] = initial_position_[ 4 ] - stability_;
        mid_position1_[ 3 ] = ( final_position_[ 3 ] - initial_position_[ 3 ] ) * ( shift_body_phase + 2 * swing_legs_phase ) + initial_position_[ 3 ];
        mid_position2_[ 3 ] = ( final_position_[ 3 ] - initial_position_[ 3 ] ) * ( 3 * shift_body_phase + 2 * swing_legs_phase ) + initial_position_[ 3 ];
        position_           = Interpolate::CubicBezier< Vec6< T > >( mid_position1_, mid_position2_, the_phase );
        velocity_           = Interpolate::CubicBezierFirstDerivative< Vec6< T > >( mid_position1_, mid_position2_, the_phase ) / the_time;
        acceleration_       = Interpolate::CubicBezierSecondDerivative< Vec6< T > >( mid_position1_, mid_position2_, the_phase ) / ( the_time * the_time );
    }
    else if ( phase <= T( 3 * shift_body_phase + 4 * swing_legs_phase ) ) {
        // swing left legs
        T the_phase         = ( phase - ( 3 * shift_body_phase + 2 * swing_legs_phase ) ) / 2 / swing_legs_phase;
        T the_time          = 2 * swing_legs_time;
        mid_position1_      = initial_position_;
        mid_position2_      = initial_position_;
        mid_position1_[ 4 ] = initial_position_[ 4 ] - stability_;
        mid_position2_[ 4 ] = initial_position_[ 4 ] - stability_;
        mid_position1_[ 3 ] = ( final_position_[ 3 ] - initial_position_[ 3 ] ) * ( 3 * shift_body_phase + 2 * swing_legs_phase ) + initial_position_[ 3 ];
        mid_position2_[ 3 ] = ( final_position_[ 3 ] - initial_position_[ 3 ] ) * ( 3 * shift_body_phase + 4 * swing_legs_phase ) + initial_position_[ 3 ];
        position_           = Interpolate::CubicBezier< Vec6< T > >( mid_position1_, mid_position2_, the_phase );
        velocity_           = Interpolate::CubicBezierFirstDerivative< Vec6< T > >( mid_position1_, mid_position2_, the_phase ) / the_time;
        acceleration_       = Interpolate::CubicBezierSecondDerivative< Vec6< T > >( mid_position1_, mid_position2_, the_phase ) / ( the_time * the_time );
    }
    else {
        // shift body
        T the_phase         = ( phase - ( 3 * shift_body_phase + 4 * swing_legs_phase ) ) / shift_body_phase;
        T the_time          = shift_body_time;
        mid_position1_      = initial_position_;
        mid_position1_[ 4 ] = initial_position_[ 4 ] - stability_;
        mid_position2_      = final_position_;
        mid_position1_[ 3 ] = ( final_position_[ 3 ] - initial_position_[ 3 ] ) * ( 3 * shift_body_phase + 4 * swing_legs_phase ) + initial_position_[ 3 ];
        mid_position2_[ 3 ] = ( final_position_[ 3 ] - initial_position_[ 3 ] ) * 1.0 + initial_position_[ 3 ];
        position_           = Interpolate::CubicBezier< Vec6< T > >( mid_position1_, mid_position2_, the_phase );
        velocity_           = Interpolate::CubicBezierFirstDerivative< Vec6< T > >( mid_position1_, mid_position2_, the_phase ) / the_time;
        acceleration_       = Interpolate::CubicBezierSecondDerivative< Vec6< T > >( mid_position1_, mid_position2_, the_phase ) / ( the_time * the_time );
    }
}
template class BodySwingTrajectory< double >;
template class BodySwingTrajectory< float >;