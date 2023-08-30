#include <iostream>

#include "utilities/interpolation.hpp"
#include "trajectory/foot_swing_trajectory.hpp"

/**
 * @brief Compute foot swing trajectory using a bezier curve
 *
 * @param phase How far along we are in the swing (0 to 1)
 * @param swing_time How long the swing should take (seconds)
 */
template < typename T > void FootSwingTrajectory< T >::ComputeSwingTrajectoryBezier( T phase, T swing_time ) {
    position_     = Interpolate::CubicBezier< Vec3< T > >( initial_position_, final_position_, phase );
    velocity_     = Interpolate::CubicBezierFirstDerivative< Vec3< T > >( initial_position_, final_position_, phase ) / swing_time;
    acceleration_ = Interpolate::CubicBezierSecondDerivative< Vec3< T > >( initial_position_, final_position_, phase ) / ( swing_time * swing_time );

    T z_position, z_velocity, z_acceleration;

    if ( phase < T( 0.5 ) ) {
        z_position     = Interpolate::CubicBezier< T >( initial_position_[ 2 ], initial_position_[ 2 ] + height_, phase * 2 );
        z_velocity     = Interpolate::CubicBezierFirstDerivative< T >( initial_position_[ 2 ], initial_position_[ 2 ] + height_, phase * 2 ) * 2 / swing_time;
        z_acceleration = Interpolate::CubicBezierSecondDerivative< T >( initial_position_[ 2 ], initial_position_[ 2 ] + height_, phase * 2 ) * 4 / ( swing_time * swing_time );
    }
    else {
        z_position     = Interpolate::CubicBezier< T >( initial_position_[ 2 ] + height_, final_position_[ 2 ], phase * 2 - 1 );
        z_velocity     = Interpolate::CubicBezierFirstDerivative< T >( initial_position_[ 2 ] + height_, final_position_[ 2 ], phase * 2 - 1 ) * 2 / swing_time;
        z_acceleration = Interpolate::CubicBezierSecondDerivative< T >( initial_position_[ 2 ] + height_, final_position_[ 2 ], phase * 2 - 1 ) * 4 / ( swing_time * swing_time );
    }

    position_[ 2 ]     = z_position;
    velocity_[ 2 ]     = z_velocity;
    acceleration_[ 2 ] = z_acceleration;
}

/**
 * @brief Compute foot swing trajectory with a polynomial curve
 *
 * @param phase How far along we are in the swing (0 to 1)
 * @param swing_time How long the swing should take (seconds)
 */
template < typename T > void FootSwingTrajectory< T >::ComputeSwingTrajectoryPolynomial( T phase, T swing_time ) {
    position_     = Interpolate::QuinticPolynomial< Vec3< T > >( initial_position_, final_position_, phase );
    velocity_     = Interpolate::QuinticPolynomialFirstDerivative< Vec3< T > >( initial_position_, final_position_, phase ) / swing_time;
    acceleration_ = Interpolate::QuinticPolynomialSecondDerivative< Vec3< T > >( initial_position_, final_position_, phase ) / ( swing_time * swing_time );

    position_[ 2 ]     = Interpolate::SepticPolynomial< T >( initial_position_[ 2 ], initial_position_[ 2 ] + height_, final_position_[ 2 ], phase );
    velocity_[ 2 ]     = Interpolate::SepticPolynomialFirstDerivative< T >( initial_position_[ 2 ], initial_position_[ 2 ] + height_, final_position_[ 2 ], phase ) / swing_time;
    acceleration_[ 2 ] = Interpolate::SepticPolynomialSecondDerivative< T >( initial_position_[ 2 ], initial_position_[ 2 ] + height_, final_position_[ 2 ], phase ) / ( swing_time * swing_time );
}

/**
 * @brief Compute foot swing trajectory using a bezier curve with a fully customized mid point
 *
 * @param phase How far along we are in the swing (0 to 1)
 * @param swing_time How long the swing should take (seconds)
 */
template < typename T > void FootSwingTrajectory< T >::ComputeSwingTrajectoryMidPosBezier( T phase, T swing_time ) {
    float local_phase        = 0.0;
    float segment_phase[ 3 ] = { 0.4, 0.2, 0.4 };
    if ( phase < T( segment_phase[ 0 ] ) ) {
        local_phase   = phase / segment_phase[ 0 ];
        local_phase   = local_phase > 1.0 ? 1.0 : local_phase;
        position_     = Interpolate::CubicBezier< Vec3< T > >( initial_position_, initial_position_ + mid_position_, local_phase );
        velocity_     = Interpolate::CubicBezierFirstDerivative< Vec3< T > >( initial_position_, initial_position_ + mid_position_, local_phase ) / ( segment_phase[ 0 ] * swing_time );
        acceleration_ = Interpolate::CubicBezierSecondDerivative< Vec3< T > >( initial_position_, initial_position_ + mid_position_, local_phase )
                        / ( segment_phase[ 0 ] * segment_phase[ 0 ] * swing_time * swing_time );
    }
    else if ( phase < T( segment_phase[ 0 ] + segment_phase[ 1 ] ) ) {
        position_ = initial_position_ + mid_position_;
        velocity_.setZero();
        acceleration_.setZero();
    }
    else {
        local_phase = ( phase - ( segment_phase[ 0 ] + segment_phase[ 1 ] ) ) / segment_phase[ 2 ];
        local_phase = local_phase > 1.0 ? 1.0 : local_phase;
        position_   = Interpolate::CubicBezier< Vec3< T > >( initial_position_ + mid_position_, final_position_, local_phase );
        velocity_   = Interpolate::CubicBezierFirstDerivative< Vec3< T > >( initial_position_ + mid_position_, final_position_, local_phase ) / ( segment_phase[ 2 ] * swing_time );
        ;
        acceleration_ = Interpolate::CubicBezierSecondDerivative< Vec3< T > >( initial_position_ + mid_position_, final_position_, local_phase )
                        / ( segment_phase[ 2 ] * segment_phase[ 2 ] * swing_time * swing_time );
    }
}

/**
 * @brief Compute foot swing trajectory with a bezier curve
 *
 * @param phase How far along we are in the swing (0 to 1)
 * @param swing_time How long the swing should take (seconds)
 */
template < typename T > void FootSwingTrajectory< T >::ComputeSwingTrajectoryBezier_downstairs( T phase, T swing_time ) {
    Vec3< T > initial_position, final_position;
    float     the_phase = 0.0;
    T         z_position, z_velocity, z_acceleration;
    if ( phase <= T( 0.6 ) ) {
        the_phase = phase / 0.6;
        if ( the_phase > 1.0 ) {
            the_phase = 1.0;
        }
        initial_position = initial_position_;
        final_position   = final_position_;
        position_        = Interpolate::CubicBezier< Vec3< T > >( initial_position, final_position, the_phase );
        velocity_        = Interpolate::CubicBezierFirstDerivative< Vec3< T > >( initial_position, final_position, the_phase ) / ( 0.6 * swing_time );
        acceleration_    = Interpolate::CubicBezierSecondDerivative< Vec3< T > >( initial_position, final_position, the_phase ) / ( 0.6 * 0.6 * swing_time * swing_time );
        if ( the_phase < 0.5 ) {
            z_position     = Interpolate::CubicBezier< T >( initial_position[ 2 ], initial_position[ 2 ] + height_, the_phase * 2 );
            z_velocity     = Interpolate::CubicBezierFirstDerivative< T >( initial_position[ 2 ], initial_position[ 2 ] + height_, the_phase * 2 ) / ( 0.6 * swing_time );
            z_acceleration = Interpolate::CubicBezierSecondDerivative< T >( initial_position[ 2 ], initial_position[ 2 ] + height_, the_phase * 2 ) / ( 0.6 * 0.6 * swing_time * swing_time );
        }
        else {
            z_position     = Interpolate::CubicBezier< T >( initial_position[ 2 ] + height_, final_position[ 2 ], the_phase * 2 - 1 );
            z_velocity     = Interpolate::CubicBezierFirstDerivative< T >( initial_position[ 2 ] + height_, final_position[ 2 ], the_phase * 2 - 1 ) / ( 0.6 * swing_time );
            z_acceleration = Interpolate::CubicBezierSecondDerivative< T >( initial_position[ 2 ] + height_, final_position[ 2 ], the_phase * 2 - 1 ) / ( 0.6 * 0.6 * swing_time * swing_time );
        }
        position_[ 2 ]     = z_position;
        velocity_[ 2 ]     = z_velocity;
        acceleration_[ 2 ] = z_acceleration;
    }
    else {
        the_phase = ( phase - 0.6 ) / 0.4;
        if ( the_phase > 1.0 ) {
            the_phase = 1.0;
        }
        initial_position = final_position_;
        final_position   = final_position_ + Vec3< T >( 0, 0, depth_ );
        position_        = Interpolate::CubicBezier< Vec3< T > >( initial_position, final_position, the_phase );
        velocity_        = Interpolate::CubicBezierFirstDerivative< Vec3< T > >( initial_position, final_position, the_phase ) / ( 0.4 * swing_time );
        acceleration_    = Interpolate::CubicBezierSecondDerivative< Vec3< T > >( initial_position, final_position, the_phase ) / ( 0.4 * 0.4 * swing_time * swing_time );
    }
}

/**
 * @brief Compute foot swing trajectory with a bezier curve for step stair, foot swing trajectory is set to be "T" shape
 *
 * @param phase How far along we are in the swing (0 to 1)
 * @param swing_time How long the swing should take (seconds)
 */
template < typename T > void FootSwingTrajectory< T >::ComputeSwingTrajectoryTstair( T phase, T swing_time ) {
    double    the_phase = 0;
    Vec3< T > initial_position, final_position;
    if ( phase <= T( 0.3 ) ) {
        initial_position = initial_position_;
        final_position   = initial_position_ + Vec3< T >( 0, 0, height_ );
        // final_position(1)=final_position_(1);
        the_phase = phase / 0.3;
        if ( the_phase > 1.0 )
            the_phase = 1.0;
        if ( use_bezier_ >= 0.9 ) {
            position_     = Interpolate::CubicBezier< Vec3< T > >( initial_position_, final_position, the_phase );
            velocity_     = Interpolate::CubicBezierFirstDerivative< Vec3< T > >( initial_position_, final_position, the_phase ) / ( 0.3 * swing_time );
            acceleration_ = Interpolate::CubicBezierSecondDerivative< Vec3< T > >( initial_position_, final_position, the_phase ) / ( 0.3 * 0.3 * swing_time * swing_time );
        }
        else {
            position_     = Interpolate::Cycloid< Vec3< T > >( initial_position_, final_position, the_phase );
            velocity_     = Interpolate::CycloidFirstDerivative< Vec3< T > >( initial_position_, final_position, the_phase ) / ( 0.3 * swing_time );
            acceleration_ = Interpolate::CycloidSecondDerivative< Vec3< T > >( initial_position_, final_position, the_phase ) / ( 0.3 * 0.3 * swing_time * swing_time );
        }
    }
    else if ( phase <= 0.6 ) {
        initial_position = initial_position_ + Vec3< T >( 0, 0, height_ );
        final_position   = Vec3< T >( final_position_( 0 ), final_position_( 1 ), initial_position_( 2 ) ) + Vec3< T >( 0, 0, height_ );
        the_phase        = ( phase - 0.3 ) / 0.3;
        if ( the_phase > 1.0 )
            the_phase = 1.0;
        if ( use_bezier_ >= 0.9 ) {
            position_     = Interpolate::CubicBezier< Vec3< T > >( initial_position, final_position, the_phase );
            velocity_     = Interpolate::CubicBezierFirstDerivative< Vec3< T > >( initial_position, final_position, the_phase ) / ( 0.3 * swing_time );
            acceleration_ = Interpolate::CubicBezierSecondDerivative< Vec3< T > >( initial_position, final_position, the_phase ) / ( 0.3 * 0.3 * swing_time * swing_time );
        }
        else {
            position_     = Interpolate::Cycloid< Vec3< T > >( initial_position, final_position, the_phase );
            velocity_     = Interpolate::CycloidFirstDerivative< Vec3< T > >( initial_position, final_position, the_phase ) / ( 0.3 * swing_time );
            acceleration_ = Interpolate::CycloidSecondDerivative< Vec3< T > >( initial_position, final_position, the_phase ) / ( 0.3 * 0.3 * swing_time * swing_time );
        }
    }
    else {
        initial_position = Vec3< T >( final_position_( 0 ), final_position_( 1 ), initial_position_( 2 ) ) + Vec3< T >( 0, 0, height_ );
        final_position   = final_position_;
        the_phase        = ( phase - 0.6 ) / 0.4;
        if ( the_phase > 1.0 )
            the_phase = 1.0;
        if ( use_bezier_ >= 0.9 ) {
            position_     = Interpolate::CubicBezier< Vec3< T > >( initial_position, final_position, the_phase );
            velocity_     = Interpolate::CubicBezierFirstDerivative< Vec3< T > >( initial_position, final_position, the_phase ) / ( 0.4 * swing_time );
            acceleration_ = Interpolate::CubicBezierSecondDerivative< Vec3< T > >( initial_position, final_position, the_phase ) / ( 0.4 * 0.4 * swing_time * swing_time );
        }
        else {
            position_     = Interpolate::Cycloid< Vec3< T > >( initial_position, final_position, the_phase );
            velocity_     = Interpolate::CycloidFirstDerivative< Vec3< T > >( initial_position, final_position, the_phase ) / ( 0.4 * swing_time );
            acceleration_ = Interpolate::CycloidSecondDerivative< Vec3< T > >( initial_position, final_position, the_phase ) / ( 0.4 * 0.4 * swing_time * swing_time );
        }
    }
}

/**
 * @brief Compute foot swing trajectory with a bezier curve for step stair, foot swing trajectory is set to be "T" shape
 *
 * @param phase How far along we are in the swing (0 to 1)
 * @param swing_time How long the swing should take (seconds)
 */
template < typename T > void FootSwingTrajectory< T >::ComputeSwingTrajectoryTsimilar( T phase, T swing_time ) {
    if ( phase > 1.0 )
        phase = 1.0;
    Vec3< T > initial_position, p1, p2, final_position;
    if ( phase <= T( 1.0 ) ) {
        initial_position = initial_position_;
        final_position   = final_position_;
        double port      = 1.0;
        p1 << initial_position_[ 0 ] - port * ( final_position_[ 0 ] - initial_position_[ 0 ] ), initial_position_[ 1 ] - port * ( final_position_[ 1 ] - initial_position_[ 1 ] ), 1.4 * height_;
        p2 << final_position_[ 0 ] + port * ( final_position_[ 0 ] - initial_position_[ 0 ] ), final_position_[ 1 ] + port * ( final_position_[ 1 ] - initial_position_[ 1 ] ), 1.4 * height_;

        position_     = Interpolate::CubicBezierFourCtrlPoint< Vec3< T > >( initial_position, p1, p2, final_position, phase );
        velocity_     = Interpolate::CubicBezierFourCtrlPointFirstDerivative< Vec3< T > >( initial_position, p1, p2, final_position, phase ) / ( swing_time );
        acceleration_ = Interpolate::CubicBezierFourCtrlPointSecondDerivative< Vec3< T > >( initial_position, p1, p2, final_position, phase ) / ( swing_time * swing_time );
    }
}

template class FootSwingTrajectory< double >;
template class FootSwingTrajectory< float >;
