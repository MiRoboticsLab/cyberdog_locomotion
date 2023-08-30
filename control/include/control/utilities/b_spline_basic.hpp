#ifndef B_SPLINE_BASIC_HPP_
#define B_SPLINE_BASIC_HPP_

#include <assert.h>
#include <iostream>
#include <stdio.h>

#define IsNumberEqual( x, y ) ( ( ( x ) - ( y ) ) * ( ( x ) - ( y ) ) < 1.e-10 )
#define SafeDeletePointer( p ) \
    if ( p ) {                 \
        delete[] p;            \
        ( p ) = NULL;          \
    }

/**
 * @brief Construct b-spline curve for all joints of given dimension
 *
 */
template < typename T, int dimension, int degree, int num_middle, int constraint_level_initial, int constraint_level_final > class BSplineBasic {
public:
    /**
     * @brief Construct a new BSplineBasic object
     *
     */
    BSplineBasic()
        : num_knots_( degree + num_middle + 2 + constraint_level_initial + constraint_level_final + 1 ), num_control_points_( num_middle + 2 + constraint_level_initial + constraint_level_final ) {
        for ( int i( 0 ); i < num_knots_; ++i )
            knots_[ i ] = 0.;
        for ( int i( 0 ); i < num_control_points_; ++i ) {
            for ( int j( 0 ); j < dimension; ++j )
                control_points_[ i ][ j ] = 0.;
        }
        if ( num_knots_ < 2 * ( degree + 1 ) ) {
            printf( "Invalid setup (num_knots, degree): %d, %d\n", num_knots_, degree );
        }
    }

    /**
     * @brief Destroy the BSplineBasic object
     *
     */
    ~BSplineBasic() {}

    /**
     * @brief Set the parameters object
     *
     * @param initial_point initial point information, 3xdimension
     * @param final_point finial point information, 3xdimension
     * @param middle_points middle point information. 1xdimension
     * @param duration duration of the spline
     * @return true
     * @return false
     */
    bool SetParameters( T* initial_point, T* final_point, T** middle_points, T duration ) {
        CalculateKnots( duration );
        CalculateConstrainedPoints( initial_point, final_point, duration );
        CalculateControlPoints( middle_points );

        return true;
    }

    /**
     * @brief Get spline position at the given time.
     *
     * @param current_time current time
     * @param result  position at the given time
     * @return true
     * @return false
     */
    bool GetCurvePoint( T current_time, T* result ) {
        int span;

        if ( current_time < knots_[ 0 ] )
            current_time = knots_[ 0 ];
        else if ( current_time > knots_[ num_knots_ - 1 ] ) {
            current_time = knots_[ num_knots_ - 1 ];
        }

        if ( !FindSpan( span, current_time ) )
            return false;

        T n[ degree + 1 ];
        BasicFunction( n, span, current_time );

        T c[ dimension ];

        for ( int j( 0 ); j < dimension; ++j ) {
            c[ j ] = 0.0;
            for ( int i( 0 ); i <= degree; ++i ) {
                c[ j ] += n[ i ] * control_points_[ span - degree + i ][ j ];
            }
        }

        for ( int i( 0 ); i < dimension; ++i ) {
            result[ i ] = c[ i ];
        }
        return true;
    }

    /**
     * @brief Get spline derivative information at the given time.
     *
     * @param current_time current time
     * @param d current degree
     * @param result velocity at given time
     * @return true
     * @return false
     */
    bool GetCurveDerivationPoint( T current_time, int d, T* result ) {
        if ( d > degree )
            return 0.0;

        if ( current_time < knots_[ 0 ] )
            current_time = knots_[ 0 ];
        else if ( current_time > knots_[ num_knots_ - 1 ] ) {
            current_time = knots_[ num_knots_ - 1 ];
        }

        T** ck = new T*[ d + 1 ];
        for ( int i( 0 ); i < d + 1; ++i ) {
            ck[ i ] = new T[ dimension ];
        }
        if ( CurveDerivation( ck, current_time, d ) ) {
            for ( int m( 0 ); m < dimension; ++m )
                result[ m ] = ck[ d ][ m ];

            for ( int p( 0 ); p < d + 1; ++p )
                delete[] ck[ p ];
            SafeDeletePointer( ck );
            return true;
        }
        else {
            for ( int p( 0 ); p < d + 1; ++p )
                delete[] ck[ p ];
            SafeDeletePointer( ck );
        }
        return false;
    }

private:
    /**
     * @brief Calculate corresponding time of every knot, construct clamped-b-spline
     *
     * @param duration duration of the action
     */
    inline void CalculateKnots( T duration ) {
        int knot_id( 0 );
        int i( 0 );
        int num_middle_knot( num_knots_ - 2 * degree - 2 );
        T   time_step = duration / ( num_middle_knot + 1 );

        // to construct clamped-b-splineaugment, make multiplicity of first and last knots equal ( degree + 1 )
        for ( i = 0; i < degree + 1; ++i )
            knots_[ knot_id++ ] = 0.0;

        // uniform knot sequence for the middle part
        for ( i = 0; i < num_middle_knot; ++i ) {
            knots_[ knot_id ] = knots_[ knot_id - 1 ] + time_step;
            ++knot_id;
        }
        // make multiplicity of the last knots equal ( degree + 1 )
        for ( i = 0; i < degree + 1; ++i )
            knots_[ knot_id++ ] = duration;
    }

    /**
     * @brief Get first order deviration of curve
     *
     * @param ck control point
     * @param current_time current time
     * @param d current degree
     * @return true
     * @return false
     */
    bool CurveDerivation( T** ck, T current_time, int d ) {
        assert( d <= degree );

        int k( 0 );
        int j( 0 );

        T** nders = new T*[ d + 1 ];

        for ( k = 0; k < d + 1; ++k )
            nders[ k ] = new T[ degree + 1 ];

        int span;
        if ( !FindSpan( span, current_time ) )
            return false;

        BasicFunctionDerivation( nders, span, current_time, d );

        for ( k = 0; k <= d; ++k ) {
            // Clean Up Column
            for ( int m( 0 ); m < dimension; ++m )
                ck[ k ][ m ] = 0.;

            for ( j = 0; j <= degree; ++j ) {
                for ( int m( 0 ); m < dimension; ++m ) {
                    ck[ k ][ m ] += nders[ k ][ j ] * control_points_[ span - degree + j ][ m ];
                }
            }
        }
        for ( k = 0; k < d + 1; ++k )
            delete[] nders[ k ];
        delete[] nders;

        return true;
    }

    /**
     * @brief Get first order of basic functions
     *
     * @param derivations first order of basic functions
     * @param current_time current time
     * @param constraint_level constraint knots
     * @return true
     * @return false
     */
    bool BasicFunctionDerivation( T** derivations, T current_time, int constraint_level ) {
        int span;
        if ( !FindSpan( span, current_time ) )
            return false;

        BasicFunctionDerivation( derivations, span, current_time, constraint_level );
        return true;
    }

    /**
     * @brief Get first order of basic functions
     *
     * @param derivations first order of basic functions
     * @param span current located knot
     * @param current_time current time
     * @param constraint_level constraint knots
     * @return true
     * @return false
     */
    bool BasicFunctionDerivation( T** derivations, int span, T current_time, int constraint_level ) {
        int j, r, k;
        int s1, s2;
        int j1, j2;
        int rk;
        int pk;

        T saved = 0.0;
        T left  = 0.0;
        T right = 0.0;
        T temp  = 0.0;
        T d     = 0.0;

        // to store the basis functions and knot differences
        T** ndu = new T*[ degree + 1 ];
        for ( j = 0; j <= degree; ++j )
            ndu[ j ] = new T[ degree + 1 ];
        // to store (in an alternating fashion) the two most recently computed
        // rows a(k,j) and a(k-1,j)
        T** a = new T*[ 2 ];
        for ( j = 0; j < 2; ++j )
            a[ j ] = new T[ degree + 1 ];

        ndu[ 0 ][ 0 ] = 1.0;
        for ( j = 1; j <= degree; ++j ) {
            saved = 0.0;
            for ( r = 0; r < j; ++r ) {
                left  = Left( span, j - r, current_time );
                right = Right( span, r + 1, current_time );

                // Lower triangle
                ndu[ j ][ r ] = right + left;
                temp          = ndu[ r ][ j - 1 ] / ndu[ j ][ r ];

                // Upper triangle
                ndu[ r ][ j ] = saved + right * temp;
                saved         = left * temp;
            }
            ndu[ j ][ j ] = saved;
        }

        // Load the basis functions
        for ( j = 0; j <= degree; ++j )
            derivations[ 0 ][ j ] = ndu[ j ][ degree ];

        // This section computes the derivatives (Eq. [2.9])
        for ( r = 0; r <= degree; ++r ) {
            s1          = 0;
            s2          = 1;
            a[ 0 ][ 0 ] = 1.0;

            // Loop to compute k-th derivative
            for ( k = 1; k <= constraint_level; ++k ) {
                d  = 0.0;
                rk = r - k;
                pk = degree - k;

                if ( r >= k ) {
                    a[ s2 ][ 0 ] = a[ s1 ][ 0 ] / ndu[ pk + 1 ][ rk ];
                    d            = a[ s2 ][ 0 ] * ndu[ rk ][ pk ];
                }

                if ( rk >= -1 )
                    j1 = 1;
                else
                    j1 = -rk;

                if ( r - 1 <= pk )
                    j2 = k - 1;
                else
                    j2 = degree - r;

                for ( j = j1; j <= j2; ++j ) {
                    a[ s2 ][ j ] = ( a[ s1 ][ j ] - a[ s1 ][ j - 1 ] ) / ndu[ pk + 1 ][ rk + j ];
                    d += a[ s2 ][ j ] * ndu[ rk + j ][ pk ];
                }

                if ( r <= pk ) {
                    a[ s2 ][ k ] = -a[ s1 ][ k - 1 ] / ndu[ pk + 1 ][ r ];
                    d += a[ s2 ][ k ] * ndu[ r ][ pk ];
                }
                derivations[ k ][ r ] = d;

                // Switch rows
                j  = s1;
                s1 = s2;
                s2 = j;
            }
        }

        // Multiply through by the correct factors
        r = degree;
        for ( k = 1; k <= constraint_level; ++k ) {
            for ( j = 0; j <= degree; ++j )
                derivations[ k ][ j ] *= r;

            r *= ( degree - k );
        }

        // Deallocate
        for ( j = 0; j <= degree; ++j )
            delete[] ndu[ j ];
        delete[] ndu;

        for ( j = 0; j < 2; ++j )
            delete[] a[ j ];
        delete[] a;

        return true;
    }

    /**
     * @brief Get basic functions of  b-spline
     *
     * @param result basic functions
     * @param current_time current time
     */
    void BasicFunction( T* result, T current_time ) {
        int span;

        if ( FindSpan( span, current_time ) ) {
            BasicFunction( result, span, current_time );
        }
    }

    /**
     * @brief Get basic functions of  b-spline
     *
     * @param result basic functions
     * @param span current located knot
     * @param current_time current time
     */
    void BasicFunction( T* result, int span, T current_time ) {
        int j, r;
        T   left  = 0.0;
        T   right = 0.0;
        T   saved = 0.0;
        T   temp  = 0.0;

        result[ 0 ] = 1.0;
        for ( j = 1; j <= degree; ++j ) {
            saved = 0.0;
            for ( r = 0; r < j; ++r ) {
                left  = Left( span, j - r, current_time );
                right = Right( span, r + 1, current_time );

                if ( ( right + left ) != 0 ) {
                    temp = result[ r ] / ( right + left );
                }

                result[ r ] = saved + right * temp;
                saved       = left * temp;
            }
            result[ j ] = saved;
        }
    }

    inline T Left( int i, int j, T current_time ) {
        return current_time - knots_[ i + 1 - j ];
    }

    inline T Right( int i, int j, T current_time ) {
        return knots_[ i + j ] - current_time;
    }

    /**
     * @brief Find located knot of current time
     *
     * @param current_knot_id current located knot index
     * @param current_time current time
     * @return true
     * @return false
     */
    bool FindSpan( int& current_knot_id, T current_time ) {
        if ( current_time < knots_[ 0 ] || knots_[ num_knots_ - 1 ] < current_time )
            return false;
        // Sequential search, not make effect
        if ( IsNumberEqual( current_time, knots_[ num_knots_ - 1 ] ) ) {
            for ( int i( num_knots_ - 2 ); i > -1; --i ) {
                if ( knots_[ i ] < current_time && current_time <= knots_[ i + 1 ] ) {
                    current_knot_id = i;
                    return true;
                }
            }
            return false;
        }
        // Binary search
        int low    = 0;
        int high   = num_knots_ - 1;
        int middle = ( low + high ) >> 1;

        while ( current_time < knots_[ middle ] || current_time >= knots_[ middle + 1 ] ) {
            if ( current_time < knots_[ middle ] )
                high = middle;
            else
                low = middle;
            middle = ( low + high ) >> 1;
        }
        current_knot_id = middle;
        return true;
    }

    /**
     * @brief Calculate control points of constrainted initial and final nodes
     *
     * @param initial_pos desired initial point
     * @param final_pos desired final point
     * @param duration duration of the curve
     */
    void CalculateConstrainedPoints( T* initial_pos, T* final_pos, T duration ) {
        // Initial and final position
        for ( int m( 0 ); m < dimension; ++m ) {
            control_points_[ 0 ][ m ]                       = initial_pos[ m ];
            control_points_[ num_control_points_ - 1 ][ m ] = final_pos[ m ];
        }
        // Initial Constraints, dimension: (constraint_level_initial + 1) x (constraint_level_initial + 2)
        T** d_mat = new T*[ constraint_level_initial + 1 ];

        for ( int i( 0 ); i < constraint_level_initial + 1; ++i )
            d_mat[ i ] = new T[ constraint_level_initial + 2 ];

        BasicFunctionDerivation( d_mat, 0., constraint_level_initial );

        T initial_constraint_pos[ dimension ];
        // Vel, Acc, ...
        for ( int j( 1 ); j < constraint_level_initial + 1; ++j ) {
            for ( int k( 0 ); k < dimension; ++k ) {
                initial_constraint_pos[ k ] = initial_pos[ j * dimension + k ];

                for ( int h( j ); h > 0; --h ) {
                    initial_constraint_pos[ k ] -= d_mat[ j ][ h - 1 ] * control_points_[ h - 1 ][ k ];
                }
                control_points_[ j ][ k ] = initial_constraint_pos[ k ] / d_mat[ j ][ j ];
            }
        }

        for ( int p( 0 ); p < constraint_level_initial + 1; ++p )
            delete[] d_mat[ p ];
        SafeDeletePointer( d_mat );

        // Final Constraints
        T** c_mat = new T*[ constraint_level_final + 1 ];

        for ( int i( 0 ); i < constraint_level_final + 1; ++i )
            c_mat[ i ] = new T[ constraint_level_final + 2 ];

        BasicFunctionDerivation( c_mat, duration, constraint_level_final );

        // Vel, Acc, ...
        int idx( 1 );
        for ( int j( num_control_points_ - 2 ); j > num_control_points_ - 2 - constraint_level_final; --j ) {
            for ( int k( 0 ); k < dimension; ++k ) {
                initial_constraint_pos[ k ] = final_pos[ idx * dimension + k ];

                for ( int h( idx ); h > 0; --h ) {
                    initial_constraint_pos[ k ] -= c_mat[ idx ][ constraint_level_final + 2 - h ] * control_points_[ num_control_points_ - h ][ k ];
                }
                control_points_[ j ][ k ] = initial_constraint_pos[ k ] / c_mat[ idx ][ constraint_level_final + 1 - idx ];
            }
            ++idx;
        }
        for ( int p( 0 ); p < constraint_level_final + 1; ++p )
            delete[] c_mat[ p ];
        SafeDeletePointer( c_mat );
    }

    /**
     * @brief Calculate control points of middle nodes
     *
     * @param middle_points desired middle points
     */
    void CalculateControlPoints( T** middle_points ) {
        for ( int i( 0 ); i < num_middle; ++i ) {
            for ( int m( 0 ); m < dimension; ++m ) {
                control_points_[ constraint_level_initial + 1 + i ][ m ] = middle_points[ i ][ m ];
            }
        }
    }

    int num_knots_;
    int num_control_points_;

    T knots_[ degree + num_middle + 2 + constraint_level_initial + constraint_level_final + 1 ];
    T control_points_[ num_middle + 2 + constraint_level_initial + constraint_level_final ][ dimension ];
};

#endif  // B_SPLINE_BASIC_HPP_
