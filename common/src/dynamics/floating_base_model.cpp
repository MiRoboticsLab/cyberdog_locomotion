#include <stdexcept>
#include <stdio.h>
#include <string>
#include <vector>

#include "dynamics/floating_base_model.hpp"
#include "math/orientation_tools.hpp"

using namespace ori;
using namespace spatial;

/**
 * @brief Apply a unit test force at a contact. Returns the inv contact inertia  in
 *        that direction and computes the resultant qdd.
 *
 * @param gc_index Index of the contact
 * @param force_ics_at_contact Unit test force expressed in inertial coordinates
 * @param dstate_out Output paramter of resulting accelerations
 * @return The 1x1 inverse contact inertia J H^{-1} J^T
 */
template < typename T > T FloatingBaseModel< T >::ApplyTestForce( const int gc_index, const Vec3< T >& force_ics_at_contact, DVec< T >& dstate_out ) {
    ForwardKinematics();
    UpdateArticulatedBodies();
    UpdateForcePropagators();
    UdpateQddEffects();

    size_t i_opsp = gc_parent_.at( gc_index );
    size_t i      = i_opsp;

    dstate_out = DVec< T >::Zero( n_dof_ );

    // Rotation to absolute coords
    Mat3< T > rai = xa_[ i ].template block< 3, 3 >( 0, 0 ).transpose();
    Mat6< T > xc  = CreateSXform( rai, gc_location_.at( gc_index ) );

    // D is one column of an extended force propagator matrix (See Wensing, 2012
    // ICRA)
    SVec< T > F = xc.transpose().template rightCols< 3 >() * force_ics_at_contact;

    T lambda_inv = 0;
    T tmp        = 0;

    // from tips to base
    while ( i > 5 ) {
        tmp = F.dot( S_[ i ] );
        lambda_inv += tmp * tmp / d_[ i ];
        dstate_out.tail( n_dof_ - 6 ) += qdd_from_subqdd_.col( i - 6 ) * tmp / d_[ i ];

        // Apply force propagator (see Pat's ICRA 2012 paper)
        // essentially, since the joint is articulated, only a portion of the force
        // is felt on the predecessor. So, while Xup^T sends a force backwards as if
        // the joint was locked, ChiUp^T sends the force backward as if the joint
        // were free
        F = chi_up_[ i ].transpose() * F;
        i = parents_[ i ];
    }

    dstate_out.head( 6 ) = invIA_5_.solve( F );
    lambda_inv += F.dot( dstate_out.head( 6 ) );
    dstate_out.tail( n_dof_ - 6 ) += qdd_from_base_accel_ * dstate_out.head( 6 );

    return lambda_inv;
}

/**
 * @brief Support function for contact inertia algorithms
 *        Computes the qdd arising from "subqdd" components
 *        If you are familiar with Featherstone's sparse Op sp
 *        or jain's innovations factorization:
 *        H = L * D * L^T
 *        These subqdd components represnt the space in the middle
 *        i.e. if H^{-1} = L^{-T} * D^{-1} * L^{1}
 *        then what I am calling subqdd = L^{-1} * tau
 *        This is an awful explanation. It needs latex.
 *
 */
template < typename T > void FloatingBaseModel< T >::UdpateQddEffects() {
    if ( qdd_effects_up_to_date_ )
        return;
    UpdateForcePropagators();
    qdd_from_base_accel_.setZero();
    qdd_from_subqdd_.setZero();

    // Pass for force props
    // This loop is semi-equivalent to a cholesky factorization on H
    // akin to Featherstone's sparse operational space algo
    // These computations are for treating the joint rates like a task space
    // To do so, F computes the dynamic effect of torues onto bodies down the tree
    //
    for ( size_t i = 6; i < n_dof_; i++ ) {
        qdd_from_subqdd_( i - 6, i - 6 ) = 1;
        SVec< T > F                      = ( chi_up_[ i ].transpose() - xup_[ i ].transpose() ) * S_[ i ];
        size_t    j                      = parents_[ i ];
        while ( j > 5 ) {
            qdd_from_subqdd_( i - 6, j - 6 ) = S_[ j ].dot( F );
            F                                = chi_up_[ j ].transpose() * F;
            j                                = parents_[ j ];
        }
        qdd_from_base_accel_.row( i - 6 ) = F.transpose();
    }
    qdd_effects_up_to_date_ = true;
}

/**
 * @brief Support function for contact inertia algorithms
 *        Comptues force propagators across each joint.
 *
 */
template < typename T > void FloatingBaseModel< T >::UpdateForcePropagators() {
    if ( force_propagators_up_to_date_ )
        return;
    UpdateArticulatedBodies();
    for ( size_t i = 6; i < n_dof_; i++ ) {
        chi_up_[ i ] = xup_[ i ] - S_[ i ] * Utot_[ i ].transpose() / d_[ i ];
    }
    force_propagators_up_to_date_ = true;
}

/**
 * @brief Support function for the ABA.
 *
 */
template < typename T > void FloatingBaseModel< T >::UpdateArticulatedBodies() {
    if ( articulated_bodies_up_to_date_ )
        return;

    ForwardKinematics();

    IA_[ 5 ] = Ibody_[ 5 ].GetMatrix();

    // loop 1, down the tree
    for ( size_t i = 6; i < n_dof_; i++ ) {
        IA_[ i ]        = Ibody_[ i ].GetMatrix();  // initialize
        Mat6< T > XJrot = JointXform( joint_types_[ i ], joint_axes_[ i ], state_.q[ i - 6 ] * gear_ratios_[ i ] );
        xuprot_[ i ]    = XJrot * Xrot_[ i ];
        Srot_[ i ]      = S_[ i ] * gear_ratios_[ i ];
    }

    // Pat's magic principle of least constraint (Guass too!)
    for ( size_t i = n_dof_ - 1; i >= 6; i-- ) {
        U_[ i ]    = IA_[ i ] * S_[ i ];
        Urot_[ i ] = Irot_[ i ].GetMatrix() * Srot_[ i ];
        Utot_[ i ] = xup_[ i ].transpose() * U_[ i ] + xuprot_[ i ].transpose() * Urot_[ i ];

        d_[ i ] = Srot_[ i ].transpose() * Urot_[ i ];
        d_[ i ] += S_[ i ].transpose() * U_[ i ];

        // articulated inertia recursion
        Mat6< T > Ia = xup_[ i ].transpose() * IA_[ i ] * xup_[ i ] + xuprot_[ i ].transpose() * Irot_[ i ].GetMatrix() * xuprot_[ i ] - Utot_[ i ] * Utot_[ i ].transpose() / d_[ i ];
        IA_[ parents_[ i ] ] += Ia;
    }

    invIA_5_.compute( IA_[ 5 ] );
    articulated_bodies_up_to_date_ = true;
}

/**
 * @brief Populate member variables when bodies are added.
 *
 * @param count (6 for fb, 1 for joint)
 */
template < typename T > void FloatingBaseModel< T >::AddDynamicsVars( int count ) {
    if ( count != 1 && count != 6 ) {
        throw std::runtime_error( "AddDynamicsVars must be called with count=1 (joint) or count=6 "
                                  "(base).\n" );
    }

    Mat6< T > eye6   = Mat6< T >::Identity();
    SVec< T > zero6  = SVec< T >::Zero();
    Mat6< T > zero66 = Mat6< T >::Zero();

    SpatialInertia< T > zero_inertia( zero66 );
    for ( int i = 0; i < count; i++ ) {
        v_.push_back( zero6 );
        vrot_.push_back( zero6 );
        a_.push_back( zero6 );
        arot_.push_back( zero6 );
        avp_.push_back( zero6 );
        avprot_.push_back( zero6 );
        c_.push_back( zero6 );
        crot_.push_back( zero6 );
        S_.push_back( zero6 );
        Srot_.push_back( zero6 );
        f_.push_back( zero6 );
        frot_.push_back( zero6 );
        fvp_.push_back( zero6 );
        fvprot_.push_back( zero6 );
        ag_.push_back( zero6 );
        agrot_.push_back( zero6 );
        IC_.push_back( zero_inertia );
        xup_.push_back( eye6 );
        xuprot_.push_back( eye6 );
        xa_.push_back( eye6 );

        chi_up_.push_back( eye6 );
        d_.push_back( 0. );
        u_.push_back( 0. );
        IA_.push_back( eye6 );

        U_.push_back( zero6 );
        Urot_.push_back( zero6 );
        Utot_.push_back( zero6 );
        pA_.push_back( zero6 );
        pArot_.push_back( zero6 );
        external_forces_.push_back( zero6 );
    }

    J_.push_back( D6Mat< T >::Zero( 6, n_dof_ ) );
    Jdqd_.push_back( SVec< T >::Zero() );

    ResizeSystemMatricies();
}

/**
 * @brief Updates the size of H, C, Cqd, G, and Js when bodies are added.
 *
 */
template < typename T > void FloatingBaseModel< T >::ResizeSystemMatricies() {
    H_.setZero( n_dof_, n_dof_ );
    C_.setZero( n_dof_, n_dof_ );
    Cqd_.setZero( n_dof_ );
    G_.setZero( n_dof_ );
    for ( size_t i = 0; i < J_.size(); i++ ) {
        J_[ i ].setZero( 6, n_dof_ );
        Jdqd_[ i ].setZero();
    }

    for ( size_t i = 0; i < Jc_.size(); i++ ) {
        Jc_[ i ].setZero( 3, n_dof_ );
        Jcdqd_[ i ].setZero();
    }
    qdd_from_subqdd_.resize( n_dof_ - 6, n_dof_ - 6 );
    qdd_from_base_accel_.resize( n_dof_ - 6, 6 );
    state_.q  = DVec< T >::Zero( n_dof_ - 6 );
    state_.qd = DVec< T >::Zero( n_dof_ - 6 );
}

/**
 * @brief Create the floating body.
 *
 * @param inertia Spatial inertia of the floating body
 */
template < typename T > void FloatingBaseModel< T >::AddBase( const SpatialInertia< T >& inertia ) {
    if ( n_dof_ ) {
        throw std::runtime_error( "Cannot add base multiple times!\n" );
    }

    Mat6< T >           eye6  = Mat6< T >::Identity();
    Mat6< T >           zero6 = Mat6< T >::Zero();
    SpatialInertia< T > zero_inertia( zero6 );
    // the floating base has 6 DOFs

    n_dof_ = 6;
    for ( size_t i = 0; i < 6; i++ ) {
        parents_.push_back( 0 );
        gear_ratios_.push_back( 0 );
        joint_types_.push_back( JointType::kNothing );  // doesn't actually matter
        joint_axes_.push_back( CoordinateAxis::X );     // doesn't actually matter
        Xtree_.push_back( eye6 );
        Ibody_.push_back( zero_inertia );
        Xrot_.push_back( eye6 );
        Irot_.push_back( zero_inertia );
        body_names_.push_back( "N/A" );
    }

    joint_types_[ 5 ] = JointType::kFloatingBase;
    Ibody_[ 5 ]       = inertia;
    gear_ratios_[ 5 ] = 1;
    body_names_[ 5 ]  = "Floating Base";

    AddDynamicsVars( 6 );
}

/**
 * @brief Create the floating body.
 *
 * @param mass Mass of the floating body
 * @param com Center of mass of the floating body
 * @param I Rotational inertia of the floating body
 */
template < typename T > void FloatingBaseModel< T >::AddBase( T mass, const Vec3< T >& com, const Mat3< T >& I ) {
    SpatialInertia< T > IS( mass, com, I );
    AddBase( IS );
}

/**
 * @brief Add a ground contact point to a model.
 *
 * @param body_id The ID of the body containing the contact point
 * @param location The location (in body coordinate) of the contact point
 * @param is_foot True if foot or not.
 * @return The ID of the ground contact point
 */
template < typename T > int FloatingBaseModel< T >::AddGroundContactPoint( int body_id, const Vec3< T >& location, bool is_foot ) {
    if ( ( size_t )body_id >= n_dof_ ) {
        throw std::runtime_error( "AddGroundContactPoint got invalid body_id: " + std::to_string( body_id ) + " nDofs: " + std::to_string( n_dof_ ) + "\n" );
    }

    // std::cout << "pt-add: " << location.transpose() << "\n";
    gc_parent_.push_back( body_id );
    gc_location_.push_back( location );

    Vec3< T > zero3 = Vec3< T >::Zero();

    pGC_.push_back( zero3 );
    vGC_.push_back( zero3 );

    D3Mat< T > J( 3, n_dof_ );
    J.setZero();

    Jc_.push_back( J );
    Jcdqd_.push_back( zero3 );
    // compute_contact_info_.push_back(false);
    compute_contact_info_.push_back( true );

    // add foot to foot list
    if ( is_foot ) {
        foot_indices_GC_.push_back( n_ground_contact_ );
        compute_contact_info_[ n_ground_contact_ ] = true;
    }

    ResizeSystemMatricies();
    return n_ground_contact_++;
}

/**
 * @brief Add the bounding points of a box to the contact model. Assumes the box is
 *        centered around the origin of the body coordinate system and is axis aligned.
 *
 * @param body_id The ID of the body containing the contact point
 * @param dims The dimensions of the ground contact box (width, length, height)
 */
template < typename T > void FloatingBaseModel< T >::AddGroundContactBoxPoints( int body_id, const Vec3< T >& dims ) {
    AddGroundContactPoint( body_id, Vec3< T >( dims( 0 ), dims( 1 ), dims( 2 ) ) / 2 );
    AddGroundContactPoint( body_id, Vec3< T >( -dims( 0 ), dims( 1 ), dims( 2 ) ) / 2 );
    AddGroundContactPoint( body_id, Vec3< T >( dims( 0 ), -dims( 1 ), dims( 2 ) ) / 2 );
    AddGroundContactPoint( body_id, Vec3< T >( -dims( 0 ), -dims( 1 ), dims( 2 ) ) / 2 );

    AddGroundContactPoint( body_id, Vec3< T >( dims( 0 ), dims( 1 ), -dims( 2 ) ) / 2 );
    AddGroundContactPoint( body_id, Vec3< T >( -dims( 0 ), dims( 1 ), -dims( 2 ) ) / 2 );
    AddGroundContactPoint( body_id, Vec3< T >( dims( 0 ), -dims( 1 ), -dims( 2 ) ) / 2 );
    AddGroundContactPoint( body_id, Vec3< T >( -dims( 0 ), -dims( 1 ), -dims( 2 ) ) / 2 );
}

/**
 * @brief Add a body.
 *
 * @param inertia The inertia of the body
 * @param rotor_inertia The inertia of the rotor the body is connected to
 * @param gear_ratio The gear ratio between the body and the rotor
 * @param parent The parent body, which is also assumed to be the body the rotor is connected to
 * @param joint_type The type of joint (prismatic or revolute)
 * @param joint_axis The joint axis (X,Y,Z), in the parent's frame
 * @param Xtree The coordinate transformation from parent to this body
 * @param Xrot The coordinate transformation from parent to this body's rotor
 * @return The body's ID (can be used as the parent)
 */
template < typename T >
int FloatingBaseModel< T >::AddBody( const SpatialInertia< T >& inertia, const SpatialInertia< T >& rotor_inertia, T gear_ratio, int parent, JointType joint_type, CoordinateAxis joint_axis,
                                     const Mat6< T >& Xtree, const Mat6< T >& Xrot ) {
    if ( ( size_t )parent >= n_dof_ ) {
        throw std::runtime_error( "AddBody got invalid parent: " + std::to_string( parent ) + " nDofs: " + std::to_string( n_dof_ ) + "\n" );
    }

    parents_.push_back( parent );
    gear_ratios_.push_back( gear_ratio );
    joint_types_.push_back( joint_type );
    joint_axes_.push_back( joint_axis );
    Xtree_.push_back( Xtree );
    Xrot_.push_back( Xrot );
    Ibody_.push_back( inertia );
    Irot_.push_back( rotor_inertia );
    n_dof_++;

    AddDynamicsVars( 1 );

    return n_dof_;
}

/**
 * @brief Add a body.
 *
 * @param inertia The inertia of the body
 * @param rotor_inertia The inertia of the rotor the body is connected to
 * @param gear_ratio The gear ratio between the body and the rotor
 * @param parent The parent body, which is also assumed to be the body the rotor is connected to
 * @param joint_type The type of joint (prismatic or revolute)
 * @param joint_axis The joint axis (X,Y,Z), in the parent's frame
 * @param Xtree The coordinate transformation from parent to this body
 * @param Xrot The coordinate transformation from parent to this body's rotor
 * @return The body's ID (can be used as the parent)
 */
template < typename T >
int FloatingBaseModel< T >::AddBody( const MassProperties< T >& inertia, const MassProperties< T >& rotor_inertia, T gear_ratio, int parent, JointType joint_type, CoordinateAxis joint_axis,
                                     const Mat6< T >& Xtree, const Mat6< T >& Xrot ) {
    return AddBody( SpatialInertia< T >( inertia ), SpatialInertia< T >( rotor_inertia ), gear_ratio, parent, joint_type, joint_axis, Xtree, Xrot );
}

/**
 * @brief Checks the validity of the floating base model.
 *
 */
template < typename T > void FloatingBaseModel< T >::Check() {
    if ( n_dof_ != parents_.size() )
        throw std::runtime_error( "Invalid dof and parents length" );
}

/**
 * @brief Compute the total mass of bodies which are not rotors.
 *
 * @return The total mass of bodies which are not rotors
 */
template < typename T > T FloatingBaseModel< T >::TotalNonRotorMass() {
    T total_mass = 0;
    for ( size_t i = 0; i < n_dof_; i++ ) {
        total_mass += Ibody_[ i ].GetMass();
    }
    return total_mass;
}

/**
 * @brief Compute the total mass of bodies which are rotors.
 *
 * @return the total mass of bodies which are rotors
 */
template < typename T > T FloatingBaseModel< T >::TotalRotorMass() {
    T total_mass = 0;
    for ( size_t i = 0; i < n_dof_; i++ ) {
        total_mass += Irot_[ i ].GetMass();
    }
    return total_mass;
}

/**
 * @brief Forward kinematics of all bodies.  Computes xup_ (from up the tree) and xa_
 *        (from absolute) Also computes S_ (motion subspace), _v (spatial velocity in
 *        link coordinates), and _c (coriolis acceleration in link coordinates).
 *
 */
template < typename T > void FloatingBaseModel< T >::ForwardKinematics() {
    if ( kinematics_up_to_date_ )
        return;

    // calculate joint transformations
    xup_[ 5 ] = CreateSXform( QuaternionToRotationMatrix( state_.body_orientation ), state_.body_position );
    v_[ 5 ]   = state_.body_velocity;
    for ( size_t i = 6; i < n_dof_; i++ ) {
        // joint xform
        Mat6< T > XJ = JointXform( joint_types_[ i ], joint_axes_[ i ], state_.q[ i - 6 ] );
        xup_[ i ]    = XJ * Xtree_[ i ];
        S_[ i ]      = JointMotionSubspace< T >( joint_types_[ i ], joint_axes_[ i ] );
        SVec< T > vJ = S_[ i ] * state_.qd[ i - 6 ];
        // total velocity of body i
        v_[ i ] = xup_[ i ] * v_[ parents_[ i ] ] + vJ;

        // Same for rotors
        Mat6< T > XJrot = JointXform( joint_types_[ i ], joint_axes_[ i ], state_.q[ i - 6 ] * gear_ratios_[ i ] );
        Srot_[ i ]      = S_[ i ] * gear_ratios_[ i ];
        SVec< T > vJrot = Srot_[ i ] * state_.qd[ i - 6 ];
        xuprot_[ i ]    = XJrot * Xrot_[ i ];
        vrot_[ i ]      = xuprot_[ i ] * v_[ parents_[ i ] ] + vJrot;

        // Coriolis accelerations
        c_[ i ]    = MotionCrossProduct( v_[ i ], vJ );
        crot_[ i ] = MotionCrossProduct( vrot_[ i ], vJrot );
    }

    // calculate from absolute transformations
    for ( size_t i = 5; i < n_dof_; i++ ) {
        if ( parents_[ i ] == 0 ) {
            xa_[ i ] = xup_[ i ];  // float base
        }
        else {
            xa_[ i ] = xup_[ i ] * xa_[ parents_[ i ] ];
        }
    }

    // ground contact points
    //  // TODO : we end up inverting the same Xa a few times (like for the 8
    //  points on the body). this isn't super efficient.
    for ( size_t j = 0; j < n_ground_contact_; j++ ) {
        if ( !compute_contact_info_[ j ] )
            continue;
        size_t    i        = gc_parent_.at( j );
        Mat6< T > xai      = InvertSXform( xa_[ i ] );  // from link to absolute
        SVec< T > vSpatial = xai * v_[ i ];

        // foot position in world
        pGC_.at( j ) = SXFormPoint( xai, gc_location_.at( j ) );
        vGC_.at( j ) = SpatialToLinearVelocity( vSpatial, pGC_.at( j ) );
    }
    kinematics_up_to_date_ = true;
}

/**
 * @brief Compute the contact Jacobians (3xn matrices) for the velocity
 *        of each contact point expressed in absolute coordinates.
 *
 */
template < typename T > void FloatingBaseModel< T >::ContactJacobians() {
    ForwardKinematics();
    BiasAccelerations();

    for ( size_t k = 0; k < n_ground_contact_; k++ ) {
        Jc_[ k ].setZero();
        Jcdqd_[ k ].setZero();

        // Skip it if we don't care about it
        if ( !compute_contact_info_[ k ] )
            continue;

        size_t i = gc_parent_.at( k );

        // Rotation to absolute coords
        Mat3< T > rai = xa_[ i ].template block< 3, 3 >( 0, 0 ).transpose();
        Mat6< T > Xc  = CreateSXform( rai, gc_location_.at( k ) );

        // Bias acceleration
        SVec< T > ac = Xc * avp_[ i ];
        SVec< T > vc = Xc * v_[ i ];

        // Correct to classical
        Jcdqd_[ k ] = SpatialToLinearAcceleration( ac, vc );

        // rows for linear velcoity in the world
        D3Mat< T > Xout = Xc.template bottomRows< 3 >();

        // from tips to base
        while ( i > 5 ) {
            Jc_[ k ].col( i ) = Xout * S_[ i ];
            Xout              = Xout * xup_[ i ];
            i                 = parents_[ i ];
        }
        Jc_[ k ].template leftCols< 6 >() = Xout;
    }
}

/**
 * @brief (Support Function) Computes velocity product accelerations of
 *        each link and rotor avp_, and avprot_.
 *
 */
template < typename T > void FloatingBaseModel< T >::BiasAccelerations() {
    if ( bias_accelerations_up_to_date_ )
        return;
    ForwardKinematics();
    // velocity product acceelration of base
    avp_[ 5 ] << 0, 0, 0, 0, 0, 0;

    // from base to tips
    for ( size_t i = 6; i < n_dof_; i++ ) {
        // Outward kinamtic propagtion
        avp_[ i ]    = xup_[ i ] * avp_[ parents_[ i ] ] + c_[ i ];
        avprot_[ i ] = xuprot_[ i ] * avp_[ parents_[ i ] ] + crot_[ i ];
    }
    bias_accelerations_up_to_date_ = true;
}

/**
 * @brief Computes the generalized gravitational force (G) in the inverse dynamics.
 *
 * @return G (n_dof_ x 1 vector)
 */
template < typename T > DVec< T > FloatingBaseModel< T >::GeneralizedGravityForce() {
    CompositeInertias();

    SVec< T > a_gravity;
    a_gravity << 0, 0, 0, gravity_[ 0 ], gravity_[ 1 ], gravity_[ 2 ];
    ag_[ 5 ] = xup_[ 5 ] * a_gravity;

    // Gravity comp force is the same as force required to accelerate
    // oppostite gravity
    G_.template topRows< 6 >() = -IC_[ 5 ].GetMatrix() * ag_[ 5 ];
    for ( size_t i = 6; i < n_dof_; i++ ) {
        ag_[ i ]    = xup_[ i ] * ag_[ parents_[ i ] ];
        agrot_[ i ] = xuprot_[ i ] * ag_[ parents_[ i ] ];

        // body and rotor
        G_[ i ] = -S_[ i ].dot( IC_[ i ].GetMatrix() * ag_[ i ] ) - Srot_[ i ].dot( Irot_[ i ].GetMatrix() * agrot_[ i ] );
    }
    return G_;
}

/**
 * @brief Computes the generalized coriolis forces (Cqd) in the inverse dynamics.
 *
 * @return Cqd (n_dof_ x 1 vector)
 */
template < typename T > DVec< T > FloatingBaseModel< T >::GeneralizedCoriolisForce() {
    BiasAccelerations();

    // Floating base force
    Mat6< T > Ifb = Ibody_[ 5 ].GetMatrix();
    SVec< T > hfb = Ifb * v_[ 5 ];
    fvp_[ 5 ]     = Ifb * avp_[ 5 ] + ForceCrossProduct( v_[ 5 ], hfb );

    for ( size_t i = 6; i < n_dof_; i++ ) {
        // Force on body i
        Mat6< T > Ii = Ibody_[ i ].GetMatrix();
        SVec< T > hi = Ii * v_[ i ];
        fvp_[ i ]    = Ii * avp_[ i ] + ForceCrossProduct( v_[ i ], hi );

        // Force on rotor i
        Mat6< T > Ir = Irot_[ i ].GetMatrix();
        SVec< T > hr = Ir * vrot_[ i ];
        fvprot_[ i ] = Ir * avprot_[ i ] + ForceCrossProduct( vrot_[ i ], hr );
    }

    for ( size_t i = n_dof_ - 1; i > 5; i-- ) {
        // Extract force along the joints
        Cqd_[ i ] = S_[ i ].dot( fvp_[ i ] ) + Srot_[ i ].dot( fvprot_[ i ] );

        // Propage force down the tree
        fvp_[ parents_[ i ] ] += xup_[ i ].transpose() * fvp_[ i ];
        fvp_[ parents_[ i ] ] += xuprot_[ i ].transpose() * fvprot_[ i ];
    }

    // Force on floating base
    Cqd_.template topRows< 6 >() = fvp_[ 5 ];
    return Cqd_;
}

/**
 * @brief Get the orientation of a specific link.
 *
 * @param link_idx The index of the link
 * @return The orientation matrix of the specified link
 */
template < typename T > Mat3< T > FloatingBaseModel< T >::GetOrientation( int link_idx ) {
    ForwardKinematics();
    Mat3< T > rai = xa_[ link_idx ].template block< 3, 3 >( 0, 0 );
    rai.transposeInPlace();
    return rai;
}

/**
 * @brief Get the position of a specific link.
 *
 * @param link_idx The index of the link
 * @return The position of the specific link
 */
template < typename T > Vec3< T > FloatingBaseModel< T >::GetPosition( const int link_idx ) {
    ForwardKinematics();
    Mat6< T > xai      = InvertSXform( xa_[ link_idx ] );  // from link to absolute
    Vec3< T > link_pos = SXFormPoint( xai, Vec3< T >::Zero() );
    return link_pos;
}

/**
 * @brief Get the position of a specific link.
 *
 * @param link_idx The index of the link
 * @param local_pos The local position vector relative to the link's frame
 * @return The position of the specific link
 */
template < typename T > Vec3< T > FloatingBaseModel< T >::GetPosition( const int link_idx, const Vec3< T >& local_pos ) {
    ForwardKinematics();
    Mat6< T > xai      = InvertSXform( xa_[ link_idx ] );  // from link to absolute
    Vec3< T > link_pos = SXFormPoint( xai, local_pos );
    return link_pos;
}

/**
 * @brief Get the linear acceleration of a point on a specified link.
 *
 * @param link_idx The index of the link
 * @param point The point on the link for which the linear acceleration is computed
 * @return The linear acceleration of the specified point on the link
 */
template < typename T > Vec3< T > FloatingBaseModel< T >::GetLinearAcceleration( const int link_idx, const Vec3< T >& point ) {
    ForwardAccelerationKinematics();
    Mat3< T > R = GetOrientation( link_idx );
    return R * SpatialToLinearAcceleration( a_[ link_idx ], v_[ link_idx ], point );
}

/**
 * @brief Get the linear acceleration of a specified link.
 *
 * @param link_idx The index of the link
 * @return The linear acceleration of the specified link
 */
template < typename T > Vec3< T > FloatingBaseModel< T >::GetLinearAcceleration( const int link_idx ) {
    ForwardAccelerationKinematics();
    Mat3< T > R = GetOrientation( link_idx );
    return R * SpatialToLinearAcceleration( a_[ link_idx ], v_[ link_idx ], Vec3< T >::Zero() );
}

/**
 * @brief Get the linear velocity of a point on a specified link.
 *
 * @param link_idx The index of the link
 * @param point The point on the link for which the linear velocity is computed
 * @return The linear velocity of the specified point on the link
 */
template < typename T > Vec3< T > FloatingBaseModel< T >::GetLinearVelocity( const int link_idx, const Vec3< T >& point ) {
    ForwardKinematics();
    Mat3< T > rai = GetOrientation( link_idx );
    return rai * SpatialToLinearVelocity( v_[ link_idx ], point );
}

/**
 * @brief Get the linear velocity of a specified link.
 *
 * @param link_idx The index of the link
 * @return The linear velocity of the specified link
 */
template < typename T > Vec3< T > FloatingBaseModel< T >::GetLinearVelocity( const int link_idx ) {
    ForwardKinematics();
    Mat3< T > rai = GetOrientation( link_idx );
    return rai * SpatialToLinearVelocity( v_[ link_idx ], Vec3< T >::Zero() );
}

/**
 * @brief Get the angular velocity of a specified link.
 *
 * @param link_idx The index of the link
 * @return The angular velocity of the specified link
 */
template < typename T > Vec3< T > FloatingBaseModel< T >::GetAngularVelocity( const int link_idx ) {
    ForwardKinematics();
    Mat3< T > rai = GetOrientation( link_idx );
    // Vec3<T> v3 =
    return rai * v_[ link_idx ].template head< 3 >();
    ;
}

/**
 * @brief Get the angular acceleration of a specified link.
 *
 * @param link_idx The index of the link
 * @return The angular acceleration of the specified link
 */
template < typename T > Vec3< T > FloatingBaseModel< T >::GetAngularAcceleration( const int link_idx ) {
    ForwardAccelerationKinematics();
    Mat3< T > rai = GetOrientation( link_idx );
    return rai * a_[ link_idx ].template head< 3 >();
}

/**
 * @brief (Support Function) Computes the composite rigid body inertia
 *        of each subtree IC_[i] contains body i, and the body/rotor
 *        inertias of all successors of body i.
 *        (key note: IC_[i] does not contain rotor i)
 *
 */
template < typename T > void FloatingBaseModel< T >::CompositeInertias() {
    if ( composite_inertias_up_to_date_ )
        return;

    ForwardKinematics();
    // initialize
    for ( size_t i = 5; i < n_dof_; i++ ) {
        IC_[ i ].SetMatrix( Ibody_[ i ].GetMatrix() );
    }

    // backward loop
    for ( size_t i = n_dof_ - 1; i > 5; i-- ) {
        // Propagate inertia down the tree
        IC_[ parents_[ i ] ].AddMatrix( xup_[ i ].transpose() * IC_[ i ].GetMatrix() * xup_[ i ] );
        IC_[ parents_[ i ] ].AddMatrix( xuprot_[ i ].transpose() * Irot_[ i ].GetMatrix() * xuprot_[ i ] );
    }
    composite_inertias_up_to_date_ = true;
}

/**
 * @brief Computes the Mass Matrix (H) in the inverse dynamics formulation.
 *
 * @return H (n_dof_ x n_dof_ matrix)
 */
template < typename T > DMat< T > FloatingBaseModel< T >::MassMatrix() {
    CompositeInertias();
    H_.setZero();

    // Top left corner is the locked inertia of the whole system
    H_.template topLeftCorner< 6, 6 >() = IC_[ 5 ].GetMatrix();

    for ( size_t j = 6; j < n_dof_; j++ ) {
        // f = spatial force required for a unit qdd_j
        SVec< T > f    = IC_[ j ].GetMatrix() * S_[ j ];
        SVec< T > frot = Irot_[ j ].GetMatrix() * Srot_[ j ];

        H_( j, j ) = S_[ j ].dot( f ) + Srot_[ j ].dot( frot );

        // Propagate down the tree
        f        = xup_[ j ].transpose() * f + xuprot_[ j ].transpose() * frot;
        size_t i = parents_[ j ];
        while ( i > 5 ) {
            // in here f is expressed in frame {i}
            H_( i, j ) = S_[ i ].dot( f );
            H_( j, i ) = H_( i, j );

            // Propagate down the tree
            f = xup_[ i ].transpose() * f;
            i = parents_[ i ];
        }

        // Force on floating base
        H_.template block< 6, 1 >( 0, j ) = f;
        H_.template block< 1, 6 >( j, 0 ) = f.adjoint();
    }
    return H_;
}

/**
 * @brief Computes the forward acceleration kinematics for the FloatingBaseModel.
 *
 */
template < typename T > void FloatingBaseModel< T >::ForwardAccelerationKinematics() {
    if ( accelerations_up_to_date_ ) {
        return;
    }

    ForwardKinematics();
    BiasAccelerations();

    // Initialize gravity with model info
    SVec< T > a_gravity            = SVec< T >::Zero();
    a_gravity.template tail< 3 >() = gravity_;

    // Spatial force for floating base
    a_[ 5 ] = -xup_[ 5 ] * a_gravity + d_state_.d_body_velocity;

    // loop through joints
    for ( size_t i = 6; i < n_dof_; i++ ) {
        // spatial acceleration
        a_[ i ]    = xup_[ i ] * a_[ parents_[ i ] ] + S_[ i ] * d_state_.qdd[ i - 6 ] + c_[ i ];
        arot_[ i ] = xuprot_[ i ] * a_[ parents_[ i ] ] + Srot_[ i ] * d_state_.qdd[ i - 6 ] + crot_[ i ];
    }
    accelerations_up_to_date_ = true;
}

/**
 * @brief Computes the inverse dynamics of the system.
 *
 * @param d_state The state derivative of the FloatingBaseModel
 * @return An n_dof_ x 1 vector. The first six entries
 *         give the external wrengh on the base, with the remaining giving the
 *         joint torques
 */
template < typename T > DVec< T > FloatingBaseModel< T >::InverseDynamics( const FBModelStateDerivative< T >& d_state ) {
    SetDState( d_state );
    ForwardAccelerationKinematics();

    // Spatial force for floating base
    SVec< T > hb = Ibody_[ 5 ].GetMatrix() * v_[ 5 ];
    f_[ 5 ]      = Ibody_[ 5 ].GetMatrix() * a_[ 5 ] + ForceCrossProduct( v_[ 5 ], hb );

    // loop through joints
    for ( size_t i = 6; i < n_dof_; i++ ) {
        // spatial momentum
        SVec< T > hi = Ibody_[ i ].GetMatrix() * v_[ i ];
        SVec< T > hr = Irot_[ i ].GetMatrix() * vrot_[ i ];

        // spatial force
        f_[ i ]    = Ibody_[ i ].GetMatrix() * a_[ i ] + ForceCrossProduct( v_[ i ], hi );
        frot_[ i ] = Irot_[ i ].GetMatrix() * arot_[ i ] + ForceCrossProduct( vrot_[ i ], hr );
    }

    DVec< T > gen_force( n_dof_ );
    for ( size_t i = n_dof_ - 1; i > 5; i-- ) {
        // Pull off compoents of force along the joint
        gen_force[ i ] = S_[ i ].dot( f_[ i ] ) + Srot_[ i ].dot( frot_[ i ] );

        // Propagate down the tree
        f_[ parents_[ i ] ] += xup_[ i ].transpose() * f_[ i ];
        f_[ parents_[ i ] ] += xuprot_[ i ].transpose() * frot_[ i ];
    }
    gen_force.template head< 6 >() = f_[ 5 ];
    return gen_force;
}

/**
 * @brief Runs the Articulated Body Algorithm (ABA) for the FloatingBaseModel.
 *
 * @param tau The joint torques
 * @param d_state The derivative of the model state
 */
template < typename T > void FloatingBaseModel< T >::RunArticulatedBodyAlgorithm( const DVec< T >& tau, FBModelStateDerivative< T >& d_state ) {
    ( void )tau;
    ForwardKinematics();
    UpdateArticulatedBodies();

    // create spatial vector for gravity
    SVec< T > a_gravity;
    a_gravity << 0, 0, 0, gravity_[ 0 ], gravity_[ 1 ], gravity_[ 2 ];

    // float-base articulated inertia
    SVec< T > iv_product = Ibody_[ 5 ].GetMatrix() * v_[ 5 ];
    pA_[ 5 ]             = ForceCrossProduct( v_[ 5 ], iv_product );

    // loop 1, down the tree
    for ( size_t i = 6; i < n_dof_; i++ ) {
        iv_product = Ibody_[ i ].GetMatrix() * v_[ i ];
        pA_[ i ]   = ForceCrossProduct( v_[ i ], iv_product );

        // same for rotors
        SVec< T > vJrot = Srot_[ i ] * state_.qd[ i - 6 ];
        vrot_[ i ]      = xuprot_[ i ] * v_[ parents_[ i ] ] + vJrot;
        crot_[ i ]      = MotionCrossProduct( vrot_[ i ], vJrot );
        iv_product      = Irot_[ i ].GetMatrix() * vrot_[ i ];
        pArot_[ i ]     = ForceCrossProduct( vrot_[ i ], iv_product );
    }

    // adjust pA for external forces
    for ( size_t i = 5; i < n_dof_; i++ ) {
        // TODO add if statement (avoid these calculations if the force is zero)
        Mat3< T > R  = RotationFromSXform( xa_[ i ] );
        Vec3< T > p  = TranslationFromSXform( xa_[ i ] );
        Mat6< T > iX = CreateSXform( R.transpose(), -R * p );
        pA_[ i ]     = pA_[ i ] - iX.transpose() * external_forces_.at( i );
    }

    // Pat's magic principle of least constraint
    for ( size_t i = n_dof_ - 1; i >= 6; i-- ) {
        u_[ i ] = tau[ i - 6 ] - S_[ i ].transpose() * pA_[ i ] - Srot_[ i ].transpose() * pArot_[ i ] - U_[ i ].transpose() * c_[ i ] - Urot_[ i ].transpose() * crot_[ i ];

        // articulated inertia recursion
        SVec< T > pa = xup_[ i ].transpose() * ( pA_[ i ] + IA_[ i ] * c_[ i ] ) + xuprot_[ i ].transpose() * ( pArot_[ i ] + Irot_[ i ].GetMatrix() * crot_[ i ] ) + Utot_[ i ] * u_[ i ] / d_[ i ];
        pA_[ parents_[ i ] ] += pa;
    }

    // include gravity and compute acceleration of floating base
    SVec< T > a0  = -a_gravity;
    SVec< T > ub  = -pA_[ 5 ];
    a_[ 5 ]       = xup_[ 5 ] * a0;
    SVec< T > afb = invIA_5_.solve( ub - IA_[ 5 ].transpose() * a_[ 5 ] );
    a_[ 5 ] += afb;

    // joint accelerations
    d_state.qdd = DVec< T >( n_dof_ - 6 );
    for ( size_t i = 6; i < n_dof_; i++ ) {
        d_state.qdd[ i - 6 ] = ( u_[ i ] - Utot_[ i ].transpose() * a_[ parents_[ i ] ] ) / d_[ i ];
        a_[ i ]              = xup_[ i ] * a_[ parents_[ i ] ] + S_[ i ] * d_state.qdd[ i - 6 ] + c_[ i ];
    }

    // output
    RotMat< T > rup         = RotationFromSXform( xup_[ 5 ] );
    d_state.d_body_position = rup.transpose() * state_.body_velocity.template block< 3, 1 >( 3, 0 );
    d_state.d_body_velocity = afb;
    // qdd is set in the for loop above
}

/**
 * @brief Apply a unit test force at a contact. Returns the inv contact inertia in
 *        that direction and computes the resultant qdd.
 *
 * @param gc_index Index of the contact
 * @param force_ics_at_contact Unit test forcoe
 * @param dstate_out Output paramter of resulting accelerations
 * @return the 1x1 inverse contact inertia J H^{-1} J^T
 */
template < typename T > T FloatingBaseModel< T >::ApplyTestForce( const int gc_index, const Vec3< T >& force_ics_at_contact, FBModelStateDerivative< T >& dstate_out ) {
    ForwardKinematics();
    UpdateArticulatedBodies();
    UpdateForcePropagators();
    UdpateQddEffects();

    size_t i_opsp = gc_parent_.at( gc_index );
    size_t i      = i_opsp;

    dstate_out.qdd.setZero();

    // Rotation to absolute coords
    Mat3< T > rai = xa_[ i ].template block< 3, 3 >( 0, 0 ).transpose();
    Mat6< T > Xc  = CreateSXform( rai, gc_location_.at( gc_index ) );

    // D is one column of an extended force propagator matrix (See Wensing, 2012
    // ICRA)
    SVec< T > F = Xc.transpose().template rightCols< 3 >() * force_ics_at_contact;

    double lambda_inv = 0;
    double tmp        = 0;

    // from tips to base
    while ( i > 5 ) {
        tmp = F.dot( S_[ i ] );
        lambda_inv += tmp * tmp / d_[ i ];
        dstate_out.qdd += qdd_from_subqdd_.col( i - 6 ) * tmp / d_[ i ];

        // Apply force propagator (see Pat's ICRA 2012 paper)
        // essentially, since the joint is articulated, only a portion of the force
        // is felt on the predecessor. So, while Xup^T sends a force backwards as if
        // the joint was locked, ChiUp^T sends the force backward as if the joint
        // were free
        F = chi_up_[ i ].transpose() * F;
        i = parents_[ i ];
    }

    // TODO: Only carry out the QR once within update Aritculated Bodies
    dstate_out.d_body_velocity = invIA_5_.solve( F );
    lambda_inv += F.dot( dstate_out.d_body_velocity );
    dstate_out.qdd += qdd_from_base_accel_ * dstate_out.d_body_velocity;

    return lambda_inv;
}

/**
 * @brief Compute the inverse of the contact inertia matrix (mxm).
 *
 * @param gc_index Index of the contact
 * @param force_ics_at_contact (3x1)
 *                             e.g. if you want the cartesian inv. contact inertia in the z_ics
 *                             force_ics_at_contact = [0 0 1]^T
 * @return The 1x1 inverse contact inertia J H^{-1} J^T
 */
template < typename T > T FloatingBaseModel< T >::InvContactInertia( const int gc_index, const Vec3< T >& force_ics_at_contact ) {
    ForwardKinematics();
    UpdateArticulatedBodies();
    UpdateForcePropagators();

    size_t i_opsp = gc_parent_.at( gc_index );
    size_t i      = i_opsp;

    // Rotation to absolute coords
    Mat3< T > rai = xa_[ i ].template block< 3, 3 >( 0, 0 ).transpose();
    Mat6< T > Xc  = CreateSXform( rai, gc_location_.at( gc_index ) );

    // D is one column of an extended force propagator matrix (See Wensing, 2012
    // ICRA)
    SVec< T > F = Xc.transpose().template rightCols< 3 >() * force_ics_at_contact;

    double lambda_inv = 0;
    double tmp        = 0;

    // from tips to base
    while ( i > 5 ) {
        tmp = F.dot( S_[ i ] );
        lambda_inv += tmp * tmp / d_[ i ];

        // Apply force propagator (see Pat's ICRA 2012 paper)
        // essentially, since the joint is articulated, only a portion of the force
        // is felt on the predecessor. So, while Xup^T sends a force backwards as if
        // the joint was locked, ChiUp^T sends the force backward as if the joint
        // were free
        F = chi_up_[ i ].transpose() * F;
        i = parents_[ i ];
    }
    lambda_inv += F.dot( invIA_5_.solve( F ) );
    return lambda_inv;
}

/**
 * @brief Compute the inverse of the contact inertia matrix (mxm).
 *
 * @param gc_index Index of the contact
 * @param force_directions (6xm) each column denotes a direction of interest
 *                         col = [ moment in i.c.s., force in i.c.s.]
 *                         e.g. if you want the cartesian inv. contact inertia
 *                         force_directions = [ 0_{3x3} I_{3x3}]^T
 *                         if you only want the cartesian inv. contact inertia in one
 *                         direction then use the overloaded version.
 * @return The mxm inverse contact inertia J H^{-1} J^T
 */
template < typename T > DMat< T > FloatingBaseModel< T >::InvContactInertia( const int gc_index, const D6Mat< T >& force_directions ) {
    ForwardKinematics();
    UpdateArticulatedBodies();
    UpdateForcePropagators();

    size_t i_opsp = gc_parent_.at( gc_index );
    size_t i      = i_opsp;

    // Rotation to absolute coords
    Mat3< T > rai = xa_[ i ].template block< 3, 3 >( 0, 0 ).transpose();
    Mat6< T > Xc  = CreateSXform( rai, gc_location_.at( gc_index ) );

    // D is a subslice of an extended force propagator matrix (See Wensing, 2012
    // ICRA)
    D6Mat< T > D = Xc.transpose() * force_directions;

    size_t m = force_directions.cols();

    DMat< T > lambda_inv = DMat< T >::Zero( m, m );
    DVec< T > tmp        = DVec< T >::Zero( m );

    // from tips to base
    while ( i > 5 ) {
        tmp = D.transpose() * S_[ i ];
        lambda_inv += tmp * tmp.transpose() / d_[ i ];

        // Apply force propagator (see Pat's ICRA 2012 paper)
        // essentially, since the joint is articulated, only a portion of the force
        // is felt on the predecessor. So, while Xup^T sends a force backwards as if
        // the joint was locked, ChiUp^T sends the force backward as if the joint
        // were free
        D = chi_up_[ i ].transpose() * D;
        i = parents_[ i ];
    }

    // TODO: Only carry out the QR once within update Aritculated Bodies
    lambda_inv += D.transpose() * invIA_5_.solve( D );

    return lambda_inv;
}

template class FloatingBaseModel< double >;
template class FloatingBaseModel< float >;
