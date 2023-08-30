#ifndef FLOATING_BASE_MODEL_HPP_
#define FLOATING_BASE_MODEL_HPP_

#include <string>
#include <vector>

#include <Eigen/StdVector>

#include "dynamics/spatial.hpp"
#include "dynamics/spatial_inertia.hpp"
#include "math/orientation_tools.hpp"

using std::vector;
using namespace ori;
using namespace spatial;

/**
 * @brief The state of a floating base model (base and joints).
 *
 */
template < typename T > struct FBModelState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Quat< T > body_orientation;
    Vec3< T > body_position;
    Vec3< T > rpy;
    SVec< T > body_velocity;  // body coordinates
    DVec< T > q;
    DVec< T > qd;

    /**
     * @brief Print the position of the body.
     *
     */
    void Print() const {
        printf( "position: %.3f %.3f %.3f\n", body_position[ 0 ], body_position[ 1 ], body_position[ 2 ] );
    }
};

/**
 * @brief The result of running the articulated body algorithm on a rigid-body floating
 *        base model.
 *
 */
template < typename T > struct FBModelStateDerivative {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vec3< T > d_body_position;
    SVec< T > d_body_velocity;
    DVec< T > qdd;
};

/**
 * @brief Class to represent a floating base rigid body model with rotors and ground
 *        contacts. No concept of state.
 *
 */
template < typename T > class FloatingBaseModel {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Initialize a floating base model with default gravity.
     *
     */
    FloatingBaseModel() : gravity_( 0, 0, -9.81 ) {}
    ~FloatingBaseModel() {}

    void AddBase( const SpatialInertia< T >& inertia );
    void AddBase( T mass, const Vec3< T >& com, const Mat3< T >& I );
    int  AddGroundContactPoint( int body_id, const Vec3< T >& location, bool is_foot = false );
    void AddGroundContactBoxPoints( int body_id, const Vec3< T >& dims );
    int  AddBody( const SpatialInertia< T >& inertia, const SpatialInertia< T >& rotor_inertia, T gear_ratio, int parent, JointType joint_type, CoordinateAxis joint_axis, const Mat6< T >& Xtree,
                  const Mat6< T >& Xrot );
    int  AddBody( const MassProperties< T >& inertia, const MassProperties< T >& rotor_inertia, T gear_ratio, int parent, JointType joint_type, CoordinateAxis joint_axis, const Mat6< T >& Xtree,
                  const Mat6< T >& Xrot );
    void Check();
    T    TotalRotorMass();
    T    TotalNonRotorMass();

    /**
     * @brief Get vector of parents, where parents[i] is the parent body of body i.
     *
     * @return Vector of parents
     */
    const std::vector< int >& GetParentVector() {
        return parents_;
    }

    /**
     * @brief Get vector of body spatial inertias.
     *
     * @return Vector of body spatial inertias
     */
    const std::vector< SpatialInertia< T >, Eigen::aligned_allocator< SpatialInertia< T > > >& GetBodyInertiaVector() {
        return Ibody_;
    }

    /**
     * @brief Get vector of rotor spatial inertias.
     *
     * @return Vector of rotor spatial inertias
     */
    const std::vector< SpatialInertia< T >, Eigen::aligned_allocator< SpatialInertia< T > > >& GetRotorInertiaVector() {
        return Irot_;
    }

    /**
     * @brief Set the gravity.
     *
     * @param g Gravity vector
     */
    void SetGravity( Vec3< T >& g ) {
        gravity_ = g;
    }

    /**
     * @brief Set the flag to enable computing contact info for a given contact point.
     *
     * @param gc_index Index of contact point
     * @param flag Enable/disable contact calculation
     */
    void SetContactComputeFlag( size_t gc_index, bool flag ) {
        compute_contact_info_[ gc_index ] = flag;
    }

    DMat< T > InvContactInertia( const int gc_index, const D6Mat< T >& force_directions );
    T         InvContactInertia( const int gc_index, const Vec3< T >& force_ics_at_contact );

    T ApplyTestForce( const int gc_index, const Vec3< T >& force_ics_at_contact, FBModelStateDerivative< T >& dstate_out );

    T ApplyTestForce( const int gc_index, const Vec3< T >& force_ics_at_contact, DVec< T >& dstate_out );

    void AddDynamicsVars( int count );

    void ResizeSystemMatricies();

    /**
     * @brief Update the state of the simulator, invalidating previous results.
     *
     * @param state The new state
     */
    void SetState( const FBModelState< T >& state ) {
        state_ = state;

        bias_accelerations_up_to_date_ = false;
        composite_inertias_up_to_date_ = false;

        ResetCalculationFlags();
    }

    /**
     * @brief Set the abad joint torque limit.
     *
     * @param joint_torque_limit The joint torque limit
     */
    void SetAbadTorqueLimit( const T& joint_torque_limit ) {
        abad_joint_torque_limit_ = joint_torque_limit;
    }

    /**
     * @brief Set the hip joint torque limit.
     *
     * @param joint_torque_limit The joint torque limit
     */
    void SetHipTorqueLimit( const T& joint_torque_limit ) {
        hip_joint_torque_limit_ = joint_torque_limit;
    }

    /**
     * @brief Set the knee joint torque limit.
     *
     * @param joint_torque_limit The joint torque limit
     */
    void SetKneeTorqueLimit( const T& joint_torque_limit ) {
        knee_joint_torque_limit_ = joint_torque_limit;
    }

    /**
     * @brief Mark all previously calculated values as invalid.
     *
     */
    void ResetCalculationFlags() {
        articulated_bodies_up_to_date_ = false;
        kinematics_up_to_date_         = false;
        force_propagators_up_to_date_  = false;
        qdd_effects_up_to_date_        = false;
        accelerations_up_to_date_      = false;
    }

    /**
     * @brief Update the state derivative of the simulator, invalidating previous results.
     *
     * @param d_state The new state derivative
     */
    void SetDState( const FBModelStateDerivative< T >& d_state ) {
        d_state_                  = d_state;
        accelerations_up_to_date_ = false;
    }

    Vec3< T > GetPosition( const int link_idx, const Vec3< T >& local_pos );
    Vec3< T > GetPosition( const int link_idx );

    Mat3< T > GetOrientation( const int link_idx );
    Vec3< T > GetLinearVelocity( const int link_idx, const Vec3< T >& point );
    Vec3< T > GetLinearVelocity( const int link_idx );

    Vec3< T > GetLinearAcceleration( const int link_idx, const Vec3< T >& point );
    Vec3< T > GetLinearAcceleration( const int link_idx );

    Vec3< T > GetAngularVelocity( const int link_idx );
    Vec3< T > GetAngularAcceleration( const int link_idx );

    void ForwardKinematics();
    void BiasAccelerations();
    void CompositeInertias();
    void ForwardAccelerationKinematics();
    void ContactJacobians();

    DVec< T > GeneralizedGravityForce();
    DVec< T > GeneralizedCoriolisForce();
    DMat< T > MassMatrix();
    DVec< T > InverseDynamics( const FBModelStateDerivative< T >& d_state );
    void      RunArticulatedBodyAlgorithm( const DVec< T >& tau, FBModelStateDerivative< T >& d_state );

    size_t        n_dof_ = 0;
    Vec3< T >     gravity_;
    vector< int > parents_;
    vector< T >   gear_ratios_;
    vector< T >   d_, u_;

    vector< JointType >                                                            joint_types_;
    vector< CoordinateAxis >                                                       joint_axes_;
    vector< Mat6< T >, Eigen::aligned_allocator< Mat6< T > > >                     Xtree_, Xrot_;
    vector< SpatialInertia< T >, Eigen::aligned_allocator< SpatialInertia< T > > > Ibody_, Irot_;
    vector< std::string >                                                          body_names_;

    size_t                     n_ground_contact_ = 0;
    vector< size_t >           gc_parent_;
    vectorAligned< Vec3< T > > gc_location_;
    vector< uint64_t >         foot_indices_GC_;

    vectorAligned< Vec3< T > > pGC_;
    vectorAligned< Vec3< T > > vGC_;

    vector< bool > compute_contact_info_;
    T              abad_joint_torque_limit_;
    T              hip_joint_torque_limit_;
    T              knee_joint_torque_limit_;

    /**
     * @brief Get the mass matrix for the system.
     *
     * @return The mass matrix
     */
    const DMat< T >& GetMassMatrix() const {
        return H_;
    }

    /**
     * @brief Get the gravity term (generalized forces).
     *
     * @return The gravity term
     */
    const DVec< T >& GetGravityForce() const {
        return G_;
    }

    /**
     * @brief Get the coriolis term (generalized forces).
     *
     * @return The coriolis term
     */
    const DVec< T >& GetCoriolisForce() const {
        return Cqd_;
    }

    // BEGIN ALGORITHM SUPPORT VARIABLES
    FBModelState< T >           state_;
    FBModelStateDerivative< T > d_state_;

    vectorAligned< SVec< T > > v_, vrot_, a_, arot_, avp_, avprot_, c_, crot_, S_, Srot_, fvp_, fvprot_, ag_, agrot_, f_, frot_;

    vectorAligned< SVec< T > > U_, Urot_, Utot_, pA_, pArot_;
    vectorAligned< SVec< T > > external_forces_;

    vectorAligned< SpatialInertia< T > > IC_;
    vectorAligned< Mat6< T > >           xup_, xa_, xuprot_, IA_, chi_up_;

    DMat< T > H_, C_;
    DVec< T > Cqd_, G_;

    vectorAligned< D6Mat< T > > J_;
    vectorAligned< SVec< T > >  Jdqd_;

    vectorAligned< D3Mat< T > > Jc_;
    vectorAligned< Vec3< T > >  Jcdqd_;

    bool kinematics_up_to_date_         = false;
    bool bias_accelerations_up_to_date_ = false;
    bool accelerations_up_to_date_      = false;

    bool composite_inertias_up_to_date_ = false;

    void UpdateArticulatedBodies();
    void UpdateForcePropagators();
    void UdpateQddEffects();

    /**
     * @brief Set all external forces to zero.
     *
     */
    void ResetExternalForces() {
        for ( size_t i = 0; i < n_dof_; i++ ) {
            external_forces_[ i ] = SVec< T >::Zero();
        }
    }

    bool articulated_bodies_up_to_date_ = false;
    bool force_propagators_up_to_date_  = false;
    bool qdd_effects_up_to_date_        = false;

    DMat< T >                               qdd_from_base_accel_;
    DMat< T >                               qdd_from_subqdd_;
    Eigen::ColPivHouseholderQR< Mat6< T > > invIA_5_;
};

#endif  // FLOATING_BASE_MODEL_HPP_
