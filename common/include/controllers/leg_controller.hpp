#ifndef LEG_CONTROLLER_HPP_
#define LEG_CONTROLLER_HPP_

#include "cpp_types.hpp"
#include "dynamics/quadruped.hpp"
#include "header/lcm_type/leg_control_command_lcmt.hpp"
#include "header/lcm_type/leg_control_data_lcmt.hpp"
#include "sim_utilities/spine_board.hpp"

/**
 * @brief Data sent from the control algorithm to the legs.
 *
 */
template < typename T > struct LegControllerCommand {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
     * @brief Construct a new Leg Controller Command object.
     *
     */
    LegControllerCommand() {
        Zero();
    }

    void Zero();

    Vec3< T > tau_feed_forward, force_feed_forward, q_des, qd_des, p_des, v_des;
    Mat3< T > kp_joint, kd_joint, kp_cartesian, kd_cartesian;
};

/**
 * @brief Data returned from the legs to the control code.
 *
 */
template < typename T > struct LegControllerData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
     * @brief Construct a new Leg Controller Data object.
     *
     */
    LegControllerData() {
        Zero();
    }

    /**
     * @brief Set the Quadruped object.
     *
     * @param quad quadruped robot
     */
    void SetQuadruped( Quadruped< T >& quad ) {
        quadruped = &quad;
    }

    void Zero();

    Vec3< T >       q, qd, p, v, q_buffer, qd_buffer, qd_buffer_10[ 10 ], v_buffer, qd_numeric, qd_numeric_buffer;
    Mat3< T >       jacobian;
    Vec3< T >       tau_estimate;
    Vec3< T >       tau_actual;
    Vec3< T >       foot_force_actual;
    Vec3< T >       foot_force_desired;
    Vec3< T >       tmp_actual;
    Quadruped< T >* quadruped;
};

/**
 * @brief Controller for 4 legs of a quadruped.
 *
 */
template < typename T > class LegController {
public:
    /**
     * @brief Construct a new Leg Controller object.
     *
     * @param quad quadruped robot
     */
    LegController( Quadruped< T >& quad ) : quadruped_( quad ) {
        for ( auto& data : datas_ ) {
            data.SetQuadruped( quadruped_ );
        }
        q_abad_lowerbound_     = quadruped_.abad_lower_bound_;
        q_abad_upperbound_     = quadruped_.abad_upper_bound_;
        q_fronthip_lowerbound_ = quadruped_.front_hip_lower_bound_;
        q_fronthip_upperbound_ = quadruped_.front_hip_upper_bound_;
        q_rearhip_lowerbound_  = quadruped_.rear_hip_lower_bound_;
        q_rearhip_upperbound_  = quadruped_.rear_hip_upper_bound_;
        q_knee_lowerbound_     = quadruped_.knee_lower_bound_;
        q_knee_upperbound_     = quadruped_.knee_upper_bound_;
    }

    void ZeroCommand();
    void EdampCommand( RobotType robot, T gain );
    void UpdateData( const SpiData* spi_data );
    void UpdateCommand( SpiCommand* spi_command );
    void UpdateCommand( SpiCommand* spi_command, int pos_limit_mode );
    /**
     * @brief Enable leg controller.
     *
     * @param enabled enable flag
     */
    void SetEnabled( bool enabled ) {
        legs_enabled_ = enabled;
    };

    /**
     * @brief Set clear_error flag.
     *
     * @param enabled enable flag
     */
    void SetErrorClear( bool enabled ) {
        clear_error_ = enabled;
    };

    void SetLcm( leg_control_data_lcmt* lcm_data, leg_control_command_lcmt* lcm_command );

    LegControllerCommand< T > commands_[ 4 ];
    LegControllerData< T >    datas_[ 4 ];
    Quadruped< T >&           quadruped_;
    bool                      legs_enabled_       = false;
    bool                      clear_error_        = false;
    bool                      zero_encoders_      = false;
    u32                       calibrate_encoders_ = 0;
    int                       q_abad_limit_[ 4 ];
    int                       q_hip_limit_[ 4 ];
    int                       q_knee_limit_[ 4 ];
    T                         q_abad_lowerbound_, q_abad_upperbound_;
    T                         q_fronthip_lowerbound_, q_fronthip_upperbound_;
    T                         q_rearhip_lowerbound_, q_rearhip_upperbound_;
    T                         q_knee_lowerbound_, q_knee_upperbound_;
};

template < typename T > Mat3< T > ComputeLegJacobian( Quadruped< T >& quad, int leg, Vec3< T > q );
template < typename T > Vec3< T > ForwardKinematic( Quadruped< T >& quad, int leg, Vec3< T > q );
template < typename T > Vec3< T > InverseKinematic( Quadruped< T >& quad, int leg, Vec3< T > p );

#endif  // LEG_CONTROLLER_HPP_
