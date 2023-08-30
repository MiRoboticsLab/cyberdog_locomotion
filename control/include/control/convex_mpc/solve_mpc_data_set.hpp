#ifndef SOLVE_MPC__SOLVE_MPC_DATA_SET_HPP_
#define SOLVE_MPC__SOLVE_MPC_DATA_SET_HPP_

#include "cpp_types.hpp"

/**
 * @brief Defined parameters of solver and physics model
 * conatant on every loop generally
 *
 */
struct MpcSolverParameters {
    double        dt_mpc;
    int           horizon_length;
    double        friction_coefficient;
    double        force_max;
    Vec3< float > inertia_body;
    float         mass;
    float         weight_state[ 12 ];
    float         weight_force[ 12 ];
    float         xy_drag[ 2 ];  // gravity direction's compensation
};

/**
 * @brief Define current states of robot and desired states on mpc table
 * update on every loop generally
 *
 */
struct MpcSolverData {
    Vec3< float >  body_pos, body_vel, body_rpy, body_omega;
    Vec4< float >  body_quat;
    Vec12< float > feet_pos;                    // feet position on absolute body frame
    float*         state_reference_trajectory;  // desired states on mpc table
    int*           gait_state_mpc_table;        // planned gait scheduler on mpc table
    float*         ctrl_reference_trajectory;   // desired controls
    bool           ctrl_reference_enabled;      // whether enable control reference
};

/**
 * @brief Optimal solution of mpc slover
 *
 */
struct MpcSolverSolution {
    Vec3< float >  optimal_force[ 4 ];          // optimal control variables on current horizon
    Vec12< float > optimal_state_next_horizon;  // optimal state variables on next horizon
};

#endif  // SOLVE_MPC__SOLVE_MPC_DATA_SET_HPP_
