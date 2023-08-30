#include "convex_mpc/solve_mpc_interface.hpp"

/**
 * @brief Construct the Solve Mpc Interface:: Solve Mpc Interface object
 * interface bridge between convexMPCLocoGait and mpc solver, has two major functions
 * firstly, transfer parameters、 data and results
 * secondly, contain two types of mpc solver, make a choose by macro definition
 *
 */
SolveMpcInterface::SolveMpcInterface( /* args */ ) {
    solve_success_       = false;
    last_horizon_length_ = 10;
    last_contact_number_ = 4 * last_horizon_length_;

    solver_ = new SolveLinearMpc( last_horizon_length_ );
    printf( "[Solver] Use Linear MPC !\n" );
}

/**
 * @brief Destroy the Solve Mpc Interface:: Solve Mpc Interface object
 *
 */
SolveMpcInterface::~SolveMpcInterface() {
    delete solver_;
}

/**
 * @brief Resize matrix dimensions、execute to solve mpc and receive optimal solution
 * resize matrix when changing horizon length or gait scheduler
 *
 */
void SolveMpcInterface::SolveDenseMpc() {
    if ( last_horizon_length_ != mpc_solver_params_.horizon_length ) {
        solver_->ResizeQpMatrix( mpc_solver_params_.horizon_length );
        last_horizon_length_ = mpc_solver_params_.horizon_length;
    }

    if ( mpc_solver_data_.ctrl_reference_enabled ) {
        solver_->SolveMpcWithCtrlRef( &mpc_solver_data_, &mpc_solver_params_ );
    }
    else {
        solver_->SolveMpc( &mpc_solver_data_, &mpc_solver_params_ );
    }
    if ( solver_->solve_success_ ) {
        GetOptimalSolution();
        solve_success_ = true;
    }
    else
        solve_success_ = false;
}

/**
 * @brief Receive optimal solution of control variables and state variables
 *
 */
void SolveMpcInterface::GetOptimalSolution() {
    for ( int leg = 0; leg < 4; leg++ ) {
        for ( int axis = 0; axis < 3; axis++ )
            optimal_solution_.optimal_force[ leg ][ axis ] = solver_->GetSolution( leg * 3 + axis );
    }

    for ( int i = 0; i < 12; i++ ) {
        optimal_solution_.optimal_state_next_horizon[ i ] = solver_->GetOptStateVar( i );
    }
}

/**
 * @brief Setup parameters of solver and physics model
 *
 * @param dt_mpc time interval of mpc horizon
 * @param horizon prediction horizon length
 * @param friction_coefficient coefficient of friction between foothold and ground
 * @param force_max maximum ground reaction force of Z direction
 * @param inertia_body inertia of body
 * @param mass robot mass
 * @param weight_state weight of state variables
 * @param weight_force weight of control variables
 * @return true
 * @return false
 */
bool SolveMpcInterface::SetupMpcSolverParams( double dt_mpc, int horizon, double friction_coefficient, double force_max, Vec3< float > inertia_body, float mass, float weight_state[ NUM_MPC_STATE_INPUT ],
                                              float weight_force[ NUM_MPC_CTRL_INPUT ] ) {
    mpc_solver_params_.dt_mpc               = dt_mpc;
    mpc_solver_params_.horizon_length       = horizon;
    mpc_solver_params_.friction_coefficient = friction_coefficient;
    mpc_solver_params_.force_max            = force_max;
    mpc_solver_params_.inertia_body         = inertia_body;
    mpc_solver_params_.mass                 = mass;
    for ( int i = 0; i < 12; i++ )
        mpc_solver_params_.weight_state[ i ] = weight_state[ i ];
    for ( int i = 0; i < NUM_MPC_CTRL_INPUT; i++ )
        mpc_solver_params_.weight_force[ i ] = weight_force[ i ];

    return true;
}

/**
 * @brief Setup parameters of solver and physics model
 *
 * @param dt_mpc time interval of mpc horizon
 * @param horizon prediction horizon length
 * @param friction_coefficient coefficient of friction between foothold and ground
 * @param force_max maximum ground reaction force of Z direction
 * @param inertia_body inertia of body
 * @param mass robot mass
 * @param weight_state weight of state variables
 * @param weight_force weight of control variables
 * @return true
 * @return false
 */
bool SolveMpcInterface::SetupMpcSolverParams( double dt_mpc, int horizon, double friction_coefficient, double force_max, Vec3< float > inertia_body, float mass, float weight_state[ NUM_MPC_STATE_INPUT ],
                                              float weight_force ) {
    mpc_solver_params_.dt_mpc               = dt_mpc;
    mpc_solver_params_.horizon_length       = horizon;
    mpc_solver_params_.friction_coefficient = friction_coefficient;
    mpc_solver_params_.force_max            = force_max;
    mpc_solver_params_.inertia_body         = inertia_body;
    mpc_solver_params_.mass                 = mass;
    for ( int i = 0; i < 12; i++ )
        mpc_solver_params_.weight_state[ i ] = weight_state[ i ];
    for ( int i = 0; i < NUM_MPC_CTRL_INPUT; i++ )
        mpc_solver_params_.weight_force[ i ] = weight_force;

    return true;
}

/**
 * @brief Update current and desired robot state
 *
 * @param body_pos position of body
 * @param body_vel velocity of body
 * @param body_rpy euler angle of body
 * @param body_omega angular velocity of body
 * @param body_quat quaternion of body
 * @param feet_pos feet position on absolute body frame
 * @param reference_trajectory desired states on mpc table
 * @param gait_state_mpc_table planned gait scheduler on mpc table
 * @return true
 * @return false
 */
bool SolveMpcInterface::UpdateMpcSolverData( Vec3< float > body_pos, Vec3< float > body_vel, Vec3< float > body_rpy, Vec3< float > body_omega, Vec4< float > body_quat, float feet_pos[ 12 ],
                                             float* reference_trajectory, int* gait_state_mpc_table ) {
    for ( int i = 0; i < 3; i++ ) {
        mpc_solver_data_.body_pos[ i ]   = body_pos[ i ];
        mpc_solver_data_.body_vel[ i ]   = body_vel[ i ];
        mpc_solver_data_.body_rpy[ i ]   = body_rpy[ i ];
        mpc_solver_data_.body_omega[ i ] = body_omega[ i ];
        mpc_solver_data_.body_quat[ i ]  = body_quat[ i ];
    }
    mpc_solver_data_.body_quat[ 3 ] = body_quat[ 3 ];
    for ( int i = 0; i < 12; i++ )
        mpc_solver_data_.feet_pos[ i ] = feet_pos[ i ];
    mpc_solver_data_.state_reference_trajectory = reference_trajectory;
    mpc_solver_data_.gait_state_mpc_table       = gait_state_mpc_table;
    mpc_solver_data_.ctrl_reference_enabled     = false;
    return true;
}

/**
 * @brief Update current and desired robot state
 *
 * @param body_pos position of body
 * @param body_vel velocity of body
 * @param body_rpy euler angle of body
 * @param body_omega angular velocity of body
 * @param body_quat quaternion of body
 * @param feet_pos feet position on absolute body frame
 * @param reference_trajectory desired states on mpc table
 * @param ctrl_reference_trajectory desired control reference
 * @param gait_state_mpc_table planned gait scheduler on mpc table
 * @return true
 * @return false
 */
bool SolveMpcInterface::UpdateMpcSolverData( Vec3< float > body_pos, Vec3< float > body_vel, Vec3< float > body_rpy, Vec3< float > body_omega, Vec4< float > body_quat, float feet_pos[ 12 ],
                                             float* reference_trajectory, float* ctrl_reference_trajectory, int* gait_state_mpc_table ) {
    for ( int i = 0; i < 3; i++ ) {
        mpc_solver_data_.body_pos[ i ]   = body_pos[ i ];
        mpc_solver_data_.body_vel[ i ]   = body_vel[ i ];
        mpc_solver_data_.body_rpy[ i ]   = body_rpy[ i ];
        mpc_solver_data_.body_omega[ i ] = body_omega[ i ];
        mpc_solver_data_.body_quat[ i ]  = body_quat[ i ];
    }
    mpc_solver_data_.body_quat[ 3 ] = body_quat[ 3 ];
    for ( int i = 0; i < 12; i++ )
        mpc_solver_data_.feet_pos[ i ] = feet_pos[ i ];
    mpc_solver_data_.state_reference_trajectory = reference_trajectory;
    mpc_solver_data_.gait_state_mpc_table       = gait_state_mpc_table;
    mpc_solver_data_.ctrl_reference_trajectory  = ctrl_reference_trajectory;
    mpc_solver_data_.ctrl_reference_enabled     = true;
    return true;
}