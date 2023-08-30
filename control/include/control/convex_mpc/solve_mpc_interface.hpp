#ifndef SOLVE_MPC__SOLVE_MPC_INTERFACE_HPP_
#define SOLVE_MPC__SOLVE_MPC_INTERFACE_HPP_

#include "solve_linear_mpc.hpp"
using namespace linear_mpc;

class SolveMpcInterface {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SolveMpcInterface( /* args */ );
    ~SolveMpcInterface();

    bool              solve_success_;
    MpcSolverSolution optimal_solution_;

    bool SetupMpcSolverParams( double dt_mpc, int horizon, double friction_coefficient, double force_max, Vec3< float > inertia_body, float mass, float weight_state[ NUM_MPC_STATE_INPUT ], float weight_force );
    bool SetupMpcSolverParams( double dt_mpc, int horizon, double friction_coefficient, double force_max, Vec3< float > inertia_body, float mass, float weight_state[ NUM_MPC_STATE_INPUT ], float weight_force[ NUM_MPC_CTRL_INPUT ] );

    bool UpdateMpcSolverData( Vec3< float > body_pos, Vec3< float > body_vel, Vec3< float > body_rpy, Vec3< float > body_omega, Vec4< float > body_quat, float feet_pos[ 12 ],
                              float* reference_trajectory, int* gait_state_mpc_table );
    bool UpdateMpcSolverData( Vec3< float > body_pos, Vec3< float > body_vel, Vec3< float > body_rpy, Vec3< float > body_omega, Vec4< float > body_quat, float feet_pos[ 12 ],
                              float* reference_trajectory, float* ctrl_reference_trajectory, int* gait_state_mpc_table );

    void SetupGravityDirectionCompensation( float xy_drag[ 2 ] ) {
        mpc_solver_params_.xy_drag[ 0 ] = xy_drag[ 0 ];
        mpc_solver_params_.xy_drag[ 1 ] = xy_drag[ 1 ];
    }

    void SolveDenseMpc();

private:
    MpcSolverParameters mpc_solver_params_;
    MpcSolverData       mpc_solver_data_;

    int last_horizon_length_;
    int last_contact_number_;

    SolveLinearMpc* solver_;

    void GetOptimalSolution();
};

#endif  // SOLVE_MPC__SOLVE_MPC_INTERFACE_HPP_