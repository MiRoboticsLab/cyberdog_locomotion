#ifndef SOLVE_MPC__SOLVE_LINEAR_MPC_HPP_
#define SOLVE_MPC__SOLVE_LINEAR_MPC_HPP_

#include <iostream>
#include <stdio.h>
#include <sys/time.h>

#include "qpOASES.hpp"

#include "utilities/timer.hpp"
#include "common_types.hpp"
#include "solve_mpc_data_set.hpp"

#define NUM_MPC_STATE_INPUT 12  // number of state in mpc
#define NUM_MPC_CTRL_INPUT 12   // number of ctrl in mpc
#define MAX_HORIZON 36          // max horizon in mpc

namespace linear_mpc {

using namespace Eigen;
using Eigen::Matrix;

class SolveLinearMpc {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SolveLinearMpc( const s16 horizon );
    ~SolveLinearMpc();

    bool solve_success_;

    void SolveMpc( MpcSolverData* prob_update, MpcSolverParameters* prob_setup );
    void SolveMpcWithCtrlRef( MpcSolverData* prob_update, MpcSolverParameters* prob_setup );

    void ResizeQpMatrix( const s16 horizon );

    float GetSolution( int index );
    float GetOptStateVar( int index );

private:
    static const int state_variables_dim_   = 13;
    static const int control_variables_dim_ = 12;
    static const int constraint_dim_        = 20;

    int horizon_length_;
    int horizon_length2_;
    int horizon_length2_m12m12_;
    int horizon_length2_m12m20_;
    int horizon_length_m12_;
    int horizon_length_m20_;

    // matrix of fixed dimension
    Matrix< fpt, state_variables_dim_, 1 > state_init_;
    // coefficient matrix of continuous state space equation
    Matrix< fpt, state_variables_dim_, state_variables_dim_ >   state_matrix_continuous_time_;
    Matrix< fpt, state_variables_dim_, control_variables_dim_ > input_matrix_continuous_time_;
    // coefficient matrix of discrete state space equation
    Matrix< fpt, state_variables_dim_, control_variables_dim_ > input_matrix_discrete_time_;
    Matrix< fpt, state_variables_dim_, state_variables_dim_ >   state_matrix_discrete_time_;
    // augmented matrix of all coefficient matrix
    Matrix< fpt, state_variables_dim_ + control_variables_dim_, state_variables_dim_ + control_variables_dim_ > state_input_augmented_matrix_, exp_state_input_augmented_matrix_;

    // matrix of dynamic dimension, resized by horizon_length_
    // state space equation on all prediction horizon
    Matrix< fpt, Eigen::Dynamic, state_variables_dim_ > state_matrix_qp_;
    Matrix< fpt, Eigen::Dynamic, Eigen::Dynamic >       input_matrix_qp_, input_matrix_trans_x_weight_matrix_state_;

    Eigen::DiagonalMatrix< fpt, Eigen::Dynamic >  weight_matrix_state_;
    Eigen::DiagonalMatrix< fpt, Eigen::Dynamic >  weight_matrix_control_;
    Matrix< fpt, Eigen::Dynamic, Eigen::Dynamic > identity_matrix_12_;

    Matrix< fpt, Eigen::Dynamic, 1 >                               state_vector_ref_;
    Matrix< fpt, Eigen::Dynamic, 1 >                               control_vector_ref_;
    Matrix< fpt, Eigen::Dynamic, 1 >                               upper_boundary_;
    Matrix< fpt, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor > constraint_matrix_qp_;
    Matrix< fpt, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor > hessian_matrix_qp_;
    Matrix< fpt, Eigen::Dynamic, 1 >                               gradient_vector_qp_;

    Matrix< fpt, Eigen::Dynamic, 1, Eigen::AutoAlign > control_vector_, state_vector_;

    qpOASES::real_t* hessian_qpoases_;
    qpOASES::real_t* gradient_qpoases_;
    qpOASES::real_t* constraint_qpoases_;
    qpOASES::real_t* lower_boundary_qpoases_;
    qpOASES::real_t* upper_boundary_qpoases_;
    qpOASES::real_t* control_solution_;

    qpOASES::real_t* hessian_qpoases_simple_;
    qpOASES::real_t* gradient_qpoases_simple_;
    qpOASES::real_t* constraint_qpoases_simple_;
    qpOASES::real_t* lower_boundary_qpoases_simple_;
    qpOASES::real_t* upper_boundary_qpoases_simple_;
    qpOASES::real_t* control_solution_simple_;
    u8               real_allocated_;

    char variable_eliminate_[ 2000 ];
    char constraint_eliminate_[ 2000 ];

    void GetStateEquationMatrix( Matrix< fpt, 3, 3 > inertia_world, fpt mass, Matrix< fpt, 3, 4 > r_feet, Matrix< fpt, 3, 3 > rotation_yaw, float xy_drag[ 2 ] );
    void DiscreteAndGetQpMatrix( fpt dt_mpc, s16 horizon );

    fpt* GetControlVarSolution();
    void GetOptStateVarAllHorizon();
    fpt* GetOptStateVarNextHorizon();

    // inner function
    inline Matrix< fpt, 3, 3 > CrossMatrix( Matrix< fpt, 3, 3 > I_inv, Matrix< fpt, 3, 1 > r ) {
        Matrix< fpt, 3, 3 > cm;
        cm << 0.f, -r( 2 ), r( 1 ), r( 2 ), 0.f, -r( 0 ), -r( 1 ), r( 0 ), 0.f;
        return I_inv * cm;
    }

    s8 NearZero( fpt a ) {
        return ( a < 0.01 && a > -.01 );
    }

    s8 NearOne( fpt a ) {
        return NearZero( a - 1 );
    }
};

}  // namespace linear_mpc

#endif  // SOLVE_MPC__SOLVE_LINEAR_MPC_HPP_