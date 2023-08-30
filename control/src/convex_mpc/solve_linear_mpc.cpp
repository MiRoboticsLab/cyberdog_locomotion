#include <unsupported/Eigen/MatrixFunctions>

#include "convex_mpc/solve_linear_mpc.hpp"

namespace linear_mpc {

// #define K_PRINT_EVERYTHING
#define BIG_NUMBER 5e10

/**
 * @brief Construct a new Solve Linear Mpc:: Solve Linear Mpc object
 *
 * @param horizon prediction horizon length
 */
SolveLinearMpc::SolveLinearMpc( const s16 horizon ) : horizon_length_( horizon ) {
    solve_success_  = false;
    real_allocated_ = 0;
    ResizeQpMatrix( horizon_length_ );
}

/**
 * @brief Destroy the Solve Linear Mpc:: Solve Linear Mpc object
 *
 */
SolveLinearMpc::~SolveLinearMpc() {
    if ( real_allocated_ ) {
        free( hessian_qpoases_ );
        free( gradient_qpoases_ );
        free( constraint_qpoases_ );
        free( lower_boundary_qpoases_ );
        free( upper_boundary_qpoases_ );
        free( control_solution_ );
        free( hessian_qpoases_simple_ );
        free( gradient_qpoases_simple_ );
        free( constraint_qpoases_simple_ );
        free( lower_boundary_qpoases_simple_ );
        free( upper_boundary_qpoases_simple_ );
        free( control_solution_simple_ );
    }
}

/**
 * @brief Resize all the dynamic matrix when initializing or changing horizon length
 *
 * @param horizon prediction horizon length
 */
void SolveLinearMpc::ResizeQpMatrix( const s16 horizon ) {
    horizon_length_ = horizon;

    int mcount = 0;
    int h2     = horizon * horizon;

    state_matrix_qp_.resize( state_variables_dim_ * horizon, Eigen::NoChange );
    mcount += state_variables_dim_ * horizon * 1;

    input_matrix_qp_.resize( state_variables_dim_ * horizon, control_variables_dim_ * horizon );
    mcount += state_variables_dim_ * h2 * control_variables_dim_;

    weight_matrix_state_.resize( state_variables_dim_ * horizon );
    mcount += state_variables_dim_ * state_variables_dim_ * h2;

    weight_matrix_control_.resize( control_variables_dim_ * horizon );
    mcount += control_variables_dim_ * control_variables_dim_ * h2;

    identity_matrix_12_.resize( control_variables_dim_ * horizon, control_variables_dim_ * horizon );
    mcount += control_variables_dim_ * control_variables_dim_ * h2;

    state_vector_ref_.resize( state_variables_dim_ * horizon, Eigen::NoChange );
    mcount += state_variables_dim_ * horizon;

    control_vector_ref_.resize( control_variables_dim_ * horizon, Eigen::NoChange );
    mcount += control_variables_dim_ * horizon;

    upper_boundary_.resize( constraint_dim_ * horizon, Eigen::NoChange );
    mcount += constraint_dim_ * horizon;

    constraint_matrix_qp_.resize( constraint_dim_ * horizon, control_variables_dim_ * horizon );
    mcount += constraint_dim_ * control_variables_dim_ * h2;

    hessian_matrix_qp_.resize( control_variables_dim_ * horizon, control_variables_dim_ * horizon );
    mcount += control_variables_dim_ * control_variables_dim_ * h2;

    gradient_vector_qp_.resize( control_variables_dim_ * horizon, Eigen::NoChange );
    mcount += control_variables_dim_ * horizon;

    control_vector_.resize( control_variables_dim_ * horizon, Eigen::NoChange );
    mcount += control_variables_dim_ * horizon * 1;

    state_vector_.resize( state_variables_dim_ * horizon, Eigen::NoChange );
    mcount += state_variables_dim_ * horizon * 1;

    printf( "[Solver] reallocated floating point numbers: %d\n", mcount );
    mcount = 0;

    state_matrix_qp_.setZero();
    input_matrix_qp_.setZero();
    control_vector_.setZero();
    state_vector_.setZero();
    weight_matrix_state_.setZero();
    weight_matrix_control_.setZero();
    state_vector_ref_.setZero();
    upper_boundary_.setZero();
    constraint_matrix_qp_.setZero();
    hessian_matrix_qp_.setZero();
    gradient_vector_qp_.setZero();
    identity_matrix_12_.setIdentity();

    if ( real_allocated_ ) {
        free( hessian_qpoases_ );
        free( gradient_qpoases_ );
        free( constraint_qpoases_ );
        free( lower_boundary_qpoases_ );
        free( upper_boundary_qpoases_ );
        free( control_solution_ );
        free( hessian_qpoases_simple_ );
        free( gradient_qpoases_simple_ );
        free( constraint_qpoases_simple_ );
        free( lower_boundary_qpoases_simple_ );
        free( upper_boundary_qpoases_simple_ );
        free( control_solution_simple_ );
    }

    hessian_qpoases_ = ( qpOASES::real_t* )malloc( control_variables_dim_ * control_variables_dim_ * horizon * horizon * sizeof( qpOASES::real_t ) );
    mcount += control_variables_dim_ * control_variables_dim_ * h2;
    gradient_qpoases_ = ( qpOASES::real_t* )malloc( control_variables_dim_ * 1 * horizon * sizeof( qpOASES::real_t ) );
    mcount += control_variables_dim_ * horizon;
    constraint_qpoases_ = ( qpOASES::real_t* )malloc( control_variables_dim_ * constraint_dim_ * horizon * horizon * sizeof( qpOASES::real_t ) );
    mcount += control_variables_dim_ * constraint_dim_ * h2;
    lower_boundary_qpoases_ = ( qpOASES::real_t* )malloc( constraint_dim_ * 1 * horizon * sizeof( qpOASES::real_t ) );
    mcount += constraint_dim_ * horizon;
    upper_boundary_qpoases_ = ( qpOASES::real_t* )malloc( constraint_dim_ * 1 * horizon * sizeof( qpOASES::real_t ) );
    mcount += constraint_dim_ * horizon;
    control_solution_ = ( qpOASES::real_t* )malloc( control_variables_dim_ * horizon * sizeof( qpOASES::real_t ) );
    mcount += control_variables_dim_ * horizon;

    hessian_qpoases_simple_ = ( qpOASES::real_t* )malloc( control_variables_dim_ * control_variables_dim_ * horizon * horizon * sizeof( qpOASES::real_t ) );
    mcount += control_variables_dim_ * control_variables_dim_ * h2;
    gradient_qpoases_simple_ = ( qpOASES::real_t* )malloc( control_variables_dim_ * 1 * horizon * sizeof( qpOASES::real_t ) );
    mcount += control_variables_dim_ * horizon;
    constraint_qpoases_simple_ = ( qpOASES::real_t* )malloc( control_variables_dim_ * constraint_dim_ * horizon * horizon * sizeof( qpOASES::real_t ) );
    mcount += control_variables_dim_ * constraint_dim_ * h2;
    lower_boundary_qpoases_simple_ = ( qpOASES::real_t* )malloc( constraint_dim_ * 1 * horizon * sizeof( qpOASES::real_t ) );
    mcount += constraint_dim_ * horizon;
    upper_boundary_qpoases_simple_ = ( qpOASES::real_t* )malloc( constraint_dim_ * 1 * horizon * sizeof( qpOASES::real_t ) );
    mcount += constraint_dim_ * horizon;
    control_solution_simple_ = ( qpOASES::real_t* )malloc( control_variables_dim_ * horizon * sizeof( qpOASES::real_t ) );
    mcount += control_variables_dim_ * horizon;

    printf( "[Solver] malloced floating point numbers: %d\n", mcount );

    real_allocated_ = 1;

    horizon_length2_        = horizon * horizon;
    horizon_length2_m12m12_ = horizon_length2_ * control_variables_dim_ * control_variables_dim_;
    horizon_length2_m12m20_ = horizon_length2_ * control_variables_dim_ * constraint_dim_;
    horizon_length_m12_     = horizon * control_variables_dim_;
    horizon_length_m20_     = horizon * constraint_dim_;

#ifdef K_PRINT_EVERYTHING
    printf( "[Solver] resize matrixs for horizon: %d\n", horizon );
#endif
}

/**
 * @brief Calculate state and input matrix of continuous state space equation
 *
 * @param inertia_world robot base's inertia on world frame
 * @param mass robot mass
 * @param r_feet foothold position on absolute body frame
 * @param rotation_yaw rotation matrix with yaw direction
 * @param xy_drag manual gain for z direction according to x-y velocity
 */
void SolveLinearMpc::GetStateEquationMatrix( Matrix< fpt, 3, 3 > inertia_world, fpt mass, Matrix< fpt, 3, 4 > position_feet, Matrix< fpt, 3, 3 > rotation_yaw, float xy_drag[ 2 ] ) {
    state_matrix_continuous_time_.setZero();
    state_matrix_continuous_time_( 11, 9 )            = xy_drag[ 0 ];
    state_matrix_continuous_time_( 11, 10 )           = xy_drag[ 1 ];
    state_matrix_continuous_time_.block( 3, 9, 3, 3 ) = Matrix< fpt, 3, 3 >::Identity();

    state_matrix_continuous_time_( 11, 12 )           = 1.f;  // augmentation for G vector
    state_matrix_continuous_time_.block( 0, 6, 3, 3 ) = rotation_yaw.transpose();

    input_matrix_continuous_time_.setZero();
    Matrix< fpt, 3, 3 > inertia_world_inverse = inertia_world.inverse();

    for ( s16 b = 0; b < 4; b++ ) {
        input_matrix_continuous_time_.block( 6, b * 3, 3, 3 ) = CrossMatrix( inertia_world_inverse, position_feet.col( b ) );
        input_matrix_continuous_time_.block( 9, b * 3, 3, 3 ) = Matrix< fpt, 3, 3 >::Identity() / mass;
    }
}

/**
 * @brief Discrete state space equation and integrate them(all horizon) to one equation
 *
 * @param dt time interval of mpc horizon
 * @param horizon prediction horizon length
 */
void SolveLinearMpc::DiscreteAndGetQpMatrix( fpt dt_mpc, s16 horizon ) {
    state_input_augmented_matrix_.setZero();
    state_input_augmented_matrix_.block( 0, 0, state_variables_dim_, state_variables_dim_ )                      = state_matrix_continuous_time_;
    state_input_augmented_matrix_.block( 0, state_variables_dim_, state_variables_dim_, control_variables_dim_ ) = input_matrix_continuous_time_;

    state_input_augmented_matrix_ = dt_mpc * state_input_augmented_matrix_;

    exp_state_input_augmented_matrix_ = state_input_augmented_matrix_.exp();

    state_matrix_discrete_time_ = exp_state_input_augmented_matrix_.block( 0, 0, state_variables_dim_, state_variables_dim_ );
    input_matrix_discrete_time_ = exp_state_input_augmented_matrix_.block( 0, state_variables_dim_, state_variables_dim_, control_variables_dim_ );
#ifdef K_PRINT_EVERYTHING
    std::cout << "State Matrix(DT): \n" << state_matrix_discrete_time_ << "\nInput Matrix(DT):\n" << input_matrix_discrete_time_ << std::endl;
#endif
    static const int horizon_max = 30;
    if ( horizon > horizon_max ) {
        throw std::runtime_error( "[Solver] horizon is too long!" );
    }

    static Matrix< fpt, state_variables_dim_, state_variables_dim_ > iteration_coeff_mat[ horizon_max + 1 ];
    iteration_coeff_mat[ 0 ].setIdentity();
    for ( int i = 1; i < horizon + 1; i++ ) {
        iteration_coeff_mat[ i ] = state_matrix_discrete_time_ * iteration_coeff_mat[ i - 1 ];
    }

    for ( s16 row = 0; row < horizon; row++ ) {
        state_matrix_qp_.block( state_variables_dim_ * row, 0, state_variables_dim_, state_variables_dim_ ) = iteration_coeff_mat[ row + 1 ];
        for ( s16 col = 0; col < horizon; col++ ) {
            if ( row >= col ) {
                s16 offset_num = row - col;
                input_matrix_qp_.block( state_variables_dim_ * row, control_variables_dim_ * col, state_variables_dim_, control_variables_dim_ ) =
                    iteration_coeff_mat[ offset_num ] * input_matrix_discrete_time_;
            }
        }
    }
}

/**
 * @brief Construct QP problem and solve it by qpoases
 *
 * @param prob_update update current robot states and reference states on mpctable
 * @param prob_setup defined params for linear mpc and physics model
 */
void SolveLinearMpc::SolveMpc( MpcSolverData* prob_update, MpcSolverParameters* prob_setup ) {
    static Matrix< fpt, 3, 3 > inertia_world, inertia_body, rotation_yaw;
    static Matrix< fpt, 3, 4 > position_feet;

    fpt yc = cos( prob_update->body_rpy[ 2 ] );
    fpt ys = sin( prob_update->body_rpy[ 2 ] );

    rotation_yaw << yc, -ys, 0, ys, yc, 0, 0, 0, 1;
    inertia_body.diagonal() = prob_setup->inertia_body;
    inertia_world           = rotation_yaw * inertia_body * rotation_yaw.transpose();
    for ( u8 row = 0; row < 3; row++ )
        for ( u8 col = 0; col < 4; col++ )
            position_feet( row, col ) = prob_update->feet_pos[ row * 4 + col ];

    // initial state
    state_init_ << prob_update->body_rpy, prob_update->body_pos, prob_update->body_omega, prob_update->body_vel, -9.81f;

    GetStateEquationMatrix( inertia_world, prob_setup->mass, position_feet, rotation_yaw, prob_setup->xy_drag );

#ifdef K_PRINT_EVERYTHING
    std::cout << "Initial State: \n" << state_init_ << std::endl;
    std::cout << "World Inertia: \n" << inertia_world << std::endl;
    std::cout << "State Matrix(CT): \n" << state_matrix_continuous_time_ << std::endl;
    std::cout << "Input Matrix(CT): \n" << input_matrix_continuous_time_ << std::endl;
#endif
    // QP matrices
    DiscreteAndGetQpMatrix( prob_setup->dt_mpc, prob_setup->horizon_length );

    // weights
    static Matrix< fpt, state_variables_dim_, 1 > weight_state;
    for ( u8 i = 0; i < 12; i++ )
        weight_state( i ) = prob_setup->weight_state[ i ];
    weight_state( 12 )              = 0.f;
    weight_matrix_state_.diagonal() = weight_state.replicate( prob_setup->horizon_length, 1 );

    // reference trajectory
    for ( s16 i = 0; i < prob_setup->horizon_length; i++ ) {
        for ( s16 j = 0; j < 12; j++ )
            state_vector_ref_( state_variables_dim_ * i + j, 0 ) = prob_update->state_reference_trajectory[ 12 * i + j ];
    }

    // friction cone constraint
    s16 k = 0;
    for ( s16 i = 0; i < prob_setup->horizon_length; i++ ) {
        for ( s16 j = 0; j < 4; j++ ) {
            upper_boundary_( 5 * k + 0 ) = BIG_NUMBER;
            upper_boundary_( 5 * k + 1 ) = BIG_NUMBER;
            upper_boundary_( 5 * k + 2 ) = BIG_NUMBER;
            upper_boundary_( 5 * k + 3 ) = BIG_NUMBER;
            upper_boundary_( 5 * k + 4 ) = prob_update->gait_state_mpc_table[ i * 4 + j ] * prob_setup->force_max;
            k++;
        }
    }
    fpt                        mu = 1.f / prob_setup->friction_coefficient;
    static Matrix< fpt, 5, 3 > friction_cone_block;
    friction_cone_block << mu, 0, 1.f, -mu, 0, 1.f, 0, mu, 1.f, 0, -mu, 1.f, 0, 0, 1.f;

    for ( s16 i = 0; i < prob_setup->horizon_length * 4; i++ ) {
        constraint_matrix_qp_.block( i * 5, i * 3, 5, 3 ) = friction_cone_block;
    }

    // computer coeff matrix of H and g for QP
    input_matrix_trans_x_weight_matrix_state_ = input_matrix_qp_.transpose() * weight_matrix_state_;
    hessian_matrix_qp_                        = 2 * ( input_matrix_trans_x_weight_matrix_state_ * input_matrix_qp_ + prob_setup->weight_force[ 0 ] * identity_matrix_12_ );
    gradient_vector_qp_                       = 2 * input_matrix_trans_x_weight_matrix_state_ * ( state_matrix_qp_ * state_init_ - state_vector_ref_ );

    memcpy( ( void* )hessian_qpoases_, ( void* )hessian_matrix_qp_.data(), sizeof( float ) * horizon_length2_m12m12_ );
    memcpy( ( void* )gradient_qpoases_, ( void* )gradient_vector_qp_.data(), sizeof( float ) * horizon_length_m12_ );
    memcpy( ( void* )constraint_qpoases_, ( void* )constraint_matrix_qp_.data(), sizeof( float ) * horizon_length2_m12m20_ );
    memcpy( ( void* )upper_boundary_qpoases_, ( void* )upper_boundary_.data(), sizeof( float ) * horizon_length_m20_ );

    for ( s16 i = 0; i < constraint_dim_ * prob_setup->horizon_length; i++ )
        lower_boundary_qpoases_[ i ] = 0.0f;
    // lower_boundary_qpoases_.setZero();  // much more time cost

    // find the index that U is zero
    s16 num_constraints     = constraint_dim_ * prob_setup->horizon_length;
    s16 num_variables       = control_variables_dim_ * prob_setup->horizon_length;
    s16 new_num_variables   = num_variables;
    s16 new_num_constraints = num_constraints;

    for ( int i = 0; i < num_constraints; i++ )
        constraint_eliminate_[ i ] = 0;

    for ( int i = 0; i < num_variables; i++ )
        variable_eliminate_[ i ] = 0;

    for ( int i = 0; i < num_constraints; i++ ) {
        if ( !( NearZero( lower_boundary_qpoases_[ i ] ) && NearZero( upper_boundary_qpoases_[ i ] ) ) )
            continue;
        float* c_row = &constraint_qpoases_[ i * num_variables ];
        for ( int j = 0; j < num_variables; j++ ) {
            if ( NearOne( c_row[ j ] ) ) {
                new_num_variables -= 3;
                new_num_constraints -= 5;
                int cs                          = ( j * 5 ) / 3 - 3;
                variable_eliminate_[ j - 2 ]    = 1;
                variable_eliminate_[ j - 1 ]    = 1;
                variable_eliminate_[ j ]        = 1;
                constraint_eliminate_[ cs ]     = 1;
                constraint_eliminate_[ cs + 1 ] = 1;
                constraint_eliminate_[ cs + 2 ] = 1;
                constraint_eliminate_[ cs + 3 ] = 1;
                constraint_eliminate_[ cs + 4 ] = 1;
            }
        }
    }

    // eliminate useless(U==0) segment for matrix H g C u
    if ( new_num_variables != num_variables ) {
        int remain_var_index[ new_num_variables ];
        int remain_con_index[ new_num_constraints ];
        int num = 0;
        for ( int i = 0; i < num_variables; i++ ) {
            if ( !variable_eliminate_[ i ] ) {
                if ( !( num < new_num_variables ) ) {
                    printf( "[Solver] bad error!\n" );
                }
                remain_var_index[ num ] = i;
                num++;
            }
        }
        num = 0;
        for ( int i = 0; i < num_constraints; i++ ) {
            if ( !constraint_eliminate_[ i ] ) {
                if ( !( num < new_num_constraints ) ) {
                    printf( "[Solver] bad error!\n" );
                }
                remain_con_index[ num ] = i;
                num++;
            }
        }
        for ( int i = 0; i < new_num_variables; i++ ) {
            int index_row                 = remain_var_index[ i ];
            gradient_qpoases_simple_[ i ] = gradient_qpoases_[ index_row ];
            for ( int j = 0; j < new_num_variables; j++ ) {
                int index_col                                        = remain_var_index[ j ];
                hessian_qpoases_simple_[ i * new_num_variables + j ] = hessian_qpoases_[ index_row * num_variables + index_col ];
            }
        }

        for ( int con = 0; con < new_num_constraints; con++ ) {
            for ( int st = 0; st < new_num_variables; st++ ) {
                float cval                                                 = constraint_qpoases_[ ( num_variables * remain_con_index[ con ] ) + remain_var_index[ st ] ];
                constraint_qpoases_simple_[ con * new_num_variables + st ] = cval;
            }
        }
        for ( int i = 0; i < new_num_constraints; i++ ) {
            int index                           = remain_con_index[ i ];
            upper_boundary_qpoases_simple_[ i ] = upper_boundary_qpoases_[ index ];
            lower_boundary_qpoases_simple_[ i ] = lower_boundary_qpoases_[ index ];
        }
    }

    // construct qp solver
    qpOASES::int_t    max_iterations = 100;
    qpOASES::QProblem problem_red( new_num_variables, new_num_constraints );
    qpOASES::Options  op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem_red.setOptions( op );

    if ( new_num_variables != num_variables ) {
        int rval = problem_red.init( hessian_qpoases_simple_, gradient_qpoases_simple_, constraint_qpoases_simple_, NULL, NULL, lower_boundary_qpoases_simple_, upper_boundary_qpoases_simple_,
                                     max_iterations );
        ( void )rval;

        int rval2 = problem_red.getPrimalSolution( control_solution_simple_ );
        if ( rval2 != qpOASES::SUCCESSFUL_RETURN ) {
            printf( "[Solver] failed to solve!\n" );
            solve_success_ = false;
        }
        else
            solve_success_ = true;

        int num = 0;
        for ( int i = 0; i < num_variables; i++ ) {
            if ( variable_eliminate_[ i ] ) {
                control_solution_[ i ] = 0.0f;
            }
            else {
                control_solution_[ i ] = control_solution_simple_[ num ];
                num++;
            }
        }
    }
    else {
        int rval = problem_red.init( hessian_qpoases_, gradient_qpoases_, constraint_qpoases_, NULL, NULL, lower_boundary_qpoases_, upper_boundary_qpoases_, max_iterations );
        ( void )rval;

        int rval2 = problem_red.getPrimalSolution( control_solution_ );
        if ( rval2 != qpOASES::SUCCESSFUL_RETURN ) {
            printf( "[Solver] failed to solve!\n" );
            solve_success_ = false;
        }
        else
            solve_success_ = true;
    }
}

/**
 * @brief Construct QP problem and solve it by qpoases
 *
 * @param prob_update update current robot states and reference states on mpctable
 * @param prob_setup defined params for linear mpc and physics model
 */
void SolveLinearMpc::SolveMpcWithCtrlRef( MpcSolverData* prob_update, MpcSolverParameters* prob_setup ) {
    static Matrix< fpt, 3, 3 > inertia_world, inertia_body, rotation_yaw;
    static Matrix< fpt, 3, 4 > position_feet;

    fpt yc = cos( prob_update->body_rpy[ 2 ] );
    fpt ys = sin( prob_update->body_rpy[ 2 ] );

    rotation_yaw << yc, -ys, 0, ys, yc, 0, 0, 0, 1;
    inertia_body.diagonal() = prob_setup->inertia_body;
    inertia_world           = rotation_yaw * inertia_body * rotation_yaw.transpose();
    for ( u8 row = 0; row < 3; row++ )
        for ( u8 col = 0; col < 4; col++ )
            position_feet( row, col ) = prob_update->feet_pos[ row * 4 + col ];

    // initial state
    state_init_ << prob_update->body_rpy, prob_update->body_pos, prob_update->body_omega, prob_update->body_vel, -9.81f;

    GetStateEquationMatrix( inertia_world, prob_setup->mass, position_feet, rotation_yaw, prob_setup->xy_drag );

#ifdef K_PRINT_EVERYTHING
    std::cout << "Initial State: \n" << state_init_ << std::endl;
    std::cout << "World Inertia: \n" << inertia_world << std::endl;
    std::cout << "State Matrix(CT): \n" << state_matrix_continuous_time_ << std::endl;
    std::cout << "Input Matrix(CT): \n" << input_matrix_continuous_time_ << std::endl;
#endif
    // QP matrices
    DiscreteAndGetQpMatrix( prob_setup->dt_mpc, prob_setup->horizon_length );

    // weights
    static Matrix< fpt, state_variables_dim_, 1 >   weight_state;
    static Matrix< fpt, control_variables_dim_, 1 > weight_force;
    for ( u8 i = 0; i < state_variables_dim_ - 1; i++ ) {
        weight_state( i ) = prob_setup->weight_state[ i ];
    }
    weight_state( 12 ) = 0.f;
    for ( u8 i = 0; i < control_variables_dim_; i++ ) {
        weight_force( i ) = prob_setup->weight_force[ i ];
    }
    weight_matrix_state_.diagonal()   = weight_state.replicate( prob_setup->horizon_length, 1 );
    weight_matrix_control_.diagonal() = weight_force.replicate( prob_setup->horizon_length, 1 );

    // reference trajectory
    for ( s16 i = 0; i < prob_setup->horizon_length; i++ ) {
        for ( s16 j = 0; j < state_variables_dim_ - 1; j++ ) {
            state_vector_ref_( state_variables_dim_ * i + j, 0 ) = prob_update->state_reference_trajectory[ 12 * i + j ];
        }
        for ( s16 k = 0; k < control_variables_dim_; k++ ) {
            if ( prob_update->ctrl_reference_enabled ) {
                control_vector_ref_( control_variables_dim_ * i + k, 0 ) = prob_update->ctrl_reference_trajectory[ control_variables_dim_ * i + k ];
            }
            else {
                control_vector_ref_( control_variables_dim_ * i + k, 0 ) = 0.0;
            }
        }
    }

    // friction cone constraint
    s16 k = 0;
    for ( s16 i = 0; i < prob_setup->horizon_length; i++ ) {
        for ( s16 j = 0; j < 4; j++ ) {
            upper_boundary_( 5 * k + 0 ) = BIG_NUMBER;
            upper_boundary_( 5 * k + 1 ) = BIG_NUMBER;
            upper_boundary_( 5 * k + 2 ) = BIG_NUMBER;
            upper_boundary_( 5 * k + 3 ) = BIG_NUMBER;
            upper_boundary_( 5 * k + 4 ) = prob_update->gait_state_mpc_table[ i * 4 + j ] * prob_setup->force_max;
            k++;
        }
    }
    fpt                        mu = 1.f / prob_setup->friction_coefficient;
    static Matrix< fpt, 5, 3 > friction_cone_block;
    friction_cone_block << mu, 0, 1.f, -mu, 0, 1.f, 0, mu, 1.f, 0, -mu, 1.f, 0, 0, 1.f;

    for ( s16 i = 0; i < prob_setup->horizon_length * 4; i++ ) {
        constraint_matrix_qp_.block( i * 5, i * 3, 5, 3 ) = friction_cone_block;
    }

    // computer coeff matrix of H and g for QP
    input_matrix_trans_x_weight_matrix_state_ = input_matrix_qp_.transpose() * weight_matrix_state_;
    hessian_matrix_qp_                        = 2 * ( input_matrix_trans_x_weight_matrix_state_ * input_matrix_qp_ + weight_matrix_control_ * identity_matrix_12_ );
    gradient_vector_qp_ = 2 * ( input_matrix_trans_x_weight_matrix_state_ * ( state_matrix_qp_ * state_init_ - state_vector_ref_ ) - weight_matrix_control_ * control_vector_ref_ );
    // hessian_matrix_qp_                        = 2 * ( input_matrix_trans_x_weight_matrix_state_ * input_matrix_qp_ + prob_setup->weight_force[ 0 ] * identity_matrix_12_ );
    // gradient_vector_qp_                       = 2 * input_matrix_trans_x_weight_matrix_state_ * ( state_matrix_qp_ * state_init_ - state_vector_ref_ );

    memcpy( ( void* )hessian_qpoases_, ( void* )hessian_matrix_qp_.data(), sizeof( float ) * horizon_length2_m12m12_ );
    memcpy( ( void* )gradient_qpoases_, ( void* )gradient_vector_qp_.data(), sizeof( float ) * horizon_length_m12_ );
    memcpy( ( void* )constraint_qpoases_, ( void* )constraint_matrix_qp_.data(), sizeof( float ) * horizon_length2_m12m20_ );
    memcpy( ( void* )upper_boundary_qpoases_, ( void* )upper_boundary_.data(), sizeof( float ) * horizon_length_m20_ );

    for ( s16 i = 0; i < constraint_dim_ * prob_setup->horizon_length; i++ )
        lower_boundary_qpoases_[ i ] = 0.0f;
    // lower_boundary_qpoases_.setZero();  // much more time cost

    // find the index that U is zero
    s16 num_constraints     = constraint_dim_ * prob_setup->horizon_length;
    s16 num_variables       = control_variables_dim_ * prob_setup->horizon_length;
    s16 new_num_variables   = num_variables;
    s16 new_num_constraints = num_constraints;

    for ( int i = 0; i < num_constraints; i++ )
        constraint_eliminate_[ i ] = 0;

    for ( int i = 0; i < num_variables; i++ )
        variable_eliminate_[ i ] = 0;

    for ( int i = 0; i < num_constraints; i++ ) {
        if ( !( NearZero( lower_boundary_qpoases_[ i ] ) && NearZero( upper_boundary_qpoases_[ i ] ) ) )
            continue;
        float* c_row = &constraint_qpoases_[ i * num_variables ];
        for ( int j = 0; j < num_variables; j++ ) {
            if ( NearOne( c_row[ j ] ) ) {
                new_num_variables -= 3;
                new_num_constraints -= 5;
                int cs                          = ( j * 5 ) / 3 - 3;
                variable_eliminate_[ j - 2 ]    = 1;
                variable_eliminate_[ j - 1 ]    = 1;
                variable_eliminate_[ j ]        = 1;
                constraint_eliminate_[ cs ]     = 1;
                constraint_eliminate_[ cs + 1 ] = 1;
                constraint_eliminate_[ cs + 2 ] = 1;
                constraint_eliminate_[ cs + 3 ] = 1;
                constraint_eliminate_[ cs + 4 ] = 1;
            }
        }
    }

    // eliminate useless(U==0) segment for matrix H g C u
    if ( new_num_variables != num_variables ) {
        int remain_var_index[ new_num_variables ];
        int remain_con_index[ new_num_constraints ];
        int num = 0;
        for ( int i = 0; i < num_variables; i++ ) {
            if ( !variable_eliminate_[ i ] ) {
                if ( !( num < new_num_variables ) ) {
                    printf( "[Solver] bad error!\n" );
                }
                remain_var_index[ num ] = i;
                num++;
            }
        }
        num = 0;
        for ( int i = 0; i < num_constraints; i++ ) {
            if ( !constraint_eliminate_[ i ] ) {
                if ( !( num < new_num_constraints ) ) {
                    printf( "[Solver] bad error!\n" );
                }
                remain_con_index[ num ] = i;
                num++;
            }
        }
        for ( int i = 0; i < new_num_variables; i++ ) {
            int index_row                 = remain_var_index[ i ];
            gradient_qpoases_simple_[ i ] = gradient_qpoases_[ index_row ];
            for ( int j = 0; j < new_num_variables; j++ ) {
                int index_col                                        = remain_var_index[ j ];
                hessian_qpoases_simple_[ i * new_num_variables + j ] = hessian_qpoases_[ index_row * num_variables + index_col ];
            }
        }

        for ( int con = 0; con < new_num_constraints; con++ ) {
            for ( int st = 0; st < new_num_variables; st++ ) {
                float cval                                                 = constraint_qpoases_[ ( num_variables * remain_con_index[ con ] ) + remain_var_index[ st ] ];
                constraint_qpoases_simple_[ con * new_num_variables + st ] = cval;
            }
        }
        for ( int i = 0; i < new_num_constraints; i++ ) {
            int index                           = remain_con_index[ i ];
            upper_boundary_qpoases_simple_[ i ] = upper_boundary_qpoases_[ index ];
            lower_boundary_qpoases_simple_[ i ] = lower_boundary_qpoases_[ index ];
        }
    }

    // construct qp solver
    qpOASES::int_t    max_iterations = 100;
    qpOASES::QProblem problem_red( new_num_variables, new_num_constraints );
    qpOASES::Options  op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem_red.setOptions( op );

    if ( new_num_variables != num_variables ) {
        int rval = problem_red.init( hessian_qpoases_simple_, gradient_qpoases_simple_, constraint_qpoases_simple_, NULL, NULL, lower_boundary_qpoases_simple_, upper_boundary_qpoases_simple_,
                                     max_iterations );
        ( void )rval;

        int rval2 = problem_red.getPrimalSolution( control_solution_simple_ );
        if ( rval2 != qpOASES::SUCCESSFUL_RETURN ) {
            printf( "[Solver] failed to solve!\n" );
            solve_success_ = false;
        }
        else
            solve_success_ = true;

        int num = 0;
        for ( int i = 0; i < num_variables; i++ ) {
            if ( variable_eliminate_[ i ] ) {
                control_solution_[ i ] = 0.0f;
            }
            else {
                control_solution_[ i ] = control_solution_simple_[ num ];
                num++;
            }
        }
    }
    else {
        int rval = problem_red.init( hessian_qpoases_, gradient_qpoases_, constraint_qpoases_, NULL, NULL, lower_boundary_qpoases_, upper_boundary_qpoases_, max_iterations );
        ( void )rval;

        int rval2 = problem_red.getPrimalSolution( control_solution_ );
        if ( rval2 != qpOASES::SUCCESSFUL_RETURN ) {
            printf( "[Solver] failed to solve!\n" );
            solve_success_ = false;
        }
        else
            solve_success_ = true;
    }
}

/**
 * @brief Solution of control variables
 *
 * @return fpt*
 */
fpt* SolveLinearMpc::GetControlVarSolution() {
    return control_solution_;
}

/**
 * @brief Computer optimized state variables of all horizon
 *
 */
void SolveLinearMpc::GetOptStateVarAllHorizon() {
    for ( int i = 0; i < horizon_length_; i++ ) {
        for ( int j = 0; j < control_variables_dim_; j++ )
            control_vector_[ control_variables_dim_ * i + j ] = control_solution_[ control_variables_dim_ * i + j ];
    }
    state_vector_ = state_matrix_qp_ * state_init_ + input_matrix_qp_ * control_vector_;
    return;
}

/**
 * @brief Computer optimized state variables of next horizon
 *
 * @return fpt*
 */
fpt* SolveLinearMpc::GetOptStateVarNextHorizon() {
    GetOptStateVarAllHorizon();
    return &state_vector_[ state_variables_dim_ ];
}

/**
 * @brief Return solution of control variables
 *
 * @param index
 * @return float
 */
float SolveLinearMpc::GetSolution( int index ) {
    fpt* qs = GetControlVarSolution();
    return qs[ index ];
}

/**
 * @brief Return optimized state variables
 *
 * @param index
 * @return float
 */
float SolveLinearMpc::GetOptStateVar( int index ) {
    fpt* xs = GetOptStateVarNextHorizon();
    return xs[ index ];
}

}  // namespace linear_mpc