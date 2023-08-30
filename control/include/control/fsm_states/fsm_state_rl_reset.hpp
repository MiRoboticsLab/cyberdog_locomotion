#ifndef FSM_STATE_RL_RESET_HPP_
#define FSM_STATE_RL_RESET_HPP_

#include <cstring>
#include <experimental/filesystem>
#include <sys/timerfd.h>

#include "controllers/leg_controller.hpp"
#include "controllers/state_estimator_container.hpp"
#include "cpu_mlp.hpp"
#include "fsm_state.hpp"

static const int kNumObsReset        = 39;
static const int kNumObsHistoryReset = 3;

/**
 * @brief FSM state for fast recovery using reinforcement learning.
 *
 */
template < typename T > class FsmStateRlReset : public FsmState< T > {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FsmStateRlReset( ControlFsmData< T >* control_fsm_data );

    // Behavior to be carried out when entering a state
    void OnEnter();

    // Run the normal behavior for the state
    void Run();

    // Checks for any transition triggers
    FsmStateName CheckTransition();

    // Manages state specific transitions
    TransitionData< T > Transition();

    // Behavior to be carried out when exiting a state
    void OnExit();

    // Remap joint pos/vel for real robot (different in isaac gym)
    void Remap( Eigen::Matrix< float, 12, 1 >& joint_data );

    // Use thread to inference rl model
    void RlModelInferenceThread();
    void CreateRlModelInferenceThread();
    void StopRlModelInference();

    void SetPolicyParams();

    void UpdateAction();
    void UpdateHistory();
    void UpdateObservation();
    void UpdateProjectedGravity( Eigen::Vector4f& robot_quat, Eigen::Vector3f& gravity_vec );

    void CalculateMotionProgress();

private:
    // Keep track of the control iterations
    int iter_ = 0;

    // Observation
    Eigen::Matrix< float, 3, 1 >  projected_gravity_;
    Eigen::Matrix< float, 12, 1 > joint_pos_;
    Eigen::Matrix< float, 12, 1 > joint_vel_;
    Eigen::Matrix< float, 12, 1 > prev_action_;

    Eigen::Matrix< float, kNumObsReset, 1 >                       obs_;
    Eigen::Matrix< float, kNumObsReset * kNumObsHistoryReset, 1 > obs_history_;
    Eigen::Matrix< float, 12, 1 >                                 action_;
    Eigen::Matrix< float, 12, 1 >                                 default_joint_pos_mid_;
    Eigen::Matrix< float, 12, 1 >                                 default_joint_pos_range_;
    Eigen::Matrix< float, 12, 1 >                                 default_joint_pos_recovery_;
    Eigen::Matrix< float, 12, 1 >                                 joint_pos_normal_;
    Eigen::Matrix< int, 12, 1 >                                   motor_remap_;
    Eigen::Vector3f                                               gravity_vec_;
    Eigen::Vector4f                                               robot_quat_;  // (w, x, y, z)
    Eigen::Matrix< float, 12, 1 >                                 des_joint_pos_;

    // Motor Kp/Kd
    Eigen::Matrix< float, 3, 3 > kp_mat_;
    Eigen::Matrix< float, 3, 3 > kd_mat_;

    // Scale
    float joint_pos_scale_;
    float joint_vel_scale_;
    float action_scale_;

    float action_clip_;
    float action_delta_clip_;
    bool  use_action_delta_clip_;
    float cos_dist_, r_roll_, r_stand_, r_dofpos_, r_dofvel_, r_reset_;  // for calculating motion progress

    double      control_dt_;   // policy control period
    std::string load_path_;    // rl model parameters path

    cyberdog::cpu_mlp::MlpFullyConnected< float, kNumObsReset * kNumObsHistoryReset, 12, cyberdog::cpu_mlp::ActivationType::tanh > policy_;

    bool         rl_model_inference_running_;
    std::thread* rl_model_inference_thread_;

    // For final posture correction
    bool  correct_posture_flag_;
    int   slower_reset_correct_progress_;
    float slower_reset_correct_kp_;
};

#endif  // FSM_STATE_RL_RESET_HPP_
