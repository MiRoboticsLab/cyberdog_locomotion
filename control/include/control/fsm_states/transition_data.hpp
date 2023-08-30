#ifndef TRANSITION_DATA_HPP_
#define TRANSITION_DATA_HPP_

#include <Eigen/Dense>

#include "cpp_types.hpp"

/**
 * @brief Struct of relevant data that can be used during transition to pass
 * data between states.
 *
 */
template < typename T > struct TransitionData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    TransitionData() {
        ResetTransitionDone();
    }

    /**
     * @brief Reset the Transition Done object
     *
     */
    void ResetTransitionDone() {
        done = false;
    }

    // Flag to mark when transition is done
    bool done           = false;
    int  ban_trans_flag = 0;  // error transition event occur
};

template struct TransitionData< double >;
template struct TransitionData< float >;

#endif  // TRANSITION_DATA_HPP_
