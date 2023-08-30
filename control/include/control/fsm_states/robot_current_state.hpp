#ifndef ROBOT_CURRENT_STATE_HPP_
#define ROBOT_CURRENT_STATE_HPP_

#include <iostream>

template < typename T > struct RobotCurrentState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int  gait_id          = 0;
    int  gait_cmd_used    = 1;
    bool gait_check_trans = true;
    bool gait_allow_trans = true;
    bool step_on_slope    = false;
};

#endif  // ROBOT_CURRENT_STATE_HPP_
