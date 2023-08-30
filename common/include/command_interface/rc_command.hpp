#ifndef RC_COMMAND_HPP_
#define RC_COMMAND_HPP_

enum AT9s_SwitchStateBool {
    AT9S_BOOL_UP   = 0,
    AT9S_BOOL_DOWN = 1,
};

enum AT9s_SwitchStateTri {
    AT9S_TRI_UP     = 0,
    AT9S_TRI_MIDDLE = 1,
    AT9S_TRI_DOWN   = 2,
};

enum T8S_SwitchStateTri {
    T8S_TRI_UP     = 0,
    T8S_TRI_MIDDLE = 1,
    T8S_TRI_DOWN   = 2,
};

enum T8S_SwitchStateBool {
    T8S_BOOL_UP   = 0,
    T8S_BOOL_DOWN = 1,
};

/**
 * @brief Remote Control Command interface, currently use AT9S
 */
class RcCommand {
public:
    AT9s_SwitchStateBool SWF, SWA, SWB, SWD;
    AT9s_SwitchStateTri  SWE, SWC, SWG;
    T8S_SwitchStateTri   CH7, CH5;
    T8S_SwitchStateBool  CH6;
    float                left_stick_x, left_stick_y;
    float                right_stick_x, right_stick_y;
    float                varB;
    int                  rc_type;
    int                  err_count;
};

#endif  // RC_COMMAND_HPP_
