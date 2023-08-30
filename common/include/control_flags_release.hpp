#ifndef CONTROL_FLAGS_RELEASE_HPP_
#define CONTROL_FLAGS_RELEASE_HPP_

#include <iostream>

// flags for control input switch
#define LCM_CONTROL 1
#define RC_CONTROL 0

#define LCM_CMD_CHANNEL_NAME "exec_request"

/**
 * @brief define a control flag X(flag_enum, flag_name, flag_num )
 * flag_enum: for cyberdog_control.
 * flag_name: for rviz display.
 * flag_num: for rviz display, same with value of flag_enum.
 */
#define MOTION_MODE                                \
    X( kInvalid = -1, "invlid", -1 )               \
    X( kOff = 0, "off", 0 )                        \
    X( kQpStand = 3, "qp stand", 3 )               \
    X( kPureDamper = 7, "pure damper", 7 )         \
    X( kLifted = 9, "lifted", 9 )                  \
    X( kLocomotion = 11, "locomotion", 11 )        \
    X( kRecoveryStand = 12, "recovery stand", 12 ) \
    X( kMotorCtrl = 15, "motor ctrl", 15 )        \
    X( kJump3d = 16, "jump 3D", 16 )               \
    X( kPoseCtrl = 21, "pose ctrl", 21 )           \
    X( kForceJump = 22, "force jump", 22 )         \
    X( kMotion = 62, "motion", 62 )                \
    X( kTwoLegStand = 64, "two leg stand", 64 )    \
    X( kRlReset = 80, "RL reset", 80 )             \
    X( kRlRapid = 81, "RL rapid", 81 )

#define X( mode, name, num ) mode,
enum MotionMode : int64_t { MOTION_MODE };
#undef X

// for rviz gui display
#define X( mode, name, num ) name,
__attribute__( ( unused ) ) static char const* motionmode_name[] = { MOTION_MODE };
#undef X

// for rviz gui display
#define X( mode, name, num ) num,
__attribute__( ( unused ) ) static int64_t motionmode_num[]{ MOTION_MODE };
#undef X

// list for rviz gui display
__attribute__( ( unused ) ) static char const* ID_list[] = { "gait ID", "jump ID", "filp ID", "motion ID", "dance ID" };

#define GAIT_ID                                         \
    X( kStand = 1, "stand", 1 )                         \
    X( kPronk = 2, "pronk", 2 )                         \
    X( kTrotMedium = 3, "trot user", 3 )                \
    X( kPassiveTrot = 4, "passive trot", 4 )            \
    X( kTrot10v4 = 5, "flying trot", 5 )                \
    X( kWalk = 6, "walk", 6 )                           \
    X( kBound = 7, "bound", 7 )                         \
    X( kPace = 8, "pace", 8 )                           \
    X( kTrot10v5 = 9, "trot 10-5", 9 )                  \
    X( kTrotFast = 10, "trot 8-4", 10 )                 \
    X( kTrot8v3 = 11, "trot 8-3", 11 )                  \
    X( kTrot8v3Follow = 12, "trot 8-3 follow", 12 )     \
    X( kTrot10v4Follow = 13, "trot 10-4 follow", 13 )   \
    X( kTrot12v6Follow = 14, "trot 12-6 follow", 14 )   \
    X( kTrot14v8Follow = 15, "trot 14-8 follow", 15 )   \
    X( kTrot16v10Follow = 16, "trot 16-10 follow", 16 ) \
    X( kTrot18v11Follow = 17, "trot 18-11 follow", 17 ) \
    X( kTrot22v14Follow = 18, "trot 22-14 follow", 18 ) \
    X( kTrot24v16Follow = 19, "trot 24-16 follow", 19 ) \
    X( kTrot12v6 = 20, "trot 12-6", 20 )                \
    X( kTrot14v8 = 21, "trot 14-8", 21 )                \
    X( kTrot16v10 = 22, "trot 16-10", 22 )              \
    X( kTrot18v11 = 23, "trot 18-11", 23 )              \
    X( kTrot20v12 = 24, "trot 20-12", 24 )              \
    X( kTrot22v14 = 25, "trot 22-14", 25 )              \
    X( kTrot24v16 = 26, "trot 24-16", 26 )              \
    X( kTrotSlow = 27, "trot slow", 27 )                \
    X( kTrot20v12Follow = 28, "trot 20-12 follow", 28 ) \
    X( kTrotAuto = 29, "trot auto", 29 )                \
    X( kStandPassive = 30, " stand passive", 30 )       \
    X( kStandNoPr = 31, "stand noPR", 31 )              \
    X( kNonPeriodic00 = 40, "nonperiodic 00", 40 )      \
    X( kNonPeriodic01 = 41, "nonperiodic 01", 41 )      \
    X( kNonPeriodic02 = 42, "nonperiodic 02", 42 )      \
    X( kNonPeriodic03 = 43, "nonperiodic 03", 43 )      \
    X( kNonPeriodic04 = 44, "nonperiodic 04", 44 )      \
    X( kNonPeriodic05 = 45, "nonperiodic 05", 45 )      \
    X( kNonPeriodic06 = 46, "nonperiodic 06", 46 )      \
    X( kNonPeriodic07 = 47, "nonperiodic 07", 47 )      \
    X( kNonPeriodic08 = 48, "nonperiodic 08", 48 )      \
    X( kNonPeriodic09 = 49, "nonperiodic 09", 49 )      \
    X( kSpecialPronk = 50, "special pronk", 50 )        \
    X( kSpecialTrot = 51, "special trot", 51 )          \
    X( kDiagonalLeft = 52, "diagonal left", 52 )        \
    X( kDiagonalRight = 53, "diagonal right", 53 )      \
    X( kTrotSwing = 55, "trot swing", 55 )              \
    X( kTrotInOut = 56, "trot in-out", 56 )             \
    X( kTrotPitch = 57, "trot pitch", 57 )              \
    X( kMoonwalkLeft = 58, "moonwalk left", 58 )        \
    X( kMoonwalkRight = 59, "moonwalk right", 59 )      \
    X( kWalkWave = 60, "walk wave", 60 )                \
    X( kFrontLiftIn = 61, "front-lift in", 61 )         \
    X( kFrontLiftLeft = 62, "front-lift left", 62 )     \
    X( kFrontLiftSwitch = 63, "front-lift switch", 63 ) \
    X( kRearLiftIn = 65, "rear-lift in", 65 )           \
    X( kRearLiftLeft = 66, "rear-lift left", 66 )       \
    X( kRearLiftSwitch = 67, "rear-lift switch", 67 )   \
    X( kBallet = 68, "ballet", 68 )                     \
    X( kBalletTrans = 69, "ballet trans", 69 )          \
    X( kPitchDownLeft = 70, "pitch down-left", 70 )     \
    X( kPitchDownRight = 71, "pitch down-right", 71 )   \
    X( kPaceStrideLeft = 72, "pace stride left", 72 )   \
    X( kPaceStrideRight = 73, "pace stride right", 73 ) \
    X( kMoonSwitchLeft = 74, "moonswitchleft", 74 )     \
    X( kMoonSwitchRight = 75, "moonswitchright", 75 )   \
    X( kUserGait00 = 80, "user gait 00", 80 )           \
    X( kUserGait01 = 81, "user gait 01", 81 )           \
    X( kUserGait02 = 82, "user gait 02", 82 )           \
    X( kUserGait03 = 83, "user gait 03", 83 )           \
    X( kUserGait04 = 84, "user gait 04", 84 )           \
    X( kUserGait05 = 85, "user gait 05", 85 )           \
    X( kUserGait06 = 86, "user gait 06", 86 )           \
    X( kUserGait07 = 87, "user gait 07", 87 )           \
    X( kUserGait08 = 88, "user gait 08", 88 )           \
    X( kUserGait09 = 89, "user gait 09", 89 )           \
    X( kUserGait10 = 90, "user gait 10", 90 )           \
    X( kUserGait11 = 91, "user gait 11", 91 )           \
    X( kUserGait12 = 92, "user gait 12", 92 )           \
    X( kUserGait13 = 93, "user gait 13", 93 )           \
    X( kUserGait14 = 94, "user gait 14", 94 )           \
    X( kUserGait15 = 95, "user gait 15", 95 )           \
    X( kUserGait16 = 96, "user gait 16", 96 )           \
    X( kUserGait17 = 97, "user gait 17", 97 )           \
    X( kUserGait18 = 98, "user gait 18", 98 )           \
    X( kUserGait19 = 99, "user gait 19", 99 )           \
    X( kUserGait20 = 100, "user gait 20", 100 )         \
    X( kUserGait = 110, "user gait", 110 )              \
    X( kLibGait01 = 111, "lib gait01", 111 )            \
    X( kLibGait02 = 112, "lib gait02", 112 )

#define X( gait, name, num ) gait,
enum GaitId : int64_t { GAIT_ID };
#undef X

// for rviz gui display
#define X( mode, name, num ) name,
__attribute__( ( unused ) ) static char const* gaitid_name[] = { GAIT_ID };
#undef X

// for rviz gui display
#define X( mode, name, num ) num,
__attribute__( ( unused ) ) static int64_t gaitid_num[]{ GAIT_ID };
#undef X

#define JUMP_ID                               \
    X( kJumpPosYaw90 = 0, "jump_yaw_p90", 0 ) \
    X( kJumpPosX60 = 1, "jump_x_p60", 1 )     \
    X( kJumpPosY20 = 2, "jump_y_p20", 2 )     \
    X( kJumpNegYaw90 = 3, "jump_yaw_n90", 3 ) \
    X( kJumpPosX30 = 4, "jump_x_p30", 4 )     \
    X( kJumpNegY20 = 5, "jump_y_n20", 5 )     \
    X( kJumpPosZ30 = 6, "jump_z_p30", 6 )     \
    X( kJumpDownStair = 9, "jump_down_stair", 9 )

#define X( jumpid, name, num ) jumpid,
enum JumpId : int64_t { JUMP_ID };
#undef X

// for rviz gui display
#define X( jumpid, name, num ) name,
__attribute__( ( unused ) ) static char const* jumpid_name[] = { JUMP_ID };
#undef X

// for rviz gui display
#define X( jumpid, name, num ) num,
__attribute__( ( unused ) ) static int64_t jumpid_num[]{ JUMP_ID };
#undef X

#define MOTION_ID                                                  \
    X( kMotionAllInOne = 0, "all_in_one", 0 )                      \
    X( kHiFiveLeft = 1, "hi five left", 1 )                        \
    X( kHiFiveRight = 2, "hi five right", 2 )                      \
    X( kSitDown = 3, "sit down", 3 )                               \
    X( kSwingHip = 4, "swing hip", 4 )                             \
    X( kSwingHead = 5, "swing head", 5 )                           \
    X( kStretchBody = 6, "stretch body", 6 )                       \
    X( kSitDownLeft = 8, "sit down left", 8 )                      \
    X( kSitDownRight = 9, "sit down right", 9 )                    \
    X( kSitDownShake = 10, "sit down shake", 10 )                  \
    X( kMotionBallet = 11, "motion ballet", 11 )                   \
    X( kMotionMoonwalk = 12, "motion moonwalk", 12 )               \
    X( kMotionFrontLift = 13, "motion front lift", 13 )            \
    X( kMotionRearLift = 14, "motion rear lift", 14 )              \
    X( kMotionPitchLeft = 15, "motion pitch left", 15 )            \
    X( kMotionPitchRight = 16, "motion pitch right", 16 )          \
    X( kMotionDiagonalRight = 17, "motion diagconal right", 17 )   \
    X( kMotionDiagonalLeft = 18, "motion diagonal left", 18 )      \
    X( kMotionWalkWave = 19, "motion walk wave", 19 )              \
    X( kMotionTrotInOut = 20, "motion trot in out", 20 )           \
    X( kMotionTrotPitch = 21, "motion trot pitch", 21 )            \
    X( kMotionFrontLiftForward = 22, "motion front lift LR", 22 )  \
    X( kMotionRearLiftForward = 23, "motion rear lift LR", 23 )    \
    X( kMotionFrontSwitch = 24, "motion front switch", 24 )        \
    X( kMotionRearSwitch = 25, "motion rear switch", 25 )          \
    X( kMotionJump3d = 26, "motion jump3D", 26 )                   \
    X( kMotionTrotSwing = 27, "motion trot swing", 27 )            \
    X( kMotionSpecialPronk = 28, "motion special pronk", 28 )      \
    X( kMotionSpecialTrot = 29, "motion special trot", 29 )        \
    X( kMotionMoonwalkLeft = 30, "motion moonwalk left", 30 )      \
    X( kMotionMoonwalkRight = 31, "motion moonwalk right", 31 )    \
    X( kMotionMoonwalkBack = 32, "motion moonwalk back", 32 )      \
    X( kMotionUpDown = 33, "motion up down", 33 )                  \
    X( kMotionPushUp = 34, "motion push up", 34 )                  \
    X( kMotionStridePositive = 60, "motion stride pos", 60 )       \
    X( kMotionStrideNegative = 61, "motion stride neg", 61 )       \
    X( kMotionPitchGainCalibration = 70, "motion pitch gain", 70 ) \
    X( kMotionUser00 = 80, "motion_user00", 80 )                   \
    X( kMotionUser01 = 81, "motion_user01", 81 )                   \
    X( kMotionUser02 = 82, "motion_user02", 82 )                   \
    X( kMotionUser03 = 83, "motion_user03", 83 )                   \
    X( kMotionUser04 = 84, "motion_user04", 84 )                   \
    X( kMotionUser05 = 85, "motion_user05", 85 )                   \
    X( kMotionUser06 = 86, "motion_user06", 86 )                   \
    X( kMotionUser07 = 87, "motion_user07", 87 )                   \
    X( kMotionUser08 = 88, "motion_user08", 88 )                   \
    X( kMotionUser09 = 89, "motion_user09", 89 )                   \
    X( kMotionUser10 = 90, "motion_user10", 90 )                   \
    X( kMotionUser11 = 91, "motion_user11", 91 )                   \
    X( kMotionUser12 = 92, "motion_user12", 92 )                   \
    X( kMotionUser13 = 93, "motion_user13", 93 )                   \
    X( kMotionUser14 = 94, "motion_user14", 94 )                   \
    X( kMotionUser15 = 95, "motion_user15", 95 )                   \
    X( kMotionUser16 = 96, "motion_user16", 96 )                   \
    X( kMotionUser17 = 97, "motion_user17", 97 )                   \
    X( kMotionUser18 = 98, "motion_user18", 98 )                   \
    X( kMotionUser19 = 99, "motion_user19", 99 )                   \
    X( kMotionUser20 = 100, "motion_user20", 100 )                 \
    X( kMotionUserGait = 110, "motion_usergait", 110 )             \
    X( kDanceSet1 = 111, "motion_set1", 111 )                      \
    X( kDanceSet2 = 112, "motion_set2", 112 )

#define X( motionid, name, num ) motionid,
enum MotionId : int64_t { MOTION_ID };
#undef X

// for rviz gui display
#define X( motionid, name, num ) name,
__attribute__( ( unused ) ) static char const* motionid_name[] = { MOTION_ID };
#undef X

// for rviz gui display
#define X( motionid, name, num ) num,
__attribute__( ( unused ) ) static int64_t motionid_num[]{ MOTION_ID };
#undef X

typedef enum LCM_PATTERN {
    LCM_GAIT_DEFAULT      = 0,
    LCM_GAIT_PASSIVE      = 1,
    LCM_GAIT_KNEEL        = 2,
    LCM_GAIT_STAND_R      = 3,
    LCM_GAIT_STAND_B      = 4,
    LCM_GAIT_AMBLE        = 5,
    LCM_GAIT_WALK         = 6,
    LCM_GAIT_TROT_MEDIUM  = 7,
    LCM_GAIT_TROT         = 8,
    LCM_GAIT_TROT_10_4    = 9,
    LCM_GAIT_GALLOP       = 10,
    LCM_GAIT_PRONK        = 11,
    LCM_GAIT_PASSIVE_TROT = 12,
    LCM_GAIT_BOUND        = 15,
    LCM_GAIT_PACE         = 16,

    LCM_GAIT_STAND      = 19,
    LCM_GAIT_TROT_24_16 = 20,

    LCM_GAIT_SPEC_PRONK = 50,
    LCM_GAIT_SPEC_TROT  = 51,

    LCM_GAIT_UPSTAIRS   = 100,
    LCM_GAIT_DOWNSTAIRS = 101,
    LCM_GAIT_SLOPE      = 102

} LCM_PATTERN;

#endif  // CONTROL_FLAGS_RELEASE_HPP_
