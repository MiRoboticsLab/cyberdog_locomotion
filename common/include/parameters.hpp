#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_

#define IMU_CALIBRATE_FILE_PATH "/mnt/misc/imu_calibrate_param.yaml"

#define OFFSET_CALIBRATE_PARAM_FILE_PATH "/mnt/misc/offset_calibrate_param.yaml"

#define POSE_CALIBRATE_PARAM_FILE_PATH "/mnt/misc/pose_calibrate_param.yaml"

#define SPEED_CALIBRATE_FILE_PATH "/mnt/misc/speed_calibrate_param_v1.1.yaml"

#define JOINTS_CALIBRATE_FILE_PATH "/mnt/misc/joints_calibrate_param.yaml"

#define SPEED_PARAMETER_CALIBRATE_CONTROL_DEBUG 1

#define SPI_YAML_SH_PATH "/robot-software/common/config/cyberdog-spi.yaml"
#define SPI_YAML_HAND_SH_PATH "/robot-software/common/config/cyberdog-spi-hand.yaml"
#define SPI_YAML_HAND_SH_BAK_PATH "/robot-software/common/config/cyberdog-spi-hand.yaml.bak"
#define MANAGER_SCRIPT_PATH "/mnt/UDISK/test/manager.sh"

#define ROBOT_INTERFACE_UPDATE_PERIOD ( 1.f / 60.f )
#define INTERFACE_LCM_NAME "interface"
#define TIMES_TO_RESEND_CONTROL_PARAM 5

#define K_WORDS_PER_MESSAGE 66
#define ZERO_FLAG 0xdead

#define BUDDA_Q_SCALE 6.f

#define MAX_STACK_SIZE 16384  // 16KB  of stack
#define TASK_PRIORITY 49      // linux priority, this is not the nice value

#define DEFAULT_TERRAIN_FILE "default-terrain.yaml"
#define DEFAULT_USER_FILE "/default-user-parameters-file.yaml"

#define SIM_LCM_NAME "simulator_state"

// Normal robot states
#define K_PASSIVE 0
#define K_BALANCE_STAND 3
#define K_LOCOMOTION 4
#define K_LOCOMOTION_TEST 5
#define K_RECOVERY_STAND 6
#define K_VISION 8
#define K_BACKFLIP 9
#define K_FRONTJUMP 11
#define K_FRONTFLIP 12

// Specific control states
#define K_JOINT_PD 51
#define K_IMPEDANCE_CONTROL 52
#define K_MOTION 62

// Extra control states
#define K_PUREDAMPER_J 71
#define K_INVALID 100

// new gait
#define K_Dance 101

#define IMU_SERIAL_COM "/dev/ttyUSB0"
#define BMS_SERIAL_COM "/dev/ttyS3"
#define EXTERNAL_IMU_SERIAL_COM "/dev/ttyS1"

#define YAML_COLLECTION_NAME_KEY "__collection-name__"

// all print control DEBUG
#define PRINT_CONVEX_MPC_DEBUG 0

#define GRAVITY 9.81

#endif  // PARAMETERS_HPP_
