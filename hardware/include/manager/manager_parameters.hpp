
#ifndef MANAGER_PARAMETERS_HPP_
#define MANAGER_PARAMETERS_HPP_

#include <string>

extern std::string exe_path;

#define IMU_CALIBRATE_COMMAND "imu_calibrate"

/* bias_sample settings */
#define BIAS_SAMPLE_COMMAND "bias_sample"
#define IMU_TEST_RELATIVE_PATH "/imu-test/"
#define IMU_CALIB_LOG_PATH "/mnt/UDISK/imu-log"

/* Test log path */
#define TEST_PROG_LOG_PATH "/mnt/UDISK/test-log"

/* execute file config path*/
#define TEST_EXE_COMMAND_JSON_PATH "/robot-software/common/config/test_execute_command.json"

/*fork configure file path*/
#define FORK_CONFIGURE_JSON_PATH "/robot-software/common/config/fork_para_conf_lists.json"
//#define FORK_CONFIGURE_JSON_PATH        "../config/fork_para_conf_lists.json"   //(debug on PC)

#define BMS_SHM_NAME "bms_abort"
#define BMS_SHM_SIZE 2
#define BMS_ABORT_REQUEST ( 0xdead )
#define BMS_ABORT_FINISH ( 0xffff )
#define BMS_SERIAL_PORT "/dev/ttyS3"
#define BMS_WAIT_APP_DELAY_TIME 6000

/* tcp server and transfer settings */
#define IP "0.0.0.0"
#define PORT "5000"
#define BUF_LEN 4096

#define DEBUG_PRINTF 1  // debug printf control

#endif  // MANAGER_PARAMETERS_HPP_
