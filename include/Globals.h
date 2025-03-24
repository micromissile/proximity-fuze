#define DEBUG 1
#define MCU_FREQ 60
#define MCU_T_MICROS (1000000.0 / MCU_FREQ) 
#define DATA_LOGGER_INTERVAL_MS 1000
// GPIO definitions
#define SD_CARD_CS 5
#define DETONATOR_GPIO_PIN 14
// Sensors global variables
#define IMU_ACCEL_CALIBRATION_SAMPLES 100 // Number of samples for calibration
#define RADAR_BAUD_BPS 115200
#define RADAR_MIN_DISTANCE 60
#define RADAR_MAX_DISTANCE 2500
#define RADAR_THR 30
// State machine global variables; format STATE_VAR_NAME
#define IDLE_IMU_ACCEL_IS_MOVING_THR 1.0 // Acceleration threshold in m/s^2
#define IDLE_IMU_IS_MOVING_PERSISTENT_COUNT 5 // Number of consecutive readings to confirm movement
#define ACQUIRING_NUM_CONFIRMATIONS 3 // Number of readings to confirm target
#define PROXIMITY_THR_M 5.0