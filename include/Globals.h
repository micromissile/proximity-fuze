#define MCU_FREQ 110
#define MCU_T_MICROS (1000000.0 / MCU_FREQ) 
#define DATA_LOGGER_INTERVAL_MS 50
// GPIO definitions
#define SD_CARD_CS 5
#define DETONATOR_GPIO_PIN 14
// Sensors global variables
#define IMU_ACCEL_CALIBRATION_SAMPLES 100 // Number of samples for calibration
// State machine global variables; format STATE_VAR_NAME
#define IDLE_IMU_ACCEL_IS_MOVING_THR 1.0 // Acceleration threshold in m/s^2
#define IDLE_IMU_IS_MOVING_PERSISTENT_COUNT 5 // Number of consecutive readings to confirm movement
#define DETONATOR_TIMER_MS 5000
#define DETONATOR_ACTIVE_TIME_MS 5000
#define FAILSAFE_TIMEOUT_MS 15 * 60 * 1000 // 15 minutes in milliseconds