/**
 * Author: Joshua Soutelo Vieira
 * Email: joshua@micro-missile.xyz
 * Last update: 23-03-2025
 * 
 * Radar based proximity fuze.
 * 
 * Implements the following state machine
 * 
 * | Current State | Event/Condition                       | Next State | Action             |
 * |---------------|---------------------------------------|------------|--------------------|
 * | IDLE          | accel_y > 1 m/s^2                     | MOVING     | -                  |
 * | MOVING        | target_id > 0                         | ACQUIRING  | -                  |
 * | ACQUIRING     | acquisition_count >= 3                | TRACKING   | -                  |
 * | TRACKING      | distance <= 5.0 m                     | DETONATING | -                  |
 * | TRACKING      | distance <= 0 || target_id == 0       | MOVING     | -                  |
 * | DETONATING    | detonation complete                   | END        | Trigger detonator  |
 * 
 */

#include <Arduino.h>
#include "Globals.h"
#include <Logger.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <time.h>
#include <DFRobot_C4001.h>

// State machine variables
enum State {IDLE, MOVING, ACQUIRING, TRACKING, DETONATING, END};
State state = IDLE;
State prev_state = IDLE;
uint32_t state_timer = 0; // Timer for state-specific delays
uint32_t failsafe_timer = 0; // Timer for 15-minute failsafe
// State machine events variables
uint8_t rocket_is_moving_counter = 0; // Counter for movement confirmation
uint8_t target_acquisition_counter = 0; // Counter for target confirmation
String state_to_str(State state)
{
  String toret = "";
  switch (state)
  {
    case IDLE: toret = "IDLE"; break;
    case MOVING: toret = "MOVING"; break;
    case ACQUIRING: toret = "ACQUIRING"; break;
    case TRACKING: toret = "TRACKING"; break;
    case DETONATING: toret = "DETONATING"; break;
    case END: toret = "END"; break;
    default: toret = ""; break;
  }
  return toret;
}

// Loggers variables
EventLogger event_logger;
DataLogger data_logger;

// IMU variables
Adafruit_MPU6050 mpu;
float a_idle[3];
sensors_event_t a; // Stores sensor acceleration data

// Radar variables
DFRobot_C4001_UART radar(&RADAR_SERIAL, RADAR_BAUD_BPS);
float distance = 0.0;
uint8_t target_id = 0;

// MCU clock frequency variable
uint32_t loop_timer;

// Initialization functions definitions
void init_loggers();
void init_imu();
void init_radar();
void init_detonator();

// State machine events functions definitions
bool is_rocket_moving();
bool is_target_detected();

// State machine actions functions definitions
void detonate();

// State machine failsafe check function
// TODO: move to StateMachine class
#ifdef DEBUG_FAILSAFE
#define FAILSAFE_TIMEOUT_MS 5 * 1000 // 5 seconds
#endif

void setup()
{
  #if DEBUG
  Serial.begin(115200);
  while (!Serial);
  #endif

  // Initialize loggers
  init_loggers();

  // Initialize 6-axis IMU MPU6050
  init_imu();

  // Initialize the radar
  init_radar();

  // Initialize the detonator
  init_detonator();

  // Start the timer
  loop_timer = micros();

  #if DEBUG
  Serial.print("SYSTEM INITALIZED IN STATE: ");
  Serial.println(state_to_str(state));
  #endif
  event_logger.log("SYSTEM INITALIZED IN STATE: " + state_to_str(state));

  // Start failsafe timer 
  failsafe_timer = millis();
  #if DEBUG
  Serial.println("FAILSAFE TIMER STARTED");
  #endif
  event_logger.log("FAILSAFE TIMER STARTED");
}

void loop()
{
  uint32_t current_time = millis();

  // Check failsafe timer
  if ((current_time - failsafe_timer) >= FAILSAFE_TIMEOUT_MS)
  {
    // Go to END state
    state = END;
    #if DEBUG
    Serial.println("FAILSAFE TRIGGERED: 15-minute timeout reached");
    #endif
    event_logger.log("FAILSAFE TRIGGERED: 15-minute timeout reached");
  }

  // Grab data from sensors
  mpu.getAccelerometerSensor()->getEvent(&a); 
  target_id = radar.getTargetNumber();
  distance = radar.getTargetRange();

  #if DEBUG
  Serial.println("State: " + state_to_str(state));
  #endif

  switch (state)
  {
    case IDLE: 
      if (is_rocket_moving())
      {
        state = MOVING;
      }
      break;
    
    case MOVING:
      if (is_target_detected())
      {
        state = ACQUIRING;
      }
      break;

    case ACQUIRING:
      if (target_id > 0)
      {
        target_acquisition_counter++;
        if (target_acquisition_counter >= ACQUIRING_NUM_CONFIRMATIONS)
        {
          state = TRACKING;
        }
      }
      else
      {
        state = MOVING;
      }
      break;

    case TRACKING:
      if (distance <= 0 || target_id <= 0)
      {
        state = MOVING;
      }
      else if (distance <= PROXIMITY_THR_M)
      {
        state = DETONATING;
      }
      break;

    case DETONATING:
      detonate();
      state = END;
      state_timer = current_time;
      break;
    
    case END:
      event_logger.close();
      data_logger.close();

      // Check failsafe
      // TODO: move to StateMachine class
      #ifdef DEBUG_FAILSAFE
      Serial.println("FAILSAFE TRIGGERED: check passed");
      #endif  

      while(1);

      break;
  }
  
  // Check for state transition
  if (state != prev_state)
  {
    // Log event
    event_logger.log("STATE TRANSITION: " + state_to_str(prev_state) + " -> " + state_to_str(state));
    // Update state
    prev_state = state;
  }

  // Log data
  if (state != END)
  {
    String data_msg = "";
    data_msg += String(micros()) + ",";
    data_msg += String(a.acceleration.x) + ",";
    data_msg += String(a.acceleration.y) + ",";
    data_msg += String(a.acceleration.z) + ",";
    data_msg += String(target_id) + ",";
    data_msg += String(distance) + ",";
    data_msg += state_to_str(state);
    data_logger.log(data_msg);
  }

  // Finish loop at the given MHz 
  while (micros() - loop_timer < MCU_T_MICROS);
  loop_timer = micros();
}

// --- Initialization functions

void init_loggers()
{
  // Initialize SD card
  if (!SD.begin(SD_CARD_CS))
  {
    #if DEBUG
    Serial.println("SD card initialization failed.");
    #endif
  }
  #if DEBUG
  Serial.println("SD card initialized.");
  #endif
  
  // Initialize loggers now that SD card is ready
  if (event_logger.init())
  {
    event_logger.log("EVENT LOGGER INITIALIZED");
  }
  #if DEBUG
  else
  {
    Serial.println("Event logger initialization failed.");
  }
  #endif
  if (data_logger.init(DATA_LOGGER_INTERVAL_MS))
  {
    // Write header of the CSV file
    String data_logger_csv_header = "";
    data_logger_csv_header += "timestamp,";
    data_logger_csv_header += "accel_x,";
    data_logger_csv_header += "accel_y,";
    data_logger_csv_header += "accel_z,";
    data_logger_csv_header += "radar_target_id,";
    data_logger_csv_header += "radar_target_distance,";
    data_logger_csv_header += "state";
    data_logger.log(data_logger_csv_header);

    event_logger.log("DATA LOGGER INITIALIZED");
  }
  #if DEBUG
  else
  {
    Serial.println("Data logger initialization failed.");
  }
  #endif
}

void init_imu()
{
  // Start connection
  if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire1))
  {
    #if DEBUG
    Serial.println("IMU initialization failed.");
    #endif
  }
  #if DEBUG
  else
  {
    Serial.println("IMU initialized.");
  }
  #endif
  // Configure
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Calibration
  float sum[3] = {0., 0., 0.};
  sensors_event_t a;
  for (int i = 0; i < IMU_ACCEL_CALIBRATION_SAMPLES; i++)
  {
    mpu.getAccelerometerSensor()->getEvent(&a); 
    sum[0] += a.acceleration.x;
    sum[1] += a.acceleration.y;
    sum[2] += a.acceleration.z;
    delay(10); // Small delay between readings, 100Hz
  }
  a_idle[0] = sum[0] / IMU_ACCEL_CALIBRATION_SAMPLES;
  a_idle[1] = sum[1] / IMU_ACCEL_CALIBRATION_SAMPLES;
  a_idle[2] = sum[2] / IMU_ACCEL_CALIBRATION_SAMPLES;

  #if DEBUG
  Serial.println("IMU calibration complete");
  #endif

  // Log IMU event
  String imu_event_msg = "";
  imu_event_msg += "IMU INITIALIZED\n";
  imu_event_msg += "IMU ACCEL RANGE: " + String(mpu.getAccelerometerRange()) + "\n";
  imu_event_msg += "IMU FILTER BANDWIDTH: " + String(mpu.getFilterBandwidth()) + "\n";
  imu_event_msg += "IMU CALIBRATION COMPLETE\n";
  imu_event_msg += "IDLE AX: " + String(a_idle[0]) + "\n";
  imu_event_msg += "IDLE AY: " + String(a_idle[1]) + "\n";
  imu_event_msg += "IDLE AZ: " + String(a_idle[2]) + "\n";
  event_logger.log(imu_event_msg);
}

void init_radar()
{
  #if DEBUG
  Serial.print("Initializing radar...");
  #endif
  // Start connection with radar
  while(!radar.begin())
  {
    #if DEBUG
    Serial.println("Radar not found.");
    #endif
    delay(1000);
  }
  #if DEBUG
  Serial.println("Radar found.");
  #endif

  // Log radar event
  String radar_event_msg = "";
  radar_event_msg += "RADAR INITIALIZED\n";
  radar_event_msg += "RADAR MIN RANGE:" + String(radar.getMinRange()) + "\n";
  radar_event_msg += "RADAR MAX RANGE:" + String(radar.getMaxRange()) + "\n";
  radar_event_msg += "RADAR THR FACTOR:" + String(radar.getThresRange()) + "\n";
  event_logger.log(radar_event_msg);
}

void init_detonator()
{
  // Initialize detonator pin low
  pinMode(DETONATOR_GPIO_PIN, OUTPUT);
  digitalWrite(DETONATOR_GPIO_PIN, LOW);
  #if DEBUG
  Serial.println("Detonator initialized.");
  #endif
  // Log detonator event
  String detonator_event_msg = "";
  detonator_event_msg += "DETONATOR INITIALIZED\n";
  detonator_event_msg += "DETONATOR PIN: " + String(DETONATOR_GPIO_PIN) + "\n";
  event_logger.log(detonator_event_msg);
}

// --- State machine events functions ---

bool is_rocket_moving()
{
  // Compute difference in y-axis
  float delta_y = a.acceleration.y - a_idle[1];
  // Check if acceleration exceeds threshold
  if (delta_y > IDLE_IMU_ACCEL_IS_MOVING_THR)
  {
    rocket_is_moving_counter++;
    if (rocket_is_moving_counter >= IDLE_IMU_IS_MOVING_PERSISTENT_COUNT)
    {
      return true;
    }
  }
  else
  {
    rocket_is_moving_counter = 0;
  }
  return false;
}

bool is_target_detected()
{
  // Check if target is detected
  if (target_id > 0)
  {
    // Initialize the acquisition counter
    target_acquisition_counter = 0;
    return true;
  }

  return false;
}

void detonate()
{
  digitalWrite(DETONATOR_GPIO_PIN, HIGH);
  delay(DETONATOR_ACTIVE_TIME_MS);
  digitalWrite(DETONATOR_GPIO_PIN, LOW);
}