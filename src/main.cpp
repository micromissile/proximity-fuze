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
// State machine events variables
uint8_t rocket_is_moving_counter = 0; // Counter for movement confirmation
uint8_t target_acquisition_counter = 0; // Counter for target confirmation
String state_to_str(State state)
{
  switch (state)
  {
    case IDLE: return "IDLE";
    case MOVING: return "MOVING";
    case ACQUIRING: return "ACQUIRING";
    case TRACKING: return "TRACKING";
    case DETONATING: return "DETONATING";
    case END: return "END";
  }
}

// Loggers variables
EventLogger event_logger;
DataLogger data_logger;
// TODO; move sd card logging to second core; keep logic + sensor collection on core 0

// IMU variables
Adafruit_MPU6050 mpu;
float a_idle[3];

// Radar variables
DFRobot_C4001_UART radar(&Serial1, RADAR_BAUD_BPS);
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


void detonate()
{
  digitalWrite(DETONATOR_GPIO_PIN, HIGH);
  delay(1000);
  digitalWrite(DETONATOR_GPIO_PIN, LOW);
}

void read_imu_data()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(0x68,6);

  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();

  float RateRoll=(float)GyroX/65.5;
  float RatePitch=(float)GyroY/65.5;
  float RateYaw=(float)GyroZ/65.5;

  
  Serial.println("IMU data:");
  Serial.print("Roll rate [°/s]=");
  Serial.println(RateRoll);
}

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

  event_logger.log("SYSTEM INITALIZED IN STATE: " + state_to_str(state));
}

void loop()
{
  uint32_t current_time = millis();

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
      // Read radar target
      target_id = radar.getTargetNumber();
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
      target_id = radar.getTargetNumber(); // Needed
      distance = radar.getTargetRange();
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
  if (!mpu.begin())
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
  // Read current acceleration
  sensors_event_t a;
  mpu.getAccelerometerSensor()->getEvent(&a); 
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
  // Read radar target
  target_id = radar.getTargetNumber();
  // Check if target is present
  if (target_id > 0)
  {
    // Initialize the acquisition counter
    target_acquisition_counter = 0;
    return true;
  }

  return false;
}