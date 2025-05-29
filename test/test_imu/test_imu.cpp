#include <unity.h>
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define IMU_ACCEL_CALIBRATION_SAMPLES 100
#define DEBUG 1
#define TEST_DURATION_MS 5000  // 5 seconds of printing

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
float a_idle[3] = {0., 0., 0.};
unsigned long start_time;

void setUp(void) {
    // Setup code to run before each test
}

void tearDown(void) {
    // Cleanup code to run after each test
}

void test_imu_initialization() {
    Serial.println("Testing IMU initialization...");
    // Test IMU initialization with Wire1
    TEST_ASSERT_TRUE(mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire1));
    
    // Configure IMU
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    // Verify configuration
    TEST_ASSERT_EQUAL(MPU6050_RANGE_8_G, mpu.getAccelerometerRange());
    TEST_ASSERT_EQUAL(MPU6050_BAND_21_HZ, mpu.getFilterBandwidth());
    Serial.println("IMU initialization successful!");
}

void test_imu_calibration() {
    Serial.println("Starting IMU calibration...");
    // Perform calibration
    float sum[3] = {0., 0., 0.};
    sensors_event_t a;
    
    for (int i = 0; i < IMU_ACCEL_CALIBRATION_SAMPLES; i++) {
        mpu.getAccelerometerSensor()->getEvent(&a);
        sum[0] += a.acceleration.x;
        sum[1] += a.acceleration.y;
        sum[2] += a.acceleration.z;
        delay(10);
    }
    
    a_idle[0] = sum[0] / IMU_ACCEL_CALIBRATION_SAMPLES;
    a_idle[1] = sum[1] / IMU_ACCEL_CALIBRATION_SAMPLES;
    a_idle[2] = sum[2] / IMU_ACCEL_CALIBRATION_SAMPLES;
    
    // Test if calibration values are within reasonable ranges
    // When the IMU is stationary, the total acceleration should be close to 1G (9.81 m/s^2)
    float total_accel = sqrt(a_idle[0]*a_idle[0] + a_idle[1]*a_idle[1] + a_idle[2]*a_idle[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.5, 9.81, total_accel);
    Serial.println("IMU calibration complete!");
}

void test_accelerometer_data() {
    // Get accelerometer data
    mpu.getAccelerometerSensor()->getEvent(&a);
    
    // Test if acceleration values are within expected range (-8G to +8G)
    // Convert to m/s^2 (1G = 9.81 m/s^2)
    float max_accel = 8.0 * 9.81;
    TEST_ASSERT_FLOAT_WITHIN(max_accel, 0, a.acceleration.x);
    TEST_ASSERT_FLOAT_WITHIN(max_accel, 0, a.acceleration.y);
    TEST_ASSERT_FLOAT_WITHIN(max_accel, 0, a.acceleration.z);
    
    // Test if readings are stable (not changing too rapidly)
    static float prev_x = 0, prev_y = 0, prev_z = 0;
    float max_change = 2.0; // Maximum allowed change in m/s^2 between readings
    
    if (prev_x != 0) { // Skip first reading
        TEST_ASSERT_FLOAT_WITHIN(max_change, prev_x, a.acceleration.x);
        TEST_ASSERT_FLOAT_WITHIN(max_change, prev_y, a.acceleration.y);
        TEST_ASSERT_FLOAT_WITHIN(max_change, prev_z, a.acceleration.z);
    }
    
    prev_x = a.acceleration.x;
    prev_y = a.acceleration.y;
    prev_z = a.acceleration.z;
}

void test_gyroscope_data() {
    // Get gyroscope data
    mpu.getGyroSensor()->getEvent(&g);
    
    // Test if angular velocity values are within expected range (-500 to +500 deg/s)
    TEST_ASSERT_FLOAT_WITHIN(500, 0, g.gyro.x);
    TEST_ASSERT_FLOAT_WITHIN(500, 0, g.gyro.y);
    TEST_ASSERT_FLOAT_WITHIN(500, 0, g.gyro.z);
    
    // Test if readings are stable when stationary
    static float prev_x = 0, prev_y = 0, prev_z = 0;
    float max_change = 5.0; // Maximum allowed change in deg/s between readings
    
    if (prev_x != 0) { // Skip first reading
        TEST_ASSERT_FLOAT_WITHIN(max_change, prev_x, g.gyro.x);
        TEST_ASSERT_FLOAT_WITHIN(max_change, prev_y, g.gyro.y);
        TEST_ASSERT_FLOAT_WITHIN(max_change, prev_z, g.gyro.z);
    }
    
    prev_x = g.gyro.x;
    prev_y = g.gyro.y;
    prev_z = g.gyro.z;
}

void test_temperature_data() {
    // Get temperature data
    mpu.getTemperatureSensor()->getEvent(&temp);
    
    // Test if temperature is within reasonable range (-40 to +85°C)
    TEST_ASSERT_FLOAT_WITHIN(85, 0, temp.temperature);
    
    // Test if temperature is stable (not changing too rapidly)
    static float prev_temp = 0;
    float max_change = 1.0; // Maximum allowed change in °C between readings
    
    if (prev_temp != 0) { // Skip first reading
        TEST_ASSERT_FLOAT_WITHIN(max_change, prev_temp, temp.temperature);
    }
    
    prev_temp = temp.temperature;
}

void print_sensor_values() {
    // Get sensor readings
    if (!mpu.getAccelerometerSensor()->getEvent(&a)) {
        Serial.println("Failed to get accelerometer data!");
        return;
    }
    if (!mpu.getGyroSensor()->getEvent(&g)) {
        Serial.println("Failed to get gyroscope data!");
        return;
    }
    if (!mpu.getTemperatureSensor()->getEvent(&temp)) {
        Serial.println("Failed to get temperature data!");
        return;
    }
    
    // Print formatted values
    Serial.println("\n--- IMU Readings ---");
    Serial.println("Acceleration (m/s^2):");
    Serial.print("  X: "); Serial.print(a.acceleration.x, 2);
    Serial.print("  Y: "); Serial.print(a.acceleration.y, 2);
    Serial.print("  Z: "); Serial.println(a.acceleration.z, 2);
    
    Serial.println("Gyroscope (deg/s):");
    Serial.print("  X: "); Serial.print(g.gyro.x, 2);
    Serial.print("  Y: "); Serial.print(g.gyro.y, 2);
    Serial.print("  Z: "); Serial.println(g.gyro.z, 2);
    
    Serial.print("Temperature: ");
    Serial.print(temp.temperature, 2);
    Serial.println(" °C");
    
    // Print calibration values
    Serial.println("\nCalibration Values (m/s^2):");
    Serial.print("  X: "); Serial.print(a_idle[0], 2);
    Serial.print("  Y: "); Serial.print(a_idle[1], 2);
    Serial.print("  Z: "); Serial.println(a_idle[2], 2);
    
    Serial.println("------------------------\n");
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    delay(1000); // Give time for serial to initialize
    Serial.println("\n\nStarting IMU Test...");
    
    // Initialize I2C
    Wire1.begin();
    Serial.println("I2C initialized");
    
    // Run tests
    UNITY_BEGIN();
    RUN_TEST(test_imu_initialization);
    RUN_TEST(test_imu_calibration);
    RUN_TEST(test_accelerometer_data);
    RUN_TEST(test_gyroscope_data);
    RUN_TEST(test_temperature_data);
    UNITY_END();
    
    // Start the timer
    start_time = millis();
    
    Serial.println("\nStarting 5-second sensor value printing...");
}

void loop() {
    // Check if 5 seconds have elapsed
    if (millis() - start_time >= TEST_DURATION_MS) {
        Serial.println("\nTest duration complete. Terminating...");
        while(1) { delay(1000); } // Stop the program
    }
    
    // Print sensor values
    print_sensor_values();
    
    // Run tests
    test_accelerometer_data();
    test_gyroscope_data();
    test_temperature_data();
    
    delay(100); // Print at 10Hz
}
