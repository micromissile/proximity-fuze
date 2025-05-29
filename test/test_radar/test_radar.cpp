#include <unity.h>
#include <Arduino.h>
#include <DFRobot_C4001.h>
#include "Globals.h"

#define TEST_DURATION_MS 5000  // 5 seconds of printing

DFRobot_C4001_UART radar(&RADAR_SERIAL, RADAR_BAUD_BPS);
unsigned long start_time;

void setUp(void) {
    // Setup code to run before each test
}

void tearDown(void) {
    // Cleanup code to run after each test
}

void test_radar_initialization() {
    Serial.println("Testing radar initialization...");
    
    // Test radar initialization
    TEST_ASSERT_TRUE(radar.begin());
    
    // Test radar configuration
    TEST_ASSERT_TRUE(radar.setDetectThres(RADAR_MIN_DISTANCE, RADAR_MAX_DISTANCE, RADAR_THR));
    
    // Verify configuration
    TEST_ASSERT_EQUAL(RADAR_MIN_DISTANCE, radar.getMinRange());
    TEST_ASSERT_EQUAL(RADAR_MAX_DISTANCE, radar.getMaxRange());
    TEST_ASSERT_EQUAL(RADAR_THR, radar.getThresRange());
    
    Serial.println("Radar initialization successful!");
}

void test_radar_data() {
    // Test if we can get target data
    uint8_t target_id = radar.getTargetNumber();
    float distance = radar.getTargetRange();
    
    // Target ID should be 0 or positive
    TEST_ASSERT_TRUE(target_id >= 0);
    
    // Distance should be within configured range
    TEST_ASSERT_TRUE(distance >= 0);
    TEST_ASSERT_TRUE(distance <= RADAR_MAX_DISTANCE/100.0); // Convert cm to m
}

void print_radar_values() {
    Serial.print("Target ID: ");
    Serial.println(radar.getTargetNumber());
    Serial.print("Distance: ");
    Serial.print(radar.getTargetRange());
    Serial.println(" m");
    Serial.print("Target Energy: ");
    Serial.println(radar.getTargetEnergy());
    Serial.println();
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    delay(1000); // Give time for serial to initialize
    Serial.println("\n\nStarting Radar Test...");
    
    // Run tests
    UNITY_BEGIN();
    RUN_TEST(test_radar_initialization);
    RUN_TEST(test_radar_data);
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
    print_radar_values();
    
    // Run data test
    test_radar_data();
    
    delay(100); // Print at 10Hz
}
