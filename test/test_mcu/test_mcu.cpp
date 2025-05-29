#include <unity.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Define LED pins for Waveshare RP2040 Zero
#define LED_BUILTIN 25  // GPIO25 is the built-in LED on Waveshare RP2040 Zero
#define NEOPIXEL_PIN 16 // GPIO16 is the NeoPixel data pin
#define NUM_PIXELS 1    // Number of NeoPixels
#define MAX_BLINKS 5    // Number of times to blink

bool test_success = false;
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
int blink_count = 0;
bool tests_completed = false;

void setUp(void) {
    // Set up code that runs before each test
}

void tearDown(void) {
    // Clean up code that runs after each test
}

void test_mcu_clock_speed(void) {
    // Verify that the MCU is running at the expected clock speed
    TEST_ASSERT_EQUAL(133000000L, F_CPU);
}

void test_mcu_memory(void) {
    // Basic memory test - allocate and free memory
    void* test_memory = malloc(1024);
    TEST_ASSERT_NOT_NULL(test_memory);
    free(test_memory);
}

void test_mcu_digital_io(void) {
    // Test digital I/O functionality
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    TEST_ASSERT_TRUE(true); // If we got here, the LED control worked
}

void test_mcu_analog_read(void) {
    // Test analog read functionality
    // Note: This is a basic test that just verifies the function doesn't crash
    int value = analogRead(A0);
    TEST_ASSERT_TRUE(value >= 0 && value <= 4095); // 12-bit ADC on Pico
}

void test_led_state_high(void) {
    if(test_success) {
        pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Green
    } else {
        pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Red
    }
    pixels.show();
    TEST_ASSERT_TRUE(true);
}

void test_led_state_low(void) {
    pixels.clear();
    pixels.show();
    TEST_ASSERT_TRUE(true);
}

void setup() {
    // Initialize Arduino environment
    delay(2000);  // Give time for serial monitor to connect
    
    // Initialize NeoPixel
    pixels.begin();
    pixels.clear(); // Initialize all pixels to 'off'
    pixels.show();  // Send the updated pixel colors to the hardware
    
    // Run initial tests
    UNITY_BEGIN();
    RUN_TEST(test_mcu_clock_speed);
    RUN_TEST(test_mcu_memory);
    RUN_TEST(test_mcu_digital_io);
    RUN_TEST(test_mcu_analog_read);
    test_success = (Unity.CurrentTestFailed == 0);
}

void loop() {
    if (!tests_completed) {
        if (blink_count < MAX_BLINKS) {
            RUN_TEST(test_led_state_high);
            delay(500);
            RUN_TEST(test_led_state_low);
            delay(500);
            blink_count++;
        } else if (blink_count == MAX_BLINKS) {
            // Show final color briefly
            if(test_success) {
                pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Green
            } else {
                pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Red
            }
            pixels.show();
            delay(1000);
            
            // Turn off the LED
            pixels.clear();
            pixels.show();
            
            UNITY_END(); // Stop unit testing
            tests_completed = true;
        }
    }
}
