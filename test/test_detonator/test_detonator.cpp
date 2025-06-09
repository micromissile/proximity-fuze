#ifdef TEST_DETONATOR

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "Globals.h"

#define NUM_BLINKS 10  // Number of green blinks before detonation
#define NEOPIXEL_PIN 16 // GPIO16 is the NeoPixel data pin
#define NUM_PIXELS 1

Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setup()
{
    // Initialize NeoPixel
    pixels.begin();
    pixels.clear();
    pixels.show();

    // Initialize detonator pin
    pinMode(DETONATOR_GPIO_PIN, OUTPUT);
    digitalWrite(DETONATOR_GPIO_PIN, LOW);

    // Countdown with green blinks
    for(int i = 0; i < NUM_BLINKS; i++) {
        pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Green
        pixels.show();
        delay(500);
        pixels.clear();
        pixels.show();
        delay(500);
    }

    // Switch to red for detonation
    pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Red
    pixels.show();

    // Activate detonator
    digitalWrite(DETONATOR_GPIO_PIN, HIGH);
    delay(DETONATOR_ACTIVE_TIME_MS);
    digitalWrite(DETONATOR_GPIO_PIN, LOW);

    // Turn off LED
    pixels.clear();
    pixels.show();
}

void loop()
{
    while (true); // Do nothing more
}

#endif