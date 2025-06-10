# Configure DFRobot_C4001

1. Hook USB-TTL converter to computer

2. Input the following commands on the device

```text
sensorStop
setRange 0.6 25
setThrFactor 30
setMicroMotion 0
setUart 115200
saveConfig
resetSystem 0
```

# Modify default Arduino pins

```bash
vim /path/to/.platformio/packages/framework-arduino-mbed/variants/RASPBERRY_PI_PICO/pins_arduino.h
```

Change the following values to they are like these:

```text
// Analog pins
// -----------
#define PIN_A0 (14u)
#define PIN_A1 (15u)
#define PIN_A2 (28u)
#define PIN_A3 (29u)

// SPI
#define PIN_SPI_MISO  (4u)
#define PIN_SPI_MOSI  (3u)
#define PIN_SPI_SCK   (2u)
#define PIN_SPI_SS    (5u)

// Wire
#define PIN_WIRE_SDA        (26u)
#define PIN_WIRE_SCL        (27u)
```

Pinout configuration from [here](https://github.com/earlephilhower/arduino-pico/blob/master/variants/waveshare_rp2040_zero/pins_arduino.h).

# Execute detonator field test

```bash
pio run -e pico_field_test_detonator --target upload
```

# Run tests

```bash
pio test -e pico_test
```