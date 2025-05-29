#include <Arduino.h>
#include <unity.h>
#include <SD.h>
#include "Globals.h"

void setUp(void)
{
  // set stuff up here
}

void tearDown(void)
{
  // clean stuff up here
}

void test_sd_read_write(void)
{
  const char* testFileName = "test.txt";
  const char* testData = "Hello SD Card!";
  
  // Write test
  File writeFile = SD.open(testFileName, FILE_WRITE);
  TEST_ASSERT_TRUE(writeFile);
  size_t bytesWritten = writeFile.print(testData);
  writeFile.close();
  TEST_ASSERT_EQUAL(strlen(testData), bytesWritten);
  
  // Read test
  File readFile = SD.open(testFileName, FILE_READ);
  TEST_ASSERT_TRUE(readFile);
  String readData = readFile.readString();
  readFile.close();
  TEST_ASSERT_EQUAL_STRING(testData, readData.c_str());
  
  // Clean up
  SD.remove(testFileName);
}

void setup()
{
  delay(2000);
  if (!SD.begin(SD_CARD_CS))
  {
    Serial.println("SD card initialization failed.");
  }
  Serial.println("SD card initialized.");

  UNITY_BEGIN();
  RUN_TEST(test_sd_read_write);
  UNITY_END();
}

void loop()
{
  // Empty loop
}