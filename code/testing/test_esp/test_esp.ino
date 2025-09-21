/*
  ESP32 Module Testing Code
  This code tests each component individually before running the full system
*/

#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

// Pin definitions
#define SD_CS_PIN 5
#define BME_SDA_PIN 21
#define BME_SCL_PIN 22
#define LED_PIN 2  // Built-in LED on most ESP32 boards

Adafruit_BME680 bme;

void setup() {
  Serial.begin(115200);
  delay(2000); // Give time for serial monitor to connect
  
  Serial.println("\n=== ESP32 MODULE TESTING ===");
  Serial.println("Starting component tests...\n");
  
  // Test 1: Basic ESP32 functionality
  testESP32Basic();
  
  // Test 2: I2C Bus
  testI2C();
  
  // Test 3: BME680 Sensor
  testBME680();
  
  // Test 4: SD Card Module
  testSDCard();
  
  // Test 5: Combined functionality
  testCombined();
  
  Serial.println("\n=== TESTING COMPLETE ===");
  Serial.println("Check results above. If all tests pass, upload the main code!");
}

void loop() {
  // Simple blink to show ESP32 is running
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}

void testESP32Basic() {
  Serial.println("TEST 1: ESP32 Basic Functionality");
  Serial.println("--------------------------------");
  
  // Test built-in LED
  pinMode(LED_PIN, OUTPUT);
  Serial.println("✓ Built-in LED configured");
  
  // Test chip info
  Serial.print("✓ Chip Model: "); Serial.println(ESP.getChipModel());
  Serial.print("✓ Chip Revision: "); Serial.println(ESP.getChipRevision());
  Serial.print("✓ CPU Frequency: "); Serial.print(ESP.getCpuFreqMHz()); Serial.println(" MHz");
  Serial.print("✓ Free Heap: "); Serial.print(ESP.getFreeHeap()); Serial.println(" bytes");
  Serial.print("✓ Flash Size: "); Serial.print(ESP.getFlashChipSize()); Serial.println(" bytes");
  
  Serial.println("✓ ESP32 basic test PASSED\n");
}

void testI2C() {
  Serial.println("TEST 2: I2C Bus Scanning");
  Serial.println("------------------------");
  
  Wire.begin(BME_SDA_PIN, BME_SCL_PIN);
  Serial.print("✓ I2C initialized on pins SDA:"); 
  Serial.print(BME_SDA_PIN); 
  Serial.print(", SCL:"); 
  Serial.println(BME_SCL_PIN);
  
  Serial.println("Scanning for I2C devices...");
  
  int deviceCount = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("✓ I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Identify known devices
      if (address == 0x76 || address == 0x77) {
        Serial.print(" (BME680 sensor)");
      }
      Serial.println();
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("✗ No I2C devices found! Check wiring.");
  } else {
    Serial.print("✓ Found "); Serial.print(deviceCount); Serial.println(" I2C device(s)");
  }
  Serial.println();
}

void testBME680() {
  Serial.println("TEST 3: BME680 Sensor");
  Serial.println("---------------------");
  
  if (!bme.begin()) {
    Serial.println("✗ BME680 sensor not found! Check wiring and I2C address.");
    Serial.println("  Default I2C address is 0x77, some modules use 0x76");
    return;
  }
  
  Serial.println("✓ BME680 sensor found and initialized");
  
  // Configure sensor
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);
  
  Serial.println("✓ BME680 sensor configured");
  
  // Test reading
  Serial.println("Taking test reading...");
  if (!bme.performReading()) {
    Serial.println("✗ Failed to perform reading");
    return;
  }
  
  Serial.println("✓ BME680 test reading successful:");
  Serial.print("  Temperature: "); Serial.print(bme.temperature); Serial.println(" °C");
  Serial.print("  Humidity: "); Serial.print(bme.humidity); Serial.println(" %");
  Serial.print("  Pressure: "); Serial.print(bme.pressure / 100.0); Serial.println(" hPa");
  Serial.print("  Gas Resistance: "); Serial.print(bme.gas_resistance); Serial.println(" Ohms");
  
  // Check if readings are reasonable
  bool readingsOK = true;
  if (bme.temperature < -40 || bme.temperature > 85) {
    Serial.println("  ⚠ Temperature reading seems unusual");
    readingsOK = false;
  }
  if (bme.humidity < 0 || bme.humidity > 100) {
    Serial.println("  ⚠ Humidity reading seems unusual");
    readingsOK = false;
  }
  if (bme.pressure < 30000 || bme.pressure > 110000) {
    Serial.println("  ⚠ Pressure reading seems unusual");
    readingsOK = false;
  }
  
  if (readingsOK) {
    Serial.println("✓ BME680 readings appear normal");
  }
  
  Serial.println();
}

void testSDCard() {
  Serial.println("TEST 4: SD Card Module");
  Serial.println("----------------------");
  
  Serial.print("Initializing SD card on CS pin "); Serial.println(SD_CS_PIN);
  
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("✗ SD card initialization failed!");
    Serial.println("  Check:");
    Serial.println("  - SD card is inserted properly");
    Serial.println("  - Wiring connections (especially CS pin)");
    Serial.println("  - SD card format (FAT16/FAT32)");
    return;
  }
  
  Serial.println("✓ SD card initialized successfully");
  
  // Get card info
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("✗ No SD card attached");
    return;
  }
  
  Serial.print("✓ SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.print("✓ SD Card Size: "); Serial.print(cardSize); Serial.println(" MB");
  
  // Test write/read
  Serial.println("Testing file write/read...");
  
  File testFile = SD.open("/test.txt", FILE_WRITE);
  if (testFile) {
    testFile.println("ESP32 SD card test");
    testFile.close();
    Serial.println("✓ Test file written successfully");
  } else {
    Serial.println("✗ Failed to open test file for writing");
    return;
  }
  
  testFile = SD.open("/test.txt");
  if (testFile) {
    String content = testFile.readString();
    testFile.close();
    if (content.indexOf("ESP32 SD card test") >= 0) {
      Serial.println("✓ Test file read successfully");
      Serial.println("✓ SD card read/write test PASSED");
    } else {
      Serial.println("✗ Test file content doesn't match");
    }
  } else {
    Serial.println("✗ Failed to open test file for reading");
  }
  
  // Clean up
  SD.remove("/test.txt");
  Serial.println();
}

void testCombined() {
  Serial.println("TEST 5: Combined Functionality");
  Serial.println("------------------------------");
  
  Serial.println("Testing sensor + SD card logging...");
  
  if (!bme.performReading()) {
    Serial.println("✗ Cannot read sensor data");
    return;
  }
  
  File dataFile = SD.open("/combined_test.csv", FILE_WRITE);
  if (!dataFile) {
    Serial.println("✗ Cannot open SD file for writing");
    return;
  }
  
  // Write header if new file
  if (dataFile.size() == 0) {
    dataFile.println("Timestamp,Temperature,Humidity,Pressure,GasResistance");
  }
  
  // Write data
  dataFile.print(millis()); dataFile.print(",");
  dataFile.print(bme.temperature); dataFile.print(",");
  dataFile.print(bme.humidity); dataFile.print(",");
  dataFile.print(bme.pressure / 100.0); dataFile.print(",");
  dataFile.println(bme.gas_resistance);
  
  dataFile.close();
  
  Serial.println("✓ Combined sensor + SD logging test PASSED");
  Serial.println("✓ Data logged to /combined_test.csv");
  
  Serial.println("\n ALL TESTS COMPLETED");
  Serial.println("Your ESP32 setup is ready for the main application");
}