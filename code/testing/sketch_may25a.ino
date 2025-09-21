 #include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

// BME680 sensor object
Adafruit_BME680 bme;

// Hardware Serial2 configuration (clean, no boot messages)
#define SERIAL2_BAUD_RATE 9600
#define RXD2 16  // GPIO16 - connect to Pi's TX
#define TXD2 17  // GPIO17 - connect to Pi's RX

// Status variables
bool sensorInitialized = false;
unsigned long lastReading = 0;
const unsigned long READING_INTERVAL = 10000; // 10 seconds

void setup() {
  // Initialize built-in LED for status indication (GPIO2 on most ESP32 boards)
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  
  // Initialize Hardware Serial2 for clean communication
  Serial2.begin(SERIAL2_BAUD_RATE, SERIAL_8N1, RXD2, TXD2);
  
  // Initialize regular Serial for debugging (optional)
  Serial.begin(115200);
  
  // Small delay for serial to initialize
  delay(1000);
  
  Serial2.println("=== ESP32 BME680 STARTING ===");
  Serial.println("ESP32 BME680 Data Transmitter Starting...");
  
  // Initialize I2C for BME680
  Wire.begin();
  
  // Initialize BME680 sensor with retry logic
  int attempts = 0;
  while (!bme.begin() && attempts < 5) {
    Serial2.println("BME680 sensor not found, retrying...");
    Serial.println("BME680 sensor not found, retrying...");
    delay(1000);
    attempts++;
  }
  
  if (attempts >= 5) {
    Serial2.println("ERROR: Could not initialize BME680 sensor after 5 attempts!");
    Serial.println("ERROR: Could not initialize BME680 sensor!");
    sensorInitialized = false;
  } else {
    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320°C for 150 ms
    
    Serial2.println("BME680 sensor initialized successfully!");
    Serial.println("BME680 sensor initialized successfully!");
    sensorInitialized = true;
    
    // LED on to indicate successful initialization
    digitalWrite(2, HIGH);
    
    // Send ready signal
    Serial2.println("SYSTEM_READY");
  }
  
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Check if it's time for a new reading
  if (currentTime - lastReading >= READING_INTERVAL) {
    if (sensorInitialized) {
      readAndSendData();
    } else {
      // Try to reinitialize sensor
      if (bme.begin()) {
        Serial2.println("BME680 sensor reconnected!");
        Serial.println("BME680 sensor reconnected!");
        sensorInitialized = true;
        digitalWrite(2, HIGH);
      } else {
        Serial2.println("ERROR: BME680 sensor still not available");
      }
    }
    lastReading = currentTime;
  }
  
  // Blink LED to show system is alive
  if (currentTime % 2000 < 100) {
    digitalWrite(2, !digitalRead(2));
  }
}

void readAndSendData() {
  // Read sensor data
  if (!bme.performReading()) {
    Serial2.println("ERROR: Failed to perform BME680 reading");
    Serial.println("ERROR: Failed to perform BME680 reading");
    return;
  }
  
  // Get current timestamp (milliseconds since boot)
  unsigned long timestamp = millis();
  
  // Validate sensor readings
  if (isnan(bme.temperature) || isnan(bme.humidity) || 
      isnan(bme.pressure) || isnan(bme.gas_resistance)) {
    Serial2.println("ERROR: Invalid sensor readings (NaN detected)");
    Serial.println("ERROR: Invalid sensor readings");
    return;
  }
  
  // Prepare data string in CSV format
  String dataString = "DATA," + String(timestamp) + "," +
                     String(bme.temperature, 2) + "," +
                     String(bme.humidity, 2) + "," +
                     String(bme.pressure / 100.0, 2) + "," +
                     String(bme.gas_resistance / 1000.0, 2);
  
  // Send data via Hardware Serial2 (clean channel)
  Serial2.println(dataString);
  
  // Also send to regular serial for debugging
  Serial.println(dataString);
  
  // Send heartbeat every 5 readings
  static int readingCount = 0;
  readingCount++;
  if (readingCount >= 5) {
    Serial2.println("HEARTBEAT," + String(timestamp));
    readingCount = 0;
  }
}

/*
 * Wiring connections for Hardware Serial2 method:
 * 
 * BME680 to ESP32:
 * VCC -> 3.3V
 * GND -> GND
 * SDA -> GPIO21
 * SCL -> GPIO22
 * 
 * 
 * This method uses Hardware Serial2 which:
 * - Has no boot message interference
 * - Provides clean data transmission
 * - Allows simultaneous debug output on Serial
 * - More reliable than USB Serial for data logging
 * 
 * On Raspberry Pi, the serial port will be /dev/ttyAMA0 or /dev/serial0