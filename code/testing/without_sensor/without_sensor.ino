/*
  ESP32 Environmental Data Sender via UART
  Sends BME680 data to Raspberry Pi via Serial/UART
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

// Pin definitions
#define BME_SDA_PIN 21
#define BME_SCL_PIN 22
#define LED_PIN 2

// Performance monitoring
struct PerformanceBenchmarks {
  unsigned long totalReadings = 0;
  unsigned long successfulReadings = 0;
  unsigned long failedReadings = 0;
  unsigned long maxLoopTime = 0;
  unsigned long minLoopTime = ULONG_MAX;
  unsigned long totalLoopTime = 0;
  unsigned long startTime = 0;
  float uptime = 0;
};

// Sensor data structure
struct SensorData {
  float temperature;
  float humidity;
  float pressure;
  float gasResistance;
  unsigned long timestamp;
  bool valid;
};

// Global objects
Adafruit_BME680 bme;
PerformanceBenchmarks perf;

// Configuration
const unsigned long READING_INTERVAL = 5000;  // 5 seconds
const unsigned long BENCHMARK_REPORT_INTERVAL = 300000; // 5 minutes

unsigned long lastReading = 0;
unsigned long lastBenchmarkReport = 0;

void setup() {
  Serial.begin(115200); // UART to Raspberry Pi
  perf.startTime = millis();
  
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize I2C for BME680
  Wire.begin(BME_SDA_PIN, BME_SCL_PIN);
  
  // Send startup message
  Serial.println("SYSTEM:ESP32_STARTING");
  
  // Initialize BME680
  if (!initializeBME680()) {
    Serial.println("ERROR:BME680_INIT_FAILED");
    while(1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  
  Serial.println("SYSTEM:ESP32_READY");
  digitalWrite(LED_PIN, HIGH); // Solid LED = ready
}

void loop() {
  unsigned long loopStartTime = millis();
  
  // Check if it's time for a sensor reading
  if (millis() - lastReading >= READING_INTERVAL) {
    SensorData data = readSensorData();
    
    if (data.valid) {
      sendDataToRaspberryPi(data);
      perf.successfulReadings++;
      
      // Quick LED blink to show data sent
      digitalWrite(LED_PIN, LOW);
      delay(50);
      digitalWrite(LED_PIN, HIGH);
    } else {
      perf.failedReadings++;
      Serial.println("ERROR:SENSOR_READ_FAILED");
    }
    
    perf.totalReadings++;
    lastReading = millis();
  }
  
  // Performance reporting
  if (millis() - lastBenchmarkReport >= BENCHMARK_REPORT_INTERVAL) {
    sendBenchmarkReport();
    lastBenchmarkReport = millis();
  }
  
  // Calculate loop performance
  unsigned long loopTime = millis() - loopStartTime;
  updateLoopPerformance(loopTime);
  
  delay(100);
}

bool initializeBME680() {
  if (!bme.begin()) {
    return false;
  }
  
  // Configure sensor
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);
  
  return true;
}

SensorData readSensorData() {
  SensorData data;
  data.timestamp = millis();
  data.valid = false;
  
  if (!bme.performReading()) {
    return data;
  }
  
  data.temperature = bme.temperature;
  data.humidity = bme.humidity;
  data.pressure = bme.pressure / 100.0; // Convert Pa to hPa
  data.gasResistance = bme.gas_resistance;
  data.valid = true;
  
  return data;
}

void sendDataToRaspberryPi(const SensorData& data) {
  // Send data in structured format for easy parsing
  Serial.print("DATA:");
  Serial.print(data.timestamp);
  Serial.print(",");
  Serial.print(data.temperature, 2);
  Serial.print(",");
  Serial.print(data.humidity, 2);
  Serial.print(",");
  Serial.print(data.pressure, 2);
  Serial.print(",");
  Serial.print(data.gasResistance, 0);
  Serial.println();
}

void updateLoopPerformance(unsigned long loopTime) {
  if (loopTime > perf.maxLoopTime) {
    perf.maxLoopTime = loopTime;
  }
  if (loopTime < perf.minLoopTime) {
    perf.minLoopTime = loopTime;
  }
  perf.totalLoopTime += loopTime;
}

void sendBenchmarkReport() {
  perf.uptime = (millis() - perf.startTime) / 1000.0;
  
  Serial.println("BENCHMARK:START");
  Serial.print("BENCHMARK:UPTIME,"); Serial.println(perf.uptime);
  Serial.print("BENCHMARK:TOTAL_READINGS,"); Serial.println(perf.totalReadings);
  Serial.print("BENCHMARK:SUCCESSFUL_READINGS,"); Serial.println(perf.successfulReadings);
  Serial.print("BENCHMARK:FAILED_READINGS,"); Serial.println(perf.failedReadings);
  
  float successRate = perf.totalReadings > 0 ? (float)perf.successfulReadings / perf.totalReadings * 100 : 0;
  Serial.print("BENCHMARK:SUCCESS_RATE,"); Serial.println(successRate);
  
  Serial.print("BENCHMARK:FREE_MEMORY,"); Serial.println(ESP.getFreeHeap());
  Serial.print("BENCHMARK:MAX_LOOP_TIME,"); Serial.println(perf.maxLoopTime);
  Serial.print("BENCHMARK:MIN_LOOP_TIME,"); Serial.println(perf.minLoopTime);
  
  float avgLoopTime = perf.totalReadings > 0 ? (float)perf.totalLoopTime / perf.totalReadings : 0;
  Serial.print("BENCHMARK:AVG_LOOP_TIME,"); Serial.println(avgLoopTime);
  
  float readingsPerMinute = perf.uptime > 0 ? (perf.successfulReadings * 60.0) / perf.uptime : 0;
  Serial.print("BENCHMARK:READINGS_PER_MIN,"); Serial.println(readingsPerMinute);
  
  Serial.println("BENCHMARK:END");
}