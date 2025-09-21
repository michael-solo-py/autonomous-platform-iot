#include <WiFi.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <time.h>

// Pin definitions based on your wiring
#define SD_CS_PIN 5
#define BME_SDA_PIN 21
#define BME_SCL_PIN 22

// Performance monitoring variables
struct PerformanceBenchmarks {
  unsigned long totalReadings = 0;
  unsigned long successfulReadings = 0;
  unsigned long failedReadings = 0;
  unsigned long sdWriteErrors = 0;
  unsigned long maxLoopTime = 0;
  unsigned long minLoopTime = ULONG_MAX;
  unsigned long totalLoopTime = 0;
  float avgLoopTime = 0;
  unsigned long memoryUsage = 0;
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
File dataFile;
File logFile;
const char* dataFileName = "/sensor_data.csv";
const char* logFileName = "/system_log.txt";

// Configuration
const unsigned long READING_INTERVAL = 5000; // 5 seconds
const unsigned long LOG_INTERVAL = 30000;    // 30 seconds for performance log
const unsigned long BENCHMARK_REPORT_INTERVAL = 300000; // 5 minutes

unsigned long lastReading = 0;
unsigned long lastLog = 0;
unsigned long lastBenchmarkReport = 0;

void setup() {
  Serial.begin(115200);
  perf.startTime = millis();
  
  Serial.println("ESP32 Environmental Data Logger Starting...");
  
  // Initialize I2C for BME680
  Wire.begin(BME_SDA_PIN, BME_SCL_PIN);
  
  // Initialize BME680
  if (!initializeBME680()) {
    Serial.println("Failed to initialize BME680!");
    while(1) delay(1000);
  }
  
  // Initialize SD Card
  if (!initializeSDCard()) {
    Serial.println("Failed to initialize SD Card!");
    while(1) delay(1000);
  }
  
  // Create CSV header if file doesn't exist
  createDataFileHeader();
  
  Serial.println("System initialized successfully!");
  logSystemEvent("System startup completed");
}

void loop() {
  unsigned long loopStartTime = millis();
  
  // Check if it's time for a sensor reading
  if (millis() - lastReading >= READING_INTERVAL) {
    SensorData data = readSensorData();
    
    if (data.valid) {
      logDataToSD(data);
      printDataToSerial(data);
      perf.successfulReadings++;
    } else {
      perf.failedReadings++;
      logSystemEvent("Sensor reading failed");
    }
    
    perf.totalReadings++;
    lastReading = millis();
  }
  
  // Performance logging
  if (millis() - lastLog >= LOG_INTERVAL) {
    updatePerformanceMetrics();
    lastLog = millis();
  }
  
  // Benchmark reporting
  if (millis() - lastBenchmarkReport >= BENCHMARK_REPORT_INTERVAL) {
    generateBenchmarkReport();
    lastBenchmarkReport = millis();
  }
  
  // Calculate loop performance
  unsigned long loopTime = millis() - loopStartTime;
  updateLoopPerformance(loopTime);
  
  delay(100); // Small delay to prevent overwhelming the system
}

bool initializeBME680() {
  if (!bme.begin()) {
    return false;
  }
  
  // Set oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150 ms
  
  return true;
}

bool initializeSDCard() {
  if (!SD.begin(SD_CS_PIN)) {
    return false;
  }
  
  Serial.println("SD Card initialized successfully");
  Serial.print("SD Card Size: ");
  Serial.print(SD.cardSize() / (1024 * 1024));
  Serial.println(" MB");
  
  return true;
}

void createDataFileHeader() {
  if (!SD.exists(dataFileName)) {
    dataFile = SD.open(dataFileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("Timestamp,Temperature(C),Humidity(%),Pressure(hPa),GasResistance(Ohms),Millis");
      dataFile.close();
      Serial.println("Created new data file with header");
    }
  }
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

void logDataToSD(const SensorData& data) {
  dataFile = SD.open(dataFileName, FILE_APPEND);
  
  if (dataFile) {
    // Format: timestamp,temp,humidity,pressure,gas,millis
    dataFile.print(data.timestamp);
    dataFile.print(",");
    dataFile.print(data.temperature, 2);
    dataFile.print(",");
    dataFile.print(data.humidity, 2);
    dataFile.print(",");
    dataFile.print(data.pressure, 2);
    dataFile.print(",");
    dataFile.print(data.gasResistance, 0);
    dataFile.print(",");
    dataFile.println(millis());
    
    dataFile.close();
  } else {
    perf.sdWriteErrors++;
    Serial.println("Error writing to SD card");
  }
}

void printDataToSerial(const SensorData& data) {
  Serial.println("--- Sensor Reading ---");
  Serial.print("Temperature: "); Serial.print(data.temperature); Serial.println(" °C");
  Serial.print("Humidity: "); Serial.print(data.humidity); Serial.println(" %");
  Serial.print("Pressure: "); Serial.print(data.pressure); Serial.println(" hPa");
  Serial.print("Gas Resistance: "); Serial.print(data.gasResistance); Serial.println(" Ohms");
  Serial.print("Timestamp: "); Serial.println(data.timestamp);
  Serial.println();
}

void updatePerformanceMetrics() {
  perf.uptime = (millis() - perf.startTime) / 1000.0; // seconds
  perf.memoryUsage = ESP.getFreeHeap();
  
  if (perf.totalReadings > 0) {
    perf.avgLoopTime = (float)perf.totalLoopTime / perf.totalReadings;
  }
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

void generateBenchmarkReport() {
  Serial.println("\n=== PERFORMANCE BENCHMARK REPORT ===");
  Serial.print("Uptime: "); Serial.print(perf.uptime); Serial.println(" seconds");
  Serial.print("Total Readings: "); Serial.println(perf.totalReadings);
  Serial.print("Successful Readings: "); Serial.println(perf.successfulReadings);
  Serial.print("Failed Readings: "); Serial.println(perf.failedReadings);
  Serial.print("Success Rate: "); 
  if (perf.totalReadings > 0) {
    Serial.print((float)perf.successfulReadings / perf.totalReadings * 100); 
    Serial.println("%");
  } else {
    Serial.println("N/A");
  }
  Serial.print("SD Write Errors: "); Serial.println(perf.sdWriteErrors);
  Serial.print("Free Memory: "); Serial.print(perf.memoryUsage); Serial.println(" bytes");
  Serial.print("Max Loop Time: "); Serial.print(perf.maxLoopTime); Serial.println(" ms");
  Serial.print("Min Loop Time: "); Serial.print(perf.minLoopTime); Serial.println(" ms");
  Serial.print("Avg Loop Time: "); Serial.print(perf.avgLoopTime); Serial.println(" ms");
  
  // Calculate readings per minute
  float readingsPerMinute = 0;
  if (perf.uptime > 0) {
    readingsPerMinute = (perf.successfulReadings * 60.0) / perf.uptime;
  }
  Serial.print("Readings per Minute: "); Serial.println(readingsPerMinute);
  
  // Log benchmark to SD card
  logBenchmarkToSD();
  
  Serial.println("===================================\n");
}

void logBenchmarkToSD() {
  File benchmarkFile = SD.open("/benchmark_log.csv", FILE_APPEND);
  
  if (benchmarkFile) {
    // Create header if new file
    if (benchmarkFile.size() == 0) {
      benchmarkFile.println("Timestamp,Uptime,TotalReadings,SuccessfulReadings,FailedReadings,SuccessRate,SDErrors,FreeMemory,MaxLoopTime,MinLoopTime,AvgLoopTime,ReadingsPerMin");
    }
    
    // Write benchmark data
    benchmarkFile.print(millis()); benchmarkFile.print(",");
    benchmarkFile.print(perf.uptime); benchmarkFile.print(",");
    benchmarkFile.print(perf.totalReadings); benchmarkFile.print(",");
    benchmarkFile.print(perf.successfulReadings); benchmarkFile.print(",");
    benchmarkFile.print(perf.failedReadings); benchmarkFile.print(",");
    
    float successRate = perf.totalReadings > 0 ? (float)perf.successfulReadings / perf.totalReadings * 100 : 0;
    benchmarkFile.print(successRate); benchmarkFile.print(",");
    
    benchmarkFile.print(perf.sdWriteErrors); benchmarkFile.print(",");
    benchmarkFile.print(perf.memoryUsage); benchmarkFile.print(",");
    benchmarkFile.print(perf.maxLoopTime); benchmarkFile.print(",");
    benchmarkFile.print(perf.minLoopTime); benchmarkFile.print(",");
    benchmarkFile.print(perf.avgLoopTime); benchmarkFile.print(",");
    
    float readingsPerMinute = perf.uptime > 0 ? (perf.successfulReadings * 60.0) / perf.uptime : 0;
    benchmarkFile.println(readingsPerMinute);
    
    benchmarkFile.close();
  }
}

void logSystemEvent(const char* event) {
  logFile = SD.open(logFileName, FILE_APPEND);
  if (logFile) {
    logFile.print(millis());
    logFile.print(" - ");
    logFile.println(event);
    logFile.close();
  }
  
  Serial.print("LOG: ");
  Serial.println(event);
}
