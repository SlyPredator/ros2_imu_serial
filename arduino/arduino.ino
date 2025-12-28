#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Define the baud rate (MUST match the ROS 2 config)
#define BAUDRATE 115200

// Define the sensor update rate (MUST match the 'rate_hz' in your ROS 2 yaml config)
const int PUBLISH_RATE_HZ = 50; 
const int PUBLISH_DELAY_MS = 1000 / PUBLISH_RATE_HZ;

// Create the MPU6050 object
Adafruit_MPU6050 mpu;

// For timing control
unsigned long previousMillis = 0;

// Use a unique name to avoid conflicts with ESP32 core macros
const float DEGREES_TO_RADIANS = 0.017453292519943295; // Pi / 180

void setup() {
  // Initialize Serial Communication
  Serial.begin(BAUDRATE);
  
  // Wait for serial port to connect. 
  while (!Serial) {
    delay(10);
  }

  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  
  // Initialize I2C with specific pins for ESP32 (SDA=21, SCL=22)
  Wire.begin(21, 22);
  delay(100);
  
  if (!mpu.begin(0x68)) { // Try address 0x68 (default)
    Serial.println("Failed to find MPU6050 at 0x68, trying 0x69...");
    if (!mpu.begin(0x69)) { // Try address 0x69 (AD0 high)
      Serial.println("Failed to find MPU6050 chip. Check wiring!");
      Serial.println("Common issues:");
      Serial.println("1. Wrong I2C pins");
      Serial.println("2. No pull-up resistors (4.7kΩ from SDA/SCL to 3.3V)");
      Serial.println("3. Power issues");
      while (1) {
        delay(1000);
      }
    }
  }
  
  // Configure MPU6050 settings for optimal performance
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Clear any old data
  Serial.flush();
  delay(100);
  
  Serial.println("MPU6050 Initialized successfully!");
  Serial.print("Publishing at ");
  Serial.print(PUBLISH_RATE_HZ);
  Serial.println(" Hz");
  Serial.println("Format: $Ax,Ay,Az,Gx,Gy,Gz");
  Serial.println("Ready for ROS2...");
}

void loop() {
  unsigned long currentMillis = millis();

  // Only publish data at the defined rate
  if (currentMillis - previousMillis >= PUBLISH_DELAY_MS) {
    previousMillis = currentMillis;

    // --- Read sensor data ---
    sensors_event_t a, g, temp;
    
    // Check if we can read data
    if (!mpu.getEvent(&a, &g, &temp)) {
      Serial.println("$ERROR,Read failed");
      return;
    }

    // --- Format and Publish Data to Serial ---
    // Format: $Ax,Ay,Az,Gx,Gy,Gz
    
    Serial.print("$");
    
    // Linear Acceleration (Ax, Ay, Az) - in m/s^2
    // Format: 3 decimal places
    Serial.print(a.acceleration.x, 3); Serial.print(",");
    Serial.print(a.acceleration.y, 3); Serial.print(",");
    Serial.print(a.acceleration.z, 3); Serial.print(",");

    // Angular Velocity (Gx, Gy, Gz) - convert from degrees/s to rad/s
    // Using unique name to avoid macro conflict
    Serial.print(g.gyro.x * DEGREES_TO_RADIANS, 6); Serial.print(",");
    Serial.print(g.gyro.y * DEGREES_TO_RADIANS, 6); Serial.print(",");
    Serial.print(g.gyro.z * DEGREES_TO_RADIANS, 6);
    
    // End the line
    Serial.println();
    
    // Optional: Flush to ensure data is sent immediately
    Serial.flush();
  }
  
  // Small delay to prevent watchdog issues
  delay(1);
}