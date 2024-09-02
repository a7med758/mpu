#include <Wire.h>
#include <math.h>

const int MPU6050_ADDR = 0x68;

float t = 0.01;        // Time step in seconds (10 ms)
float gyroprv = 0;     // Previous gyroscope yaw angle
const float alpha = 0.03; // Complementary filter coefficient
float gyro;            // Gyroscope yaw estimation
float accel;           // Accelerometer yaw estimation
float yaw;             // Final yaw angle estimation

void setup() {
  Wire.begin(); // Initialize I2C communication
  Serial.begin(9600); // Initialize serial communication for debugging

  // Wake up the MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // Power management register
  Wire.write(0);    // Set to 0 to wake up the MPU6050
  Wire.endTransmission(true); // Corrected: Added missing semicolon
}

void loop() {
  // Read accelerometer data
  int16_t accelX = readMPU6050(0x3B); // X-axis acceleration high byte
  int16_t accelY = readMPU6050(0x3D); // Y-axis acceleration high byte

  // Read gyroscope data
  int16_t gyroZ = readMPU6050(0x47);  // Z-axis angular velocity high byte

  // Convert raw gyro data to degrees/sec (assuming gyro sensitivity)
  float gyroZ_rate = gyroZ / 131.0;  // 131.0 is a common sensitivity scale factor (change if needed)

  // Gyroscope-based yaw update
  gyro = gyroprv + (gyroZ_rate * t);

  // Accelerometer-based yaw calculation
  accel = atan2((float)accelY, (float)accelX); // Use atan2 to handle all quadrants

  // Complementary filter to fuse gyroscope and accelerometer data
  yaw = (1 - alpha) * gyro + alpha * accel;

  // Update previous yaw
  gyroprv = gyro;

  // Print the computed yaw
  Serial.println(yaw);
  delay(t * 1000); // Convert seconds to milliseconds
}

// Function to read data from MPU6050
int16_t readMPU6050(uint8_t reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg); // Set register address to read from
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 2, true); // Request 2 bytes of data

  int16_t data = Wire.read() << 8 | Wire.read(); // Combine high and low bytes
  return data;
}
