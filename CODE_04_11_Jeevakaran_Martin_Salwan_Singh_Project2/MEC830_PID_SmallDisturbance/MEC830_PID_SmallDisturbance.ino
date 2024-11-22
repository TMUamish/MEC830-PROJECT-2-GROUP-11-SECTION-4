#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

// PID Parameters
float Kp = 120;    // Proportional gain (adjust for faster response)
float Ki = 1.3;     // Integral gain (adjust to reduce steady-state error)
float Kd = 4;    // Derivative gain (adjust for smoother damping)

float setpoint = 0.0;      // Desired angle (upright position)
float integral = 0.0;      // Integral term
float previousError = 0.0; // Previous error for derivative calculation
float integralMax = 50;    // Limit for integral windup

// Motor Pins
#define PWMA 5    // Right motor PWM
#define PWMB 6    // Left motor PWM
#define AIN 7     // Right motor direction
#define BIN 8     // Left motor direction
#define STBY 3    // H-Bridge standby

unsigned long previousTime = 0;
unsigned long controlInterval = 5; // Control loop interval in ms (faster sampling)
float angleX = 0;                  // Angle in the X direction
float filteredAngle = 0;           // Filtered angle
float alpha = 0.95;                // Complementary filter weight

void setup() {
  Serial.begin(115200); // Faster debugging
  Wire.begin();

  // Initialize the MPU6050
  Serial.println("Initializing MPU6050...");
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print("MPU6050 initialization failed with code: ");
    Serial.println(status);
    while (1); // Stop the program if initialization fails
  }

  Serial.println("MPU6050 initialized successfully!");

  // Calibrate the sensor
  Serial.println("Calibrating MPU6050... Keep the device still.");
  delay(1000);
  mpu.calcOffsets();
  Serial.println("Calibration complete!");

  // Initialize motor pins
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN, OUTPUT);
  pinMode(BIN, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // Enable motors

  setpoint = mpu.getAngleX();
}

void loop() {
  unsigned long currentTime = micros(); // Use micros() for higher precision
  if (currentTime - previousTime >= controlInterval * 1000) {
    float dt = (currentTime - previousTime) / 1000000.0; // Time step in seconds
    previousTime = currentTime;

    mpu.update(); // Update sensor readings as quickly as possible

    // Complementary filter for stabilizing angle measurement
    angleX = mpu.getAngleX();
    filteredAngle = alpha * (filteredAngle + mpu.getGyroX() * dt) + (1 - alpha) * angleX;

    // Calculate the error
    float error = setpoint - filteredAngle;

    // Calculate integral term with windup prevention
    integral += error * dt;
    if (integral > integralMax) integral = integralMax;
    else if (integral < -integralMax) integral = -integralMax;

    // Calculate derivative term
    float derivative = (error - previousError) / dt;

    // Calculate PID output
    float output = Kp * error + Ki * integral + Kd * derivative;

    // Update previous error
    previousError = error;

    // Motor control
    int motorSpeed = constrain(abs(output), 0, 255); // Constrain speed to 0-255 (PWM range)

    if (output > 0) {
      // Move forward
      digitalWrite(AIN, HIGH);    // Right motor forward
      digitalWrite(BIN, HIGH);    // Left motor forward
      analogWrite(PWMA, motorSpeed);
      analogWrite(PWMB, motorSpeed);
    } else {
      // Move backward
      digitalWrite(AIN, LOW);    // Right motor backward
      digitalWrite(BIN, LOW);    // Left motor backward
      analogWrite(PWMA, motorSpeed);
      analogWrite(PWMB, motorSpeed);
    }

    // Print debug information
    Serial.print("Angle X: ");
    Serial.print(filteredAngle);
    Serial.print(" | Error: ");
    Serial.print(error);
    Serial.print(" | Output: ");
    Serial.println(output);
  }
}
