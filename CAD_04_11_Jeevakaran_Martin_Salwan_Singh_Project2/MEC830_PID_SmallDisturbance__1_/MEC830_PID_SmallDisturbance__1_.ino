#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

float Kp = 120, Ki = 1.3, Kd = 4; // Adjusted PID gains
float setpoint = 0.0, integral = 0.0, previousError = 0.0;
float integralMax = 50; // Integral windup prevention
float filteredAngle = 0.0, alpha = 0.98; // Complementary filter weight
unsigned long previousTime = 0;
unsigned long controlInterval = 2; // Faster loop interval

#define PWMA 5
#define PWMB 6
#define AIN 7
#define BIN 8
#define STBY 3

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.begin();
  mpu.calcOffsets();
  
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN, OUTPUT);
  pinMode(BIN, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  setpoint = mpu.getAngleX(); // Initial upright position
}

void loop() {
  unsigned long currentTime = micros();
  if (currentTime - previousTime >= controlInterval * 1000) {
    float dt = (currentTime - previousTime) / 1000000.0;
    previousTime = currentTime;

    mpu.update();
    float angleX = mpu.getAngleX();
    filteredAngle = alpha * (filteredAngle + mpu.getGyroX() * dt) + (1 - alpha) * angleX;

    float error = setpoint - filteredAngle;

    // Integral with windup prevention
    if (abs(error) < 15) { // Avoid integral action during large disturbances
      integral += error * dt;
      integral = constrain(integral, -integralMax, integralMax);
    } else {
      integral = 0;
    }

    float derivative = -mpu.getGyroX(); // Anti-derivative kick

    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    int motorSpeed = constrain(abs(output), 0, 255);
    if (abs(output) < 10) motorSpeed = 0; // Deadband for noise

    if (output > 0) {
      digitalWrite(AIN, HIGH);
      digitalWrite(BIN, HIGH);
      analogWrite(PWMA, motorSpeed);
      analogWrite(PWMB, motorSpeed);
    } else {
      digitalWrite(AIN, LOW);
      digitalWrite(BIN, LOW);
      analogWrite(PWMA, motorSpeed);
      analogWrite(PWMB, motorSpeed);
    }

    // Debugging
    Serial.print("Angle: ");
    Serial.print(filteredAngle);
    Serial.print(" | Error: ");
    Serial.print(error);
    Serial.print(" | Output: ");
    Serial.println(output);
  }
}
