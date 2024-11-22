#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

float output = 0;

#define PWMA 5    // Controls power to right motor
#define PWMB 6    // Controls power to left motor
#define DIRA 7    // Controls direction of right motor, HIGH = FORWARD, LOW = REVERSE
#define DIRB 8    // Controls direction of left motor, HIGH = FORWARD, LOW = REVERSE
#define STBY 3    // Place H-Bridge in standby if LOW, Run if HIGH

float angleX = 0;       // Angle in the X direction
float angVelX = 0;      // Angular velovity in the X direction
float positionX = 0;    // Position in the X direction
float velocityX = 0;    // Velocity in the X direction

float setpoint = 0;

//Pole Placement Code
//==================================================================
float K[1][4] = {
    {-3.1623, -3.9827, 15.2011, 3.3551}     //Found in MATLAB
};

float state[4] = {0, 0, 0, 0};

float controlInput = 0;

float computeControlInput() {
    float u = 0;
    for (int i = 0; i < 4; i++) {
        u += -K[0][i] * state[i];
    }
    return u;
}
//==================================================================

void setup() {
  Serial.begin(115200);
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
  mpu.calcOffsets(); // Calculate offsets (gyro and accel)
  Serial.println("Calibration complete!");

  pinMode(PWMA, OUTPUT);     //set IO pin mode OUTPUT
  pinMode(PWMB, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  digitalWrite(STBY, HIGH);  //Enable Motors to run
  digitalWrite(PWMA, LOW);  // Fully on 
  digitalWrite(PWMA, LOW);  // Fully on

  setpoint = mpu.getAngleX();
}

void loop() {

  mpu.update(); // Update sensor readings

  // Get the angle in X direction
  angleX = mpu.getAngleX();

  // Get the angular velocity in X direction
  angVelX = mpu.getGyroX();

  //Pole Placement Controller
  //==================================================================
  state[0] = 0;               // Cart position
  state[1] = 0;               // Cart velocity
  state[2] = angleX;                  // Pendulum angle
  state[3] = angVelX;                 // Pendulum angular velocity

  output = computeControlInput()*2;
  //==================================================================
 
  // Motor control
  int motorSpeed = constrain(abs(output), 0, 255); // Constrain speed to 0-255 (PWM range)

  if (angleX < setpoint) {
    // Move all motors forward to correct backward tilt
    digitalWrite(DIRA, HIGH);    // Forward direction on Right
    digitalWrite(DIRB, HIGH);    // Forward direction on Left
    analogWrite(PWMA, motorSpeed);   // Power on Right
    analogWrite(PWMB, motorSpeed);   // Power on Left
  } else {
    // Move all motors backward to correct forward tilt
    digitalWrite(DIRA, LOW);    // Reverse direction on Right
    digitalWrite(DIRB, LOW);    // Reverse direction on Left
    analogWrite(PWMA, motorSpeed);   // Power on Right
    analogWrite(PWMB, motorSpeed);   // Power on Left
  }

  // Print debug information
  Serial.print("Angle X: ");
  Serial.print(angleX);
  Serial.print(" | Output: ");
  Serial.println(output);
}
