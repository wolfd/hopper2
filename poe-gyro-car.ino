#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Servo.h>

MPU6050 mpu6050(Wire);

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

long timer = 0;

int escPin1 = 9;
int escPin2 = 10;
int escPin3 = 11;
int escPin4 = 6;

int minPulseRate        = 900;
int maxPulseRate        = 2000;
int throttleChangeDelay = 50;
int maxPulseRateMinusOne = maxPulseRate - 1;

float cmd1;
float cmd2;
float cmd3;
float cmd4;

void setup() {
  initEscs();

  Serial.begin(112500);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  mpu6050.update();

  Serial.print("\tangleX : "); Serial.print(mpu6050.getAngleX());
  Serial.print("\tangleY : "); Serial.print(mpu6050.getAngleY());
  Serial.print("\tangleZ : "); Serial.println(mpu6050.getAngleZ());

  // Update the command variables
  gyroControlUpdate(mpu6050.getAngleX());

  // Write the updated commands to the wheels
  esc1.write(cmd1);
  esc2.write(cmd2);
  esc3.write(cmd3);
  esc4.write(cmd4);
}

float constrainSig(input) {
  return min(max(input, -maxPulseRateMinusOne), maxPulseRateMinusOne);
}

// Control parameters
float kp = 0.1;
float ki = 0;
float kd = 0;
float jsig = 1.36;
float angleDes = 0.0;
float angleErrorAccum = 0.0;

void gyroControlUpdate(float xAngle) {
  // Other local variables
  float s;                        // empty speed holder
  float err = angleDes - xAngle;  // angle error term

  float frontSig;  // empty front wheel signal holder
  float rearSig;   // empty rear wheel signal holder

  s = (kp * err) + (ki * angleErrorAccum) + (kd * xAngle);
  // Update global error accumulator
  angleErrorAccum -= err;

  Serial.print("Error: "); Serial.print(angleErrorAccum);

  //frontSig = -jsig*s;
  //rearSig = jsig*s;

  // Now let's generate the signal for each wheel
  frontSig = jsig*s;
  rearSig = -jsig*s;

  // Signal limiter
  frontSig = constrainSig(frontSig);
  rearSig = constrainSig(rearSig);

  // Set front wheel speeds
  cmd1 = frontSig;
  cmd2 = frontSig;
  Serial.print("\tFront : "); Serial.print(frontSig);

  // Set back wheel speeds
  cmd3 = rearSig;
  cmd4 = rearSig;
  Serial.print("\tRear : ");  Serial.print(rearSig);
}

void writeTo4Escs(int throttle) {
  esc1.write(throttle);
  esc2.write(throttle);
  esc3.write(throttle);
  esc4.write(throttle);
}

void initEscs() {
  esc1.attach(escPin1, minPulseRate, maxPulseRate);
  esc2.attach(escPin2, minPulseRate, maxPulseRate);
  esc3.attach(escPin3, minPulseRate, maxPulseRate);
  esc4.attach(escPin4, minPulseRate, maxPulseRate);

  //Init motors with 0 value
  writeTo4Escs(0);
}
