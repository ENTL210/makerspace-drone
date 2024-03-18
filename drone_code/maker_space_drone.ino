/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu; // create an accelerometer object

Servo ESC1;  // create servo object to control a servo
Servo ESC2;  // create servo object to control a servo
Servo ESC3;  // create servo object to control a servo
Servo ESC4;  // create servo object to control a servo

// Initialize values for calculation purposes...
double gravity = 9.81;
double pi = 3.141592;

// Accelerometers...
double xAccel, yAccel, zAccel = 0;
double pitchAngle, rollAngle = 0;
double stationaryGravity;

// Initialize time variable
double now, dt, lastTime;

// PID Loops...
double pitchKp = 0.35, pitchKd = 0.12;
double rollKp = 0.42, rollKd = 0.18;
double pitchEquilibrium = 0, rollEquilibrium = 0;
double pitchLastError, rollLastError;
double pitchOutput, rollOutput;


// Initialize the speed of each motor...
float spd1, spd2, spd3, spd4 = 60;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  // Setting the range & bandwith of the accelerometer...
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Calibrate the motor if needed
  // motorCalibration(); 
}

void loop() {
  readAccelerometer();

  // Calculate the stationary gravity
  stationaryGravity = sqrt(xAccel * xAccel + yAccel * yAccel + zAccel * zAccel);

  // Calculate the roll...
  rollAngle = -atan2f(yAccel, stationaryGravity) * (180 / pi);

  // Compute the pitch angle...
  pitchAngle = -asinf(xAccel / stationaryGravity) * (180 / pi);

  pitchOutput = calculatePID(pitchAngle, pitchLastError, pitchEquilibrium, pitchKp, pitchKd);
  rollOutput = calculatePID(rollAngle, rollLastError, rollEquilibrium, rollKp, rollKd);

  spd1 = 60 + pitchOutput - rollOutput;
  spd2 = 60 + pitchOutput + rollOutput;
  spd3 = 60 - pitchOutput + rollOutput;
  spd4 = 60 - pitchOutput - rollOutput;

  ESC1.write(spd1); //Set the ESC signal to minimum
  ESC2.write(spd2); //Set the ESC signal to minimum
  ESC3.write(spd3); //Set the ESC signal to minimum
  ESC4.write(spd4); //Set the ESC signal to minimum

  // Print out the pitch & roll angles 
  Serial.print("Picth Angle: ");
  Serial.print(pitchAngle);
  Serial.print(", Roll Angle: ");
  Serial.print(rollAngle);
  Serial.println(" degrees");
  Serial.print("Output: ");
  Serial.print(pitchOutput);
  Serial.print(", ");
  Serial.print(rollOutput);
  Serial.println("");
}

double calculatePID(double input, double prevError, double setPoint, double kP, double kD) {
  now = millis();
  dt = now - lastTime;
  double error = setPoint - input;
  double proportional = error;
  double derivative = (error - prevError) / dt;
  double output = proportional * kP + derivative * kD;
  prevError = error;
  lastTime = now;
  delay(100);
  return output;
}

void readAccelerometer() {
  // Read mutiple data points of an axis to reduced noise...
  float sampleSize  = 10; 
  float rawX, rawY, rawZ;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  rawX = 0;
  rawY = 0;
  rawZ = 0;

  for (float i = 0; i < sampleSize; i++) {
    rawX += a.acceleration.x - 0.6;
    rawY += a.acceleration.y - 0.2;
    rawZ += a.acceleration.z;
  }

  xAccel = (rawX / sampleSize);
  yAccel = (rawY / sampleSize);
  zAccel = (rawZ / sampleSize);

  delay(200);
}

void motorCalibration() {
  int throttle = 0;

  ESC1.attach(9,600,2500);
  ESC2.attach(6, 600,2500);
  ESC3.attach(3,600,2500);
  ESC4.attach(5, 600, 2500);

  //Due to problems with the ESC recognising the maximum
  //position at the default settings, the figures after
  //the pin number are the microsecond signals for the 
  //minimum and maximum that the ESC will recognise.
  // 600 and 2250 work.
  throttle = 180;  //Set the throttle to maximum
  ESC1.write(throttle); //Set the ESC signal to maximum
  ESC2.write(throttle); //Set the ESC signal to maximum
  ESC3.write(throttle); //Set the ESC signal to maximum
  ESC4.write(throttle); //Set the ESC signal to maximum
  //At this point the ESC's power should be connected.
  delay(5000);  //Allow the user time to connect the battery to
  //the ESC.  
  throttle = 0;  //Set the throttle to zero
  ESC1.write(throttle); //Set the ESC signal to minimum
  ESC2.write(throttle); //Set the ESC signal to minimum
  ESC3.write(throttle); //Set the ESC signal to minimum
  ESC4.write(throttle); //Set the ESC signal to minimum
  delay(5000);  // allow a 5 second delay for the ESC to signal

  //Set throttle to the neutral position.
  throttle = 60;  
  ESC1.write(throttle); //Set the ESC signal to neutral
  ESC2.write(throttle); //Set the ESC signal to neutral
  ESC3.write(throttle); //Set the ESC signal to neutral
  ESC4.write(throttle); //Set the ESC signal to neutral
  delay(3000);   //The ESC should now be calibrated.
}
