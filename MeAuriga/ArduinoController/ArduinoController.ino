/**
 * @file    Explorer.ino
 * @author  dani.gutierrez@gmail.com
 * @version V1.2.0
 * @date    2024/03/07
 * @brief   mRobot Explorer - Digital Twin for Arduino C with improved logic from ArduinoController.cs.
 */
#include <stdlib.h>
#include <Wire.h>
#include "MeAuriga.h"
#include <Math.h>
#include <TFminiS.h>
#include <Ewma.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>


// Modules and sensors
MeBluetooth bluetooth(PORT_16);           // Bluetooth
MeUltrasonicSensor ultraSensor(PORT_10);  // Ultrasonic module
MeEncoderOnBoard motor1(SLOT1);           // Left Motor
MeEncoderOnBoard motor2(SLOT2);           // Right Motor
MeGyro gyro(1, 0x69);                     // Gyroscope
#define tfSerial Serial2                  // Lidar
TFminiS tfmini(tfSerial);                 // Lidar
//--

/*****************/
/*      Common   */
/*****************/
#define MIN_DISTANCE 75
#define MIN_DISTANCE_MULT 1.1
#define MAX_SPEED 0
#define WIDE_SCAN_ANGLE 180
#define FORWARD_SCAN_ANGLE 60
#define EXTENSION_I2C_ID 2
#define MAIN_CONTROLLER_I2C_ID 1
#define SERIAL_BAUD 115200
#define TURN_SPEED_MULT 1.4
#define WAIT_SERVO_POSITION 1500
#define SCAN_MIN_DISTANCE 999999

int distanceLidar;
int distanceUltrasonic;
int angle = 0;
int state = 0;  // 0 - stopped, 1 - forward, 2 - backward, 3 - turn left, 4 - turn right, 5 - scan direction

int currentScanDirection = 1;  // Left = -1, Right = 1
bool currentScanObstacleDetected = false;
bool obstacleDetected = false;
int currentScanMaxDistance = -1;
int currentScanMinDistance = SCAN_MIN_DISTANCE;
int currentScanMaxDistanceAngle = -1;
int minDistance = -1;

int maxDistanceAngle = -1;
bool waitNextScan = false;

static unsigned long lastSensorsRead = 0;
static unsigned long lastLogSent = 0;

/*****************/
/*    MAIN       */
/*****************/
void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.setClock(400000);

  if (bluetooth.available()) {
    bluetooth.println("AT+RESET");
  }

  //I2C
  Wire.begin(MAIN_CONTROLLER_I2C_ID);
  Wire.setWireTimeout();

  // Lidar
  tfSerial.begin(SERIAL_BAUD);

  // Gyro
  gyro.begin();

  // Enable motor encoder interrupts
  attachInterrupt(
    motor1.getIntNum(), []() {
      motor1.pulsePosPlus();
    },
    RISING);
  attachInterrupt(
    motor2.getIntNum(), []() {
      motor2.pulsePosPlus();
    },
    RISING);
  // --

  // Set motors PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  //--

  // Motor configuration
  motor1.setPulse(9);
  motor2.setPulse(9);
  motor1.setRatio(39.267);
  motor2.setRatio(39.267);
  motor1.setPosPid(1.8, 0, 1.2);
  motor2.setPosPid(1.8, 0, 1.2);
  motor1.setSpeedPid(0.18, 0, 0);
  motor2.setSpeedPid(0.18, 0, 0);
  // --

  sendScanMaxAngle(FORWARD_SCAN_ANGLE);
}

void loop() {
  if (millis() - lastSensorsRead > 250) {
    requestScanAngle();
    lastSensorsRead = millis();
  }
  
  updateDistance();
  detectObstacles();
  move();

  if (millis() - lastLogSent > 100) {
    log();
    lastLogSent = millis();
  }
}

void log() {
  Serial.print("State: ");
  Serial.print(state);
  Serial.print(", Lidar: ");
  Serial.print(distanceLidar);
  Serial.print(", Angle: ");
  Serial.print(angle);
  Serial.print(", ObstacleDetected: ");
  Serial.print(obstacleDetected);
  Serial.print(", CurrentScanMaxDistance: ");
  Serial.print(currentScanMaxDistance);
  Serial.print(", CurrentScanMaxDistanceAngle: ");
  Serial.print(currentScanMaxDistanceAngle);
  Serial.print(", MaxDistanceAngle: ");
  Serial.print(maxDistanceAngle);
  Serial.print(", WaitNextScan: ");
  Serial.print(waitNextScan);
  Serial.print(", leftMotorSpeed: ");
  Serial.print(motor1.getCurrentSpeed());
  Serial.print(", rightMotorSpeed: ");
  Serial.print(motor2.getCurrentSpeed());
  Serial.println();

  if (bluetooth.available())
  {
    StaticJsonDocument<256> tel;
    tel["State"] = state;
    tel["Lidar"] = distanceLidar;
    tel["Ultrasonic"] = distanceUltrasonic;
    tel["Angle"] = angle;
    tel["ObstacleDetected"] = obstacleDetected;
    tel["CurrentScanMaxDistance"] = currentScanMaxDistance;
    tel["CurrentScanMaxDistanceAngle"] = currentScanMaxDistanceAngle;
    tel["MaxDistanceAngle"] = maxDistanceAngle;
    tel["WaitNextScan"] = waitNextScan;
    tel["LeftMotorSpeed"] = motor1.getCurrentSpeed();
    tel["RightMotorSpeed"] = motor2.getCurrentSpeed();

    // Serialize and send
    String json;
    serializeJson(tel, json);
    bluetooth.println(json);
  }
}

/*****************/
/*      Lidar    */
/*****************/
void updateDistance() {
  tfmini.readSensor();
  int dist = tfmini.getDistance();
  int strength = tfmini.getStrength();
  int temperature = tfmini.getTemperature();

  // Check for and handle any errors.
  if (dist >= 0) {
    distanceLidar = dist;
  }
}

/*****************/
/*  Ultrasonic   */
/*****************/
void updateUltrasonic() {
  distanceUltrasonic = ultraSensor.distanceCm();
}

/*****************/
/*      Gyro     */
/*****************/
void updateGyro() {
  gyro.update();
}

/*****************/
/*    Movement   */
/*****************/
void move(float leftSpeed, float rightSpeed) {
  // TODO: use setCurrentSpeed() instead
  motor1.setMotorPwm((int)-leftSpeed);  // Inverted due to tank tread orientation
  motor2.setMotorPwm((int)rightSpeed);
}

/*********************/
/* I2C Communication */
/*********************/
void requestScanAngle() {
  Wire.requestFrom(EXTENSION_I2C_ID, 10);
  if (Wire.available() >= 10) {
    char receivedBuffer[10];
    int index = 0;

    while (Wire.available() && index < sizeof(receivedBuffer) - 1) {
      receivedBuffer[index++] = Wire.read();  // Read bytes into buffer
    }
    receivedBuffer[index] = '\0';  // Null-terminate the string
    angle = atoi(receivedBuffer);  // Convert string to int
  }
  
}

void sendScanMaxAngle(int openAngle) {
  Wire.beginTransmission(EXTENSION_I2C_ID);  // Reset servo angle
  Wire.write(openAngle);
  Wire.endTransmission();
}

/**************/
/* Move Logic */
/**************/
void detectObstacles() {
  if (angle < 0 && currentScanDirection == 1) {
    if (!waitNextScan) {
      maxDistanceAngle = currentScanMaxDistanceAngle;
      obstacleDetected = currentScanMinDistance < minDistance ? true : false;
      
    }
    currentScanDirection = -1;
    currentScanMaxDistanceAngle = -1;
    currentScanMaxDistance = -1;
    currentScanMinDistance = SCAN_MIN_DISTANCE;
    waitNextScan = false;
  }

  if (angle > 0 && currentScanDirection == -1) {
    if (!waitNextScan) {
      maxDistanceAngle = currentScanMaxDistanceAngle;
      obstacleDetected = currentScanMinDistance < minDistance ? true : false;
    }

    currentScanDirection = 1;
    currentScanMaxDistanceAngle = -1;
    currentScanMaxDistance = -1;
    currentScanMinDistance = SCAN_MIN_DISTANCE;
    waitNextScan = false;
  }
  currentScanMaxDistanceAngle = distanceLidar > currentScanMaxDistance ? abs(angle) : currentScanMaxDistanceAngle;
  currentScanMinDistance = min(currentScanMinDistance, distanceLidar);
  currentScanMaxDistance = max(currentScanMaxDistance, distanceLidar);
}

void move() {
  if (state == 0)  // Stopped
  {
    if (!obstacleDetected) 
    {
      state = 1;
      minDistance = MIN_DISTANCE;
    } 
    else 
    {
      minDistance = MIN_DISTANCE * MIN_DISTANCE_MULT;
      sendScanMaxAngle(WIDE_SCAN_ANGLE);
      waitNextScan = true;
      maxDistanceAngle = -1;
      state = 5;  // Find farthest direction
    }
  } 
  else if (state == 1)  // Forward
  {
    if (obstacleDetected) 
    {
      move(0, 0);
      state = 0;
    } 
    else 
    {
      move(MAX_SPEED, MAX_SPEED);
    }
  } 
  else if (state == 2)  // Backward
  {
    if (!obstacleDetected) 
    {
      state = 4;  // Turn right after backing
    } 
    else 
    {
      move(-MAX_SPEED, -MAX_SPEED);
    }
  } 
  else if (state == 3)  // Turn left
  {
    if (!obstacleDetected) 
    {
      state = 0;
    } 
    else 
    {
      move(MAX_SPEED * TURN_SPEED_MULT, -MAX_SPEED * TURN_SPEED_MULT);
    }
  } 
  else if (state == 4)  // Turn right
  {
    if (!obstacleDetected) 
    {
      state = 0;
    } 
    else 
    {
      move(-MAX_SPEED * TURN_SPEED_MULT, MAX_SPEED * TURN_SPEED_MULT);
    }
  } 
  else if (state == 5)  // Find farthest direction
  {
    if (maxDistanceAngle != -1) 
    {
      state = (maxDistanceAngle < 90) ? 3 : 4;
      sendScanMaxAngle(FORWARD_SCAN_ANGLE);
      waitNextScan = true;
      obstacleDetected = true;
      delay(WAIT_SERVO_POSITION);
    }
  }
}
