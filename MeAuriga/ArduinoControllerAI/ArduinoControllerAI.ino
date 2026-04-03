/**
 * @file    ArduinoControllerAI.ino
 * @author  dani.gutierrez@gmail.com
 * @version V1.2.0
 * @date    2026/04/03
 * @brief   mRobot Explorer - AI driven version.
 */
#include <Wire.h>
#include "MeAuriga.h"
#include <TFminiS.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>


// Modules and sensors
MeBluetooth bluetooth(PORT_16);           // Bluetooth
MeUltrasonicSensor ultraSensor(PORT_10);  // Ultrasonic module
MeEncoderOnBoard motor1(SLOT1);           // Left Motor
MeEncoderOnBoard motor2(SLOT2);           // Right Motor
#define tfSerial Serial2                  // Lidar
TFminiS tfmini(tfSerial);                 // Lidar
//--

/*****************/
/*      Common   */
/*****************/
#define FORWARD_SCAN_ANGLE 60
#define EXTENSION_I2C_ID 2
#define MAIN_CONTROLLER_I2C_ID 1
#define SERIAL_BAUD 115200

#define REQUEST_COMMAND_DELAY 100
#define READ_SENSORS_DELAY 10


int distanceLidar;
int distanceUltrasonic;
int angle = 0;

float leftMotorCurrentSpeed;
float rightMotorCurrentSpeed;
static unsigned long lastRequestSent = 0;
static unsigned long lastSensorsRead = 0;


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
  if (millis() - lastSensorsRead > READ_SENSORS_DELAY) {
    requestScanAngle();
    lastSensorsRead = millis();
    updateDistance();
  }
  
  move();

  if (millis() - lastRequestSent > REQUEST_COMMAND_DELAY) {
    sendTelemetry();
    getCommands();

    lastRequestSent = millis();
  }
}

void sendTelemetry() {
  StaticJsonDocument<256> tel;
  tel["Lidar"] = distanceLidar;
  tel["Angle"] = angle;
  tel["LeftMotorSpeed"] = -leftMotorCurrentSpeed;
  tel["RightMotorSpeed"] = rightMotorCurrentSpeed;

  // Serialize and send
  String json;
  serializeJson(tel, json);
  bluetooth.println(json);
}

void getCommands() {
  int readData = 0;
  int count = 0;
  char buffer[256];

  while (bluetooth.available() && count < sizeof(buffer) - 1) {
    readData = bluetooth.read();
    if (readData != -1) {
      buffer[count] = (char)readData;
      count++;
      buzzerOn();
    }
    delay(1);
  }

  if (count > 0) {
    buffer[count] = '\0';
    StaticJsonDocument<256> com;
    DeserializationError derror = deserializeJson(com, buffer);

    if (!derror) {
      leftMotorCurrentSpeed = com["LeftMotorSpeed"];
      rightMotorCurrentSpeed = com["RightMotorSpeed"];
    }
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
/*    Movement   */
/*****************/
void move() {
  motor1.setMotorPwm(-leftMotorCurrentSpeed);  // Inverted due to tank tread orientation
  motor2.setMotorPwm(rightMotorCurrentSpeed);
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
