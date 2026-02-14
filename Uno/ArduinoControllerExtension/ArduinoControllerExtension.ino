#include <Wire.h>
#include <time.h>
#include <stdlib.h>
#include <Servo.h>

#define EXTENSION_I2C_ID 2
#define MAIN_CONTROLLER_I2C_ID 1
#define SERVO_PORT_ID 9
#define SERVO_SPEED 0.5
#define SERVO_SPEED_MULT 0.5
#define SERIAL_BAUD 115200

Servo servo;
float angle = 0;
float speed = 0;
int direction = 1;
int minAngle = 0;
int maxAngle = 180;
int x = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);

  // I2C messaging
  Wire.begin(EXTENSION_I2C_ID);
  //Wire.setWireTimeout();
  Wire.onReceive(receiveServoAngle);
  Wire.onRequest(sendCurrentAngle);

  // Servo
  servo.attach(SERVO_PORT_ID);
}

void log() {
  Serial.print("MinAngle: ");
  Serial.print(minAngle);
  Serial.print(", MaxAngle: ");
  Serial.print(maxAngle);

  Serial.print(", Angle: ");
  Serial.print(angle);
  Serial.println();
}

// Receive I2C commands
void receiveServoAngle(int howMany) {
  int openAngle = Wire.read();

  if (openAngle == 0) {
    speed = SERVO_SPEED;
    minAngle = maxAngle = 90;
  } else {
    minAngle = 90 - openAngle / 2;
    maxAngle = 90 + openAngle / 2;

    if (openAngle > 65) {
      speed = SERVO_SPEED * SERVO_SPEED_MULT;
    } else {
      speed = SERVO_SPEED;
    }
  }
}

void sendCurrentAngle() {
  char buffer[10];
  sprintf(buffer, "%d", (int)angle * direction);
  Wire.write(buffer);
}

void loop() {
  //Servo motor
  angle += speed * direction;

  if (angle >= maxAngle) {
    angle = maxAngle;
    direction = -1;
  } else if (angle <= minAngle) {
    angle = minAngle;
    direction = 1;
  }
  servo.write(angle);

  log();
}
