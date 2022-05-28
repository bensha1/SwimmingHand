#ifndef SH_SERVO_H
#define SH_SERVO_H

#include <ESP32Servo.h>
#define Servo_PWM 5

Servo MG995_Servo;
void init_servo();
void servo_write(int angle);

void init_servo() {
  MG995_Servo.attach(Servo_PWM);
}

void servo_write(int angle) {
  MG995_Servo.write(angle);
}

#endif //SH_SERVO_H
