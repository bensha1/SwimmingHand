/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "hand_model.h"
#include "EloquentTinyML.h"
#include <eloquent_tinyml/tensorflow.h>
#include "state_history.h"

#include <ESP32Servo.h>
#define Servo_PWM 5

#define N_INPUTS 6
#define N_OUTPUTS 4
#define TENSOR_ARENA_SIZE 32*1024


Eloquent::TinyML::TensorFlow::TensorFlow<N_INPUTS, N_OUTPUTS, TENSOR_ARENA_SIZE> tf;
int predicted = 0;

#include "Wire.h"

TwoWire mpuwire1(0);
TwoWire mpuwire2(1);
MPU6050 accelgyro2(0x68, &mpuwire1);
MPU6050 accelgyro1(0x68, &mpuwire2);

// MPU control/status vars
uint8_t devStatus1, devStatus2;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize1, packetSize2;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer1[64]; // FIFO storage buffer
uint8_t fifoBuffer2[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float pred_ypr[6];
Servo servo;
StateHistory history(8);

int servo_angles[] = {60, 0, 30, 0};

void setup() {
  tf.begin(hand_model);
    Serial.begin(230400);
    servo.attach(Servo_PWM);
    mpuwire1.begin(21, 22);
    mpuwire2.begin(25, 26);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro1.initialize();
    accelgyro2.initialize();

    packetSize1 = init_dmp(accelgyro1, 1, -1647, -199, 3001, 60, -37, 50);
    packetSize2 = init_dmp(accelgyro2, 2, -551, -765, 3677, 140, 52, 32);
}

int init_dmp(MPU6050& mpu, int i, int xacc, int yacc, int zacc, int xgyro, int ygyro, int zgyro) {
  Serial.print("dmp ");
  Serial.println(i);
  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer again
  int devStatus = mpu.dmpInitialize();
  
  mpu.setXAccelOffset(xacc);
  mpu.setYAccelOffset(yacc);
  mpu.setZAccelOffset(zacc);
  mpu.setXGyroOffset(xgyro);
  mpu.setYGyroOffset(ygyro);
  mpu.setZGyroOffset(zgyro);
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.print(F("Enabling DMP "));
    Serial.print(i);
    Serial.println(" ...");
    mpu.setDMPEnabled(true);
    
    // get expected DMP packet size for later comparison
    return mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    return -1;
  }
}

int get_prediction(float* predict_arr, int* max_i) {
  float max_val = -200.0;
  *max_i = -1;
  for (int i = 0; i < 4; i++) {
    if (predict_arr[i] > max_val) {
      max_val = predict_arr[i];
      *max_i = i;
    }
  }
  Serial.print("out: ");
  Serial.print(*max_i);
  Serial.print(" servo angle: ");
  Serial.print(servo_angles[*max_i]);
  return servo_angles[*max_i];
}

void print_ypr(MPU6050& mpu, int i) {
    Serial.print("mpu ");
    Serial.print(i);
    Serial.print(" ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI);
    Serial.print("\t");
    Serial.println();
}

void print_pred(float* prediction_arr) {
    Serial.print("perdict: {");
    Serial.print(prediction_arr[0]);
    Serial.print(", ");
    Serial.print(prediction_arr[1]);
    Serial.print(", ");
    Serial.print(prediction_arr[2]);
    Serial.print(", ");
    Serial.print(prediction_arr[3]);
    Serial.print(", ");
    Serial.print("} ");
}


void main_loop(MPU6050& mpu, int i, int packetSize, uint8_t *fifoBuffer) {
  static bool first = true;

  mpu.resetFIFO();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  // wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize){
    fifoCount = mpu.getFIFOCount();
  }

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      pred_ypr[(i-1)*3 + 0] = (ypr[0] * 180 / M_PI);
      pred_ypr[(i-1)*3 + 1] = (ypr[1] * 180 / M_PI);
      pred_ypr[(i-1)*3 + 2] = (ypr[2] * 180 / M_PI);

      if (i == 2) {
        float prediction_arr[4];
        tf.predict(pred_ypr, prediction_arr);
        print_pred(prediction_arr);

        int state;
        int prediction = get_prediction(prediction_arr, &state);
        Serial.print(" prediction: "); Serial.print(state); Serial.print(" current correction: "); Serial.println(history.current_correction);
        history.update_state(state);
        int prev;
        if (history.has_state_changed(state, &prev)) {
            Serial.print("changing state from "); Serial.print(prev); Serial.print(" to "); Serial.println(state);
            history.change_state(state);
            servo.write(servo_angles[history.current_correction]);
        }

        if (first) {
            Serial.println("Setting servo to initial position");
            first = false;
            servo.write(servo_angles[history.current_correction]);
      }
    }
}


void loop() {
    main_loop(accelgyro1, 1, packetSize1, fifoBuffer1);
    main_loop(accelgyro2, 2, packetSize2, fifoBuffer2);
  delay(80);
}
