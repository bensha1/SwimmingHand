// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

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

/* SwimmingHand: changed PID for loop in MPU6050.cpp to 10 */

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "SH_servo.h"
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "hand_model.h"
#include "EloquentTinyML.h"
#include <eloquent_tinyml/tensorflow.h>

#define N_INPUTS 6
#define N_OUTPUTS 4
#define TENSOR_ARENA_SIZE 32*1024


Eloquent::TinyML::TensorFlow::TensorFlow<N_INPUTS, N_OUTPUTS, TENSOR_ARENA_SIZE> tf;
int predicted = 0;

#include "Wire.h"

TwoWire mpuwire1(0);
TwoWire mpuwire2(1);
MPU6050 accelgyro3(0x69, &mpuwire1);
MPU6050 accelgyro2(0x68, &mpuwire1);
MPU6050 accelgyro1(0x68, &mpuwire2);

// MPU control/status vars
uint8_t devStatus1, devStatus2, devStatus3;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize1, packetSize2, packetSize3;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer1[64]; // FIFO storage buffer
uint8_t fifoBuffer2[64]; // FIFO storage buffer
uint8_t fifoBuffer3[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float pred_ypr[6];

float ranges[13][3] = {{-180.0, -165.0, -180.0},
                {-164.999, -135.0, -150.0},
                {-134.999, -105.0, -120.0},
                {-104.999, -75.0, -90.0},
                {-74.999, -45.0, -60.0},
                {-44.999, -15.0, -30.0},
                {-14.999, 15.0, 0.0},
                {14.999, 45.0, 30.0},
                {44.999, 75.0, 60.0},
                {74.999, 105.0, 90.0},
                {104.999, 135.0, 120.0},
                {134.999, 165.0, 150.0},
                {164.999, 180.0, 180.0}};
// 0, 60, 120, 180, 240, 300
//int servo_angles[] = {30, 0, 0, 30, 0, 60};/
int servo_angles[] = {60, 0, 30, 0};
int valid_transitions[6][2] = {{0, 1},
                              {1, 2},
                              {2, 3},
                              {3, 4},
                              {4, 5},
                              {5, 0}};

void setup() {
  tf.begin(hand_model);
    Serial.begin(115200);
    init_servo();
    mpuwire1.begin(21, 22);
    mpuwire2.begin(25, 26);


    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro1.initialize();
    accelgyro2.initialize();

//    packetSize1 = init_dmp(accelgyro1, 1, -4733, -6623, 12661, 136, -174, -12);
    packetSize1 = init_dmp(accelgyro1, 1, -1647, -199, 3001, 60, -37, 50);
    packetSize2 = init_dmp(accelgyro2, 2, -551, -765, 3677, 140, 52, 32);
//    packetSize3 = init_dmp(accelgyro3, 3, -1773, 1793, 4855, 62, -37, 53);/
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

float fix_ypr(float f) {
  for (int i = 0; i < 13; i++) {
    if (ranges[i][0] <= f && f <= ranges[i][1]) {
//      Serial.print("changed ");
//      Serial.print(f);
//      Serial.print(" to: ");
//      Serial.println(ranges[i][2]);
      return ranges[i][2];
    }
  }
  return f;
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

bool is_valid_transition(int current, int next) {
  for (int i = 0; i < 6; i++) {
    if (valid_transitions[i][0] == current && valid_transitions[i][1] == next) {
      return true;
    }
  }
  return false;
}

unsigned int prev_ms = millis();
bool allow_timed_transition(int old_val, int new_val) {
  if (old_val == new_val) {
    return true;
  }
  
  unsigned int new_ms = millis();
  Serial.print("\nold = ");
  Serial.print(prev_ms);
  Serial.print(" new = ");
  Serial.println(new_ms);
  bool rc = false;
  if (new_ms > prev_ms + 400) {
      rc = true;
      prev_ms = new_ms;
  }
  return rc;
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
//        Serial.print(prediction_arr[4]);
//        Serial.print(", ");
//        Serial.print(prediction_arr[5]);
    Serial.print("} ");
}

void main_loop(MPU6050& mpu, int i, int packetSize, uint8_t *fifoBuffer) {
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
    print_ypr(mpu, i);

      pred_ypr[(i-1)*3 + 0] = (ypr[0] * 180 / M_PI);
      pred_ypr[(i-1)*3 + 1] = (ypr[1] * 180 / M_PI);
      pred_ypr[(i-1)*3 + 2] = (ypr[2] * 180 / M_PI);

      /*pred_ypr[(i-1)*2 + 0] = (ypr[1] * 180 / M_PI);*/
      /*pred_ypr[(i-1)*2 + 1] = (ypr[2] * 180 / M_PI);*/
      
      if (i == 2) {
        float prediction_arr[4];
        tf.predict(pred_ypr, prediction_arr);
        print_pred(prediction_arr);

        int category = 0;
        int prediction = get_prediction(prediction_arr, &category);
        Serial.print(" current: ");
        Serial.println(predicted);
//        if (is_valid_transition(predicted, category)) {/
         /*if (allow_timed_transition(predicted, category)) {*/
            /*servo_write(prediction);*/
            /*predicted = category;*/
         /*}*/
        
        /*}/*/
        if (prediction != predicted) {
          servo_write(prediction);
          predicted = prediction;
        } 
      }
}


void loop() {
    main_loop(accelgyro1, 1, packetSize1, fifoBuffer1);
    main_loop(accelgyro2, 2, packetSize2, fifoBuffer2);
//    main_loop(accelgyro3, 3, packetSize3, fifoBuffer3);
  delay(80);
}
