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
//        Serial.print(prediction_arr[4]);
//        Serial.print(", ");
//        Serial.print(prediction_arr[5]);
    Serial.print("} ");
}

unsigned int cycle_time = 0;

bool is_new_cycle() {
    return (history.compare_lower_half(3) && history.compare_higher_half(0));
}

void update_cycle_time() {
    static unsigned int cycle_start_time = millis();
    static bool first_time = true;
    
    if (first_time) {
        first_time = false;
        return;
    }

    unsigned int current_time = millis();
    cycle_time = current_time - cycle_start_time;
    cycle_start_time = current_time;
    
}

#define STATES_NUMBER 4
float timings[STATES_NUMBER][2] = {{60, 0.25}, {0, 0.25}, {30, 0.25}, {0, 0.25}};
void simulate_motion(bool start_new_cycle) {
    static int i;
    static unsigned int state_start_time = 0;

    unsigned int current_time = millis();
    if (start_new_cycle) {
        i = 0;
        state_start_time = current_time;
        if (cycle_time != 0) {
            servo.write(timings[i][0]);
        }
        return;
    }
    
    if (cycle_time == 0) { // never finished a cycle
        return;
    }
    
    // is previous state done?
    unsigned int target_time = state_start_time + (cycle_time * timings[i][1]);
    if (current_time >= target_time) {
        if (i == STATES_NUMBER - 1) {
            // cycle is done but we didn't detect a new cycle yet, so idle on 0 here
            servo.write(0);
            return;
        }

        Serial.print("moving to state "); Serial.println(i);
        i++;
        state_start_time = current_time;
        servo.write(timings[i][0]);
    } else {
        Serial.print("in state "); Serial.println(i);
        return;
    }
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
    /*print_ypr(mpu, i);*/

      pred_ypr[(i-1)*3 + 0] = (ypr[0] * 180 / M_PI);
      pred_ypr[(i-1)*3 + 1] = (ypr[1] * 180 / M_PI);
      pred_ypr[(i-1)*3 + 2] = (ypr[2] * 180 / M_PI);

      /*pred_ypr[(i-1)*2 + 0] = (ypr[1] * 180 / M_PI);*/
      /*pred_ypr[(i-1)*2 + 1] = (ypr[2] * 180 / M_PI);*/
      
      if (i == 2) {
        float prediction_arr[4];
        tf.predict(pred_ypr, prediction_arr);
        print_pred(prediction_arr);

        int state;
        int prediction = get_prediction(prediction_arr, &state);
        if (state == 2) {
            Serial.println(" skipping state 2");
            return;
        }
        history.update_state(state);
        Serial.print(" current: ");
        Serial.println(predicted);
        
        if (is_new_cycle()) {
            Serial.println("Start of new cycle detected");
            update_cycle_time();
            simulate_motion(true);
        } else {
            simulate_motion(false);
        }
      }
}


void loop() {
    main_loop(accelgyro1, 1, packetSize1, fifoBuffer1);
    main_loop(accelgyro2, 2, packetSize2, fifoBuffer2);
//    main_loop(accelgyro3, 3, packetSize3, fifoBuffer3);
  delay(80);
}
