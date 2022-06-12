/* SwimmingHand: changed PID for loop in MPU6050.cpp to 10 */

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps612.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


TwoWire mpuwire1(0);
// TwoWire mpuwire2(1);
// MPU6050 accelgyro3(0x69, &mpuwire1);
MPU6050 accelgyro2(0x68, &mpuwire1);
// MPU6050 accelgyro1(0x68, &mpuwire2);


// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO


// MPU control/status vars
uint16_t packetSize1, packetSize2, packetSize3;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer1[64]; // FIFO storage buffer
uint8_t fifoBuffer2[64]; // FIFO storage buffer
uint8_t fifoBuffer3[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float pred_ypr[3];

int servo_angles[] = {30, 0, 0, 60};



void setup() {
  Serial.begin(115200);
  mpuwire1.begin(21, 22);


    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro2.initialize();

    packetSize2 = init_dmp(accelgyro2, 2, -2005, -1937, 2944, 23, -46, 42);
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

void print_ypr(int i, float ypr[3]) {
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

bool has_time_elapsed(unsigned int original, unsigned int target) {
  unsigned int current = millis();
  return ((current - original) >= target);
}

void get_statistics(float min_ypr[3], float max_ypr[3], float avg_ypr[3], int* samples) {
  main_loop(accelgyro2, 2, packetSize2, fifoBuffer2);
  (*samples)++;
  for (int i = 0; i < 3; i++) ypr[i] = ypr[i] * 180 / M_PI; // rad to degree

  if ((*samples) == 1) {
    for (int i = 0; i < 3; i++) min_ypr[i] = ypr[i];
    for (int i = 0; i < 3; i++) max_ypr[i] = ypr[i];
    for (int i = 0; i < 3; i++) avg_ypr[i] = ypr[i];
    return;
  }
  for (int i = 0; i < 3; i++) {
    if (min_ypr[i] > ypr[i]) min_ypr[i] = ypr[i];
  }
  for (int i = 0; i < 3; i++) {
    if (max_ypr[i] < ypr[i]) max_ypr[i] = ypr[i];
  }
  for (int i = 0; i < 3; i++) avg_ypr[i] = avg_ypr[i] + (ypr[i] - avg_ypr[i]) / (*samples);
}

void run_sampling(int seconds, int src, int dest = 0, bool dynamic = false) {
  float min_ypr[3] = {0}, max_ypr[3] = {0}, avg_ypr[3] = {0};
  int samples = 0;
  Serial.print("Sampling for "); Serial.print(seconds); Serial.println(" seconds");
  unsigned int start_time = millis();
  while (!has_time_elapsed(start_time, seconds * 1000)) {
    get_statistics(min_ypr, max_ypr, avg_ypr, &samples);
  }

  if (dynamic) {
    Serial.print("Dynamic between "); Serial.print(src); Serial.print(" and "); Serial.print(dest); Serial.print(" results: ");
  } else {
    Serial.print("Static position "); Serial.print(src); Serial.print(" results: ");
  }

  Serial.print("min: ["); Serial.print(min_ypr[0]); Serial.print(", "); Serial.print(min_ypr[1]); Serial.print(", "); Serial.print(min_ypr[2]); Serial.print("]\t");
  Serial.print("max: ["); Serial.print(max_ypr[0]); Serial.print(", "); Serial.print(max_ypr[1]); Serial.print(", "); Serial.print(max_ypr[2]); Serial.print("]\t");
  Serial.print("avg: ["); Serial.print(avg_ypr[0]); Serial.print(", "); Serial.print(avg_ypr[1]); Serial.print(", "); Serial.print(avg_ypr[2]); Serial.println("]");
  Serial.print("Number of samples: "); Serial.println(samples);

}

void benchmark_position(int src, int dest = 0, bool dynamic = false) {
  if (dynamic) {
    Serial.print("Please move device between "); Serial.print(src); Serial.print(" and "); Serial.println(dest);
  } else {
    Serial.print("Please set device to position "); Serial.println(src);
  }

  delay(5000);
  Serial.println("benchmark will resume in 5 seconds");
  delay(5000);
  run_sampling(10, src, dest, dynamic); // sample for 10 seconds in position

  if (dynamic) return;

  for (int i = 0; i < 2; i++) {
    Serial.print("Please move device around then return to position "); Serial.println(src);
    delay(5000);
    Serial.println("benchmark will resume in 5 seconds");
    delay(5000);
    run_sampling(10, src);
  }
}

void static_benchmark() {
  Serial.println("Starting static benchmark");
  benchmark_position(0);
  benchmark_position(90);
  benchmark_position(180);
}

void dynamic_benchmark() {
  Serial.println("Starting dynamic benchmark");
  benchmark_position(0, 90, true);
  benchmark_position(90, 180, true);
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
//   print_ypr(i, ypr);
}


void loop() {
  static_benchmark();
  dynamic_benchmark();
  Serial.println("------ done ------");
  while(1) delay(1000);
}
