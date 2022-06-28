#include "MPU9250.h"
#include <SoftWire.h>


MPU9250_<SoftWire> mpu1, mpu2;

#define SDA_1 (21)
#define SCL_1 (22)
#define SDA_2 (36)
#define SCL_2 (39)
#define SDA_3 (34)
#define SCL_3 (35)
char swTxBuffer1[64];
char swRxBuffer1[64];
char swTxBuffer2[64];
char swRxBuffer2[64];

SoftWire sw1(SDA_1, SCL_1);
SoftWire sw2(SDA_2, SCL_2);

void setup_mpu(MPU9250_<SoftWire> mpu, int i, SoftWire w) {
  if (!(mpu.setup(0x68, MPU9250Setting(), w))) {  // change to your own address
        Serial.print("MPU ");
        Serial.print(i, DEC);
        Serial.print(" connection failed. Please check your connection\r\n");
        delay(5000);
    }
}

void setup() {
    Serial.begin(115200);
    sw1.setTxBuffer(swTxBuffer1, sizeof(swTxBuffer1));
    sw1.setRxBuffer(swRxBuffer1, sizeof(swRxBuffer1));
    sw2.setTxBuffer(swTxBuffer2, sizeof(swTxBuffer2));
    sw2.setRxBuffer(swRxBuffer2, sizeof(swRxBuffer2));
    sw1.setDelay_us(5);
    sw2.setDelay_us(5);
    sw1.begin();
    sw2.begin();
    delay(2000);

    setup_mpu(mpu1, 1, sw1);
    setup_mpu(mpu2, 2, sw2);

    // calibrate anytime you want to
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu1.verbose(true);
    mpu2.verbose(true);
    delay(5000);
    mpu1.calibrateAccelGyro();
    mpu2.calibrateAccelGyro();

//    Serial.println("Mag calibration will start in 5sec.");
//    Serial.println("Please Wave device in a figure eight until done.");
//    delay(5000);
//    mpu.calibrateMag();

    print_calibration(&mpu1);
    print_calibration(&mpu2);
    mpu1.selectFilter(QuatFilterSel::MAHONY);
    mpu1.setFilterIterations(15);
    mpu1.verbose(false);
    mpu2.selectFilter(QuatFilterSel::MAHONY);
    mpu2.setFilterIterations(15);
    mpu2.verbose(false);
}

void mpu_update(MPU9250* mpu) {
    if (mpu->update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            print_roll_pitch_yaw(mpu);
            prev_ms = millis();
        }
    }  
}

void loop() {
  mpu_update(&mpu1);
  mpu_update(&mpu2);
}

void print_roll_pitch_yaw(MPU9250 *mpu) {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu->getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu->getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu->getRoll(), 2);
}

void print_calibration(MPU9250* mpu) {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu->getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu->getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu->getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu->getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu->getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu->getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu->getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu->getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu->getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu->getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu->getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu->getMagScaleZ());
    Serial.println();
}
