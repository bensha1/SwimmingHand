#include "MPU9250.h"
MPU9250 mpu1, mpu2;

//MPU9250Setting setting();

//MPU9250 mpu1, mpu2, mpu3;
#define SDA_2 (16)
#define SCL_2 (17)
#define SDA_3 (34)
#define SCL_3 (35)

//TwoWire MPU_Wire1 = TwoWire(1);
TwoWire MPU_Wire2 = TwoWire(1);

void setup_mpu(MPU9250& mpu, int i, TwoWire& w) {
  if (!(mpu.setup(0x68, MPU9250Setting(), w))) {  // change to your own address
//   if (!(mpu.setup(0x68, setting, w))) {
        while (1) {
            Serial.print("MPU ");
            Serial.print(i, DEC);
            Serial.print(" connection failed. Please check your connection");
            Serial.println();
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(115200);
//    Wire.begin();
//    MPU_Wire1.begin(21, 22);
    MPU_Wire2.begin(SDA_2, SCL_2);
//    MPU_Wire2.begin(SDA_2, SCL_2);
//    Wire1.begin(SDA_2, SCL_2);
//    Wire2.begin(SDA_3, SCL_3);
    delay(2000);
    
//    mpu1.verbose(true);
    mpu2.verbose(true);
//    setup_mpu(mpu1, 1, MPU_Wire1);
    setup_mpu(mpu2, 2, MPU_Wire2);
//    setup_mpu(mpu2, 2, Wire1);
//    setup_mpu(&mpu3, 3, &Wire2);

    // calibrate anytime you want to
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
//    mpu2.verbose(true);
//    mpu3.verbose(true);
    delay(5000);
//    mpu1.calibrateAccelGyro();
    mpu2.calibrateAccelGyro();
//    mpu2.calibrateAccelGyro();
//    mpu3.calibrateAccelGyro();

    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    mpu2.calibrateMag();

//    print_calibration(mpu1);
    print_calibration(mpu2);
//    print_calibration(&mpu3);
//    mpu1.selectFilter(QuatFilterSel::MAHONY);
//    mpu1.setFilterIterations(15);
//    mpu1.verbose(false);
    mpu2.selectFilter(QuatFilterSel::MAHONY);
    mpu2.setFilterIterations(15);
    mpu2.verbose(false);
//    mpu3.selectFilter(QuatFilterSel::MAHONY);
//    mpu3.setFilterIterations(15);
//    mpu3.verbose(false);
}

void mpu_update(MPU9250& mpu) {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            print_roll_pitch_yaw(mpu);
            prev_ms = millis();
        }
    }  
}

void loop() {
//  mpu_update(mpu1);
  mpu_update(mpu2);
//  mpu_update(&mpu3);
}

void print_roll_pitch_yaw(MPU9250& mpu) {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
}

void print_calibration(MPU9250& mpu) {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}
