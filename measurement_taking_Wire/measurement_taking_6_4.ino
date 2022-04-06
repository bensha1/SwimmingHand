#include "MPU9250.h"

//MPU9250Setting setting();

MPU9250 mpu1, mpu2, mpu3;
#define SDA_2 (16)
#define SCL_2 (17)
#define SDA_3 (25)
#define SCL_3 (26)

TwoWire MPU_Wire1 = TwoWire(1);
//TwoWire MPU_Wire2 = TwoWire(1);
TwoWire MPU_Wire3 = TwoWire(0);

void setup_mpu(MPU9250& mpu, int i, TwoWire& w, uint8_t addr) {
  if (!(mpu.setup(addr, MPU9250Setting(), w))) {  // change to your own address
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
    MPU_Wire1.begin(21, 22);
//    MPU_Wire2.begin(SDA_2, SCL_2);
    MPU_Wire3.begin(SDA_3, SCL_3);
//    Wire1.begin(SDA_2, SCL_2);
//    Wire2.begin(SDA_3, SCL_3);
    delay(2000);
    
    
    mpu1.verbose(true);
    
    mpu2.verbose(true);
    
    mpu3.verbose(true);

    Serial.println("Sensor 1 Setup:");
    setup_mpu(mpu1, 1, MPU_Wire1, 0x69);
    Serial.println("Sensor 2 Setup:");
    setup_mpu(mpu2, 2, MPU_Wire1, 0x68);
    Serial.println("Sensor 3 Setup:");
    setup_mpu(mpu3, 3, MPU_Wire3, 0x68);
//    setup_mpu(&mpu3, 3, &Wire2);

    Serial.println("Sensor 1 Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
//    mpu2.verbose(true);
//    mpu3.verbose(true);
    delay(5000);
//    mpu1.calibrateAccelGyro();
    mpu1.calibrateAccelGyro();
    
    // calibrate anytime you want to
    Serial.println("Sensor 2 Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
//    mpu2.verbose(true);
//    mpu3.verbose(true);
    delay(5000);
//    mpu1.calibrateAccelGyro();
    mpu2.calibrateAccelGyro();
//    mpu2.calibrateAccelGyro();
//    mpu3.calibrateAccelGyro();

    // calibrate anytime you want to
    Serial.println("Sensor 3 Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
//    mpu2.verbose(true);
//    mpu3.verbose(true);
    delay(5000);
//    mpu1.calibrateAccelGyro();
    mpu3.calibrateAccelGyro();
//    mpu2.calibrateAccelGyro();
//    mpu3.calibrateAccelGyro();

    Serial.println("Sensor 1  Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    mpu1.calibrateMag();
    print_calibration(mpu1);

    mpu1.selectFilter(QuatFilterSel::MAHONY);
    mpu1.setFilterIterations(15);
    mpu1.verbose(false);

    Serial.println("Sensor 2  Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    mpu2.calibrateMag();
    print_calibration(mpu2);

    mpu2.selectFilter(QuatFilterSel::MAHONY);
    mpu2.setFilterIterations(15);
    mpu2.verbose(false);
    
    Serial.println("Sensor 3  Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    mpu3.calibrateMag();

//    print_calibration(mpu1);
    print_calibration(mpu3);
//    print_calibration(&mpu3);
//    mpu1.selectFilter(QuatFilterSel::MAHONY);
//    mpu1.setFilterIterations(15);
//    mpu1.verbose(false);
    mpu3.selectFilter(QuatFilterSel::MAHONY);
    mpu3.setFilterIterations(15);
    mpu3.verbose(false);
//    mpu3.selectFilter(QuatFilterSel::MAHONY);
//    mpu3.setFilterIterations(15);
//    mpu3.verbose(false);
}

void mpu_update(int i,MPU9250& mpu) {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 300) {
            Serial.print("Sensor ");
            Serial.print(i, DEC);
            Serial.print(" : ");
            print_roll_pitch_yaw(mpu);
            prev_ms = millis();
        }
    }  
}

void loop() {
  mpu_update(1, mpu1);
  mpu_update(2, mpu2);
  mpu_update(3, mpu3);
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
