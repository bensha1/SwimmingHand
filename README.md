# <img src="https://user-images.githubusercontent.com/75077005/180175601-372a96b2-a3f9-471a-9e38-975cca952ac1.png" data-canonical-src="https://user-images.githubusercontent.com/75077005/180175601-372a96b2-a3f9-471a-9e38-975cca952ac1.png" width="120" height="120" />  SwimmingHand

**Helping amputees swim naturally**


A dynamic prosthetic arm that enables above-elbow amputees to mimic a natural swimming stroke.  
Current prosthetic arms remain static regardless of a swimmer's position in the water making them cumbersome to use.  
Swimming Hand uses a neural network running on the prosthetic to predict the desired hand position and moves the hand accordingly creating a natural stroke.  

## Contents

- swimming_hand - Final code of the hand, ready to be uploaded to the ESP.
- mpu_calibrations - Results of calibrations of the MPU sensors, and code that we used to compare the accuracy of different MPU's.
- data sicence - Parser for the serial output that is used when gathering training data, and the tensorflow model.
- old folders - Things we tried while working, like different MPU libraries.

## Libraries

- I2Cdev
  - To install: clone from git repository https://github.com/jrowberg/i2cdevlib (this library is not in Arudino's library manager)  
  - Copy `Arduino/I2Cdev` and `Arduino/MPU6050` directories into your local libraries directory.  
  - Our project used the most up to date version as of June 2022.
- ESP32Servo v0.11.0  

## Wiki

Check out our wiki!

[Wiki](https://github.com/bensha1/SwimmingHand/wiki)
