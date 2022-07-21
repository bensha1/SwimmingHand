# <img src="https://user-images.githubusercontent.com/75077005/180175601-372a96b2-a3f9-471a-9e38-975cca952ac1.png" data-canonical-src="https://user-images.githubusercontent.com/75077005/180175601-372a96b2-a3f9-471a-9e38-975cca952ac1.png" width="120" height="120" />  SwimmingHand

**Helping amputees swim naturally**


A dynamic prosthetic arm that enables above-elbow amputees to mimic a natural swimming stroke. 
Current prosthetic arms remain static regardless of a swimmer's position in the water making them cumbersome to use. 
Swimming Hand uses a neural network running on the prosthetic to predict the desired hand position and moves the hand accordingly creating a natural stroke.

## contents

- mpu_calibrations - calibrations code for the mpu and the output values file
- data sicence - parser for the serial output and the tensorflow model
- swimming_hand - code of the hand to upload to ESP
- old folders - things we tried while working, like different MPU sensors

## libraries

- I2Cdev
- MPU6050
- ESP32Servo
