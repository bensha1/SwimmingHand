#include<Wire.h>
 
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ, AcX1, AcY1, AcZ1, Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
 
double x;
double y;
double z;
double x1;
double _y1;
double z1;


#define SDA_1 (21)
#define SCL_1 (22)
#define SDA_2 (26)
#define SCL_2 (27)

void setup(){
Wire.begin();
Wire1.begin(SDA_2, SCL_2);
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
Wire1.beginTransmission(MPU_addr);
Wire1.write(0x6B);
Wire1.write(0);
Wire1.endTransmission(true);
Serial.begin(9600);
}
void loop(){
Wire.beginTransmission(MPU_addr);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true);
AcX=Wire.read()<<8|Wire.read();
AcY=Wire.read()<<8|Wire.read();
AcZ=Wire.read()<<8|Wire.read();
int xAng = map(AcX,minVal,maxVal,-90,90);
int yAng = map(AcY,minVal,maxVal,-90,90);
int zAng = map(AcZ,minVal,maxVal,-90,90);

Wire1.beginTransmission(MPU_addr);
Wire1.write(0x3B);
Wire1.endTransmission(false);
Wire1.requestFrom(MPU_addr,14,true);
AcX1=Wire1.read()<<8|Wire1.read();
AcY1=Wire1.read()<<8|Wire1.read();
AcZ1=Wire1.read()<<8|Wire1.read();
int xAng1 = map(AcX1,minVal,maxVal,-90,90);
int yAng1 = map(AcY1,minVal,maxVal,-90,90);
int zAng1 = map(AcZ1,minVal,maxVal,-90,90);

x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

x1= RAD_TO_DEG * (atan2(-yAng1, -zAng1)+PI);
_y1= RAD_TO_DEG * (atan2(-xAng1, -zAng1)+PI);
z1= RAD_TO_DEG * (atan2(-yAng1, -xAng1)+PI);
Serial.print("AngleX= ");
Serial.println(x);
 
Serial.print("AngleY= ");
Serial.println(y);
 
Serial.print("AngleZ= ");
Serial.println(z);

Serial.print("AngleX1= ");
Serial.println(x1);
 
Serial.print("AngleY1= ");
Serial.println(_y1);
 
Serial.print("AngleZ1= ");
Serial.println(z1);
Serial.println("-----------------------------------------");
delay(2000);
}
