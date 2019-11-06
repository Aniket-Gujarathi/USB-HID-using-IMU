#include <HardWire.h>
#include <Wire.h>
#include <WireBase.h>


#include <Wire.h>
int Gyro_X, Gyro_Y, Gyro_Z;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    //Wire.setClock(400000);
    Wire.begin();
    delay(250);

    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
}

void loop() {
  // put your main code here, to run repeatedly:
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    Gyro_X = Wire.read() << 8 | Wire.read();
    Gyro_Y = Wire.read() << 8 | Wire.read();
    Gyro_Z = Wire.read() << 8 | Wire.read();
    Serial.print  ("X = ");
    Serial.println(Gyro_X);
    Serial.print("Y = ");
    Serial.println(Gyro_Y);
    Serial.print("Z = ");
    Serial.println(Gyro_Z);
    delay(250);
}
