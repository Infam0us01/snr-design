#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include "math.h"
LSM6DSO myIMU; //Default constructor is I2C, addr 0x6B
float x, y, z;                        //three axis acceleration data
float roll, pitch;       //Roll & Pitch are the angles which rotate by the axis X and y

void RP_calculate(){
  roll = atan2(y , z) * 57.3;
  pitch = atan2((- x) , sqrt(y * y + z * z)) * 57.3;
  //Serial.println(roll);
  //Serial.println(pitch);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  delay(10);

  if( myIMU.begin(0x6A,Wire) )
    Serial.println("Ready.");
  else { 
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  if( myIMU.initialize(BASIC_SETTINGS) )
    Serial.println("Loaded Settings.");
}

void loop() {
  // put your main code here, to run repeatedly:
  x = myIMU.readFloatAccelX();
  y = myIMU.readFloatAccelY();
  z = myIMU.readFloatAccelZ();
  //Serial.println(x,10);

  RP_calculate();
  Serial.println(pitch,10);
  //Serial.println(roll,10);

  delay(100);
}
