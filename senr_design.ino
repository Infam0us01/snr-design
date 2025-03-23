
//Reference
const int analogIn = A0; // potentiometer 1
const int analogIn2 = A1 // Potentiometer 2
float ref = 1;

//Sensor Input
#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include "PID_v1.h"
//#include "SPI.h"

LSM6DSO myIMU; //Default constructor is I2C, addr 0x6B

//PID
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // LSM6DSO
  Serial.begin(115200);
  delay(500); 
  
  Wire.begin();
  delay(10);
  if( myIMU.begin() )
    Serial.println("Ready.");
  else { 
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  if( myIMU.initialize(BASIC_SETTINGS) )
    Serial.println("Loaded Settings.");

  //PID
  //initialize the variables we're linked to
  Input = analogRead(PIN_INPUT);
  Setpoint = 100;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);


}

void loop() {
 
  A_r1 = (analogRead(analogIn))/1024;
  A_r2 = analogRead(analogIn2)/2048;
  ref_pos = A_r1 * sin(2*pi()*A_r2*millis()); //ref sine wave

  //LSM6DSO
  //Get all parameters
  float xAccel1 = myIMU.readFloatAccelX() - 1.994; //these were the numbers that were showing up when I was attached to the wrong communication ports
  float yAccel1 = myIMU.readFloatAccelY() - 1.994;
  float zAccel1 = myIMU.readFloatAccelZ() - 1.994;

  float xGyro1 = myIMU.readFloatGyroX() - 286.073;
  float yGyro1 = myIMU.readFloatGyroY()- 286.073;
  float zGyro1 = myIMU.readFloatGyroZ()- 286.073;

  delay(1000);

  float xAccel2 = myIMU.readFloatAccelX()- 1.994; 
  float yAccel2 = myIMU.readFloatAccelY()- 1.994;
  float zAccel2 = myIMU.readFloatAccelZ()- 1.994;

  float xGyro2 = myIMU.readFloatGyroX()- 286.073;
  float yGyro2 = myIMU.readFloatGyroY()- 286.073;
  float zGyro2 = myIMU.readFloatGyroZ()- 286.073;

  // change in values
  float changeXAccel = xAccel2 - xAccel1;
  float changeYAccel = yAccel2 - yAccel1;
  float changeZAccel = zAccel2 - zAccel1;

  float changeXGyro = xGyro2 - xGyro1;
  float changeYGyro = yGyro2 - yGyro1;
  float changeZGyro = zGyro2 - zGyro1;

  //change in angular position
  float changeXAngle = changeXGyro * 1000;

  myPID.Compute();
  float changeYAngle = changeYGyro * 1000;
  float changeZAngle = changeZGyro * 1000;

  //print values

  Serial.print("\nAccelerometer:\n");
  Serial.print(" X = ");
  Serial.println(myIMU.readFloatAccelX(), 3);
  Serial.print(" Y = ");
  Serial.println(myIMU.readFloatAccelY(), 3);
  Serial.print(" Z = ");
  Serial.println(myIMU.readFloatAccelZ(), 3);

  Serial.print("\nGyroscope:\n");
  Serial.print(" X = ");
  Serial.println(myIMU.readFloatGyroX(), 3);
  Serial.print(" Y = ");
  Serial.println(myIMU.readFloatGyroY(), 3);
  Serial.print(" Z = ");
  Serial.println(myIMU.readFloatGyroZ(), 3);

  Serial.print("\nThermometer:\n");
  Serial.print(" Degrees F = ");
  Serial.println(myIMU.readTempF(), 3);
  
  delay(1000);
  



}
