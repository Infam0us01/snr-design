
//Reference
const int analogIn = A0; // potentiometer 1
const int analogIn2 = A1; // Potentiometer 2
float ref_pos = 1;
double A_r1 =1;
double A_r2 = 1;

//Sensor Input
#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include "math.h"
//#include "SPI.h"

LSM6DSO myIMU; //Default constructor is I2C, addr 0x6B

//PID
double setpoint, input;


// double count = 0; //set the counts of the encoder
// double angle = 0;//set the angles
// boolean A,B;
// byte state, statep;

double pwm = 9; // this is the PWM pin for the motor for how much we move it to correct for its error
const int dir = 4; //DIR pin to control the direction of the motor (clockwise/counter-clockwise)

double Kp = 1;
double Ki = 0;
double Kd = 0;

float last_error = 0;
float error = 0;
float changeError = 0;
float totalError = 0;
float pidTerm = 0;
float pidTerm_scaled = 0;// if the total gain we get is not in the PWM range we scale it down so that it's not bigger than |255|
long prev_time;

// void setup() {
//   Serial.begin(9600);
//   pinMode(2, INPUT);//encoder pins
//   pinMode(3, INPUT);
//   attachInterrupt(0,Achange,CHANGE); //interrupt pins for encoder
//   attachInterrupt(1,Bchange,CHANGE); 
//   pinMode(pwm, OUTPUT);
//   pinMode(dir, OUTPUT);

// }
void Angle2Position(){

}

void PIDcalculation(){
  //angle = (0.9 * count); //count to angle conversion
  error = setpoint - input;
  
  changeError = error - last_error; // derivative term
  totalError += error; //accumalate errors to find integral term
  pidTerm = (Kp * error) + (Ki * totalError) + (Kd * changeError);//total gain
  pidTerm = constrain(pidTerm, -255, 255);//constraining to appropriate value
  pidTerm_scaled = abs(pidTerm);//make sure it's a positive value

  last_error = error;
}

void setup() {
  // LSM6DSO
  Serial.begin(115200);
  Serial.println("CLEARSHEET");
  Serial.println("LABEL,changeXAngle,changeYAngle,changeZAngle,Time");
  delay(500); //Do I need this?
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  
  Wire.begin();
  delay(10);
  myIMU.begin(0x6A,Wire);
  myIMU.initialize(BASIC_SETTINGS);
  // if( myIMU.begin(0x6A,Wire) )
  //   Serial.println("Ready.");
  // else { 
  //   Serial.println("Could not connect to IMU.");
  //   Serial.println("Freezing");
  // }

  // if( myIMU.initialize(BASIC_SETTINGS) )
  //   Serial.println("Loaded Settings.");

  //PID 
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);


}

void loop() {
  A_r1 = (analogRead(analogIn))/1024;
  A_r2 = analogRead(analogIn2)/2048;
  ref_pos = A_r1 * sin(2*M_PI*A_r2*millis()); //ref sine wave
  setpoint = ref_pos;

  //LSM6DSO
  //Get all parameters
  long prev_time = millis();
  float xAccel1 = myIMU.readFloatAccelX() - 1.994; //these were the numbers that were showing up when I was attached to the wrong communication ports
  float yAccel1 = myIMU.readFloatAccelY() - 1.994;
  float zAccel1 = myIMU.readFloatAccelZ() - 1.994;

  float xGyro1 = myIMU.readFloatGyroX();
  float yGyro1 = myIMU.readFloatGyroY();
  float zGyro1 = myIMU.readFloatGyroZ();

  delay(10);

  float xAccel2 = myIMU.readFloatAccelX()- 1.994; 
  float yAccel2 = myIMU.readFloatAccelY()- 1.994;
  float zAccel2 = myIMU.readFloatAccelZ()- 1.994;

  float xGyro2 = myIMU.readFloatGyroX();
  float yGyro2 = myIMU.readFloatGyroY();
  float zGyro2 = myIMU.readFloatGyroZ();

  // change in values
  float changeXAccel = xAccel2 - xAccel1;
  float changeYAccel = yAccel2 - yAccel1;
  float changeZAccel = zAccel2 - zAccel1;

  float changeXGyro = xGyro2 - xGyro1;
  float changeYGyro = yGyro2 - yGyro1;
  float changeZGyro = zGyro2 - zGyro1;

  //change in angular position
  long time = millis()-prev_time;
  float changeXAngle = changeXGyro * time;
  float changeYAngle = changeYGyro * time;
  float changeZAngle = changeZGyro * time;
  input = changeXAngle;

  PIDcalculation();// find PID value
  
  if (pidTerm >0) {
    digitalWrite(dir, HIGH);// Forward motion
    
  } else {
    digitalWrite(dir, LOW);//Reverse motion
    
  }

  analogWrite(pwm, pidTerm_scaled);

  // Serial.println(" X ANGLE: ");
  // Serial.println(changeXAngle);
  // Serial.println(" Y ANGLE: ");
  // Serial.println(changeYAngle);
  // Serial.println(" Z ANGLE: ");
  // Serial.println(changeZAngle);

  Serial.println( (String) "DATA,"+ changeXAngle + "," + changeYAngle + "," + changeZAngle + "," + millis());


  // delay(100);

  //print values

  // Serial.print("\nAccelerometer:\n");
  // Serial.print(" X = ");
  // Serial.println(myIMU.readFloatAccelX(), 3);
  // Serial.print(" Y = ");
  // Serial.println(myIMU.readFloatAccelY(), 3);
  // Serial.print(" Z = ");
  // Serial.println(myIMU.readFloatAccelZ(), 3);

  // Serial.print("\nGyroscope:\n");
  // Serial.print(" X = ");
  // Serial.println(myIMU.readFloatGyroX(), 3);
  // Serial.print(" Y = ");
  // Serial.println(myIMU.readFloatGyroY(), 3);
  // Serial.print(" Z = ");
  // Serial.println(myIMU.readFloatGyroZ(), 3);

  // Serial.print("\nThermometer:\n");
  // Serial.print(" Degrees F = ");
  // Serial.println(myIMU.readTempF(), 3);
  
  delay(100);
  



}
