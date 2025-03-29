#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include "math.h"
LSM6DSO myIMU; //Default constructor is I2C, addr 0x6B
float x, y, z;                        //three axis acceleration data
float roll, pitch;       //Roll & Pitch are the angles which rotate by the axis X and y

//Reference
const int analogIn = A0; // potentiometer 1
const int analogIn2 = A1; // Potentiometer 2
float ref_pos = 1;
double A_r1 =1;
double A_r2 = 1;
double position;


//Sensor Input


//PID
double setpoint, input;

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


void Angle2Position(){
  //use geometry to get position from pitch
  // position = ___;
}

void RP_calculate(){
  roll = atan2(y , z) * 57.3;
  pitch = atan2((- x) , sqrt(y * y + z * z)) * 57.3;
  //Serial.println(roll);
  //Serial.println(pitch);
}

void PIDcalculation(){
  error = setpoint - input;
  
  changeError = error - last_error; // derivative term
  totalError += error; //accumalate errors to find integral term
  pidTerm = (Kp * error) + (Ki * totalError) + (Kd * changeError);//total gain
  pidTerm = constrain(pidTerm, -255, 255);//constraining to appropriate value
  pidTerm_scaled = abs(pidTerm);//make sure it's a positive value

  last_error = error;
}

void setup() {
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

  //PID 
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);


}

void loop() {
  A_r1 = (analogRead(analogIn))/1024;
  A_r2 = analogRead(analogIn2)/2048;
  ref_pos = A_r1 * sin(2*M_PI*A_r2*millis()); //ref sine wave
  setpoint = ref_pos;

  x = myIMU.readFloatAccelX();
  y = myIMU.readFloatAccelY();
  z = myIMU.readFloatAccelZ();
  //Serial.println(x,10);

  RP_calculate();
  Serial.println(pitch,10);
  //Serial.println(roll,10);

  // PID

  input = position;

  PIDcalculation();// find PID value
  
  if (pidTerm >0) {
    digitalWrite(dir, HIGH);// Forward motion
    
  } else {
    digitalWrite(dir, LOW);//Reverse motion
    
  }

  analogWrite(pwm, pidTerm_scaled);

  delay(100);

}
