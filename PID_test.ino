/*
PEK, Noah Karow
Mar 28, 2025
Senior Design Proj: Fish Robot Inc

This code will allow us to do open loop control, as we test out accelerometer and motor drivers
Diff from other code: no potentiometers, or PID. I also got rid of the roll and pitch function
*/

// Libraries
#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include "math.h"

// object: accelerometer
LSM6DSO myIMU; //Default constructor is I2C, addr 0x6B

// ****************PINS**********************************************************
// motor pins are all digital. 
// driver 1 controls pins 1 & 4, driver 2 controls coils 2 & 3

const int forwardDriver1 = 4; //turquoise. DIR pin to control the direction of the motor (clockwise/counter-clockwise)
const int reverseDriver1 = 5; // blue. driver 1
const int pwmDriver1 = 3; // orange. driver 1, this is the PWM pin for the motor for how much we move it to correct for its error

const int forwardDriver2 = 12; //driver 2, turquoise 
const int reverseDriver2 = 13;// driver 2, blue
const int pwmDriver2 = 11; // driver 2, orange


float x, y, z;           //three axis acceleration data
float roll, pitch;       //Roll & Pitch are the angles which rotate by the axis X and y


//PID
double setpoint, input;


double Kp = 1;
double Ki = 0;
double Kd = 0;

float last_error;
float error;
float changeError;
float totalError;
float pidTerm;
float pidTerm_scaled;// if the total gain we get is not in the PWM range we scale it down so that it's not bigger than |255|

double mag_pos;

void Angle2Position(float angle){
  double r = .045;
  double L = .125;
  mag_pos = (r * sqrt((pow(L,2))*cos(angle)+((pow(L,2))/4) * (1 - pow(cos(angle),2))))/((L*cos(angle))/2);
}

void RP_calculate(){
  roll = atan2(y , z) * 57.3;
  pitch = atan2((- x) , sqrt(y * y + z * z)) * 57.3;
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

void MD_Control(float pidTerm){
//***********************Motor Driver Control****************************************
  if (pidTerm > 0) // coils 1 & 4: Forward. coils 2 & 3: Reverse
  {
    digitalWrite(forwardDriver1, HIGH); // coils 1 & 4
    digitalWrite(reverseDriver1, LOW);  // coils 1 & 4
    
    digitalWrite(forwardDriver2, HIGH); // coils 2 & 3
    digitalWrite(reverseDriver2, LOW);  // coils 2 & 3
  } 
  else  // coils 1 & 4: Reverse. coils 2 & 3: Forward
  {
    digitalWrite(forwardDriver1, LOW); // coils 1 & 4
    digitalWrite(reverseDriver1, HIGH);  // coils 1 & 4
    
    digitalWrite(forwardDriver2, LOW); // coils 2 & 3
    digitalWrite(reverseDriver2, HIGH);  // coils 2 & 3
  }

  OCR2A = pidTerm_scaled; // motor driver 2 set to appropriate value
  OCR2B = pidTerm_scaled; // motor driver 1 set to appropriate value
}


void setup() {

  Serial.begin(115200);
  delay(500); //Do I need this?
  
  // Accelerometer
  Wire.begin();
  delay(10);

  if( myIMU.begin(0x6B,Wire) )
  {
    Serial.println("Ready.");
  }
  else 
  { 
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  if( myIMU.initialize(BASIC_SETTINGS) )
  {
    Serial.println("Loaded Settings.");
  }

  // inputs and outputs
  //pinMode(analogIn, INPUT); // blue potentiometer
  //pinMode(analogIn2, INPUT);  // orange potentiometer
  
  pinMode(pwmDriver1, OUTPUT);
  pinMode(forwardDriver1, OUTPUT);
  pinMode(reverseDriver1, OUTPUT);
  pinMode(pwmDriver2, OUTPUT);
  pinMode(forwardDriver2, OUTPUT);
  pinMode(reverseDriver2, OUTPUT);

  // PWM
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); // COM: enables non-inverted PWM, WGM: selects fast pwm
  TCCR2B = _BV(CS22); // prescaler: 64 -> freq ~ 1kHz. **********CHANGE depending on inductance of coils
  OCR2A = 0; // initially off. motor driver 2
  OCR2B = 0; // initially off. motor driver 1
}

void loop() {
  //******************Variables******************************************************
  setpoint = 1;
  x = myIMU.readFloatAccelX(); 
  y = myIMU.readFloatAccelY();
  z = myIMU.readFloatAccelZ();

  RP_calculate();
  Angle2Position(pitch);
  Serial.println(mag_pos);
  input = mag_pos;
  PIDcalculation();
  MD_Control(pidTerm);
 
  delay(100);
  }



