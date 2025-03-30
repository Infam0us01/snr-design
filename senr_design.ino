// Noah Karow March 27, 2025
// PEK changes Mar 28, 2025
// PEK got much of this from Noah's github. PEK changing motor driver code for directionality

/* Things PEK changed: organized some set up code,
 added all motor driver pins and their pin mode settings,
 uncommented the if/else logic around the accelerometer setup,
 removed "=0" from global variable initializations so vars would not be overwritten,
 added to pidTerm >/< 0 logic so all directions will change,
 added some comments for organization and clarification,
 deleted some content PEK doesn't think will be needed, 
 changed PWM code from analogWrite to directly setting the duty cycle (OCR2A/OCR2B).
*/

//Sensor Input
#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include "math.h"


LSM6DSO myIMU; //Default constructor is I2C, addr 0x6B

// ****************PINS**********************************************************
// potentiometer pins
const int analogIn = A0; // potentiometer 1. blue
const int analogIn2 = A1; // Potentiometer 2. white

// motor pins. these are all digital. driver 1 controls pins 1 & 4, driver 2 controls coils 2 & 3
const int pwmDriver1 = 3; // orange. driver 1, this is the PWM pin for the motor for how much we move it to correct for its error
const int forwardDriver1 = 4; //turquoise. DIR pin to control the direction of the motor (clockwise/counter-clockwise)
const int reverseDriver1 = 5; // blue. driver 1

const int pwmDriver2 = 11; // driver 2, orange
const int forwardDriver2 = 12; //driver 2, turquoise 
const int reverseDriver2 = 13;// driver 2, blue

// ****************VARIABLES**********************************************************// VARIABLES
float ref_pos = 1;
double A_r1 = 1;
double A_r2 = 1;

float x, y, z;           //three axis acceleration data
float roll, pitch;       //Roll & Pitch are the angles which rotate by the axis X and y


//PID
double setpoint, input;
// double count = 0; //set the counts of the encoder
// double angle = 0;//set the angles
// boolean A,B;
// byte state, statep;

double Kp = 1;
double Ki = 0;
double Kd = 0;

float last_error;
float error;
float changeError;
float totalError;
float pidTerm;
float pidTerm_scaled;// if the total gain we get is not in the PWM range we scale it down so that it's not bigger than |255|


double Angle2Position(float angle)
{
  double r = 1;
  double L = 1;
  //angle = int(angle);
  //use geometry to get position from pitch
  // position = ___;
  double position = (r * sqrt((pow(L,2))*cos(angle)+((pow(L,2))/4) * (1 - pow(cos(angle),2))))/((L*cos(angle))/2);
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

void setup() {

  Serial.begin(115200);
  // LSM6DSO
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

  // inputs and outputs
  pinMode(analogIn, INPUT); // blue potentiometer
  pinMode(analogIn2, INPUT);  // orange potentiometer
  
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
  A_r1 = (analogRead(analogIn))/1024; // calculating amplitude of position (cm). *****Needs to change based on actual max/min 
  A_r2 = analogRead(analogIn2)/2048; // calculating freq (Hz) of travelling wave **** could change but probs not necessary
  ref_pos = A_r1 * sin(2*M_PI*A_r2*millis()); //ref sine wave
  setpoint = ref_pos;

  //LSM6DSO
  //Get all parameters
  x = myIMU.readFloatAccelX();
  y = myIMU.readFloatAccelY();
  z = myIMU.readFloatAccelZ();

  RP_calculate();
  Serial.println(pitch,10);
  //Serial.println(roll,10);

  // PID
  double position = Angle2Position(pitch);
  input = position;
  PIDcalculation();// find PID value

  
  if (pidTerm > 0) // coils 1 & 4: Forward. coils 2 & 3: Reverse
  {
    digitalWrite(forwardDriver1, HIGH); // coils 1 & 4
    digitalWrite(reverseDriver1, LOW);  // coils 1 & 4
    
    digitalWrite(forwardDriver2, LOW); // coils 2 & 3
    digitalWrite(reverseDriver2, HIGH);  // coils 2 & 3
  } 
  else  // coils 1 & 4: Reverse. coils 2 & 3: Forward
  {
    digitalWrite(forwardDriver1, LOW); // coils 1 & 4
    digitalWrite(reverseDriver1, HIGH);  // coils 1 & 4
    
    digitalWrite(forwardDriver2, HIGH); // coils 2 & 3
    digitalWrite(reverseDriver2, LOW);  // coils 2 & 3
    
  }

  OCR2A = pidTerm_scaled; // motor driver 2 set to appropriate value
  OCR2B = pidTerm_scaled; // motor driver 1 set to appropriate value

  delay(100);

}