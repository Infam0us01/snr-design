

// Libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ========== CONSTANTS ========== //
#define BNO055_ADDRESS 0x28    // I2C address of IMU
#define SERIAL_BAUD_RATE 115200
#define LOOP_RATE_HZ 100       // Control loop frequency
#define LOOP_PERIOD_MS (1000 / LOOP_RATE_HZ)
double CONSTANT_FORCE;
double CONSTANT_FORCE_FACTOR;



// ****************PINS**********************************************************
// motor pins are all digital. 
// driver 1 controls pins 1 & 4, driver 2 controls coils 2 & 3

const int forwardDriver1 = 4; //turquoise. DIR pin to control the direction of the motor (clockwise/counter-clockwise)
const int reverseDriver1 = 5; // blue. driver 1
const int pwmDriver1 = 3; // orange. driver 1, this is the PWM pin for the motor for how much we move it to correct for its error

const int forwardDriver2 = 12; //driver 2, turquoise 
const int reverseDriver2 = 13;// driver 2, blue
const int pwmDriver2 = 11; // driver 2, orange

//PID
double setpoint, input;


double Kp = .5;
double Ki = 0;
double Kd = 0;

float last_error;
float error;
float changeError;
float totalError;
float pidTerm;
float pidTerm_scaled;// if the total gain we get is not in the PWM range we scale it down so that it's not bigger than |255|


int potPinblue = A0;
double potValblue = 0;

int potPinwhite = A1;
double potValwhite = 0;



// ========== GLOBAL OBJECTS ========== //
//PIDController positionPID;
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS, &Wire);
unsigned long lastTime = 0;

void PIDcalculation(){
  error = setpoint - input;
  
  changeError = error - last_error; // derivative term
  totalError += error; //accumalate errors to find integral term
  pidTerm = (Kp * error) + (Ki * totalError) + (Kd * changeError);//total gain
  pidTerm = constrain(pidTerm, -255, 255);//constraining to appropriate value
  pidTerm_scaled = abs(pidTerm);//make sure it's a positive value

  last_error = error;

}

void MD_one(int direction, int value){ //coils 1 and 4

  if (direction == 1){ //go right
    digitalWrite(forwardDriver1, LOW); // coils 1 & 4
    digitalWrite(reverseDriver1, HIGH);  // coils 1 & 4
  }
  else{ //go left
    digitalWrite(forwardDriver1, HIGH); // coils 1 & 4
    digitalWrite(reverseDriver1, LOW);  // coils 1 & 4
  }
  OCR2B = value;
}

void MD_two(int direction, int value){ //coils 2 and 3

  if (direction == 1){ //go right
    digitalWrite(forwardDriver2, HIGH); // coils 2 & 3
    digitalWrite(reverseDriver2, LOW);  // coils 2 & 3
  }
  else{ //go left
    digitalWrite(forwardDriver2, LOW); // coils 2 & 3
    digitalWrite(reverseDriver2, HIGH);  // coils 2 & 3
  }
  OCR2A = value;
}

void MD_Control(float pidTerm,float direction){
//***********************Motor Driver Control****************************************
  if (direction > 0) // coils 1 & 4: Forward. coils 2 & 3: Reverse
  {
    // Go right
    MD_one(1,CONSTANT_FORCE);
    MD_two(0,pidTerm);

  } 
  else if (direction ==0){
    // Go to the center
    MD_one(1,CONSTANT_FORCE);
    MD_two(0,CONSTANT_FORCE);
  }
  else  // coils 1 & 4: Reverse. coils 2 & 3: Forward
  {
    // Go left
    MD_one(1,pidTerm);
    MD_two(0,CONSTANT_FORCE);
  }
}


void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   
  delay(1000);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);

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
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
  lastTime = currentTime;
  
  // Skip iteration if time delta too small
  if (dt < 0.001) return;
  setpoint = map(
    analogRead(potPinblue), 
    0, 1023, 
    -10, 10) ;// Scale to calibrated range

  CONSTANT_FORCE= map(
    analogRead(potPinwhite), 
    0, 1023, 
    0, 255) ;// Scale to calibrated range

  Serial.print("Stiffness");
  Serial.println(CONSTANT_FORCE);

  
 
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  double angle = euler.x();


  if (angle > 300) // actuator has rotated counterclockwise
  { 
    angle = angle - 360;
  }

  
  input = angle;


  PIDcalculation();

  Serial.print("Error: ");
  Serial.print(error);

  MD_Control(pidTerm_scaled,setpoint);


  Serial.print("Set:");
  Serial.print(setpoint);
  Serial.print("° Ang:");
  Serial.print(angle,1);
  Serial.print("° Vel:");

 
  // Maintain constant loop rate
  delay(max(0, LOOP_PERIOD_MS - (millis() - currentTime)));
  }



