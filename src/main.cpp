#include <Arduino.h>

#include "ArduPID.h"

#define BAUD_RATE 115200

#define ENCODER_A 2
#define ENCODER_B 3
#define MOTOR_CW 9
#define MOTOR_CCW 10
#define MOTOR_PWM 11
#define POT_INPUT A0

const float DEGREE = 49/360;

// const float K = 0.87913043478260869565217391304348 / 4;
// const float tauD = (1034996-1000000)/1000000;
// const float tau = 1361012;

double setPoint;
double output;
double Kc, Ti, Td;
int deltaT;
int encoderCount, encoderCountPrev;
float timeNow, timePrev;
double angularVel;
double startTime;
bool encAPressed;


void encoder(int& thisEncCount);

void setup() {
  Serial.begin(9600);

  //Initiate variables
  encoderCount=0;
  encoderCountPrev=0;
  timeNow = micros();
  timePrev = micros();
  
  startTime = millis();
  encAPressed=false;

  Kc = 0.9 * tau / (K * tauD);
  Ti = 3.3*tauD;
  Td = 0;

  angularVel=0;
  

  //myController.setOutputLimits(0,255);

  
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(ENCODER_A, INPUT); // ENCODER_A as Input
  pinMode(ENCODER_B, INPUT); // ENCODER_B as Input

  pinMode(MOTOR_CW, OUTPUT); // MOTOR_CW as Output
  pinMode(MOTOR_CCW, OUTPUT); // MOTOR_CW as Output


}

void loop() 
{

  
}



void encoder(int& thisEncCount) {

  if(digitalRead(ENCODER_A) && !encAPressed)
  {
    timeNow = micros();

    if(digitalRead(ENCODER_B))
    {
      thisEncCount++;
      angularVel = -1000000/(timeNow-timePrev);
    }
    else
    {
      thisEncCount--;
      timeNow = micros();
      angularVel = 1000000/(timeNow-timePrev);
    }


    //Serial.println(String(timeNow - startTime) + "," + String(angularVel) + "\n");
    timePrev = timeNow;
    encAPressed=true;
  
  }
  
  if(digitalRead(ENCODER_A)==false)
  {
    encAPressed=false;
  }

}