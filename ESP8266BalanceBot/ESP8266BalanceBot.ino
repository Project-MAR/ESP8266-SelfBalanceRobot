#include "I2Cdev.h"
#include "MPU6050.h"
#include <PID_v1.h>

#define LED_BuiltIn  2

/*
 * Motor1x == Right Motor
 * 1A = HIGH, 1B = LOW   ==> Forward
 * 1A = LOW,  1B = HIGH  ==> Backward
 * 
 * Motor2x == Left Motor
 * 2A = LOW,  2B = HIGH  ==> Forward
 * 2A = HIGH, 2B = LOW   ==> Backward 
 * 
 * NodeMCU Pin D5, D6, D7, D8 == Arduino Pin 14, 12, 13, 15
 */
#define Motor1A      12
#define Motor1B      14

#define Motor2A      15
#define Motor2B      13

#define X            0
#define Y            1
#define Z            2

#define ACCELEROMETER_SENSITIVITY   8192.0
#define GYROSCOPE_SENSITIVITY       65.536
#define M_PI                        3.14159265359

/*
 * SCL = D1 (Arduino Pin 5)
 * SDA = D2 (Arduino Pin 4)
 */
MPU6050 MPU6000;

unsigned long PreviousMillis = 0;
unsigned long CurrentMillis  = 0;
unsigned long Duration       = 0;

int16_t Acc[3];
int16_t Gyo[3];

int blinkcount = 0;
bool blinkState = false;
bool testIOState = false;
bool isMPUConnectSuccess = false;
bool BalanceTick = false;

double Angle; 
double pitch, roll;

double Output;

double Kp=900; 
double Kd=60;
double Ki=10;
double Setpoint = -1.3;

PID BalancePID(&pitch, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() 
{
    Serial.begin(115200);

    pinMode(Motor1A, OUTPUT);
    pinMode(Motor1B, OUTPUT);
    pinMode(Motor2A, OUTPUT);
    pinMode(Motor2B, OUTPUT);
    Drive(0, 0, 0, 0);
    
    Wire.begin();
    MPU6000.initialize();

    Serial.println("Testing MPU6000 connections...");

    isMPUConnectSuccess =  MPU6000.testConnection();
    if(isMPUConnectSuccess)
    {
      Serial.println("MPU6050 connection successful");
    }
    else
    {
      Serial.println("MPU6050 connection failed");
    }
    pinMode(LED_BuiltIn, OUTPUT);

    BalancePID.SetMode(AUTOMATIC);
    BalancePID.SetOutputLimits(-1023, 1023);
    BalancePID.SetSampleTime(10);
}

void Drive(int LeftMotorPower, int RightMotorPower, int LEFT_MIN_SPEED, int RIGHT_MIN_SPEED)
{
  int LeftMotorOffset = 10;

  // Set Motor Power Offset
  // For me, Left Motor seem to be less power than Right Motor
  if(LeftMotorPower > 0)
      LeftMotorPower += 10;
  else if (LeftMotorPower < 0)
      LeftMotorPower -= 10;

  // Set Minimum Speed
  if (LeftMotorPower > 0)
  {
    if(LeftMotorPower < LEFT_MIN_SPEED)
    {
      //Serial.println("Use Min Value for Left Forward");
      LeftMotorPower = LEFT_MIN_SPEED;
    }
  }
  else if(LeftMotorPower < 0)
  {
    if(-LeftMotorPower < LEFT_MIN_SPEED)
    {
      //Serial.println("Use Min Value for Left Backward");
      LeftMotorPower = -LEFT_MIN_SPEED;
    }
  }
  
  if (RightMotorPower > 0)
  {
    if(RightMotorPower < RIGHT_MIN_SPEED)
    {
      //Serial.println("Use Min Value for Right Forward");
      RightMotorPower = RIGHT_MIN_SPEED;
    }
  }
  else if(RightMotorPower < 0)
  {
    if(-RightMotorPower < RIGHT_MIN_SPEED)
    {
      //Serial.println("Use Min Value for Right Backward");
      RightMotorPower = -RIGHT_MIN_SPEED;
    }
  }

  // Max PWM Range
  if(abs(LeftMotorPower) > 1023)
  {
      if(LeftMotorPower > 0)
        LeftMotorPower = 1023;
      else
        LeftMotorPower = -1023;
  }
  
  if(abs(RightMotorPower) > 1023)
  {
      if(RightMotorPower > 0)
        RightMotorPower = 1023;
      else
        RightMotorPower = -1023;
  }

  // Drive
  if(LeftMotorPower > 0)
  {
    //Left Motor => Forward
    analogWrite(Motor2A, 0);
    analogWrite(Motor2B, LeftMotorPower);
  }
  else if(LeftMotorPower < 0)
  {
    //Left Motor => Backward
    analogWrite(Motor2A, -LeftMotorPower);
    analogWrite(Motor2B, 0);
  }
  else
  {
    analogWrite(Motor2A, 0);
    analogWrite(Motor2B, 0);
  }

  if(RightMotorPower > 0)
  {
    // Right Motor Forward
    analogWrite(Motor1A, RightMotorPower);
    analogWrite(Motor1B, 0);
  }
  else if(RightMotorPower < 0)
  {
    // Right Motor Backward
    analogWrite(Motor1A, 0);
    analogWrite(Motor1B, -RightMotorPower);
  }
  else
  {
    analogWrite(Motor1A, 0);
    analogWrite(Motor1B, 0);
  }
}

void updateMPU(int16_t *Ax, int16_t *Ay, int16_t *Az, int16_t *Gx, int16_t *Gy, int16_t *Gz, unsigned long *PreviousMillis, unsigned long *CurrentMillis, unsigned long *Duration)
{
  //MPU6000.getMotion6(&Acc[X], &Acc[Y], &Acc[Z], &Gyo[X], &Gyo[Y], &Gyo[Z]);
  MPU6000.getAcceleration(Ax, Ay, Az);

  *CurrentMillis = millis();
  MPU6000.getRotation(Gx, Gy, Gz);
  *Duration = *CurrentMillis - *PreviousMillis;
  *PreviousMillis = *CurrentMillis;
}

/*
 * Original: http://www.pieter-jan.com/node/11
 * Author :  Pieter-Jan
 * Date   : Fri, 26/04/2013
 */
void ComplementaryFilter(short accData[3], short gyrData[3], double *pitch, double *roll, double dt)
{
    float pitchAcc, rollAcc;               
 
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    *pitch += ((double)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt;   // Angle around the X-axis
    *roll -= ((double)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis
 
    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {
        // Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f((double)accData[1], (double)accData[2]) * 180 / M_PI;
        *pitch = *pitch * 0.98 + pitchAcc * 0.02;
 
        // Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f((double)accData[0], (double)accData[2]) * 180 / M_PI;
        *roll = *roll * 0.98 + rollAcc * 0.02;
    }
} 

void BalanceNow()
{
    updateMPU(&Acc[X], &Acc[Y], &Acc[Z], &Gyo[X], &Gyo[Y], &Gyo[Z], &PreviousMillis, &CurrentMillis, &Duration);
    ComplementaryFilter(Acc, Gyo, &pitch, &roll, Duration/1000.0);
    Drive(int(Output), int(Output), 0, 0);
}

void loop() 
{
  delay(0);
  if(isMPUConnectSuccess)
  {
    if(BalancePID.Compute())
    {
    BalanceNow();
    }
    
    // If Pitch too much, dont's Drive
    if(abs(pitch) > 40)
    {
    Drive(0, 0, 0, 0);
    } 
    else
    {
    //Drive(int(Output), int(Output), 0, 0);
    // Print to Serial Plotter
    //Serial.print(pitch);
    //Serial.print(" ");
    //Serial.println(Setpoint);
    }
  }
  else
  {
    Drive(0, 0, 0, 0);
  }

  blinkcount++;
  if(blinkcount >= 1000)
  {
    blinkcount = 0;
    blinkState = !blinkState;
    digitalWrite(LED_BuiltIn, blinkState);
  }
  if(!isMPUConnectSuccess)
  {
     digitalWrite(LED_BuiltIn, HIGH);
  }
}
