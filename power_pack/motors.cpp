#include <Arduino.h>
#include <PID_v1.h>
#include <pot_box.h>
#include "motors.h"
#include "pins.h"

//Handy wrapper for using stream-like serial printing
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }

//Constant terms
#define PID_POT_SENSITIVITY 0.02
#define ANALOG_TO_VOLTAGE 0.004892494
#define PRESSURE_SENSITIVITY 1386.2
#define PRESSURE_INTERCEPT 1246.7

#define ANALOG_READ_TO_PRESSURE(x) (((double)x * ANALOG_TO_VOLTAGE * PRESSURE_SENSITIVITY) - PRESSURE_INTERCEPT)

typedef struct tMotorConfig {
  tMotorConfig() : 
  mThrottlePin(-1), 
  mPressureInputPin(-1), 
  mLoadInputPin(-1), 
  mActive(false), 
  mSampleAvg(0.0f),
  mP(0.0f), 
  mI(0.0f), 
  mD(0.0f), 
  mPressureSetpoint(1500),
  mThrottle(0),
  mPIDController(&mSampleAvg, &mThrottle, &mPressureSetpoint, 0, 0, 0, DIRECT)
  {
    mPIDController.SetMode(AUTOMATIC);
    mPIDController.SetSampleTime(50);
  }
 
  uint8_t mThrottlePin;
  uint8_t mPressureInputPin;
  uint8_t mLoadInputPin;
  double mSampleAvg;
  float mP;
  float mI;
  float mD;
  double mPressureSetpoint;
  double mThrottle;
  bool mActive;
  
  PID mPIDController;
} tMotorConfig;

namespace ProsthesisMotors
{
 
  tMotorConfig mKneeMotorConfig;
  tMotorConfig mHipMotorConfig;
  ProsthesisPotBox mPotBox(POT_BOX_INTERRUPT_ID, POT_BOX_INTERRUPT_PIN, POT_BOX_P_PIN, POT_BOX_I_PIN, POT_BOX_D_PIN, 0, 0, 0);
  
  int mPotBoxConnected = false;
  int mPotBoxConnectionDirty = false;
  
  void PotBoxConnectionDirtyCB()
  {
    mPotBoxConnectionDirty = true;
  }
  
  void Initialize()
  {
    mPotBox.SetConnectionDirtyCallback(PotBoxConnectionDirtyCB);
    mPotBox.AttemptReconnect(&mPotBoxConnected);
  }
  
  void SetKneeMotorPinout(int kneeThrottleOut, int kneePressureIn, int kneeLoadIn)
  {
    mKneeMotorConfig.mThrottlePin = kneeThrottleOut;
    mKneeMotorConfig.mPressureInputPin = kneePressureIn;
    mKneeMotorConfig.mLoadInputPin = kneeLoadIn;
  }
  
  void SetHipMotorPinout(int hitThrottleOut, int hipPressureIn, int hidLoadIn)
  {
    mHipMotorConfig.mThrottlePin = hitThrottleOut;
    mHipMotorConfig.mPressureInputPin = hipPressureIn;
    mHipMotorConfig.mLoadInputPin = hidLoadIn;    
  }
  
  void ToggleKneeMotorControl(bool toggle)
  {
    mKneeMotorConfig.mActive = toggle;
    analogWrite(mKneeMotorConfig.mThrottlePin, 255);
  }
  
  void ToggleHipMotorControl(bool toggle)
  {
    mHipMotorConfig.mActive = toggle;
    analogWrite(mHipMotorConfig.mThrottlePin, 255);
  }
 
  void UpdateMotors()
  { 
    if (mPotBoxConnectionDirty || !mPotBoxConnected)
    {
      mPotBox.AttemptReconnect(&mPotBoxConnected);
      mPotBoxConnectionDirty = false;
    }
    
   // if (mPotBoxConnected)
    {
      int intP;
      int intI;
      int intD;
      mPotBox.GetPID(&intP, &intI, &intD);
      mKneeMotorConfig.mP = intP * PID_POT_SENSITIVITY;
      mKneeMotorConfig.mI = intI * PID_POT_SENSITIVITY;
      mKneeMotorConfig.mD = intD * PID_POT_SENSITIVITY;
      mKneeMotorConfig.mPIDController.SetTunings(mKneeMotorConfig.mP, mKneeMotorConfig.mI, mKneeMotorConfig.mD);
      //Serial << "Read pot box " << intP << " " << intI << " " << intD << "\n";
    }
    
    if (mKneeMotorConfig.mActive)
    {
      //Use exponential averaging from http://bleaklow.com/2012/06/20/sensor_smoothing_and_optimised_maths_on_the_arduino.html
      mKneeMotorConfig.mSampleAvg = 0.1 * ANALOG_READ_TO_PRESSURE(analogRead(mKneeMotorConfig.mPressureInputPin)) + 0.9 * mKneeMotorConfig.mSampleAvg;
      if (mKneeMotorConfig.mPIDController.Compute())
      {
        //Serial << "Knee: T: " << (int)mKneeMotorConfig.mThrottle << ". Press: " << mKneeMotorConfig.mSampleAvg << "PID: " << mKneeMotorConfig.mP << ", " << mKneeMotorConfig.mI << ", " << mKneeMotorConfig.mD << "\n";
        analogWrite(mKneeMotorConfig.mThrottlePin, mKneeMotorConfig.mThrottle);
      }
    }
    
    if (mHipMotorConfig.mActive)
    {
      //Use exponential averaging from http://bleaklow.com/2012/06/20/sensor_smoothing_and_optimised_maths_on_the_arduino.html
      mHipMotorConfig.mSampleAvg = 0.1 * ANALOG_READ_TO_PRESSURE(analogRead(mHipMotorConfig.mPressureInputPin)) + 0.9 * mHipMotorConfig.mSampleAvg;
      if (mHipMotorConfig.mPIDController.Compute())
      {
        //Serial << "Hip: T: " << (int)mKneeMotorConfig.mThrottle << ". Press: " << mKneeMotorConfig.mSampleAvg << "PID: " << mKneeMotorConfig.mP << ", " << mKneeMotorConfig.mI << ", " << mKneeMotorConfig.mD << "\n";        
        analogWrite(mHipMotorConfig.mThrottlePin, mHipMotorConfig.mThrottle);
      }
    }
  } 
}