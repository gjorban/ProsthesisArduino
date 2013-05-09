#include <pot_box.h>
#include <Wire.h>
#include <MsTimer2.h>
#include <PID_v1.h>
#include <MotorController.h>

#define INPUT_PRESSURE_PIN 0
#define P_PIN 5
#define I_PIN 6
#define D_PIN 7
#define PWM_PIN 13
#define POT_BOX_INTERRUPT_ID 0
#define POT_BOX_INTERRUPT_PIN 2

#define SETPOINT_PLUS_PIN 25
#define SETPOINT_MINUS_PIN 24
#define LOADSENSE_PLUS_PIN 23
#define LOADSENSE_MINUS_PIN 22

#define DISPLAY_INPUT_PRESSURE 0x70 //This is the default address of the OpenSegment with both solder jumpers open
#define DISPLAY_SETPOINT_PRESSURE 0x72
#define CLEAR_DISPLAY 0x76 // Command to clear the 7seg display (sparkfun version)

#define SYSTEM_PRINT_TIME_MS 1000 //ms
#define INTERRUPT_TIME_MS 1 //ms
#define SAMPLE_TIME_MS 1 //ms

#define INITIAL_SETPOINT_PSI 1500 //psi

double mPIDInput = 0;
double mPIDSetpoint = 0;
double mPIDOutput = 0;
double mPIDP = 0;
double mPIDI = 0;
double mPIDD = 0;

boolean mChangeSetpointFlag = false; //if switch is pressed

unsigned long mLastDebugPrintMilli = 0;
boolean mPrintDebugStats = false;

MotorController MController(mPIDInput, mPIDOutput, mPIDSetpoint, mPIDP, mPIDI, mPIDD, DIRECT, P_PIN, I_PIN, D_PIN, INPUT_PRESSURE_PIN, PWM_PIN, POT_BOX_INTERRUPT_ID, POT_BOX_INTERRUPT_PIN);

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(DISPLAY_INPUT_PRESSURE);
  Wire.write(CLEAR_DISPLAY);
  Wire.endTransmission();
  Wire.beginTransmission(DISPLAY_SETPOINT_PRESSURE);
  Wire.write(CLEAR_DISPLAY);
  Wire.endTransmission();
  
  MController.SetSetpoint(INITIAL_SETPOINT_PSI);
  MController.Initialize(SAMPLE_TIME_MS);
  MsTimer2::set(INTERRUPT_TIME_MS,TimedCommands);
  MsTimer2::start();
}//end setup()

void TimedCommands()
{
  MController.Calculate();
  if( (millis()-mLastDebugPrintMilli) > SYSTEM_PRINT_TIME_MS)
  {
    mLastDebugPrintMilli = millis();
    mPrintDebugStats = true;
  }
}//end TimedCommands()

void loop()
{
  MController.UpdateState();
  ReadFromInterface();
  if (mPrintDebugStats == true)
  {
    Print_Info_Seg();
    Print_Info_Ser();
    mPrintDebugStats = false;
  }
}//end loop()

void Print_Info_Ser()
{
  Serial.print("P:");Serial.println(MController.GetP());
  Serial.print("I:");Serial.println(MController.GetI());
  Serial.print("D:");Serial.println(MController.GetD());
  Serial.print("In:");Serial.println(MController.GetInput());
  Serial.print("Set:");Serial.println(MController.GetSetpoint());
  Serial.print("Out:");Serial.println(MController.GetOutput());
}//end Print_Info()

void Print_Info_Seg()
{
  WriteValue(DISPLAY_INPUT_PRESSURE, (int) MController.GetInput());
  WriteValue(DISPLAY_SETPOINT_PRESSURE, (int) MController.GetSetpoint());
}

void WriteValue(int address, int value)
{
  Wire.beginTransmission(address);
  Wire.write(0x79);
  Wire.write(0x00);
  Wire.write(value/1000);
  value %= 1000;
  Wire.write(value/100);
  value %= 100;
  Wire.write(value/10);
  value %= 10;
  Wire.write(value);
  Wire.endTransmission();
}

void ReadFromInterface()
{
  //for setpoint switches
  if( mChangeSetpointFlag == false && digitalRead(SETPOINT_PLUS_PIN) == HIGH )
  {
    MController.AddToSetpoint(50);
    mChangeSetpointFlag = true;
  }
  else if( mChangeSetpointFlag == false && digitalRead(SETPOINT_MINUS_PIN) == HIGH )
  {
    MController.AddToSetpoint(-50);
    mChangeSetpointFlag = true;
  }
  else if(digitalRead(SETPOINT_PLUS_PIN) == LOW && digitalRead(SETPOINT_MINUS_PIN) == LOW)
  {
    mChangeSetpointFlag = false;
  }
}//end Interface()


