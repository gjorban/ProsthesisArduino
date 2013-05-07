#include <pot_box.h>
#include <Wire.h>
#include <MsTimer2.h>
#include <PID_v1.h>
#include <Motor_Controller_2.h>

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

#define DISPLAY_ADDRESS1 0x70 //This is the default address of the OpenSegment with both solder jumpers open
#define DISPLAY_ADDRESS2 0x72

#define SYSTEM_PRINT_TIME 1000 //ms
#define INTERRUPT_TIME 1 //ms
#define SAMPLE_TIME 1 //ms

#define INITIAL_SETPOINT 1500 //psi

double pidInput = 0;
double pidSetpoint = 0;
double pidOutput = 0;
double pidP = 0;
double pidI = 0;
double pidD = 0;

boolean changeSetpointFlag = false; //if switch is pressed

unsigned long printTimer = 0;
boolean serialPrintFlag = false;

MotorController MController(pidInput, pidOutput, pidSetpoint, pidP, pidI, pidD, DIRECT, P_PIN, I_PIN, D_PIN, INPUT_PRESSURE_PIN, PWM_PIN, POT_BOX_INTERRUPT_ID, POT_BOX_INTERRUPT_PIN);

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(DISPLAY_ADDRESS1);
  Wire.write('v');
  Wire.endTransmission();
  Wire.beginTransmission(DISPLAY_ADDRESS2);
  Wire.write('v');
  Wire.endTransmission();
  
  MController.SetSetpoint(INITIAL_SETPOINT);
  MController.Initialize(SAMPLE_TIME);
  MsTimer2::set(INTERRUPT_TIME,TimedCommands);
  MsTimer2::start();
}//end setup()

void TimedCommands()
{
  MController.Calculate();
  if( (millis()-printTimer) > SYSTEM_PRINT_TIME)
  {
    printTimer = millis();
    serialPrintFlag = true;
  }
}//end TimedCommands()

void loop()
{
  MController.Iterate();
  Interface();
  if (serialPrintFlag == true)
  {
    Print_Info_Seg();
    Print_Info_Ser();
    serialPrintFlag = false;
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
  WriteValue(DISPLAY_ADDRESS1, MController.GetInput());
  WriteValue(DISPLAY_ADDRESS2, MController.GetSetpoint());
}

void WriteValue(int address, double value)
{
  int truncatedValue = (int)value;
  
  Wire.beginTransmission(address);
  Wire.write(0x79);
  Wire.write(0x00);
  Wire.write(truncatedValue/1000);
  truncatedValue %= 1000;
  Wire.write(truncatedValue/100);
  truncatedValue %= 100;
  Wire.write(truncatedValue/10);
  truncatedValue %= 10;
  Wire.write(truncatedValue);
  Wire.endTransmission();
}

void Interface()
{
  //for setpoint switches
  if( changeSetpointFlag == false && digitalRead(SETPOINT_PLUS_PIN) == HIGH )
  {
    MController.AddToSetpoint(50);
    changeSetpointFlag = true;
  }
  else if( changeSetpointFlag == false && digitalRead(SETPOINT_MINUS_PIN) == HIGH )
  {
    MController.AddToSetpoint(-50);
    changeSetpointFlag = true;
  }
  else if(digitalRead(SETPOINT_PLUS_PIN) == LOW && digitalRead(SETPOINT_MINUS_PIN) == LOW)
  {
    changeSetpointFlag = false;
  }
}//end Interface()


