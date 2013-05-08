#include "Motor_Controller.h"

MotorController* MotorController::sMotorCont=NULL;

MotorController::MotorController(double tInput, double tOutput, double tSetpoint,
                                 double tP, double tI, double tD, int tDirection,
		                         int tPPin, int tIPin, int tDPin,
		                         int tInputPin, int tOutputPin,
								 int tBoxInterruptID, int tBoxConnectionPin)

	:PID_Controller(&mcInput, &mcOutput, &mcSetpoint, tP, tI, tD, tDirection)
{
    mcInput = tInput;
	mcOutput = tOutput;
	mcSetpoint = tSetpoint;
	mcP = tP;
	mcI = tI;
	mcD = tD;
	mcPPin = tPPin;
	mcIPin = tIPin;
	mcDPin = tDPin;
	mcInputPin = tInputPin;
	mcOutputPin = tOutputPin;
	mcBoxInterruptID = tBoxInterruptID;
	mcBoxConnectionPin = tBoxConnectionPin;
	mcBoxConnectionDirty = true;
	sMotorCont = this;
}

void MotorController::Initialize(int time)
{
  mcInput = 0;
  mcInputTrack = 0;
  for(int i=0; i<INPUT_AVG_COUNT; i++)
  {
    mcInputArray[i] = 0;
  }
  
  PID_Controller.SetMode(AUTOMATIC);
  PID_Controller.SetSampleTime(time);
  PID_PotBox = ProsthesisPotBox(mcBoxInterruptID, mcBoxConnectionPin, mcPPin, mcIPin, mcDPin, mcP, mcI, mcD);
  PID_PotBox.SetConnectionDirtyCallback(WrapperForMarkConnectionDirty);

  PID_PotBox.AttemptReconnect(&mcBoxIsConnected);
}//end Initialize(int)

void MotorController::MarkConnectionDirty()
{
  mcBoxConnectionDirty = true;
}//end MarkConnectionDirty()

void MotorController::WrapperForMarkConnectionDirty()
{
  sMotorCont->MarkConnectionDirty();
}//end WrapperForMarkConnectionDirty()

void MotorController::Iterate()
{
  mcInput = GetPressure(mcInputPin);
  
  if (mcBoxConnectionDirty)
  {
    PID_PotBox.AttemptReconnect(&mcBoxIsConnected);
  }
  
  int intP;
  int intI;
  int intD;
  PID_PotBox.GetPID(&intP, &intI, &intD);
  mcP = intP * PID_POT_SENSITIVITY;
  mcI = intI * PID_POT_SENSITIVITY;
  mcD = intD * PID_POT_SENSITIVITY;
  PID_Controller.SetTunings(mcP,mcI,mcD);
}//end Iterate()

bool MotorController::Calculate()
{
  PID_Controller.Compute();
  analogWrite(mcOutputPin,mcOutput);
}//end Calculate()

double MotorController::GetPressure(int pin)
{
  mcInputTrack++;
  double total = 0;
  mcInputArray[(mcInputTrack%INPUT_AVG_COUNT)] = analogRead(pin);
  for(int i=0;i<INPUT_AVG_COUNT; i++)
  {
    total += mcInputArray[i];
  }
  return ((double)total/(double)INPUT_AVG_COUNT)*(ANALOG_TO_VOLTAGE*PRESSURE_SENSITIVITY)-PRESSURE_INTERCEPT;
}//end GetPressure(int)

void MotorController::SetSetpoint(double newSet)
{
  mcSetpoint = newSet;
}//end SetSetpoint(double)

void MotorController::AddToSetpoint(double addition)
{
  mcSetpoint = mcSetpoint + addition;
}//end AddToSetpoint(double)

double MotorController::GetP(){return mcP;}
double MotorController::GetI(){return mcI;}
double MotorController::GetD(){return mcD;}
double MotorController::GetInput(){return mcInput;}
double MotorController::GetSetpoint(){return mcSetpoint;}
double MotorController::GetOutput(){return mcOutput;}
int MotorController::GetBoxIsConnected(){return mcBoxIsConnected;}
bool MotorController::GetBoxConnectionDirty(){return mcBoxConnectionDirty;}
