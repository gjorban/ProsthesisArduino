//#define DEBUG_CBS 1
#include <Arduino.h>
#include <aJSON.h>
#include <command_processor.h>
#include "DeviceState.h"
#include "motor_state.h"

//Trivial change

//Handy wrapper for using stream-like serial printing
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }

void TransitionToState(DeviceStates toState);
void DeviceDisabledSlice();
void DeviceActiveSlice();
void DeviceFaultSlice();

const char* kArduinoID = "mcon";
const int kActiveLEDPin = 13;

aJsonStream mJSONSerialStream(&Serial);
unsigned long mLastTelem;
unsigned long mLastMillis;
unsigned long mActiveMillis;
int mTelemetryPeriodMS;
bool mTelemetryEnabled = false;

MotorState::tMotorControllerState mDeviceState;

void setup()
{
  CommandProcessor::SetTypeIDRequestCallback(IDRequestCallback);
  CommandProcessor::SetTelemetryRequestCallback(TelemetryEnableCb);
  CommandProcessor::SetEnableToggleRequestCallback(InitializeDeviceCb);
  Serial.begin(9600);
  
  //Initialize our device state
  mDeviceState.State = Uninitialized;
  for (int i = 0; i < NUM_MOTOR_CONTROLLERS; ++i)
  {
     mDeviceState.Currents[i] = 0.0f;
     mDeviceState.Millivolts[i] = 0;     
     mDeviceState.OutputPressure[i] = 0.0f;
     mDeviceState.LoadPressure[i] = 0.0f;
     mDeviceState.MotorDutyCycle[i] = 0.0f;     
  }
  mDeviceState.IsLoadSense = false;
  
  pinMode(kActiveLEDPin, OUTPUT);
  digitalWrite(kActiveLEDPin, LOW);
  
  mTelemetryPeriodMS = 50;
  mLastTelem = 0;
}

void loop()
{
   if (mJSONSerialStream.available()) 
   {
     /* First, skip any accidental whitespace like newlines. */
     mJSONSerialStream.skip();
   }

  if (mJSONSerialStream.available()) 
  {
     /* Something real on input, let's take a look. */
     aJsonObject *msg = aJson.parse(&mJSONSerialStream);
     CommandProcessor::ProcessMessage(msg);
     aJson.deleteItem(msg);    
  }

  //Soft realtime telemetry. Who cares about missed deadlines for these? The mission critical stuff goes into the interrupt CBs
  if (mTelemetryEnabled && millis() - mLastTelem > mTelemetryPeriodMS)
  {
    const char *deviceState = mDeviceState.State == Active ? "Enabled" : "Disabled";
#if DEBUG_CBS    
    Serial << "Telem. Diff is: " << millis() - mLastTelem << "ms. Device is " << deviceState << "\n";    
#endif    
    
    aJsonObject *msg = MotorState::MotorStateToJSON(mDeviceState);
    if (msg != NULL)
    {
      CommandProcessor::SendMessage(msg);
      aJson.deleteItem(msg);
    }
    mLastTelem = millis(); 
  }
  
  switch (mDeviceState.State)
  {
   case Disabled:
    DeviceDisabledSlice();
    break;
   
   case Active:
    DeviceActiveSlice();
    break;
   
   case Fault:
    DeviceFaultSlice();
    break; 
    
   default:
    break;
  }
}

aJsonObject * IDRequestCallback(aJsonObject *msg)
{
  aJsonObject *ackMsg = CommandProcessor::CreateCommandAckMessage();
  aJson.addItemToObject(ackMsg, CommandProcessor::PacketKeys::kArduinoID, aJson.createItem(kArduinoID));
  
  aJson.addItemToObject(ackMsg, CommandProcessor::PacketKeys::kTelemetryState, aJson.createItem(mTelemetryEnabled));
  aJson.addItemToObject(ackMsg, CommandProcessor::PacketKeys::kDeviceState, aJson.createItem(mDeviceState.State));
  return ackMsg;
}

aJsonObject * TelemetryEnableCb(aJsonObject *msg, bool enable)
{
  //No ACK necessary for this
  mTelemetryEnabled = enable;
  aJsonObject *telemPeriod = aJson.getObjectItem(msg, CommandProcessor::PacketKeys::kTelemetryPeriod);
  if (telemPeriod != NULL)
  {
     mTelemetryPeriodMS = telemPeriod->valueint;
#if DEBUG_CBS
     const char *enableText = mTelemetryEnabled ? "on" : "off";
     const char *deviceState = mDeviceState.State == Active ? "Enabled" : "Disabled";
     Serial << "Setting telemetry period to " << mTelemetryPeriodMS << "ms. Telemetry is " << enableText << ". Device is " << deviceState << "\n";
#endif
  }
  return NULL;
}

aJsonObject * InitializeDeviceCb(aJsonObject *msg, bool enable)
{
  aJsonObject *enableValue = aJson.getObjectItem(msg, CommandProcessor::PacketKeys::kEnable);
  
  if (enableValue != NULL && mDeviceState.State != Fault)
  {
    if (enableValue->valuebool)
    {
      TransitionToState(Active);
    }
    else
    {
      TransitionToState(Disabled);
    } 
  }
  
#if DEBUG_CBS
  Serial << "Device initialise " << enable << "\n";
#endif

  return NULL;
}

void TransitionToState(DeviceStates toState)
{
  //TODO: Perform state transitions here
  DeviceStates fromState = mDeviceState.State;
  aJsonObject *msg = NULL;
  bool successful = false;
  switch (fromState)
  {
  case Uninitialized:
    if (toState != Uninitialized)
    {
      successful = true; 
    }
    break;
    
  case Disabled:
    if (toState != Uninitialized)
    {
      successful = true; 
    }    
    break;
  
  case Active:
    if (toState != Uninitialized)
    {
      successful = true; 
    }  
    break;
    
  //No recovery from faults
  case Fault:
    break;
  } 
  
  if (successful && toState == Active)
  {
    digitalWrite(kActiveLEDPin, HIGH);    
  }
  else
  {
    digitalWrite(kActiveLEDPin, LOW);  
  }
  
  if (successful)
  {
    msg = aJson.createObject();
    aJson.addItemToObject(msg, CommandProcessor::PacketKeys::kCommandID, aJson.createItem(CommandProcessor::CommandIDs::kStateChange));
    aJson.addItemToObject(msg, CommandProcessor::PacketKeys::kChangeFrom , aJson.createItem(mDeviceState.State));
    aJson.addItemToObject(msg, CommandProcessor::PacketKeys::kChangeTo , aJson.createItem(toState));
    mDeviceState.State = toState;    
  }
  
  if (msg != NULL)
  {
    CommandProcessor::SendMessage(msg);
    aJson.deleteItem(msg);
  }
}

void DeviceDisabledSlice()
{
  mLastMillis = millis();
}

#define RAD_2_DEG(x) ((double)x * 180 / 3.141)

void DeviceActiveSlice()
{
  unsigned long currMillis = millis();
  mActiveMillis += currMillis - mLastMillis;
  
  for (int i = 0; i < NUM_MOTOR_CONTROLLERS; ++i)
  {
    mDeviceState.Currents[i] = constrain(cos(RAD_2_DEG(mActiveMillis) / 1000), 0, 1);
    mDeviceState.Millivolts[i] = constrain(cos(RAD_2_DEG(mActiveMillis + 45) / 1000), 0, 1) * 1000;
    
    mDeviceState.OutputPressure[i] = constrain(sin(RAD_2_DEG(mActiveMillis) / 1000), 0, 1);
    mDeviceState.FlowRate[i] = constrain(sin(RAD_2_DEG(mActiveMillis + 45) / 1000), 0, 1);    
  }
  
  mLastMillis = currMillis;
}

void DeviceFaultSlice()
{
  mLastMillis = millis();  
}

