// Program that takes the information from the actual throttle grip and creates the USB Joystick object on the 
// Arduino Leonardo or Arduino Micro based on the library from Matthew Heironimus.
//


#define DEBUG 0

#include "Joystick.h"
#include <SPI.h>

#define MAXBUTTONS 24
#define MAXJOYSTICKS 5
#define HALFPERIOD 25
#define SPICLOCKSPEED 25//in SPI data transfer in Hz

#define throttleInner_pin A6
#define throttleOuter_pin A3
#define dataPin  14 //TBD 
#define loadPin  10 //TBD
#define clockPin  15 //TBD
#define slaveSelectPin 10;

#define tdcXDirection 0
#define tdcYDirection 1
#define antennaElevationDirection 2
#define innerThrottleDirection 3
#define outerThrottleDirection 4

enum dataStream{CAGE_UNCAGE = 0, DISPENSE_FWD, DISPENSE_AFT, SPEEDBRAKE_EXTEND, SPEEDBRAKE_RETRACT, TDC_DEPRESS, 
                COMS_UP, COMS_RIGHT, COMS_DOWN, COMS_LEFT, COMS_DEPRESS} buttonDataStream;
                
const uint8_t useSPI = 1;

// Create Joystick
Joystick_ Joystick;

class throttleAssembly
{

  private:

  uint8_t loadPointer=0;
  
  public: 

  bool buttons[MAXBUTTONS];
  

  void setButtonValue(bool buttonValue, dataStream selectedButton)
  {
    buttons[selectedButton] = buttonValue;
  }

  bool getButtonValue(dataStream selectedButton)
  {
    return(buttons[selectedButton]);
  }
  
};

throttleAssembly throttleQuadrent;




void parallelLoad()
{
  digitalWrite(loadPin,LOW);
  delayMicroseconds(20);
  digitalWrite(loadPin,HIGH);
}

bool readValue()
{
  //Set the clock high
  digitalWrite(clockPin, HIGH);

  //Read the value
  return(digitalRead(dataPin));
}



// Set to true to test "Auto Send" mode or false to test "Manual Send" mode.
//const bool testAutoSendMode = true;
const bool testAutoSendMode = true;

const unsigned long gcCycleDelta = 1000;
const unsigned long gcAnalogDelta = 25;
const unsigned long gcButtonDelta = 500;
unsigned long gNextTime = 0;
unsigned int gCurrentStep = 0;

void testSingleButtonPush(unsigned int button)
{
  if (button > 0)
  {
    Joystick.releaseButton(button - 1);
  }
  if (button < 32)
  {
    Joystick.pressButton(button);
  }
}

void testMultiButtonPush(unsigned int currentStep) 
{
  for (int button = 0; button < 32; button++)
  {
    if ((currentStep == 0) || (currentStep == 2))
    {
      if ((button % 2) == 0)
      {
        Joystick.pressButton(button);
      } else if (currentStep != 2)
      {
        Joystick.releaseButton(button);
      }
    } // if ((currentStep == 0) || (currentStep == 2))
    if ((currentStep == 1) || (currentStep == 2))
    {
      if ((button % 2) != 0)
      {
        Joystick.pressButton(button);
      } else if (currentStep != 2)
      {
        Joystick.releaseButton(button);
      }
    } // if ((currentStep == 1) || (currentStep == 2))
    if (currentStep == 3)
    {
      Joystick.releaseButton(button);
    } // if (currentStep == 3)
  } // for (int button = 0; button < 32; button++)
}

void testXYAxis(unsigned int currentStep)
{
  int xAxis;
  int yAxis;
  
  if (currentStep < 256)
  {
    xAxis = currentStep - 127;
    yAxis = -127;
    Joystick.setXAxis(xAxis);
    Joystick.setYAxis(yAxis);
  } 
  else if (currentStep < 512)
  {
    yAxis = currentStep - 256 - 127;
    Joystick.setYAxis(yAxis);
  }
  else if (currentStep < 768)
  {
    xAxis = 128 - (currentStep - 512);
    Joystick.setXAxis(xAxis);
  }
  else if (currentStep < 1024)
  {
    yAxis = 128 - (currentStep - 768);
    Joystick.setYAxis(yAxis);
  }
  else if (currentStep < 1024 + 128)
  {
    xAxis = currentStep - 1024 - 127;
    Joystick.setXAxis(xAxis);
    Joystick.setYAxis(xAxis);
  }
}

void testZAxis(unsigned int currentStep)
{
  if (currentStep < 128)
  {
    Joystick.setZAxis(-currentStep);
  } 
  else if (currentStep < 256 + 128)
  {
    Joystick.setZAxis(currentStep - 128 - 127);
  } 
  else if (currentStep < 256 + 128 + 127)
  {
    Joystick.setZAxis(127 - (currentStep - 383));
  } 
}

void testHatSwitch(unsigned int currentStep)
{
  if (currentStep < 8)
  {
    Joystick.setHatSwitch(0, currentStep * 45);
  }
  else if (currentStep == 8)
  {
    Joystick.setHatSwitch(0, -1);
  }
  else if (currentStep < 17)
  {
    Joystick.setHatSwitch(1, (currentStep - 9) * 45);
  }
  else if (currentStep == 17)
  {
    Joystick.setHatSwitch(1, -1);
  }
  else if (currentStep == 18)
  {
    Joystick.setHatSwitch(0, 0);
    Joystick.setHatSwitch(1, 0);
  }
  else if (currentStep < 27)
  {
    Joystick.setHatSwitch(0, (currentStep - 18) * 45);
    Joystick.setHatSwitch(1, (8 - (currentStep - 18)) * 45);
  }
  else if (currentStep == 27)
  {
    Joystick.setHatSwitch(0, -1);
    Joystick.setHatSwitch(1, -1);
  }
}

void testThrottleRudder(unsigned int value)
{
  Joystick.setThrottle(value);
  Joystick.setRudder(value);
}

void testXYZAxisRotation(unsigned int degree)
{
  Joystick.setRxAxis(degree);
  Joystick.setRyAxis(degree);
  Joystick.setRzAxis(degree * 2);
}

void setup() {

  // Set Range Values
  Joystick.setXAxisRange(60, 807);
  Joystick.setYAxisRange(-127, 127);
  Joystick.setZAxisRange(-127, 127);
  Joystick.setRxAxisRange(60, 807);
  Joystick.setRyAxisRange(360, 0);
  Joystick.setRzAxisRange(0, 720);
  Joystick.setThrottleRange(0, 255);
  Joystick.setRudderRange(255, 0);
  
  Joystick.begin();

  Wire.begin();        // join i2c bus (address optional for master)
  
  Serial.begin(9600);
  Serial.println("Joystick Test");
  
  pinMode(dataPin, INPUT);
  pinMode(loadPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(throttleInner_pin, INPUT_PULLUP);
  pinMode(throttleOuter_pin, INPUT_PULLUP);
  pinMode(13, OUTPUT);



  
}
 int16_t adcValue;
void loop() 
{
  uint32_t serialShiftData;
  int i = 0;

  uint16_t intialoffset[MAXJOYSTICKS];
  const int8_t scaleFactor[]= {1,1,1,1,1};

  
  intialoffset[innerThrottleDirection] = 0;
  intialoffset[outerThrottleDirection] = 0;

  
while(true)
{
  
    //Read the two throttle quandrant positions.
    
    adcValue = analogRead(throttleInner_pin);
    //Convert
    adcValue = scaleFactor[innerThrottleDirection]*(adcValue - intialoffset[innerThrottleDirection]);
    Joystick.setRxAxis(adcValue);
    #ifdef DEBUG
    Serial.print("Throttle Inner: ");
    if (adcValue>=0)
    {
       Serial.print("+");
    }
    Serial.print(adcValue);
    #endif
    
    adcValue = analogRead(throttleOuter_pin);
    adcValue = scaleFactor[outerThrottleDirection]*(adcValue - intialoffset[outerThrottleDirection]);
    Joystick.setRyAxis(adcValue);
    #ifdef DEBUG
    Serial.print("Throttle Outer: ");
    if (adcValue>=0)
    {
       Serial.print("+");
    }
    Serial.print(adcValue);
    #endif

    //Get the button states and other analog values from the grip

    //Add in the last two buttons, and the light toggle switch.



    
    #ifdef DEBUG
      Serial.println("");
    #endif
    
    #ifndef DEBUG
    Joystick.sendState();
    #endif
    //  Turn indicator light on.
    digitalWrite(13, 1);
  }

}
