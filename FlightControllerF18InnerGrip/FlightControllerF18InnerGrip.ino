#include <Wire.h>


//#define DEBUG
//#define DEBUG_LVL2
//#define DEBUG_LVL3  //average values, and sums

#define MAXBUTTONS 11
#define MAXANALOG 3
#define ANALOGMAXSAMPLES 50

#define joystickXPin A3
#define joystickYPin A2
#define AntennaElevation_Pin  A0


#define cageUncagePin         2
#define dispenseFwdPin        7
#define dispenseAftPin        8
#define speedBrakeExtendPin   3
#define speedBrakeRetractPin  4
#define tdcDepress            A1

#define comsUpPin             10
#define comsLeftPin           13
#define comsRightPin          11
#define comsDownPin           12
#define comsDepress           9


#define XDirection    0
#define YDirection    1
#define AntennaElevationDirection  2

enum dataStream{CAGE_UNCAGE = 0, DISPENSE_FWD, DISPENSE_AFT, SPEEDBRAKE_EXTEND, SPEEDBRAKE_RETRACT, TDC_DEPRESS, 
                COMS_UP, COMS_RIGHT, COMS_DOWN, COMS_LEFT, COMS_DEPRESS} buttonDataStream;

uint8_t pinsArray[] = {cageUncagePin, dispenseFwdPin, dispenseAftPin, speedBrakeExtendPin, speedBrakeRetractPin, tdcDepress, comsUpPin, comsRightPin, comsDownPin, comsLeftPin, comsDepress};


String pinsName[] = {" Cage:"," DISP_FW:"," DISP_AFT:"," SB_EXT:"," SB_RET:"," TDC DEP:"," Coms_UP:", " Coms_Right:", " Coms_Down:", " Coms_Left:", " Coms_Depress:"};

uint8_t sizeofpinsArray = sizeof(pinsArray)/sizeof(pinsArray[0]);

volatile bool buttonState[2][MAXBUTTONS];
volatile uint16_t analogArray[MAXANALOG];


enum dataCommands{SENDDATA = 0, SETUP};
volatile dataCommands I2CCommand;

class thrustmasterStick
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


void updateAverageArray(uint16_t analogArray[MAXANALOG][ANALOGMAXSAMPLES],uint8_t analogChannel, uint16_t newValue)
{
  uint8_t i;

  for (i=ANALOGMAXSAMPLES-1;i>0;i--)
  {
    analogArray[analogChannel][i] = analogArray[analogChannel][i-1];

  }
  analogArray[analogChannel][0] = newValue;
}

uint16_t getArrayAverage(uint16_t analogArray[MAXANALOG][ANALOGMAXSAMPLES],uint8_t analogChannel)
{
  uint8_t i;
  uint32_t averageValue=0;

  for (i=0;i<ANALOGMAXSAMPLES;i++)
  {
    averageValue += analogArray[analogChannel][i];
  }
  averageValue = averageValue/ANALOGMAXSAMPLES;

  #ifdef DEBUG_LVL3
   Serial.print("Channel ");
   Serial.print(analogChannel);
   Serial.print(" AVG: ");
   Serial.println(averageValue);
  #endif
  return(averageValue);
 
}


void setup() {

  Wire.begin(4);
  Wire.onReceive(I2CreceiveHandler);
  Wire.onRequest(I2CrequestHandler);

  Serial.begin(115200);
  Serial.println("Inner Throttle Test");
 
  pinMode(comsUpPin, INPUT_PULLUP);
  pinMode(comsLeftPin, INPUT_PULLUP);
  pinMode(comsRightPin, INPUT_PULLUP);
  pinMode(comsDownPin, INPUT_PULLUP);
  pinMode(comsDepress, INPUT_PULLUP);
  pinMode(joystickXPin, INPUT_PULLUP);
  pinMode(joystickYPin, INPUT_PULLUP);
  pinMode(AntennaElevation_Pin, INPUT_PULLUP);
  pinMode(cageUncagePin, INPUT_PULLUP);
  pinMode(dispenseFwdPin, INPUT_PULLUP);
  pinMode(dispenseAftPin, INPUT_PULLUP);
  pinMode(speedBrakeExtendPin, INPUT_PULLUP);
  pinMode(speedBrakeRetractPin, INPUT_PULLUP);
  pinMode(tdcDepress, INPUT_PULLUP);



}
 int16_t adcValue;
void loop() 
{
  volatile uint16_t analogArrayAverage[MAXANALOG][ANALOGMAXSAMPLES];
  uint32_t serialShiftData;
  int i = 0;

  uint16_t intialoffset[3];
  const int8_t scaleFactor[]= {1, 1, 1};

  
  intialoffset[XDirection] = analogRead(joystickXPin);
  intialoffset[YDirection] = analogRead(joystickYPin);
  intialoffset[AntennaElevationDirection] = analogRead(AntennaElevation_Pin);     

  
{
  
    adcValue = analogRead(joystickXPin);
    //adcValue = scaleFactor[XDirection]*(adcValue - intialoffset[XDirection]);
    
    updateAverageArray(analogArrayAverage, XDirection, adcValue);
    analogArray[XDirection] = getArrayAverage(analogArrayAverage,XDirection);
    #ifdef DEBUG
    Serial.print("Joystick X: ");
    if (adcValue>=0)
    {
       Serial.print("+");
    }
    Serial.print(adcValue);
    #endif
    adcValue = analogRead(joystickYPin);
    //adcValue = scaleFactor[YDirection]*(adcValue - intialoffset[YDirection]);
    updateAverageArray(analogArrayAverage, YDirection, adcValue);
    analogArray[YDirection] = getArrayAverage(analogArrayAverage,YDirection);

    #ifdef DEBUG
    Serial.print(" Joystick Y: ");
    if (adcValue>=0)
    {
       Serial.print("+");
    }
    Serial.print(adcValue);
    #endif
    adcValue = analogRead(AntennaElevation_Pin);
    //adcValue = scaleFactor[AntennaElevationDirection]*(adcValue - intialoffset[AntennaElevationDirection]);
    updateAverageArray(analogArrayAverage, AntennaElevationDirection, adcValue);
    analogArray[AntennaElevationDirection] = getArrayAverage(analogArrayAverage,AntennaElevationDirection);

    
    #ifdef DEBUG
    Serial.print(" Antenna Elevation: ");
    if (adcValue>=0)
    {
       Serial.print("+");
    }
    Serial.print(adcValue);
    #endif

    //Read in button states
    for (i=0;i<sizeofpinsArray;i++)
    {
      buttonState[0][i] = !digitalRead(pinsArray[i]);

    #ifdef DEBUG
      Serial.print(pinsName[i]);
      Serial.print(buttonState[0][i]);
    #endif

      if(buttonState[1][i] != buttonState[0][i])
      {
        //Something changed
          if(buttonState[0][i] == 1)
          {
            //This is the change to ON
          }else
          {
            //This is a change to OFF
          }

         buttonState[1][i] = buttonState[0][i];
      }
    }

    
    #ifdef DEBUG
      Serial.println("");
    #endif
  }

}


void I2CreceiveHandler(int sizeofRequest)
{
  if(Wire.available()>0)
  {
    I2CCommand = Wire.read();
  }
}

void I2CrequestHandler()
{
  
  int i;
  uint8_t outArray[2];

  #ifdef DEBUG_LVL2
    Serial.println("RequestHandler");
  #endif
  

  //Send the analog Sticks
  for (i=0;i<MAXANALOG;i++)
  { 
    convertToBytes(analogArray[i],outArray);
    Wire.write(outArray[0]);
    Wire.write(outArray[1]);
  }

  //Send the buttons
  for (i=0;i<sizeofpinsArray;i++)
  {    
    Wire.write(buttonState[0][i]);
  }
}

void convertToBytes(uint16_t invalue, uint8_t *outArray)
{
  outArray[0] = (invalue&0x00FF);
  outArray[1] = (invalue&0xFF00)>>8;
}
