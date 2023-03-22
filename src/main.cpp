#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <cmath>


// ------------ CONSTANTS ---------------

#define max_notes 1.0;
#define displayLength 5

const uint32_t interval = 100; //Display update interval
const float interuptFreq = 1000;
const double hz = 440;
const double freq_diff = pow(2, (1.0 / 12.0));
const double a = pow(2.0, 32) / interuptFreq;
const char *notes[] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };

const float envelopeGradient = (1.0/interuptFreq);
const double b = pow(2.0, 31);

// Pin definitions
// Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

//Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

//Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

//Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

//Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;
uint8_t modeDecrease;
uint32_t tempphaseAcc;


// ------------ DATA STRUCTURES ---------------

uint32_t sine_table[501];

class Note {
  public:
    Note() {};
    Note(int power) {
      frequency = hz*pow(freq_diff, power);
      period = 1/frequency;
      stepSize = frequency * a;
      phaseAcc = 0;
      isPressed = false;
      envelope = 0;
      envelopeState = 0;
      envelopeActive = false;
      time = 0;
      wave = 3;
    }
    
    uint32_t incrementAndGetPhaseAcc() {
      if (isPressed || (envelope > 0 && envelopeActive)) nextPhaseAcc();
      else {
        phaseAcc = 0;
        time = 0;
      }
      if (envelopeActive) {
        setEnvelope();
        return phaseAcc*envelope;
      }
      else return phaseAcc;
    }

    bool getIsPressed() { return isPressed || (envelope > 0 && envelopeActive); }
    float getEnvelope() { return envelope; }

    void pressed() { __atomic_store_n(&isPressed, true, __ATOMIC_RELAXED); }
    void released() { __atomic_store_n(&isPressed, false, __ATOMIC_RELAXED); }

    void envelopeActiveNotes(bool v) {
      envelopeActive = v;
    }

  private:
    uint32_t stepSize;
    float time;
    float frequency;
    float period;
    int wave; // 0 is sawtooth, 1 is square and 2 is sine

    uint32_t phaseAcc;
    bool isPressed;
    float envelope;
    int envelopeState; // 0 is initial increasing phase, 1 is first decreasing phase, 2 is constant phase and 3 is the final decreasing phase
    bool envelopeActive;

    void nextPhaseAcc() {
      uint32_t oldphaseAcc;
      

      if (wave == 0) phaseAcc += stepSize;
      else if (wave == 2) {
        phaseAcc = sine_table[(int)(frequency*time*500)];
        time += envelopeGradient;
      
        if (time >= period) time = 0;
      }

      else if (wave == 1) { 
        
        if ((phaseAcc + stepSize > oldphaseAcc) && !(modeDecrease)) {

            
            oldphaseAcc = phaseAcc;
            phaseAcc += stepSize; // increasing phaseAcc until before it overflows  by reaching its maximum value. From this point, decrease phaseAcc

        }
        else {          
                              //once it is about to overflow, set, modeDecrease to 1 to change the state to decrease the phaseAcc.
          modeDecrease = 1;

        }
        if( (modeDecrease  == 1) && ((phaseAcc - stepSize) < (oldphaseAcc))) {    //if phaseAcc hasnt yet gone back down to zero, and modeDecrease is activated, decrease the phaseAcc.
          oldphaseAcc = phaseAcc;
          phaseAcc -= stepSize;
        }
        else{
            modeDecrease =  0; 
            //At this point the phaseAcc is set

        }


    }

  else if (wave == 3){
    

            
            //oldphaseAcc = tempphaseAcc;
            
            tempphaseAcc += stepSize; 
            phaseAcc  = (tempphaseAcc & (1 << 31));

 

  }

  }
    void changeEnvelope(float incr) {
      if (envelope <= 1 && envelope >= 0) envelope += incr*envelopeGradient;
      if (envelope > 1) envelope = 1;
      else if (envelope < 0) envelope = 0;
    }
    void setEnvelope() {
      if (envelopeState == 0 && isPressed && envelope >= 1)  envelopeState = 1;
      else if (envelopeState == 1 && isPressed && envelope <= 0.5) envelopeState = 2;
      else if (!isPressed) envelopeState = 3;
      else if (envelopeState == 3 && isPressed) envelopeState = 0;

      switch (envelopeState)
      {
        case 0:
          changeEnvelope(5);
          break;

        case 1: case 3:
          changeEnvelope(-2);
          break;
        
        default:
          break;
      }
    }
};

class Octave {
  public:
    Octave() {};
    Octave(int octave) {
      for (int i=0; i<12; i++) 
        notes[i] = new Note (i-9 + octave*12);
    }

    uint32_t getNextTotalPhaseAcc() {
      int totalPressed = 0;
      // float totalEnvelope = 0;
      uint32_t totalPhaseAcc = 0;
      for (int i=0; i<12; i++) 
        if (notes[i]->getIsPressed()) 
          totalPressed++;
      for (int i=0; i<12; i++) 
        if (notes[i]->getIsPressed()) {
          
          totalPhaseAcc += notes[i]->incrementAndGetPhaseAcc() / totalPressed;
          // totalEnvelope += notes[i]->getEnvelope();
        }
      // if (totalPhaseAcc > 0) Serial.println(totalPhaseAcc);
      return (uint32_t)(totalPhaseAcc);
    }

    void envelopeActiveOctave(bool v) {
      for (int i=0; i<12; i++) notes[i]->envelopeActiveNotes(v);
    }

    void pressNote(int noteIndex) { notes[noteIndex]->pressed(); }
    void releaseNote(int noteIndex) { notes[noteIndex]->released(); }

  private:
    Note* notes[12];
};

Octave octave (0);

class DisplayItem {
  public:
    DisplayItem() {}
    DisplayItem(char* pName, char* pLevels[2], int pCurrentLevel) {//std::function<void(int)> function) {
      name = pName;
      levels[0] = pLevels[0];
      levels[1] = pLevels[1];
      currentLevel = pCurrentLevel;
      // type = pType;
      // function(currentLevel);
    }

    int changeLevel(int inc) {
      currentLevel += inc;
      if (currentLevel > 1) currentLevel = 1;
      else if (currentLevel < 0) currentLevel = 0;
      // function(currentLevel);
      return currentLevel;
    }

    // char* getDisplayString() {
    //   char* out = new char[50];
    //   sprintf(out, "%s: %s",name,levels[currentLevel]);
    //   return out;
    // }

    char* getName() {
      return name;
    }
    char* getCurrentLevel() {
      return levels[currentLevel];
    }
    int getCurrentLevelInt() {
      return currentLevel;
    }

  private:
    char* name;
    char* levels[2];
    int currentLevel;
    // int type;
    // std::function<void(int)> function;

    // void envelopeActive(int level) {
    //   // Serial.println("hi");
      
    // }
    // void filteringActive(int level) {
    //   // octave.envelopeActiveOctave(level == 1);
    // }
};

class Knob {
  public:
    Knob() {}

    Knob(int pCurrentRotation, int pLowerLimit, int pUpperLimit) {
      currentRotation = pCurrentRotation;
      lowerLimit = pLowerLimit;
      upperLimit = pUpperLimit;
      currentTransition = 0;
    }

    int getRotation() {return currentRotation;}

    void changeRotation(int pTransition = 2) {
      if (pTransition != 2) currentTransition = pTransition;

      if (currentRotation < upperLimit && currentTransition == 1) __atomic_store_n(&currentRotation, currentRotation+currentTransition, __ATOMIC_RELAXED);
      else if (currentRotation > lowerLimit && currentTransition == -1)  __atomic_store_n(&currentRotation, currentRotation+currentTransition, __ATOMIC_RELAXED);;
    }

  private:
    int currentRotation;
    int currentTransition;
    int lowerLimit;
    int upperLimit;
};


// ------------ MAIN CODE ---------------


volatile uint8_t keyArray[7];
SemaphoreHandle_t keyArrayMutex;
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);
int currentMenuRotation = 0;

// Setip Knobs
Knob knobs[4] = {
  {0,0,displayLength-1},
  {0,0,10},
  {0,0,2},
  {3,0,8}
};


void changingLevel(int currentLevel, int type) {
  Serial.print(currentLevel);
  Serial.print(" ");
  Serial.println(type);
  switch (type) {
    case 0:
      octave.envelopeActiveOctave(currentLevel == 1);
      break;
    default:
      break;
  } //getCurrentLevel()
}

char* boolLevels[2] = {(char*)"Off",(char*)"On"};
DisplayItem displayItems[displayLength] = {
  {(char*)"Envelope",boolLevels,1},
  {(char*)"Filtering",boolLevels,0},
  {(char*)"Item 1",boolLevels,0},
  {(char*)"Item 2",boolLevels,0},
  {(char*)"Item 3",boolLevels,0}
};


//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

uint8_t readCols(){
  return (uint8_t)digitalRead(C0_PIN) | ((uint8_t)digitalRead(C1_PIN)<<1) | ((uint8_t)digitalRead(C2_PIN)<<2) | ((uint8_t)digitalRead(C3_PIN)<<3);
}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN,LOW);
  digitalWrite(RA0_PIN,(rowIdx & (1 << 0)) ? HIGH : LOW);
  digitalWrite(RA1_PIN,(rowIdx & (1 << 1)) ? HIGH : LOW);
  digitalWrite(RA2_PIN,(rowIdx & (1 << 2)) ? HIGH : LOW);
  digitalWrite(REN_PIN,HIGH);
}

void sampleISR() {
  int32_t Vout = (octave.getNextTotalPhaseAcc() >> 24) - 128;
  Vout = Vout >> (8 - knobs[3].getRotation());
   Serial.println(Vout+128);
  analogWrite(OUTR_PIN, Vout + 128);
}

void scanKeysTask(void * pvParameters) {

  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    for (uint8_t i=0; i<5; i++) {
      setRow(i);
      delayMicroseconds(3);
      
      uint8_t keyArrayPrev = keyArray[i];
      keyArray[i] = readCols();

      switch (i) {
        case 0: case 1: case 2:
          for (int j=0; j<4; j++)
            if (!(keyArray[i] & (1 << j))) octave.pressNote(i*4+j);
            else octave.releaseNote(i*4+j);
          break;
        case 3: case 4:
          // Serial.print("");
          for (int j=0; j<2; j++) {
            if (
              ( (keyArrayPrev & (1 << 2*j)) ^ (keyArray[i] & (1 << 2*j)) ) == (1 << 2*j) &&
              ( (keyArrayPrev & (1 << (2*j+1))) ^ (keyArray[i] & (1 << (2*j+1))) ) == (1 << (2*j+1))
            ) knobs[ (i-4)*-2 + 1 + (j*-1) ].changeRotation(2);
            else if (( (keyArrayPrev & (1 << 2*j)) ^ (keyArray[i] & (1 << 2*j)) ) == (1 << 2*j))
              knobs[ (i-4)*-2 + 1 + (j*-1) ].changeRotation( (((keyArray[i] & (1 << 2*j)) >> 2*j) ^ ((keyArray[i] & (1 << (2*j+1))) >> (2*j+1))) ? 1 : -1 );
          }
          break;
      }
    }
    xSemaphoreGive(keyArrayMutex);
  }
}

void displayUpdateTask(void *pvParameters)
{
      const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
      TickType_t xLastWakeTime = xTaskGetTickCount();

      while (1)
      {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        u8g2.clearBuffer();


        // VOLUME
        u8g2.setFont(u8g2_font_open_iconic_play_1x_t);
        int volume = knobs[3].getRotation();
        if (volume > 4) u8g2.drawUTF8(110,10,"\u004F");
        else if (volume == 0) u8g2.drawUTF8(108,10,"\u0051");
        else u8g2.drawUTF8(109,10,"\u0050");
        // char* volumeString = new char[8];
        // for (int i=0; i<volume; i++) strcat(volumeString,"|");
        // u8g2.drawStr(2,20,volumeString);

        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.setCursor(122,10);
        u8g2.print(volume);

        // WAVE TYPE
        int wave = knobs[2].getRotation();
        
        u8g2.setFont(u8g2_font_8x13_t_symbols);
        switch (wave) {
          case 0:
            u8g2.drawStr(115,20,"/");
            u8g2.drawStr(119,20,"|");
            break;
          case 1:
            u8g2.drawStr(108,21,"|");
            u8g2.drawStr(112,12,"_");
            u8g2.drawStr(115,21,"|");
            u8g2.drawStr(119,20,"_");
            u8g2.drawStr(122,21,"|");
            break;
          case 2:
            u8g2.drawUTF8(111,20,"\u25E0");
            u8g2.drawUTF8(119,20,"\u25E1");
            break;
          default:
            break;
        }
        // u8g2.drawUTF8(111,20,"\u25E0");
        // u8g2.drawUTF8(119,20,"\u25E1");

        u8g2.setFont(u8g2_font_6x10_tf);

        // MENU
        int indexes[2];
        int cursor;
        int menuIndex = knobs[0].getRotation();
        int incIndex = knobs[1].getRotation();

        if (currentMenuRotation != incIndex) {
          int newLevel = displayItems[menuIndex].changeLevel(incIndex - currentMenuRotation);
          changingLevel(newLevel,menuIndex);
          currentMenuRotation = incIndex;
        }

        if (menuIndex == 0) {
          indexes[0] = 0;
          indexes[1] = 1;
          // indexes[2] = 2;
          cursor = 1;
        } else if (menuIndex == displayLength-1) {
          indexes[0] = displayLength-2;
          indexes[1] = displayLength-1;
          // indexes[2] = displayLength-1;
          cursor = 2;
        } else {
          // indexes[0] = menuIndex-1;
          indexes[0] = menuIndex;
          indexes[1] = menuIndex+1;
          cursor = 1;
        }

        for (int i=0; i<2; i++) {
          u8g2.drawStr(12, 10*(i+1), displayItems[indexes[i]].getName());
          u8g2.drawStr(72, 10*(i+1), displayItems[indexes[i]].getCurrentLevel());
        }
        u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
        u8g2.drawUTF8(2,10*cursor,"\u0042");
        
        
        u8g2.setFont(u8g2_font_6x10_tf);

        int count = 0;
        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
        for (int i = 0; i < 3; i++)
        {
        
          for (int j = 0; j < 4; j++)
            if (!(keyArray[i] & (1 << j)))
            {
              u8g2.drawStr(2+15*count, 30, notes[(i * 4) + j]);
              // u8g2.setCursor(2+40*count, 30);
              count++;
            }
        }
        xSemaphoreGive(keyArrayMutex);

        u8g2.sendBuffer();
        digitalToggle(LED_BUILTIN);
      }
}

void setup() {
  // put your setup code here, to run once:

  // fill_sintable();
  for (int index = 0; index < 501; index++) {
    sine_table[index] = b*(sin(2*PI * index / 500.0)+1);
  }

  // Init display items
  for (int i=0; i<displayLength; i++){
    Serial.println(i);
    changingLevel(displayItems[i].getCurrentLevelInt(), i);
  }

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  u8g2.enableUTF8Print();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(interuptFreq, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  //Initialise UART
  Serial.begin(115200);
  Serial.println("Starting ...");



  keyArrayMutex = xSemaphoreCreateMutex();

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,		/* Function that implements the task */
    "scanKeys",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority */
    &scanKeysHandle );  /* Pointer to store the task handle */

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
    displayUpdateTask,		/* Function that implements the task */
    "displayUpdate",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &displayUpdateHandle );  /* Pointer to store the task handle */

  vTaskStartScheduler();

}

void loop() {
}
