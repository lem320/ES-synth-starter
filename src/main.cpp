#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>


// ------------ CONSTANTS ---------------

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


#define max_notes 1.0;
#define displayLength 5

const float interuptFreq = 22000;
const double hz = 440;
const double freq_diff = pow(2, (1.0 / 12.0));
const double a = pow(2.0, 32) / interuptFreq;
const char *notes[] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };

const float envelopeGradient = (1.0/interuptFreq);
const double b = pow(2.0, 31);


//CAN
volatile uint8_t TX_Message[8] = {0};
uint8_t RX_Message[8]={0};
uint8_t keypressrelease[4] = {0} ;
uint32_t octaveindex;
uint32_t octaveadd;
uint8_t boardlocation;
uint8_t firstkeyboard;
uint8_t secondkeyboard;


volatile uint8_t threekeyboards_check = 0;

QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;
SemaphoreHandle_t CAN_TX_Semaphore;


// ------------ DATA STRUCTURES ---------------

uint32_t sine_table[5001];

class Note {
  public:
    Note() {};
    Note(int power) {
      for (int i=0; i<3; i++) {
        frequencies[i] = hz*pow(freq_diff, power+(i-1)*12);
        periods[i] = 1/frequencies[i];
        stepSizes[i] = frequencies[i] * a;
      }
      setTone(-1);
      phaseAcc = 0;
      isPressed = false;
      envelope = 0;
      envelopeState = 0;
      envelopeActive = false;
      time = 0;
      wave = 0;
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
    void changeWave(int pWave) {
      wave = pWave;
    } 

    void setTone(int octave) {
      stepSize = stepSizes[octave+1];
      frequency = frequencies[octave+1];
      period = periods[octave+1];
    }

  private:
    float frequencies[3];
    float periods[3];
    uint32_t stepSizes[3];

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
      if (wave == 0) phaseAcc += stepSize;
      else if (wave == 2) {
        phaseAcc = sine_table[(int)(frequency*time*50)];
        time += envelopeGradient;
        if (time >= period) time = 0;
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
    Octave(int pOctave) {
      octave = pOctave;
      for (int i=0; i<12; i++) 
        notes[i] = new Note (i-9);
    }

    uint32_t getNextTotalPhaseAcc() {
      int totalPressed = 0;
      uint32_t totalPhaseAcc = 0;
      for (int i=0; i<12; i++) 
        if (notes[i]->getIsPressed()) 
          totalPressed++;
      for (int i=0; i<12; i++) 
        if (notes[i]->getIsPressed()) 
          totalPhaseAcc += notes[i]->incrementAndGetPhaseAcc() / totalPressed;

      return (uint32_t)(totalPhaseAcc);
    }

    void envelopeActiveOctave(bool v) {
      for (int i=0; i<12; i++) notes[i]->envelopeActiveNotes(v);
    }
    void changeWaveOctave(int pWave) {
      for (int i=0; i<12; i++) notes[i]->changeWave(pWave);
    }
    int getOctave() {
      return octave;
    }

    void pressNote(int noteIndex) { notes[noteIndex]->pressed(); }
    void releaseNote(int noteIndex) { notes[noteIndex]->released(); }

    void changeOctave(int pOctave) {
      octave = pOctave;
      Serial.println(octave);
      for (int i=0; i<12; i++) 
        notes[i]->setTone(octave); // +octave*12
    }

  private:
    Note* notes[12];
    int octave;
};

Octave octave (-1);

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

bool keyboardIsMaster = false;
int volumeSent = 0;

void changingLevel(int currentLevel, int type) {
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


// CAN FUNCITONS
void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

void CAN_TX_Task (void * pvParameters) {

	uint8_t msgOut[8];
	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
	}
}

void decodeTask(void * pvParameters){
  while(1) {
  xQueueReceive(msgInQ, RX_Message, portMAX_DELAY); 

  //need to recieve the command if there is a three board keyboard... need to be careful in the case that it isnt a three board keyboard, because there is a chance it will get stuck
  // threekeyboards_check = RX_Message[3];

  }
}




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
  int volume = (keyboardIsMaster) ? knobs[3].getRotation() : volumeSent;
  Vout = Vout >> (8 - volume);
  // Serial.println(Vout+128);
  analogWrite(OUTR_PIN, Vout + 128);
}

void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  int8_t keyxor;
  int notenumber = 0; 
  uint8_t txmessage_nonvolatile[8] = {0,0,0,0,0,0,0,0};

  bool leftKeyboard, rightKeyboard;
  // int threeKeyboardCheck = 0; // 0 means unknown, 1 means false and 2 means true


  while (1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    for (uint8_t i=0; i<7; i++) {
      setRow(i);
      delayMicroseconds(3);
      
      uint8_t keyArrayPrev = keyArray[i];
      keyArray[i] = readCols();

      // threeKeyboardCheck = RX_Message[3];

      switch (i) {
        case 0: case 1: case 2:
          for (int j=0; j<4; j++)
            if (!(keyArray[i] & (1 << j))) octave.pressNote(i*4+j);
            else octave.releaseNote(i*4+j);
          break;
        case 3: case 4:
          for (int j=0; j<2; j++) {
            if (
              ( (keyArrayPrev & (1 << 2*j)) ^ (keyArray[i] & (1 << 2*j)) ) == (1 << 2*j) &&
              ( (keyArrayPrev & (1 << (2*j+1))) ^ (keyArray[i] & (1 << (2*j+1))) ) == (1 << (2*j+1))
            ) knobs[ (i-4)*-2 + 1 + (j*-1) ].changeRotation(2);
            else if (( (keyArrayPrev & (1 << 2*j)) ^ (keyArray[i] & (1 << 2*j)) ) == (1 << 2*j))
              knobs[ (i-4)*-2 + 1 + (j*-1) ].changeRotation( (((keyArray[i] & (1 << 2*j)) >> 2*j) ^ ((keyArray[i] & (1 << (2*j+1))) >> (2*j+1))) ? 1 : -1 );
          }
          break;
        case 5:
          leftKeyboard = keyArray[i] & (1 << 3);
          break;
        case 6:
          rightKeyboard = keyArray[i] & (1 << 3);
          break;
      }
    }


    xSemaphoreGive(keyArrayMutex);

    int newMaster = 0;

    // __atomic_store_n(&keyboardIsMaster, false, __ATOMIC_RELAXED);


    if (leftKeyboard) __atomic_store_n(&keyboardIsMaster, true, __ATOMIC_RELAXED);
    else __atomic_store_n(&keyboardIsMaster, false, __ATOMIC_RELAXED);

    bool changed = false;

    if (!leftKeyboard && !rightKeyboard) {
      TX_Message[0] = 1;
      txmessage_nonvolatile[0] = TX_Message[0];
      
      TX_Message[1] = RX_Message[1];
      txmessage_nonvolatile[1] = TX_Message[1];
      TX_Message[2] = RX_Message[2];
      txmessage_nonvolatile[2] = TX_Message[2];
      TX_Message[3] = RX_Message[3];
      txmessage_nonvolatile[3] = TX_Message[3];
      changed = true;
    }

    if (keyboardIsMaster & !rightKeyboard) {
      TX_Message[0] = RX_Message[0];
      txmessage_nonvolatile[0] = TX_Message[0];

      TX_Message[1] = knobs[3].getRotation();
      txmessage_nonvolatile[1] = TX_Message[1];

      TX_Message[2] = knobs[2].getRotation();
      txmessage_nonvolatile[2] = TX_Message[2];

      TX_Message[3] = displayItems[0].getCurrentLevelInt();
      txmessage_nonvolatile[3] = TX_Message[3];

      changed = true;
    } else if (!leftKeyboard) {
      volumeSent = RX_Message[1];
      octave.changeWaveOctave(RX_Message[2]);
      changingLevel(RX_Message[3],0);

      // Serial.print("rx ");
      Serial.println(RX_Message[0]);
      int octaveSent = (!leftKeyboard && !rightKeyboard) ? 0 :
                         ( (RX_Message[0] == 0) ? 0 : 1 );
      if (octaveSent != octave.getOctave()) octave.changeOctave(octaveSent);
    }
    Serial.println(octave.getOctave());

    if (changed) xQueueSend( msgOutQ, txmessage_nonvolatile, portMAX_DELAY);



    // if (keyboardIsMaster && RX_Message[0] == 2 && (leftKeyboard || rightKeyboard)) {
    //   TX_Message[0] = 1;
    //   txmessage_nonvolatile[0] = TX_Message[0];
    // }

    // if (keyboardIsMaster && RX_Message[1] == 1) __atomic_store_n(&keyboardIsMaster, false, __ATOMIC_RELAXED);;

    // // If MIDDLE keyboard or no other keyboards attached, go straight to MASTER
    // if ( (!leftKeyboard && !rightKeyboard) || (leftKeyboard && rightKeyboard) ) {
    //   __atomic_store_n(&keyboardIsMaster, true, __ATOMIC_RELAXED);
    //   newMaster = 1;
    // }
    // else {
    //   int threeKeyboardCheck = RX_Message[0];

    //   // If threeKeyboardCheck uninitialised, wait and see if a MIDDLE keyboard sets it
    //   if (threeKeyboardCheck == 0) {
    //     delayMicroseconds(100000);
    //     threeKeyboardCheck = RX_Message[0];
    //   }
    //   // If threeKeyboardCheck still uninitialised or if there are not three keyboards and we are the left-most keyboard, we are MASTER
    //   if (threeKeyboardCheck != 2 && leftKeyboard) {
        
    //     if (!keyboardIsMaster) newMaster = 1;
    //     __atomic_store_n(&keyboardIsMaster, true, __ATOMIC_RELAXED);
    //   }

    // }

    // // Serial.print(" ");
    // // Serial.print(keyboardIsMaster);
    // // Serial.print(" ");
    // // Serial.print(leftKeyboard);
    // // Serial.print(" ");
    // // Serial.print(rightKeyboard);
    // // Serial.print(" ");
    // // Serial.print(RX_Message[0]);
    // // Serial.print(" ");
    // // Serial.println(RX_Message[1]);


    // // if (!keyboardIsMaster) {
    // //   if (threeKeyboardCheck == 0) delayMicroseconds(1000);
    // //   if (RX_Message[0] == 0 && !rightKeyboard) keyboardIsMaster = true;
    // // }


    // if (RX_Message[1] == 1 && keyboardIsMaster) __atomic_store_n(&keyboardIsMaster, true, __ATOMIC_RELAXED);

    // if (leftKeyboard && rightKeyboard) {
    //   // CAN_Init(false);
    //   if (keyboardIsMaster) {
    //     if (!leftKeyboard && !rightKeyboard) {
    //       TX_Message[0] = 2;
    //       txmessage_nonvolatile[0] = TX_Message[0];
    //     } else {
    //       TX_Message[0] = 1;
    //       txmessage_nonvolatile[0] = TX_Message[0];
    //     }
    //     TX_Message[1] = newMaster;
    //     txmessage_nonvolatile[1] = TX_Message[1];
    //     TX_Message[2] = knobs[3].getRotation();
    //     txmessage_nonvolatile[2] = TX_Message[2];

    //     xQueueSend( msgOutQ, txmessage_nonvolatile, portMAX_DELAY);
    //   } else {
    //     volumeSent = RX_Message[2];

    //     int octaveSent = (!leftKeyboard && !rightKeyboard) ? 0
    //                         : ( (!leftKeyboard) ? 1 : -1 );
    //     if (octaveSent != octave.getOctave()) octave.changeOctave(octaveSent);
    //   }
    // } else {
    //   // CAN_Init(true);
    // }

  }
}
int currentWave = 0;
void displayUpdateTask(void *pvParameters)
{
      const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
      TickType_t xLastWakeTime = xTaskGetTickCount();

      while (1)
      {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        u8g2.clearBuffer();
        if (keyboardIsMaster) {
          


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
          if (wave != currentWave) {
            octave.changeWaveOctave(wave);
            currentWave = wave;
          }
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

          digitalToggle(LED_BUILTIN);
        }

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
      }
}
void setup() {
  // CAN
  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);

  CAN_Init(false);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);
  setCANFilter(0x123,0x7ff);
  CAN_Start();

  // fill_sintable();
  for (int index = 0; index < 5001; index++) {
    sine_table[index] = b*(sin(2*PI * (index / 5000.0) * 100)+1); // freq = 100
  }

  // Init display items
  for (int i=0; i<displayLength; i++){
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
    4,			/* Task priority */
    &scanKeysHandle );  /* Pointer to store the task handle */

  TaskHandle_t decodeTaskHandle  = NULL;
  xTaskCreate(
    decodeTask ,		/* Function that implements the task */
    "decodeTask ",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority */
    &decodeTaskHandle);  /* Pointer to store the task handle */    

    TaskHandle_t CAN_TX_TaskHandle = NULL;
  xTaskCreate(
    CAN_TX_Task,		/* Function that implements the task */
    "CAN_TX_queue",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    3,			/* Task priority */
    &CAN_TX_TaskHandle );  /* Pointer to store the task handle */

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
