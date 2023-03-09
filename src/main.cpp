#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>

//Constants
  const uint32_t interval = 100; //Display update interval

  const double hz = 440;
  const double freq_diff = pow(2, (1.0 / 12.0));
  const double a = pow(2.0, 32) / 22000;
  const int32_t stepSizes[] = {
      hz * pow(freq_diff, -9) * a,
      hz *pow(freq_diff, -8) * a,
      hz *pow(freq_diff, -7) * a,
      hz *pow(freq_diff, -6) * a,
      hz *pow(freq_diff, -5) * a,
      hz *pow(freq_diff, -4) * a,
      hz *pow(freq_diff, -3) * a,
      hz *pow(freq_diff, -2) * a,
      hz *pow(freq_diff, -1) * a,
      hz *a,
      hz *freq_diff *a,
      hz *pow(freq_diff, 2) * a,
  };
  const char *notes[] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
  volatile int32_t currentStepSize;
  // int32_t phaseAcc[10] = {0,0,0,0,0,0,0,0,0,0};
  volatile int32_t phaseAccTotal;

  volatile uint8_t keyArray[7];

  SemaphoreHandle_t keyArrayMutex;

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

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

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

class Note {
  public:
    Note() {};
    Note(int power) {
      stepSize = hz * pow(freq_diff, power) * a;
      phaseAcc = 0;
      isPressed = false;
    }
    
    int32_t incrementAndGetPhaseAcc() {
      if (isPressed) phaseAcc += stepSize;
      else phaseAcc = 0;

      return phaseAcc;
    }

    bool getIsPressed() { return isPressed; }

    void pressed() { __atomic_store_n(&isPressed, true, __ATOMIC_RELAXED); }
    void released() { __atomic_store_n(&isPressed, false, __ATOMIC_RELAXED); }

  private:
    int32_t stepSize;
    int32_t phaseAcc;
    bool isPressed;
};

class Octave {
  public:
    Octave() {};
    Octave(int octave) {
      for (int i=0; i<12; i++) 
        {notes[i] = new Note (i-9 + octave*12);}
    }

    int32_t getTotalPhaseAcc() {
      int totalPressed = 0;
      int32_t totalPhaseAcc = 0;
      for (int i=0; i<12; i++) 
        if (notes[i]->getIsPressed()) {
          totalPressed++;
          totalPhaseAcc += notes[i]->incrementAndGetPhaseAcc();
        }
      return totalPhaseAcc/totalPressed;
    }

    void pressNote(int noteIndex) { notes[noteIndex]->pressed(); }
    void releaseNote(int noteIndex) { notes[noteIndex]->released(); }

  private:
    Note* notes[12];
};

// Setip Knobs
Knob knobs[4] = {
  {0,0,10},
  {0,0,10},
  {0,0,10},
  {4,0,8}
};

Octave octave (0);

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

int32_t phaseAcc2 = 0;
void sampleISR() {
  int32_t Vout = (octave.getTotalPhaseAcc() >> 24) - 128;
  Vout = Vout >> (8 - knobs[3].getRotation());
  analogWrite(OUTR_PIN, Vout + 128);
}


void scanKeysTask(void * pvParameters) {
  int32_t phaseAcc[5];

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
          Serial.print("");
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
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(2, 10, "Hello World!");
        
        u8g2.setCursor(2,20);
        u8g2.print(knobs[0].getRotation());
        u8g2.setCursor(12,20);
        u8g2.print(knobs[1].getRotation());
        u8g2.setCursor(22,20);
        u8g2.print(knobs[2].getRotation());
        u8g2.setCursor(32,20);
        u8g2.print(knobs[3].getRotation());

        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
        for (int i = 0; i < 3; i++)
        {
        
          for (int j = 0; j < 4; j++)
            if (!(keyArray[i] & (1 << j)))
            {
              u8g2.drawStr(2, 30, notes[(i * 4) + j]);
              u8g2.setCursor(2, 30);
            }
        }
        xSemaphoreGive(keyArrayMutex);

        u8g2.sendBuffer();
        digitalToggle(LED_BUILTIN);
      }
}

void setup() {
  // put your setup code here, to run once:

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
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  //Initialise UART
  Serial.begin(9600);
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