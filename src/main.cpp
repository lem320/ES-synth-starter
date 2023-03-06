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
  const char *notes[] = {
      "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
  volatile int32_t currentStepSize;

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

      if (currentRotation < upperLimit && currentTransition == 1) __atomic_store_n(&currentRotation, currentRotation+pTransition, __ATOMIC_RELAXED);
      else if (currentRotation > lowerLimit && currentTransition == -1)  __atomic_store_n(&currentRotation, currentRotation+pTransition, __ATOMIC_RELAXED);;
    }

  private:
    int currentRotation;
    int currentTransition;
    int lowerLimit;
    int upperLimit;
};

//SETUP KNOBS
// Knob knobs[4] = {
//   {},
//   {},
//   {},
//   {4,0,8}
// };
Knob knob3 (4,0,8);

const double hz = 440;
const double freq_diff = pow(2,(1.0/12.0));
const double a = pow(2.0,32)/22000;
const int32_t stepSizes [] = {
  hz*pow(freq_diff,-9)*a,
  hz*pow(freq_diff,-8)*a,
  hz*pow(freq_diff,-7)*a,
  hz*pow(freq_diff,-6)*a,
  hz*pow(freq_diff,-5)*a,
  hz*pow(freq_diff,-4)*a,
  hz*pow(freq_diff,-3)*a,
  hz*pow(freq_diff,-2)*a,
  hz*pow(freq_diff,-1)*a,
  hz*a,
  hz*freq_diff*a,
  hz*pow(freq_diff,2)*a,
};
const char* notes [] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
volatile int32_t currentStepSize;

volatile uint8_t keyArray[7];
volatile int8_t knobRotation[4];
volatile int8_t knobRotationState[4];

SemaphoreHandle_t keyArrayMutex;

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

void setRow(uint8_t rowIdx){\
  digitalWrite(REN_PIN,LOW);
  digitalWrite(RA0_PIN,(rowIdx & (1 << 0)) ? HIGH : LOW);
  digitalWrite(RA1_PIN,(rowIdx & (1 << 1)) ? HIGH : LOW);
  digitalWrite(RA2_PIN,(rowIdx & (1 << 2)) ? HIGH : LOW);
  digitalWrite(REN_PIN,HIGH);
}

void sampleISR() {
  static int32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  Vout = Vout >> (8 - knob3.getRotation());
  analogWrite(OUTR_PIN, Vout + 128);
}


void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    // u8g2.clearBuffer();
    // u8g2.setFont(u8g2_font_ncenB08_tr);
    // u8g2.drawStr(2,10,"Hello World!");

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    float localCurrentStepSize = 0;
    for (uint8_t i=0; i<4; i++) {
      setRow(i);
      delayMicroseconds(3);
      
      uint8_t keyArrayPrev = keyArray[i];
      keyArray[i] = readCols();

      switch (i) {
        case 0: case 1: case 2:
          for (int j=0; j<4; j++)
            if (!(keyArray[i] & (1 << j)))
              localCurrentStepSize = stepSizes[(i*4)+j];
          break;
        case 3:
          Serial.print("");
          if (
            ( (keyArrayPrev & (1 << 0)) ^ (keyArray[i] & (1 << 0)) ) == 1 &&
            ( (keyArrayPrev & (1 << 1)) ^ (keyArray[i] & (1 << 1)) ) == 2
          ) knob3.changeRotation(2);
          else if (( (keyArrayPrev & (1 << 0)) ^ (keyArray[i] & (1 << 0)) ) == 1)
            knob3.changeRotation( ((keyArray[i] & (1 << 0)) ^ ((keyArray[i] & (1 << 1)) >> 1)) ? 1 : -1 );
          break;
      }


      
    }
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);


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
        u8g2.print(knob3.getRotation());

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

  // Default Volume
  // knobRotation[3] = 4;

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