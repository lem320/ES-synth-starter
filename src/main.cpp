#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>
#define operation_mode 0 //sender, used operation mode 1 for reciever. As a sender, it should send the note numbers, octaves and if its pressed or released, but not actually play a sound. 
                        // as a reciever, it should do be able to recieve information from the sender and its own keys, and play sounds accordingly.

//Constants
  const uint32_t interval = 100; //Display update interval

  const double hz = 440;
  const double freq_diff = pow(2, (1.0 / 12.0));
  const double a = pow(2.0, 32) / 22000;
  const char *notes[] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
  const float interuptFreq = 22000;

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

//CAN
volatile uint8_t TX_Message[8] = {0};
uint8_t RX_Message[8]={0};
uint8_t keypressrelease[4] = {0} ;
uint8_t keyboard_location = 0;
uint32_t octaveindex;
uint32_t octaveadd;
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;
SemaphoreHandle_t CAN_TX_Semaphore;


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

const float envelopeGradient = (1.0/interuptFreq);

class Note {
  public:
    Note() {};
    Note(int power) {
      stepSize = hz * pow(freq_diff, power) * a;
      phaseAcc = 0;
      isPressed = false;
      envelope = 0;
      envelopeState = 0;
    }
    
    uint32_t incrementAndGetPhaseAcc() {
      setEnvelope();
      if (envelope > 0) phaseAcc += stepSize;
      else phaseAcc = 0;
      return phaseAcc*envelope;
    }

    bool getIsPressed() { return isPressed || (envelope > 0); }

    void pressed() { __atomic_store_n(&isPressed, true, __ATOMIC_RELAXED); }
    void released() { __atomic_store_n(&isPressed, false, __ATOMIC_RELAXED); }

  private:
    uint32_t stepSize;
    uint32_t phaseAcc;
    bool isPressed;
    float envelope;
    int envelopeState; // 0 is initial increasing phase, 1 is first decreasing phase, 2 is constant phase and 3 is the final decreasing phase


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
        changeEnvelope(-1);
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
        notes[i] = new Note (i-9 + (octave)*12);
    }

    uint32_t getNextTotalPhaseAcc() {
      int totalPressed = 0;
      uint32_t totalPhaseAcc = 0;
      for (int i=0; i<12; i++) {
      
        if (notes[i]->getIsPressed()) 
          totalPressed++;
          totalPhaseAcc += notes[i]->incrementAndGetPhaseAcc();
        
      }
      // Serial.println(totalPhaseAcc);
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

Octave octave[8] {0,1,2,4,5,6,7};


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
  int32_t Vout = (octave[0].getNextTotalPhaseAcc() >> 24) - 128;
  Vout = Vout >> (8 - knobs[3].getRotation());
  analogWrite(OUTR_PIN, Vout + 128);
}
// void sampleISR() {
  
//   uint32_t count;
//   uint32_t Vout;
  
//   for(int i = 0; i<1; i++){
//     octaveindex = octave[i].getNextTotalPhaseAcc();
//     if(octaveindex > 0){
//       octaveadd = octaveindex + octaveadd;
//       count++;
//       // Serial.println(octaveindex);
//     }
//   }
//   Vout = (octaveadd >> 24) - 128;
//   Vout = Vout >> (8 - knobs[3].getRotation());
//  // Serial.println(Vout);
  
//   analogWrite(OUTR_PIN, Vout + 128);
// }

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
  //if key is releasd, then set current step size to zero 
  
  for (int i = 3; i < 6; i++){

    // for (int j=0; j<4; j++){            
    //     if (!(RX_Message[i] & (1 << j))){
    //       octave[RX_Message[1]].pressNote(((i-3)*4)+j); 
    //     }
    //     else {

    //       octave[RX_Message[1]].releaseNote(((i-3)*4)+j);
    //     }
                
    // }

  }
  }
}

void scanKeysTask(void * pvParameters) {
  int32_t phaseAcc[5];
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int8_t keyxor;
  
  int notenumber = 0; 
  uint8_t txmessage_nonvolatile[8] = {0,0,0,0,0,0,0,0};
  uint8_t boardlocation;

  while (1) {    
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);

     uint8_t outBits[] = {0,0,0,1,1,1,1}; 
    

    for (uint8_t i=0; i<7; i++) {
      setRow(i);
      digitalWrite(OUT_PIN,outBits[i]); //Set value to latch in DFF
      digitalWrite(REN_PIN,1);          //Enable selected row
      delayMicroseconds(3);
      uint8_t keyArrayPrev = keyArray[i];
      uint8_t recieved_keypress;
      keyArray[i] = readCols();
      digitalWrite(REN_PIN,0);          //Disable selected row
      keyxor = keyArrayPrev ^ keyArray[i];
      recieved_keypress = RX_Message[2];
      
      
      switch (i) {
        case 0: case 1: case 2: 
        
          for (int j=0; j<4; j++){

            

            if(!(keyArray[5] & (1 << 3)) ){    //if dectected keyboard is west, set boardloction to 1
                boardlocation = 1;
                //Serial.println("west");
              
              }        
              else if(!(keyArray[6] & (1 << 3))){   // if detected keyboard is east, set boardloction to 2
               boardlocation = 0 ;
               //Serial.println("east");
            }
            

            

            if (!(keyArray[i] & (1 << j))){
              keypressrelease[j] = 1;
              
              octave[boardlocation].pressNote((i*4)+j);
              notenumber = (i*4)+j;
            }
            else {
              delayMicroseconds(3);
              octave[boardlocation].releaseNote((i*4)+j);
              keypressrelease[j] = 0;
            }
            
          } 
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

          // case 5: case 6:
          // for (int j = 3; j < 4; j++){
          //   if((i == 5) & (j == 3) & !(keyArray[i] & (1 << j)) ){    //if dectected keyboard is west, set boardloction to 1
          //       boardlocation = 1;
          //       Serial.println("west");
              
          //     }        
          //     else if((i == 6) & (j == 3) & !(keyArray[i] & (1 << j))){   // if detected keyboard is east, set boardloction to 2
          //      boardlocation = 0 ;
          //      Serial.println("east");
          //   }
          // }

          // break;


      }

      if(keyxor != 0){
            uint8_t jIndex = 'R';
            for (int j=0; j<4; j++){
              if (keypressrelease[j] == 1){
                jIndex = 'P';
              }
            }

            //Serial.println(jIndex);

            TX_Message[0] = jIndex;
            TX_Message[1] = 0;
            TX_Message[2] = notenumber;
            TX_Message[3] = keyArray[0];
            TX_Message[4] = keyArray[1];
            TX_Message[5] = keyArray[2];
            TX_Message[6] = keyboard_location;
            txmessage_nonvolatile[0] = TX_Message[0];
            txmessage_nonvolatile[1] = TX_Message[1];
            txmessage_nonvolatile[2] = TX_Message[2];
            txmessage_nonvolatile[3] = TX_Message[3];
            txmessage_nonvolatile[4] = TX_Message[4];
            txmessage_nonvolatile[5] = TX_Message[5];
            txmessage_nonvolatile[6] = TX_Message[6];

            xQueueSend( msgOutQ, txmessage_nonvolatile, portMAX_DELAY);
            //CAN_TX(0x123, txmessage_nonvolatile);
            
            
          }
    }
    xSemaphoreGive(keyArrayMutex);
  }
}

void displayUpdateTask(void *pvParameters)
{
      const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
      TickType_t xLastWakeTime = xTaskGetTickCount();
      uint32_t ID;

      while (1)
      {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.setCursor(66,30);
        u8g2.print((char) RX_Message[0]);
        u8g2.print(RX_Message[1]);
        u8g2.print(RX_Message[2]);
        
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
  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);

  CAN_Init(true);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);
  setCANFilter(0x123,0x7ff);
  CAN_Start();

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
  sampleTimer->setOverflow(interuptFreq, HERTZ_FORMAT);
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
