//--------- Controls ----------//

# define GROUNDED_PIN -1

ADCTouchSensor touch[8] = {
  ADCTouchSensor(PA0, GROUNDED_PIN),
  ADCTouchSensor(PA1, GROUNDED_PIN),
  ADCTouchSensor(PA2, GROUNDED_PIN),
  ADCTouchSensor(PA3, GROUNDED_PIN),
  ADCTouchSensor(PA4, GROUNDED_PIN),
  ADCTouchSensor(PA5, GROUNDED_PIN),
  ADCTouchSensor(PA6, GROUNDED_PIN),
  ADCTouchSensor(PA7, GROUNDED_PIN),
};

const uint8_t touchPins[8] = { PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7 };

//#define PLAY_PAUSE PC14
#define recordLEDPin PA8

const uint8_t buttonPins[10] = { 0, 0, 0, 0, 0, 0, 0, 0, PC14, PC15 };


//--------- Controls setup ----------//

void setupControls() {
// for (uint8_t i; i < 8; i++) {
//  ADCTouchSensor touch[i] = ADCTouchSensor(touchPins[i], GROUNDED_PIN);
// }
  touch[0].begin();
  touch[1].begin();
  touch[2].begin();
  touch[3].begin();
  touch[4].begin();
  touch[5].begin();
  touch[6].begin();
  touch[7].begin();


  pinMode(buttonPins[8], INPUT_PULLUP);
  pinMode(buttonPins[9], INPUT_PULLUP);
  pinMode(recordLEDPin, OUTPUT);
  digitalWrite(recordLEDPin, LOW);

};

//--------- button interrupt ------//

volatile bool buttonLast[10] = { 1 };
volatile bool button[10] = { 1 };
const int numReadings = 64;
volatile uint_fast8_t buttonTrigger = B00000000;
volatile uint_fast8_t playTrigger = 0;

int bDelay;

void controlInterrupt() {
  button[8] = digitalRead(buttonPins[8]);
  button[9] = digitalRead(buttonPins[9]);
  // if (button == 0 && buttonLast == 1) {
  //   buttonTrigger = B00000001;
  //   bDelay = 0;
  // }
  // if (button == 1 && bDelay > 10) {
  //   buttonTrigger = B00000000;
  // }
  // bDelay++;
  // buttonLast = button;

  // PLAY/PAUSE
  if (button[8] == 0 && buttonLast[8] == 1) {
    //if pressed on this tick;

    MODE = (MODE == 1) ? 0 : 1;
    RECORD = (MODE == 0) ? 0 : RECORD;
    digitalWrite(recordLEDPin, RECORD);
    Serial.print("MODE ");
    Serial.println(MODE);
    WriteSPI('m', MODE, 0);
  }
  buttonLast[8] = button[8];

  // RECORD
  if (button[9] == 0 && buttonLast[9] == 1) {
    RECORD = (RECORD == 1) ? 0 : 1;
    Serial.print("RECORD ");
    Serial.println(RECORD);
    WriteSPI('r', RECORD, 0);
    digitalWrite(recordLEDPin, RECORD);
  }
  buttonLast[9] = button[9];

  // TEMPO
  // if (button[9] == 0 && buttonLast[9] == 1) {
  //   MODE = (MODE == 2) ? 1 : 2;
  //   Serial.print("MODE ");
  //   Serial.println(MODE);
  //   WriteSPI('m', MODE, 0);
  //   //digitalWrite(recordLEDPin, HIGH);
  // }
  // buttonLast[9] = button[9];

}


void readTouch() {
  const uint16_t S = 50;
  if(touch[0].read() > S) {
    Serial.print("trig0 ");
    Serial.println(touch[0].read());
    buttonTrigger |= B00000001;
  }
  if(touch[1].read() > S) {
    Serial.print("trig1 ");
    Serial.println(touch[1].read());
    buttonTrigger |= B00000010;
  }
  if(touch[2].read() > S) {
    Serial.print("trig2 ");
    Serial.println(touch[2].read());
    buttonTrigger |= B00000100;
  }
  if(touch[3].read() > S) {
    Serial.print("trig3 ");
    Serial.println(touch[3].read());
    buttonTrigger |= B00001000;
  }
  if(touch[4].read() > S) {
    Serial.print("trig4 ");
    Serial.println(touch[4].read());
    buttonTrigger |= B00010000;
  }
  if(touch[5].read() > S) {
    Serial.print("trig5 ");
    Serial.println(touch[5].read());
    buttonTrigger |= B00100000;
  }
  if(touch[6].read() > S) {
    Serial.print("trig6 ");
    Serial.println(touch[6].read());
    buttonTrigger |= B01000000;
  }
    if(touch[7].read() > S) {
    Serial.print("trig7 ");
    Serial.println(touch[7].read());
    buttonTrigger |= B10000000;
  }
}

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total

int averageAnalogReadings(uint8_t inputPin) {
  total = total - readings[readIndex];
  readings[readIndex] = analogRead(inputPin);
  total = total + readings[readIndex];
  readIndex++;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  return total >> 6;
}

