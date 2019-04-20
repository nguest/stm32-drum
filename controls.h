//--------- Controls ----------//

# define GROUNDED_PIN -1
ADCTouchSensor button0 = ADCTouchSensor(PA0, GROUNDED_PIN);
ADCTouchSensor button1 = ADCTouchSensor(PA1, GROUNDED_PIN);

//#define PLAY_PAUSE PC14
#define recordLEDPin PA8

const uint8_t buttonPins[10] = { 0, 0, 0, 0, 0, 0, 0, 0, PC14, PC15 };



//--------- Controls setup ----------//

void setupControls() {
  // button0.begin();
  // button1.begin();

  pinMode(buttonPins[8], INPUT_PULLUP);
  pinMode(buttonPins[9], INPUT_PULLUP);
  pinMode(recordLEDPin, OUTPUT);

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
    MODE = (MODE == 1) ? 0 : 1;
    Serial.print("MODE ");
    Serial.println(MODE);

  }
  buttonLast[8] = button[8];
  // RECORD
  if (button[9] == 0 && buttonLast[9] == 1) {
    RECORD = (RECORD == 1) ? 0 : 1;
    Serial.print("RECORD ");
    Serial.println(RECORD);
    digitalWrite(recordLEDPin, HIGH);
  }
  buttonLast[9] = button[9];


  //tempo = averageAnalogReadings(PA4) << 7;

  if(button0.read() > 40) {
    //Serial.println("trig");
    buttonTrigger |= B00000001;
  };
  if(button1.read() > 40) {
    //Serial.println("trig");
    buttonTrigger |= B00000010;
  }
    // else {
  // //   buttonTrigger = B00000000;
  // // }
  //mode = digitalRead(PA0);

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
