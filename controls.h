//--------- Controls ----------//

# define GROUNDED_PIN -1
ADCTouchSensor button0 = ADCTouchSensor(PA0, GROUNDED_PIN);
ADCTouchSensor button1 = ADCTouchSensor(PA1, GROUNDED_PIN);

// pinMode(PA5, INPUT_PULLUP);
// pinMode(PA4, INPUT_ANALOG);


//--------- Controls setup ----------//

void setupControls() {
  button0.begin();
  button1.begin();
};

//--------- button interrupt ------//

bool buttonLast = 1;
bool button = 1;
const int numReadings = 64;
volatile uint_fast8_t buttonTrigger = B00000000;


int bDelay;

void controlInterrupt() {
  button = digitalRead(PA5);
  if (button == 0 && buttonLast == 1) {
    buttonTrigger = B00000001;
    bDelay = 0;
  }
  if (button == 1 && bDelay > 10) {
    buttonTrigger = B00000000;
  }
  bDelay++;
  buttonLast = button;

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
