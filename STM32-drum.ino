#include "CircularBuffer.h"
#include "wavetables16.h"
#include "GPIOWriteFast.h"
#include <ADCTouchSensor.h>
#include "patterns.h"
// #include <Adafruit_SH1106.h>


//--------- Display ----------//

PC_13 LED;
//Adafruit_SH1106 display(OLED_RESET);


//--------- Controls ----------//

# define GROUNDED_PIN -1
ADCTouchSensor button0 = ADCTouchSensor(PA0, GROUNDED_PIN);
ADCTouchSensor button1 = ADCTouchSensor(PA1, GROUNDED_PIN); 

//--------- Timers ----------//

#define AUDIO_CHANNEL_1_PIN PB1//PB8 - must relate to the timer chosen for PWM
#define AUDIO_PWM_TIMER 3//4
// The timer used for running the audio update loop. NOTE: Timer 3 appears to clash with SPI DMA transfers under some circumstances
#define AUDIO_UPDATE_TIMER 2
#define CONTROL_TIMER 4//3

HardwareTimer audioUpdateTimer(AUDIO_UPDATE_TIMER);
HardwareTimer audioPwmTimer(AUDIO_PWM_TIMER);
HardwareTimer controlTimer(CONTROL_TIMER);

#define AUDIO_BITS 12
#define HALF_AUDIO_BITS (1 << AUDIO_BITS)/2
#define MAX_CARRIER_FREQ (F_CPU / (1 << AUDIO_BITS)) // e.g 72M / 256 for 8 bit = 281250. 256 = 1 leftshift 8
#define AUDIO_RATE 22050



void setup() {
  Serial.begin(115200);

  //--------- Audio Update Timer setup ----------//

  // audio update timer is running at 16kHz or whatever, but audio_pwm_timer is the thing that is doing the output at/near full speed
  audioUpdateTimer.pause();
  //audio_update_timer.setPeriod(1000000UL / AUDIO_RATE);
  
  uint32_t prescaler = F_CPU/AUDIO_RATE/AUDIO_BITS;
  audioUpdateTimer.setPrescaleFactor(prescaler);
  uint32_t overflow = F_CPU/AUDIO_RATE/prescaler;
  audioUpdateTimer.setOverflow(overflow);
  
  audioUpdateTimer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  audioUpdateTimer.setCompare(TIMER_CH1, 1); // Interrupt 1 count after each update
  audioUpdateTimer.attachCompare1Interrupt(pwmAudioOutput);
  audioUpdateTimer.refresh();
  audioUpdateTimer.resume();

  //--------- Audio PWM Timer setup ----------//

  pinMode(AUDIO_CHANNEL_1_PIN, PWM);

  #if MAX_CARRIER_FREQ < (AUDIO_RATE * 5)
    // Generate as fast a carrier as possible
    audioPwmTimer.setPrescaleFactor(1);
  #else
    // No point in generating arbitrarily high carrier frequencies. In fact, if
    // there _is_ any headroom, give the PWM pin more time to swing from HIGH to LOW and BACK, cleanly
    audioPwmTimer.setPrescaleFactor((int)MAX_CARRIER_FREQ / (AUDIO_RATE * 4));

  #endif
  // Allocate enough room to write all intended bits
    audioPwmTimer.setOverflow(1 << AUDIO_BITS);

  //--------- Control Timer setup ----------//

  controlTimer.pause();
  //  buttonTimer.setPrescaleFactor(F_CPU/AUDIO_RATE/AUDIO_BITS);
  //  buttonTimer.setOverflow(S);
  controlTimer.setPeriod(40000);
  controlTimer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  controlTimer.setCompare(TIMER_CH1, 1); // Interrupt 1 count after each update
  controlTimer.attachCompare1Interrupt(controlInterrupt);
  controlTimer.refresh();
  controlTimer.resume();

  pinMode(PA5, INPUT_PULLUP);
  pinMode(PA4, INPUT_ANALOG);

  //--------- Display setup ----------//

  LED.pinMode(OUTPUT);

    // init OLED
  //display.begin(SH1106_SWITCHCAPVCC, 0x3C);

  // display.display();
  // delay(2000);
  // display.clearDisplay();

  //     display.setTextSize(1);
  // display.setTextColor(WHITE);
  // display.setCursor(0,0);
  // display.println("Hello, world!");

  //--------- Controls setup ----------//

  button0.begin();
  button1.begin();
    delay(1000);


}

//--------- Ringbuffer parameters ----------//

const uint8_t BUFFERSIZE = 256;
const uint8_t BUFFERSIZE_M1 = BUFFERSIZE - 1;
uint16_t Ringbuffer[256];
uint8_t RingWrite = 0;
uint8_t RingRead = 0;
volatile uint8_t RingCount = 0;

uint16_t sampleCount[NUM_SAMPLES];
uint16_t samplePointer[NUM_SAMPLES];

//--------- Sequencer/Play parameters ----------//

uint8_t MODE = 1;
volatile long tempo = 400000;
uint_fast8_t trigger = B00000000;
volatile uint_fast8_t buttonTrigger = B00000000;


//--------- button interrupt ------//

bool buttonLast = 1;
bool button = 1;
const int numReadings = 64;

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

  tempo = averageAnalogReadings(PA4) << 7;

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
//int average = 0;                // the average

int averageAnalogReadings(uint8_t inputPin) {
  total = total - readings[readIndex];
  readings[readIndex] = analogRead(inputPin);
  total = total + readings[readIndex];
  readIndex++;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  return total >> 6;
  //return average;
}

//--------- pwmAudioOutput ------//

void pwmAudioOutput() {
  if (RingCount) { //If entry in FIFO..
    pwmWrite(AUDIO_CHANNEL_1_PIN, Ringbuffer[RingRead++] >> 4); //Output LSB of 16-bit DAC
    RingCount--;
  }
}

//--------- Write Buffer and sequence ----------//

void play() {
  // everything here optimized for processing speed not code quality - inline faster?


  /* -------sample buffer write------------ */

  int32_t sampleTotal = 0; // has to be signed
  uint_fast8_t stepCount = 0;
  uint32_t tempoCount = 1;

  sampleCount[NUM_SAMPLES] = { 0 };

  while(1) {

    if (RingCount < BUFFERSIZE_M1) { // if space in ringbuffer
      sampleTotal = 0;
      for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
        if (sampleCount[i]) {
          int16_t sample = (wavetables16[i][samplePointer[i]++]) - (1 << 15);//) >> 8;
          sample = ((sample * gain[i]) >> 8);
          sampleTotal += sample; // 
          sampleCount[i] --;
        }      
      }    
     //  hard clip - must be faster than `constrain()`?
//      if (sampleTotal < -(1 << 15))
//        sampleTotal = -(1 << 15);
//      if (sampleTotal > (1 << 15))
//        sampleTotal = (1 << 15);
      Ringbuffer[RingWrite] = (sampleTotal + (1 << 15));
      RingWrite++;
      RingCount++;
    }

  /* -------LED------------------ */

    if (tempoCount >= 1 << 17) {
      fastWrite(LED, 0);
    } else {
      fastWrite(LED, 1);     
    }

  /* -------sequencer------------ */
 
    if (MODE == 1) {
      if (!(tempoCount--)) { // every "tempo" ticks, do the thing  
        tempoCount = tempo; // set it back to the tempo ticks

        trigger = livePattern[stepCount++];

        if (buttonTrigger != B00000000) {
          livePattern[stepCount] |= buttonTrigger;
        }

        if (stepCount > patternLength) stepCount = 0;
        // read the pattern bytes, each bit triggers a sample
        for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
          if (trigger & 1<<i) {

            samplePointer[i] = 0;
            sampleCount[i] = wavetableLengths16[i]; // number of bytes in sample
          }
        } 
      }
    }
    else {
      stepCount = 0;
      tempoCount = 1;
      for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
        if (buttonTrigger & 1<<i) {
          //Serial.println("trig");

          samplePointer[i] = 0;
          sampleCount[i] = wavetableLengths16[i]; // number of bytes in sample
        }
      }
    }
  }
/* ----------------------------- */
}

//--------- Main Loop ----------//

void loop() {
  play();    
}
