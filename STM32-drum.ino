#include "CircularBuffer.h"
#include "wavetables16.h"

#define AUDIO_CHANNEL_1_PIN PB8
#define AUDIO_PWM_TIMER 4
// The timer used for running the audio update loop. NOTE: Timer 3 appears to clash with SPI DMA transfers under some circumstances
#define AUDIO_UPDATE_TIMER 2
#define LOOP_TIMER 3


HardwareTimer audioUpdateTimer(AUDIO_UPDATE_TIMER);
HardwareTimer audioPwmTimer(AUDIO_PWM_TIMER);
HardwareTimer buttonTimer(LOOP_TIMER);


#define AUDIO_BITS 12
#define HALF_AUDIO_BITS (1 << AUDIO_BITS)/2
#define MAX_CARRIER_FREQ (F_CPU / (1 << AUDIO_BITS)) // e.g 72M / 256 for 8 bit = 281250. 256 = 1 leftshift 8
#define AUDIO_RATE 22050

//CircularBuffer<unsigned int> output_buffer;  // fixed size 256

void setup() {
  Serial.begin(115200); 

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
//
//
  buttonTimer.pause();
//  buttonTimer.setPrescaleFactor(F_CPU/AUDIO_RATE/AUDIO_BITS);
//  buttonTimer.setOverflow(S);
  buttonTimer.setPeriod(10000);
  buttonTimer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  buttonTimer.setCompare(TIMER_CH1, 1); // Interrupt 1 count after each update
  buttonTimer.attachCompare1Interrupt(buttonInterrupt);
  buttonTimer.refresh();
  buttonTimer.resume();

  pinMode(PA5, INPUT_PULLUP);
}

//--------- Ringbuffer parameters ----------

const uint8_t BUFFERSIZE = 256;
const uint8_t BUFFERSIZE_M1 = BUFFERSIZE - 1;
uint16_t Ringbuffer[256];
uint8_t RingWrite = 0;
uint8_t RingRead = 0;
volatile uint8_t RingCount = 0;

uint16_t sampleCount[NUM_SAMPLES];
uint16_t samplePointer[NUM_SAMPLES];

//--------- Sequencer/Play parameters ----------

uint8_t MODE = 1;
long tempo = 300000;
const uint8_t patternLength = 7;
uint_fast8_t trigger = B00000000;
uint_fast8_t buttonTrigger = B00000000;

const uint8_t pattern[16] = {
  B00000001,
  B00100000,
  B00100000,
  B00100000,
  //
  B00000010,
  B00100000,
  B00100000,
  B00000001,
  //
  B00100001,
  B00100100,
  B00100000,
  B00000010,
  //
  B00100000,
  B00010001,
  B00100000,
  B00010000,
};

uint8_t livePattern[8] = {
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,  
};

const uint8_t pattern2[16] = {
  B01000001,
  B00000100,
  B01000001,
  B00000000,
  B01000010,
  B00000000,
  B01000100,
  B01000000,
  B01000001,
  B00000100,
  B01000000,
  B00000000,
  B01000011,
  B00000000,
  B01000001,
  B01000000,
};

//--------- button interrupt ------//


bool buttonLast = 1;
bool button = 1;

int delay1;
void buttonInterrupt() {
  button = digitalRead(PA5);
  if (button == 0 && buttonLast == 1) {
    buttonTrigger = B00000001;
    delay1 = 0;
  }
  if (button == 1 && delay1 > 10) {
    buttonTrigger = B00000000;
  }
  delay1++;
  buttonLast = button;
  //Serial.println(buttonTrigger);
}

//--------- pwmAudioOutput ------//

static void pwmAudioOutput() {
  if (RingCount) { //If entry in FIFO..
    pwmWrite(AUDIO_CHANNEL_1_PIN, Ringbuffer[RingRead++] >> 4); //Output LSB of 16-bit DAC
    RingCount--;
  }
}

//--------- Write Buffer and sequence ----------//

void play() {
  // everything here optimized for processing speed not code quality

  /* -------sample buffer write------------ */

  int32_t sampleTotal = 0; // has to be signed
  uint_fast8_t stepCount = 0;
  uint32_t tempoCount = 1;
  uint32_t playCount = 1;

  sampleCount[NUM_SAMPLES] = { 0 };

  while(1) { 
    if (RingCount < BUFFERSIZE_M1) { // if space in ringbuffer
      sampleTotal = 0;
      for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
        if (sampleCount[i]) {
          int16_t sample = (wavetables16[i][samplePointer[i]++]) - (1 << 15);//) >> 8;
          sample = ((sample * gain[i]) >> 8);
          sampleTotal += sample; // 
          sampleCount[i]--;
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

  /* -------sequencer------------ */
 
    if (MODE == 1) {
      if (!(tempoCount--)) { // every "tempo" ticks, do the thing
        tempoCount = tempo; // set it back to the tempo ticks
        trigger = livePattern[stepCount++]; //

        if (buttonTrigger & 1) {
          Serial.println("trig");
          livePattern[stepCount] = B00000001;
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
          if (trigger & 1<<i) {
            //Serial.println("trig");

            samplePointer[i] = 0;
            sampleCount[i] = wavetableLengths16[i]; // number of bytes in sample
          }
        } 
        // if (trigger & 1<<0) {
        //   samplePointer[0] = 0;
        //   sampleCount[0] = wavetableLengths16[0]; // number of bytes in sample
        // }
    }
  }
  
/* ----------------------------- */
}

//--------- Main Loop ----------//

void loop() {
  play();    
}
