#include "CircularBuffer.h"
#include "wavetables16.h"

#define AUDIO_CHANNEL_1_PIN PB8
#define AUDIO_PWM_TIMER 4
// The timer used for running the audio update loop. NOTE: Timer 3 appears to clash with SPI DMA transfers under some circumstances
#define AUDIO_UPDATE_TIMER 2

HardwareTimer audio_update_timer(AUDIO_UPDATE_TIMER);
HardwareTimer audio_pwm_timer(AUDIO_PWM_TIMER);

#define AUDIO_BITS 12
#define HALF_AUDIO_BITS (1 << AUDIO_BITS)/2
#define MAX_CARRIER_FREQ (F_CPU / (1 << AUDIO_BITS)) // e.g 72M / 256 for 8 bit = 281250. 256 = 1 leftshift 8
#define AUDIO_RATE 22050

//CircularBuffer<unsigned int> output_buffer;  // fixed size 256

void setup() {
  Serial.begin(115200); 

  // audio update timer is running at 16kHz or whatever, but audio_pwm_timer is the thing that is doing the output at/near full speed
  audio_update_timer.pause();
  //audio_update_timer.setPeriod(1000000UL / AUDIO_RATE);
  
  uint32_t prescaler = F_CPU/AUDIO_RATE/AUDIO_BITS;
  audio_update_timer.setPrescaleFactor(prescaler);
  uint32_t overflow = F_CPU/AUDIO_RATE/prescaler;
  audio_update_timer.setOverflow(overflow);
  
  audio_update_timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  audio_update_timer.setCompare(TIMER_CH1, 1); // Interrupt 1 count after each update
  audio_update_timer.attachCompare1Interrupt(pwmAudioOutput);
  audio_update_timer.refresh();
  audio_update_timer.resume();


  pinMode(AUDIO_CHANNEL_1_PIN, PWM);
         
  #if MAX_CARRIER_FREQ < (AUDIO_RATE * 5)
    // Generate as fast a carrier as possible
    audio_pwm_timer.setPrescaleFactor(1);
  #else
    // No point in generating arbitrarily high carrier frequencies. In fact, if
    // there _is_ any headroom, give the PWM pin more time to swing from HIGH to LOW and BACK, cleanly
    audio_pwm_timer.setPrescaleFactor((int)MAX_CARRIER_FREQ / (AUDIO_RATE * 4));

  #endif
  // Allocate enough room to write all intended bits
    audio_pwm_timer.setOverflow(1 << AUDIO_BITS);
}

//--------- Ringbuffer parameters ----------

const uint8_t BUFFERSIZE = 256;
const uint8_t BUFFERSIZE_M1 = BUFFERSIZE - 1;
uint_fast16_t Ringbuffer[256];
uint8_t RingWrite = 0;
uint8_t RingRead = 0;
volatile uint8_t RingCount = 0;

uint16_t sampleCount[NUM_SAMPLES];
uint16_t samplePointer[NUM_SAMPLES];

//--------- Sequencer parameters ----------

long tempo = 1000000;
const unsigned char patternLength = 7;

const unsigned char pattern[8] = {
  B00000001,
  B00100000,
  B00000010,
  B00100000,
  B00000001,
  B00100000,
  B00000010,
  B00010001,
};



const unsigned char pattern2[16] = {
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

//--------- pwmAudioOutput ------//

static void pwmAudioOutput() {
  if (RingCount) { //If entry in FIFO..
    pwmWrite(AUDIO_CHANNEL_1_PIN, (uint16_t)Ringbuffer[RingRead++] >> 4); //Output LSB of 16-bit DAC
    RingCount--;
  }
}

//--------- Write Buffer and sequence ----------//

void play() {
  // everything here optimized for processing speed not code quality

  /* -------sample buffer write------------ */

  uint_fast16_t sampleTotal;
  uint_fast8_t stepCount = 0;
  uint32_t tempoCount = 1;
  sampleCount[NUM_SAMPLES] = { 0 };

  while(1) { 
    if (RingCount < BUFFERSIZE_M1) { // if space in ringbuffer
      for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
        if (sampleCount[i]) {
          sampleTotal = (uint_fast16_t)wavetables16[i][samplePointer[i]] - (1 << 15);// - gainReduction[i])); // 
          samplePointer[i]++;
          sampleCount[i]--;
        }      
      }
     //  hard clip - must be faster than `constrain()`?
//      if (sampleTotal < -(1 << 15))
//        sampleTotal = -(1 << 15);
//      if (sampleTotal > (1 << 15))
//        sampleTotal = (1 << 15);
      Ringbuffer[RingWrite] = sampleTotal + (1 << 15);
      RingWrite++;
      RingCount++;
    }

  /* -------sequencer------------ */

    if (!(tempoCount--)) { // every "tempo" ticks, do the thing
      tempoCount = tempo; // set it back to the tempo ticks
      uint_fast8_t trigger = pattern[stepCount++]; //
//      Serial.println(audio_pwm_timer.getOverflow());
//      Serial.println(audio_pwm_timer.getPrescaleFactor());
      Serial.println(Ringbuffer[RingWrite]);
      

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
  
/* ----------------------------- */
}

//--------- Main Loop ----------//

void loop() {   
   play();
}
