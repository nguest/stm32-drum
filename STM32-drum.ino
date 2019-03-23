#include "CircularBuffer.h"
#include "wavetables.h"


#define AUDIO_CHANNEL_1_PIN PB8
#define AUDIO_PWM_TIMER 4
// The timer used for running the audio update loop. NOTE: Timer 3 appears to clash with SPI DMA transfers under some circumstances
#define AUDIO_UPDATE_TIMER 2

HardwareTimer audio_update_timer(AUDIO_UPDATE_TIMER);
HardwareTimer audio_pwm_timer(AUDIO_PWM_TIMER);

#define AUDIO_BITS 8
#define HALF_AUDIO (1 << AUDIO_BITS)/2
#define MAX_CARRIER_FREQ (F_CPU / (1 << AUDIO_BITS)) // e.g 72M / 256 for 8 bit = 281250. 256 = 1 leftshift 8
#define AUDIO_RATE 22050

//CircularBuffer<unsigned int> output_buffer;  // fixed size 256

void setup() {
   Serial.begin(115200); 

  // audio update timer is running at 16kHz or whatever, but audio_pwm_timer is the thing that is doing the output at (probably) full speed
  audio_update_timer.pause();
  audio_update_timer.setPeriod(1000000UL / AUDIO_RATE);
  audio_update_timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  audio_update_timer.setCompare(TIMER_CH1, 1); // Interrupt 1 count after each update
  audio_update_timer.attachCompare1Interrupt(pwmAudioOutput);
  audio_update_timer.refresh();
  audio_update_timer.resume();

//  audio_pwm_timer.pause();
//  audio_pwm_timer.setPrescaleFactor(1);
//  audio_pwm_timer.setOverflow(1 << AUDIO_BITS);
//  audio_pwm_timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
//  audio_pwm_timer.setCompare(TIMER_CH1, 1); // Interrupt 1 count after each update
//  audio_pwm_timer.attachCompare1Interrupt(play);
//  audio_pwm_timer.refresh();
//  audio_pwm_timer.resume();
  

  pinMode(AUDIO_CHANNEL_1_PIN, PWM);
         
  #if MAX_CARRIER_FREQ < (AUDIO_RATE * 5)
    // Generate as fast a carrier as possible
    audio_pwm_timer.setPrescaleFactor(10);
  #else
    // No point in generating arbitrarily high carrier frequencies. In fact, if
    // there _is_ any headroom, give the PWM pin more time to swing from HIGH to LOW and BACK, cleanly
    audio_pwm_timer.setPrescaleFactor((int)MAX_CARRIER_FREQ / (AUDIO_RATE * 5));
  #endif
  // Allocate enough room to write all intended bits
    audio_pwm_timer.setOverflow(1 << AUDIO_BITS);
    if (!Serial) delay(1000);
       Serial.print("hello");


}

//--------- Ringbuf parameters ----------

const uint8_t BUFFERSIZE = 256;
uint8_t Ringbuffer[256];
uint8_t RingWrite = 0;
uint8_t RingRead = 0;
volatile uint8_t RingCount = 0;

uint16_t sampleCount;
uint16_t samplePointer;

//--------- Sequencer parameters ----------

long tempo = 1000000;
const unsigned char patternLength = 15;

const unsigned char pattern[16] = {
  B01000000,
  B00000000,
  B01000000,
  B00000000,
  B00100000,
  B00000000,
  B00100000,
  B00000000,
  B00000001,
  B00000000,
  B00000001,
  B00000001,
  B00000001,
  B00000001,
  B00000001,
  B00000001,
};

//--------- pwmAudioOutput ----------

static void pwmAudioOutput() {
  if (RingCount) { //If entry in FIFO..
    pwmWrite(AUDIO_CHANNEL_1_PIN, Ringbuffer[RingRead++]); //Output LSB of 16-bit DAC
    RingCount--;
  }
}

//--------- Main Loop ----------
//
void loop() {   
   play();
}

void play() {
  int16_t sampleTotal;
  sampleCount = 0;
  uint8_t stepCount = 0;
  long tempoCount = 1;
  while(1) { 
  //sampleCount = sizeof(kick);
  if (RingCount < 255) {
    if (sampleCount) {
      sampleTotal = (int)kick[samplePointer];
      samplePointer++;
      sampleCount--;
    }
    // hard clip
//    if (sampleTotal < -HALF_AUDIO-1)
//      sampleTotal = -HALF_AUDIO-1;
//    if (sampleTotal > HALF_AUDIO-1)
//      sampleTotal = HALF_AUDIO-1;     
    Ringbuffer[RingWrite] = sampleTotal;
    RingWrite++;
    RingCount++;
  }

/* ----------------------------- */
      if (!(tempoCount--)) { // every "tempo" ticks, do the thing
        tempoCount = tempo; // set it back to the tempo ticks
        uint8_t trigger = pattern[stepCount++]; //
    
        if (stepCount > patternLength) stepCount = 0;
        // read the pattern bytes, each one triggers a sample
        if (trigger & 1) {
          samplePointer = 0;
          sampleCount = sizeof(kick); // number of bytes in sample
        }
      }
  }
/* ----------------------------- */
}

//void playSequencer() {
//
//  
//  if (!(tempoCount--)) { // every "tempo" ticks, do the thing
//    tempoCount = tempo; // set it back to the tempo ticks
//    uint8_t trigger = pattern[stepCount++]; //
//
//    if (stepCount > patternLength) stepCount = 0;
//    // read the pattern bytes, each one triggers a sample
//    if (trigger & 1) {
//      samplePointer = 0;
//      sampleCount = sizeof(kick); // number of bytes in sample
//    }
//  }
//}
