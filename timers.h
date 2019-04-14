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


//--------- pwmAudioOutput ------//

void pwmAudioOutput() {
  if (RingCount) { //If entry in FIFO..
    pwmWrite(AUDIO_CHANNEL_1_PIN, Ringbuffer[RingRead++] >> 4); //Output LSB of 16-bit DAC
    RingCount--;
  }
}

//--------- Audio Update Timer setup ----------//

void setupTimers() {

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

  // //--------- Audio PWM Timer setup ----------//

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
  //nvic_irq_set_priority(NVIC_USART1, 1);

  controlTimer.pause();
  controlTimer.setPeriod(40000);
  controlTimer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  controlTimer.setCompare(TIMER_CH1, 1); // Interrupt 1 count after each update
  controlTimer.attachCompare1Interrupt(controlInterrupt);
  controlTimer.refresh();
  controlTimer.resume();

}

