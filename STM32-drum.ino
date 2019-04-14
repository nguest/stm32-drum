#include "CircularBuffer.h"
#include "wavetables16.h"
#include "GPIOWriteFast.h"
#include <ADCTouchSensor.h>
#include "patterns.h"
#include <SPI.h>

#define SPI1_NSS_PIN PB12    //SPI_1 Chip Select pin is PA4. You can change it to the STM32 pin you want.
//#define SPI2_NSS_PIN PB12   //SPI_2 Chip Select pin is PB12. You can change it to the STM32 pin you want.

//SPIClass SPI_2(2); //Create an instance of the SPI Class called SPI_2 that uses the 2nd SPI Port
byte data;

#define SS PA4


//--------- Display ----------//

PC_13 LED;

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

  //--------- SPI setup ----------//

  pinMode(SS,OUTPUT); // Puts SS as Output
  SPI.begin(); // Begins the SPI commnuication
  SPI.setClockDivider(SPI_CLOCK_DIV32); // Sets clock for SPI communication at 16 (72/16=4.5Mhz)
  digitalWrite(SS,HIGH); // Setting SlaveSelect as HIGH (So master doesnt connnect with slave)


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

  // pinMode(PA5, INPUT_PULLUP);
  // pinMode(PA4, INPUT_ANALOG);

  //--------- Display setup ----------//

  LED.pinMode(OUTPUT);
  //Wire.begin();
    delay(1000);




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
volatile long tempo = 200000;
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
  byte MasterSend,MasterReceive;


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

        MasterSend = (byte)stepCount;
        digitalWrite(SS, LOW); // Starts communication with Slave connected to master
        MasterReceive = SPI.transfer(MasterSend); // Send the mastersend value to slave also receives value from slave
        digitalWrite(SS, HIGH);

  
        trigger = livePattern[stepCount++];

        // if (buttonTrigger != B00000000) {
        //   livePattern[stepCount] |= buttonTrigger;
        // }

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

void sendSPI() {
  digitalWrite(SPI1_NSS_PIN, LOW); // manually take CSN low for SPI_1 transmission
  data = SPI.transfer(0x55); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  digitalWrite(SPI1_NSS_PIN, HIGH); // manually take CSN high between spi transmissions
  Serial.println(data);
}


//--------- Main Loop ----------//

void loop() {
  play();    
}


// void setup() {

//   // Setup SPI 1
//   SPI.begin(); //Initialize the SPI_1 port.
//   SPI.setBitOrder(MSBFIRST); // Set the SPI_1 bit order
//   SPI.setDataMode(SPI_MODE0); //Set the  SPI_2 data mode 0
//   SPI.setClockDivider(SPI_CLOCK_DIV16);      // Slow speed (72 / 16 = 4.5 MHz SPI_1 speed)
//   pinMode(SPI1_NSS_PIN, OUTPUT);

//   // Setup SPI 2
//   SPI_2.begin(); //Initialize the SPI_2 port.
//   SPI_2.setBitOrder(MSBFIRST); // Set the SPI_2 bit order
//   SPI_2.setDataMode(SPI_MODE0); //Set the  SPI_2 data mode 0
//   SPI_2.setClockDivider(SPI_CLOCK_DIV16);  // Use a different speed to SPI 1
//   pinMode(SPI2_NSS_PIN, OUTPUT);


// }

// void loop() {

//   sendSPI();
//   sendSPI2();

//   delayMicroseconds(10);    //Delay 10 micro seconds.
// }

// void sendSPI()
// {
//   digitalWrite(SPI1_NSS_PIN, LOW); // manually take CSN low for SPI_1 transmission
//   data = SPI.transfer(0x55); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
//   digitalWrite(SPI1_NSS_PIN, HIGH); // manually take CSN high between spi transmissions
// }


// void sendSPI2()
// {
//   digitalWrite(SPI2_NSS_PIN, LOW); // manually take CSN low for SPI_2 transmission
//   data = SPI_2.transfer(0x55); //Send the HEX data 0x55 over SPI-2 port and store the received byte to the <data> variable.
//   digitalWrite(SPI2_NSS_PIN, HIGH); // manually take CSN high between spi transmissions
// }