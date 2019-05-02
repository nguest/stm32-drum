#include "wavetables16.h"
#include "GPIOWriteFast.h"
#include <ADCTouchSensor.h>
#include "globals.h"
#include "patterns.h"
#include "comms.h"
#include "controls.h"
#include "timers.h"



//--------- Setup ----------//

PC_13 LED;

void setup() {
  Serial.begin(115200);

  setupSPI();
  setupTimers();
  setupControls();

  LED.pinMode(OUTPUT);
  delay(1000);

}

//--------- Write Buffer and sequence ----------//

void play() {
  // everything here optimized for processing speed not code quality - inline faster?

  /* -------write sample buffer ------------ */

  int32_t sampleTotal = 0; // has to be signed
  uint_fast8_t stepCount = 0;
  uint32_t tempoCount = 1;

  sampleCount[NUM_SAMPLES] = { 0 };
  uint16_t MasterSend;
  byte MasterReceive;


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
        readTouch(); // read touch buttons

        MasterReceive = WriteSPI('s', stepCount, livePattern[stepCount]);
        // digitalWrite(SS, LOW); 
        // MasterReceive = SPI.transfer(stepCount);
        // delayMicroseconds(20);
        // digitalWrite(SS, HIGH); 

        trigger = livePattern[stepCount++];
        //Serial.print("RECORD ");Serial.print(RECORD);Serial.print(" -- ");Serial.println(buttonTrigger);
        if (RECORD && (buttonTrigger != B00000000)) {
          livePattern[stepCount] ^= buttonTrigger;
          buttonTrigger = B00000000;
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


    /* -------player------------ */  
    else if (MODE == 0){
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
    else {
      stepCount = 0;
      tempoCount = 1;
      Serial.println(joystick);
    }
  }
/* ----------------------------- */
}

//--------- Main Loop ----------//

void loop() {
  play();    
}