#include <SPI.h>
#include <U8g2lib.h>
#include "globals.h"
#include "render.h"


void setup (void) {
  Serial.begin(38400);

  setUpDisplay();
  setupSPI();

}


void setupSPI() {
  pinMode(MISO, OUTPUT);
  pinMode(2, INPUT);

  SPCR |= _BV(SPE); // turn on SPI in slave mode
  SPCR |= _BV(SPIE);  // turn on interrupts
  attachInterrupt (0, ss_falling, FALLING); // interrupt for SS falling edge
}

//--------- SS state interrupt ----------//

void ss_falling () {
  if (index == sizeof(buffer)) {
    index = 0;
    command = 0;
  }
}

//--------- SPI interrupt routine ----------//

ISR (SPI_STC_vect) {
  byte c = SPDR;
 
  switch (command) {
    case 0: // no command? then this is the command
      command = c;
      SPDR = 0;
      // index = 0; ?
      break;
   
    case 's': // sending stepIndex & pattern data
      buffer[index++] = c;
      SPDR = tempo;
      break;
      
    case 'm':
      MODE = c;
      SPDR = tempo;
      index = 2;
      Serial.print(" MODE ");Serial.println(MODE);
      break;
      
    case 'r':
      RECORD = c;
      SPDR = tempo;
      index = 2;
      break;
  }
  pattern[buffer[0]] = buffer[1]; // write 2nd byte
}


//--------- control functions ----------//

volatile int jDelay;

int readJoystick(int wait) {
  if (jDelay > wait) {
    int x = analogRead(joystickXPin);
    int y = analogRead(joystickYPin);
  
    //Serial.print(x);Serial.print(" ");Serial.println(y);
    uint8_t threshold = 100;

    if (x < threshold) {
      joystick.x = 1;
    }
    if (x > 1023 - threshold) {
      joystick.x = -1;
    }
    if (y < threshold) {
      joystick.y = -1;
    }
    if (y > 1023 - threshold) {
      joystick.y = 1;
    }
    jDelay = 0;
    //Serial.print(joystick.x);Serial.print(" ");Serial.println(joystick.y);

  }

  jDelay++;

}


//--------- main loop ----------//

void loop (void) {
//  Serial.print("buff ");Serial.print(buffer[0]);Serial.print("  ");Serial.println(buffer[1]);
//  Serial.print(" MODE ");Serial.println(MODE);

  
  if (MODE == 0) {
    readJoystick(50000);
    renderCursor();
  }
  if (MODE == 1) {
    readJoystick(10);
    renderPattern();
  }

}
