#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

byte command = 0;
byte buffer[2];
volatile int index;
uint8_t pattern[16];


void setup (void) {
  Serial.begin(38400);
  pinMode(MISO, OUTPUT);
  pinMode(2, INPUT);
  SPCR |= _BV(SPE); // turn on SPI in slave mode
  SPCR |= _BV(SPIE);  // turn on interrupts
  attachInterrupt (0, ss_falling, FALLING); // interrupt for SS falling edge

  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
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
      break;
   
    case 's': // sending stepIndex & pattern data
      buffer[index++] = c;
      SPDR = 50;
      break;
  }
  pattern[buffer[0]] = buffer[1]; // write 2nd byte
}

//--------- render functions ----------//


void renderPattern() {
  const uint8_t offsetY = 10;
  u8g2.clearBuffer();          // clear the internal memory
  //u8g2.clearDisplay();

  for (uint8_t j = 0; j < 8; j++) {
    for (uint8_t i = 0; i < 16; i++) {    
      if (pattern[i] & 1<<j) {
        u8g2.drawBox(i*8 + 1, j*8 + offsetY + 1, 6, 6);
      }
    }
  }
  //renderStatus();
  renderIndicator();
}

void renderIndicator() {
  // display stepCount number
  char tmp_string[8];
  itoa(buffer[0], tmp_string, 10);
  u8g2.drawStr(0,10, tmp_string);

  // display moving box
  u8g2.setDrawColor(2); // XOR
  u8g2.drawBox((buffer[0])*8,10,8,54);
  
  u8g2.sendBuffer();

}


//--------- main loop ----------//

void loop (void) {
//  Serial.print("buff ");Serial.print(buffer[0]);Serial.print("  ");Serial.println(buffer[1]);
//  delay(800);
  renderPattern();
}
