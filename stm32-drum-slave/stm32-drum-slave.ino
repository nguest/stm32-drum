#include <U8g2lib.h>
#include <SPI.h>

#define joystickXPin A2
#define joystickYPin A3

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SH1106_128X64_NONAME_1 k_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


byte command = 0;
byte buffer[2];
volatile int index;
uint8_t pattern[16];
byte MODE = 1;
byte RECORD = 0;
const int offsetY = 8;
volatile uint8_t tempo = 40;

struct Joystick {
  volatile int x;
  volatile int y;
};
Joystick joystick;

struct Cursor {
  int x;
  int y;
};
Cursor cursor1;
int cursorX;
int cursorY;

void setup (void) {
  Serial.begin(38400);
  pinMode(MISO, OUTPUT);
  pinMode(2, INPUT);

  pinMode(joystickXPin, INPUT);
  pinMode(joystickYPin, INPUT);

  SPCR |= _BV(SPE); // turn on SPI in slave mode
  SPCR |= _BV(SPIE);  // turn on interrupts
  attachInterrupt (0, ss_falling, FALLING); // interrupt for SS falling edge

  u8g2.begin();
  u8g2.clearBuffer();
 // u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFont(u8g2_font_5x7_tr);
  Serial.print(F("INIT"));

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
    case 't':
      MODE = c;
      SPDR = tempo;
      index = 2;
  }
  pattern[buffer[0]] = buffer[1]; // write 2nd byte
}

//--------- render functions ----------//

void renderPattern(void) {
  u8g2.clearBuffer();          // clear the internal memory
  //u8g2.clearDisplay();

  for (uint8_t j = 0; j < 8; j++) {
    for (uint8_t i = 0; i < 16; i++) {    
      if (pattern[i] & 1<<j) {
        u8g2.drawBox(i*8 + 1, j*8 + offsetY + 1, 6, 6);
      }
    }
  }
  renderStatus();
  if(MODE == 1) renderStep();
  //Serial.println(MODE);
  
}

void renderStep(void) {
  // display stepCount number
  char tmp_string[8];
  itoa(buffer[0], tmp_string, 10);
  u8g2.drawStr(0, 6, tmp_string);

  char tmp_string2[3];
  itoa(tempo, tmp_string2, 10);
  u8g2.drawStr(80, 6, tmp_string2);

  u8g2.setDrawColor(2); // XOR
  u8g2.drawBox((buffer[0])*8,offsetY,8,54);
    
  u8g2.sendBuffer();
}

void renderStatus(void) {
  char tmp_string[7];

  switch (MODE) {
    case 0:
      u8g2.drawStr(30, 6, "PAUSED");
      break;
    case 1:
    case 2:
      u8g2.drawStr(30, 6, "PLAYING");
      break;
  }
}

void renderCursor(int dx, int dy) {
  if (dx == 0 && dy == 0) return;
  u8g2.clearBuffer();
  
  cursorX += dx;
  cursorY += dy;

  cursorX = constrain(cursorX, 0, 15);
  cursorY = constrain(cursorY, 0, 7);
//  
 // Serial.print(cursorX);Serial.print(" cursor ");Serial.println(cursorY);
  u8g2.drawBox(cursorX*8, cursorY*8, 8, 8);
  u8g2.sendBuffer();
  //u8g2.updateDisplayArea(0, 0, 24, 24);

}

volatile int tDelay;

void renderTempo() {

  if (tDelay > 10) {
    Serial.println(MODE);

    //u8g2.clearBuffer();
    tempo = constrain(tempo + joystick.x, 30, 50);
    Serial.print("Tempo ");Serial.println(joystick.x);
    char tmp_string[4];
    itoa(tempo, tmp_string, 10);
//    u8g2.drawStr(80, 6, tmp_string);
//    u8g2.sendBuffer();

    joystick.x = 0;
    tDelay = 0;
  }
  tDelay++;
}


//--------- control functions ----------//

volatile int jDelay;

int readJoystick(int wait) {
  if (jDelay > wait) {
    int x = analogRead(joystickXPin);
    int y = analogRead(joystickYPin);
  
    //Serial.println(y);
    uint8_t threshold = 100;

    if (x < threshold) {
      joystick.x = 1;
    }
    if (x > 1023 - threshold) {
      joystick.x = -1;
    }
    if (y < threshold) {
      joystick.y = 1;
    }
    if (y > 1023 - threshold) {
      joystick.y = -1;
    }
   // Serial.print(joystick.x);Serial.print(F(" J "));Serial.println(joystick.y);
    //renderCursor(joystick.x,joystick.y);
    jDelay = 0;
  }
//  joystick.x = 0;
//  joystick.y = 0;

  jDelay++;

}


//--------- main loop ----------//

void loop (void) {
//  Serial.print("buff ");Serial.print(buffer[0]);Serial.print("  ");Serial.println(buffer[1]);
//  Serial.print(" MODE ");Serial.println(MODE);
  //delay(100);
  //Serial.println();

  
     //Serial.println(x);
  if (MODE == 0) {
    readJoystick(10000);
  }
  if (MODE == 1) {
    readJoystick(10);
    renderTempo();
    renderPattern();
  }

}
