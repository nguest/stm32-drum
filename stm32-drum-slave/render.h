#include <U8g2lib.h>

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


void setUpDisplay() {
  u8g2.begin();
  u8g2.clearBuffer();
 // u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFont(u8g2_font_5x7_tr);
  Serial.print(F("INIT"));

}

//--------- render functions ----------//

void readTempo() {
  tempo = constrain(tempo + joystick.y, 30, 50);
  joystick = (Joystick){ x: 0, y: 0 };
}

void renderStatus(void) {
  char tmp_string[7];

  switch (MODE) {
    case 0:
      u8g2.drawStr(30, 6, "PAUSED");
      break;
    case 1:
      u8g2.drawStr(30, 6, "PLAYING");
      break;
  }
  if (RECORD) {
    u8g2.drawDisc(80, 2, 3);
  }
}

void renderStep(void) {
  // display stepCount number
  char str1[8];
  itoa(buffer[0], str1, 10);
  u8g2.drawStr(0, 6, str1);

  u8g2.setDrawColor(2); // XOR
  u8g2.drawBox((buffer[0])*8,offsetY,8,54);
    
  u8g2.sendBuffer();
}

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
  readTempo();

  char str2[3];
  itoa(tempo, str2, 10);
  u8g2.drawStr(100, 6, str2);
  u8g2.drawBox(92, 4, 3, 2);

  u8g2.drawLine(95, 0, 95, 4);

  
  renderStatus();
  if(MODE == 1) renderStep();
  
}

void renderCursor() {
  if (joystick.x == 0 && joystick.y == 0) return;
  u8g2.clearBuffer();
  
  cursor.x += joystick.x;
  cursor.y += joystick.y;

  cursor.x = constrain(cursor.x, 0, 15);
  cursor.x = constrain(cursor.x, 0, 7);

  renderPattern();
  
  //Serial.print(cursor.x);Serial.print(" cursor ");Serial.println(cursor.y);
  u8g2.drawBox(cursor.x*8,offsetY,8,54);

  u8g2.sendBuffer();
  //u8g2.updateDisplayArea(0, 0, 24, 24);
  joystick = (Joystick){ x: 0, y: 0 };
}
