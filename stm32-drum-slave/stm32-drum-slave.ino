#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


void setup() {
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
}

void loop() {
  u8g2.clearBuffer();          // clear the internal memory
  //u8g2.setContrast(10); 0-255
  u8g2.drawStr(0,40,"Hello World! lorem ");  // write something to the internal memory
  u8g2.drawBox(5,10,20,10);
  u8g2.setDrawColor(2); // XOR
  u8g2.drawBox(10,40,8,8);
  u8g2.sendBuffer();          // transfer internal memory to the display
  delay(1000);  
}
