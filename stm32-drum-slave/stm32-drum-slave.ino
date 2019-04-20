#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

volatile boolean received;
volatile byte SlaveReceived, Slavesend;
uint8_t index;
uint8_t buffer[2];
uint8_t pattern[16];

void setup() {
  Serial.begin(9600);
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);

  Serial.println("init");
  pinMode(MISO,OUTPUT);                   // Sets MISO as OUTPUT (Have to Send data to Master IN (STM32F103C8)
  SPCR |= _BV(SPE);                       // Turn on SPI in Slave Mode
  received = false;
  SPI.attachInterrupt();                  // Interuupt ON is set for SPI commnucation
}

ISR (SPI_STC_vect) {
  received = false;
  SlaveReceived = SPDR;                   // Value received from master STM32F103C8 is stored in variable slavereceived
  //Serial.print("SPDR ");Serial.println(SlaveReceived);
  //Serial.println(SlaveReceived);
  if (index < sizeof buffer) {
    buffer[index++] = SlaveReceived;
  }
  if(SlaveReceived == 255) {
    received = true;
  }
  pattern[buffer[0]] = buffer[1];
//  if (!(SPSR & (1<<SPIF)) == 0) {     // Wait the end of the transmission
//    received = true;                        // Sets received as True 
//  }



}

void renderIndicator() {
  
}

void renderPattern() {
  const uint8_t offsetY = 8;
  u8g2.clearBuffer();          // clear the internal memory
  //u8g2.clearDisplay();

  for (uint8_t i = 0; i < 8; i++) {
    for (uint8_t j = 0; j < 16; j++) {    
      if (pattern[j] & 1<<i) {
        u8g2.drawBox(j*8,i*8+offsetY,8,8);
      }
    }
  }
  u8g2.sendBuffer();
}

void loop() {
  if (received) {
    received = false;
    Serial.print(buffer[0]); Serial.print(" "); Serial.println(buffer[1], BIN);
    index = 0;
  }
  
  Slavesend = 17; // random byte
  SPDR = Slavesend;
  
  
  char tmp_string[8];
  itoa(buffer[0], tmp_string, 10);

//  u8g2.clearBuffer();          // clear the internal memory
//  //u8g2.setContrast(10); 0-255
//  u8g2.drawStr(0,40, tmp_string);  // write something to the internal memory
//  u8g2.drawBox((buffer[0]-1)*8,0,8,3);
//  //u8g2.setDrawColor(2); // XOR
//  //u8g2.drawBox(10,40,8,8);
//  u8g2.sendBuffer();          // transfer internal memory to the display
//  //delay(20);  
  renderPattern();
}
