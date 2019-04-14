#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

volatile boolean received;
volatile byte SlaveReceived, Slavesend;
uint8_t index;
uint8_t buffer[2];

void setup() {
  Serial.begin(9600);
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);

//  Wire.begin(7);                // join i2c bus with address #8
//  Wire.onReceive(receiveEvent); // register event

  //setupSPI();

  Serial.println("init");
  pinMode(MISO,OUTPUT);                   // Sets MISO as OUTPUT (Have to Send data to Master IN (STM32F103C8)

  SPCR |= _BV(SPE);                       // Turn on SPI in Slave Mode
  received = false;
  SPI.attachInterrupt();                  // Interuupt ON is set for SPI commnucation
}

ISR (SPI_STC_vect) {                   // Inerrrput routine function
  SlaveReceived = SPDR;                   // Value received from master STM32F103C8 is stored in variable slavereceived
  //Serial.println(SlaveReceived);
  if (index < sizeof buffer) {
    buffer[index++] = SlaveReceived;
  }
  received = true;
//  if (!(SPSR & (1<<SPIF)) == 0) {     // Wait the end of the transmission
//    received = true;                        // Sets received as True 
//  }



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

  u8g2.clearBuffer();          // clear the internal memory
  //u8g2.setContrast(10); 0-255
  u8g2.drawStr(0,40, tmp_string);  // write something to the internal memory
  u8g2.drawBox(buffer[0]*8,0,8,3);
  u8g2.setDrawColor(2); // XOR
  //u8g2.drawBox(10,40,8,8);
  u8g2.sendBuffer();          // transfer internal memory to the display
  //delay(20);  
}
