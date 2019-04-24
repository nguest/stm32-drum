#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Encoder.h>


U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

int tempo = 200;
volatile boolean received;
volatile byte SlaveReceived, Slavesend;
uint8_t index;
uint8_t buffer[2];
uint8_t pattern[16];

long oldPosition  = -999;
int pulses, A_SIG=0, B_SIG=1;

//Encoder encoder(2,3);

void setup() {
  Serial.begin(9600);
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);

  Serial.println("init");
  pinMode(MISO,OUTPUT);                   // Sets MISO as OUTPUT (Have to Send data to Master IN (STM32F103C8)
  SPCR |= _BV(SPE);                       // Turn on SPI in Slave Mode
  received = false;
  SPI.attachInterrupt();                  // Interuupt ON is set for SPI commnucation
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(0, A_RISE, RISING);
  attachInterrupt(1, B_RISE, RISING);

  
}

ISR (SPI_STC_vect) {
  received = false;
  SlaveReceived = SPDR;                   // Value received from master STM32F103C8 is stored in variable slavereceived
  //Serial.print("SPDR ");Serial.println(SlaveReceived);
  //Serial.println(SlaveReceived);
  if (SlaveReceived == 's') {
    Serial.println("um");
    return;
  }
  if (index < sizeof buffer) {
    buffer[index++] = SlaveReceived;
  }
 // if (SlaveReceived == 255) {
   // received = true;
  //}
  if (index == sizeof buffer) {
    received = true;
  }
  
  pattern[buffer[0]] = buffer[1]; // write 2nd byte
//  if (!(SPSR & (1<<SPIF)) == 0) {     // Wait the end of the transmission
//    received = true;                        // Sets received as True 
//  }



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

void renderStatus() {
  //u8g2.setDrawColor(0);
//  u8g2.drawFrame(24,10,80,44);
//  u8g2.setDrawColor(0);
//  u8g2.drawBox(25,11,78,42);

 // u8g2.drawRBox(50, 10,25,40, 40);
}

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
  renderStatus();
  renderIndicator();

}
//
//void onEncoderChange() {
//  //long newPosition = encoder.read();
//  Serial.print(digitalRead(2));Serial.print(" ");Serial.println(digitalRead(3));
// // Serial.println(newPosition);
////  if (newPosition != oldPosition) {
////    oldPosition = newPosition;
////    
////  }
//
//}
void A_RISE(){
 detachInterrupt(0);
 A_SIG=1;
 
 if(B_SIG==0)
 pulses++;//moving forward
 if(B_SIG==1)
 pulses--;//moving reverse
 Serial.println(pulses);
 attachInterrupt(0, A_FALL, FALLING);
}

void A_FALL(){
  detachInterrupt(0);
 A_SIG=0;
 
 if(B_SIG==1)
 pulses++;//moving forward
 if(B_SIG==0)
 pulses--;//moving reverse
 Serial.println(pulses);
 attachInterrupt(0, A_RISE, RISING);  
}

void B_RISE(){
 detachInterrupt(1);
 B_SIG=1;
 
 if(A_SIG==1)
 pulses++;//moving forward
 if(A_SIG==0)
 pulses--;//moving reverse
 Serial.println(pulses);
 attachInterrupt(1, B_FALL, FALLING);
}

void B_FALL(){
 detachInterrupt(1);
 B_SIG=0;
 
 if(A_SIG==0)
 pulses++;//moving forward
 if(A_SIG==1)
 pulses--;//moving reverse
 Serial.println(pulses);
 attachInterrupt(1, B_RISE, RISING);
}


void loop() {
  if (received) {
    received = false;
    Serial.print(buffer[0]); Serial.print(" "); Serial.println(buffer[1], BIN);
    index = 0;
  }
  
  Slavesend = 17; // random byte
  SPDR = Slavesend;
  
  //onEncoderChange();


//  u8g2.clearBuffer();          // clear the internal memory
//  //u8g2.setContrast(10); 0-255
//  u8g2.drawStr(0,40, tmp_string);  // write something to the internal memory
//  u8g2.drawBox((buffer[0]-1)*8,0,8,3);
//  //u8g2.setDrawColor(2); // XOR
//  //u8g2.drawBox(10,40,8,8);
//  u8g2.sendBuffer();          // transfer internal memory to the display
//  //delay(20);
  //renderIndicator();
  renderPattern();
  
}
