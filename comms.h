#include <SPI.h>

#define SS PB12 //PA14 // chip select

//SPIClass SPI(PB5,PB4,PB3); // MOSI, MISO, CLK


//--------- SPI setup ----------//

void setupSPI() {
  pinMode(SS,OUTPUT); // Puts chipselect as Output
  
  SPI.setModule(2); // use SPI-2 - PB15/14/13/12
  SPI.setClockDivider(SPI_CLOCK_DIV64); // Sets clock for SPI communication at 16 (72/16=4.5Mhz)
  SPI.begin(); // Begins the SPI commnuication

  digitalWrite(SS,HIGH); // Setting SlaveSelect as HIGH (So master doesnt connnect with slave)
}

//--------- SPI Write ----------//

byte transferAndWait (volatile byte what) {
  byte receive = SPI.transfer(what);
  delayMicroseconds(20);
  return receive;
} // end of transferAndWait

uint8_t WriteSPI(char command, uint8_t var1, uint8_t var2) {
  digitalWrite(SS, LOW); // Starts communication with Slave connected to master
  //SPI.transfer(command);
  //delayMicroseconds(20);
  byte a = transferAndWait(command);
  int b = transferAndWait(var1);
  tempo = a * 10000;
  byte c = transferAndWait(var2);
  //byte d = transferAndWait(0);
  //uint8_t MasterReceive = transferAndWait(var2);

  Serial.print(command);Serial.print(" ");Serial.print(var1);Serial.print(" "); Serial.println(var2);
  Serial.print("abc ");Serial.print(a);Serial.print(" ");Serial.print(b);Serial.print(" ");Serial.println(c);Serial.print(" ");

  digitalWrite(SS, HIGH);
  //return MasterReceive;
};

