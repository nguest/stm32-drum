#include <SPI.h>

#define SS PB12 //PA14 // chip select

//SPIClass SPI(PB5,PB4,PB3); // MOSI, MISO, CLK
//SPI(SS);



//SPI_2.begin();



//--------- SPI setup ----------//

void setupSPI() {
  pinMode(SS,OUTPUT); // Puts chipselect as Output
  
  SPI.setModule(2); // use SPI-2 - PB15/14/13/12
  SPI.setClockDivider(SPI_CLOCK_DIV32); // Sets clock for SPI communication at 16 (72/16=4.5Mhz)
  SPI.begin(); // Begins the SPI commnuication

  digitalWrite(SS,HIGH); // Setting SlaveSelect as HIGH (So master doesnt connnect with slave)
}

//--------- SPI Write ----------//

uint8_t WriteSPI(char command, uint8_t var1, uint8_t var2) {
  digitalWrite(SS, LOW); // Starts communication with Slave connected to master
  SPI.transfer(command);
  delayMicroseconds(20);
  SPI.transfer(var1); // Send the mastersend value to slave also receives value from slave
  delayMicroseconds(20);
  uint8_t MasterReceive = SPI.transfer(var2); // Send the mastersend value to slave also receives value from slave
  delayMicroseconds(20);
  //SPI.transfer((byte)255);
  digitalWrite(SS, HIGH);
  return MasterReceive;
};