#include <SPI.h>

#define SS PA4

//--------- SPI setup ----------//

void setupSPI() {
  pinMode(SS,OUTPUT); // Puts SS as Output
  SPI.begin(); // Begins the SPI commnuication
  SPI.setClockDivider(SPI_CLOCK_DIV64); // Sets clock for SPI communication at 16 (72/16=4.5Mhz)
  digitalWrite(SS,HIGH); // Setting SlaveSelect as HIGH (So master doesnt connnect with slave)
}