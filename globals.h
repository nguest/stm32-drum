//SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));


//--------- Ringbuffer parameters ----------//

const uint8_t BUFFERSIZE = 256;
const uint8_t BUFFERSIZE_M1 = BUFFERSIZE - 1;
uint16_t Ringbuffer[256];
uint8_t RingWrite = 0;
uint8_t RingRead = 0;
volatile uint8_t RingCount = 0;

uint16_t sampleCount[NUM_SAMPLES];
uint16_t samplePointer[NUM_SAMPLES];

//--------- Sequencer/Play parameters ----------//

uint8_t MODE = 1; // 1 play / 0 pause /
bool RECORD = 0;
volatile long tempo = 4000000;
uint_fast8_t trigger = B00000000;

//--------- Control parameters ----------//

volatile uint8_t joystick;
