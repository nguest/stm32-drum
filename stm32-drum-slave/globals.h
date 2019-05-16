#define joystickXPin A2
#define joystickYPin A0

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
Cursor cursor;
