//--------- Patterns ----------------------//

uint8_t alivePattern[16] = {
  B00000001,
  B00100000,
  B00100000,
  B00100000,
  //
  B00000010,
  B00100000,
  B00100000,
  B00000001,
  //
  B00100001,
  B00100100,
  B00100000,
  B00000010,
  //
  B00100000,
  B00010001,
  B00100000,
  B00010000,
};

uint8_t livePattern[8] = {
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,  
};

const uint8_t patternLength = sizeof(livePattern) - 1;

