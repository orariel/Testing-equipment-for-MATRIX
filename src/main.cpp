#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define ARRAY_LEN(x) (sizeof(x) / sizeof(x[0]))

/* -------------------- Pins -------------------- */
const int IN1 = 4;
const int IN2 = 7;
const int IN3 = 8;
const int StartPin = 2;
const int DelayX = 2200;

/* -------------------- State Machine -------------------- */
enum state {
  IDLE,
  OPEN_COLUMNS,
  OPEN_ROWS,
  OPEN_CELL_FROM_LEFT,
  OPEN_CELL_FROM_RIGHT,
  OPEN_CELL_BY_CELL,
  CHECK_INPUT,
  TEST_ALL
};

volatile state StateMachine = CHECK_INPUT;

/* -------------------- ISR -------------------- */
void startBitISR() {
  StateMachine = CHECK_INPUT;
}

/* -------------------- Globals -------------------- */
int bitInOne, bitInTwo, bitInThree, start_bit;

/* -------------------- LUTs -------------------- */
uint8_t rowLUTforOPENrowsONLY[][2] = {
  {0x26, 0b11111110},
  {0x26, 0b11111101},
  {0x26, 0b11111011},
  {0x26, 0b11110111},
  {0x26, 0b11101111},
  {0x26, 0b11011111},
  {0x26, 0b10111111},
  {0x26, 0b01111111},
  {0x26, 0b11111111},
  {0x27, 0b11111110},
  {0x27, 0b11111101},
  {0x27, 0b11111011},
  {0x27, 0b11111111}
};

uint8_t rowLUT[][2] = {
  {0x26, 0b11111110},
  {0x26, 0b11111101},
  {0x26, 0b11111011},
  {0x26, 0b11110111},
  {0x26, 0b11101111},
  {0x26, 0b11011111},
  {0x26, 0b10111111},
  {0x26, 0b01111111},
  {0x27, 0b11111110},
  {0x27, 0b11111101},
  {0x27, 0b11111011}
};

uint8_t colLUT[][2] = {
  {0x22, 0b11011111},
  {0x22, 0b11101111},
  {0x22, 0b11110111},
  {0x22, 0b01111011},
  {0x22, 0b11111101},
  {0x22, 0b11111110},
  {0x23, 0b11111101},
  {0x23, 0b11111110},
  {0x21, 0b11111101},
  {0x21, 0b11111110},
  {0x20, 0b01111111},
  {0x20, 0b10111111},
  {0x20, 0b11011111},
  {0x20, 0b11101111},
  {0x20, 0b11110111},
  {0x20, 0b11111011},
  {0x20, 0b11111101},
  {0x20, 0b11111110},
  {0x20, 0b11111111}
};

uint8_t colLUTRR[][2] = {
  {0x20, 0b11111110},
  {0x20, 0b11111101},
  {0x20, 0b11111011},
  {0x20, 0b11110111},
  {0x20, 0b11101111},
  {0x20, 0b11011111},
  {0x20, 0b10111111},
  {0x20, 0b01111111},
  {0x21, 0b11111110},
  {0x21, 0b11111101},
  {0x23, 0b11111110},
  {0x23, 0b11111101},
  {0x22, 0b11111110},
  {0x22, 0b11111101},
  {0x22, 0b01111011},
  {0x22, 0b11110111},
  {0x22, 0b11101111},
  {0x22, 0b11011111},
  {0x20, 0b11111111}
};

uint8_t offROWLUT[][2] = {
  {0x26, 0b11111111},
  {0x27, 0b11111111}
};

uint8_t offCOLLUT[][2] = {
  {0x22, 0b11111111},
  {0x23, 0b11111111},
  {0x21, 0b11111111},
  {0x20, 0b11111111}
};

uint8_t offLUT[][2] = {
  {0x20, 0b11111111},
  {0x21, 0b11111111},
  {0x22, 0b11111111},
  {0x23, 0b11111111},
  {0x26, 0b11111111},
  {0x27, 0b11111111}
};

/* -------------------- I2C Helpers -------------------- */
void writeByteToI2C(uint8_t deviceAddress, uint8_t data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(data);
  Wire.endTransmission();
}

/* -------------------- Control Functions -------------------- */
void closeAll() {
  for (size_t i = 0; i < ARRAY_LEN(offLUT); i++) {
    writeByteToI2C(offLUT[i][0], offLUT[i][1]);
  }
}

void ColClose() {
  for (size_t i = 0; i < ARRAY_LEN(offCOLLUT); i++) {
    writeByteToI2C(offCOLLUT[i][0], offCOLLUT[i][1]);
  }
}

void RowsClose() {
  for (size_t i = 0; i < ARRAY_LEN(offROWLUT); i++) {
    writeByteToI2C(offROWLUT[i][0], offROWLUT[i][1]);
  }
}

void ColumnsCheck() {
  for (size_t i = 0; i < ARRAY_LEN(colLUT); i++) {
    ColClose();
    writeByteToI2C(colLUT[i][0], colLUT[i][1]);
    delay(100);
  }
  ColClose();
}

void RowsCheck() {
  for (size_t i = 0; i < ARRAY_LEN(rowLUTforOPENrowsONLY); i++) {
    writeByteToI2C(rowLUTforOPENrowsONLY[i][0],
                   rowLUTforOPENrowsONLY[i][1]);
    delay(i == 8 ? 100 : DelayX);
  }
}

void OpenCellLeft(int delay_before, int delay_after) {
  for (size_t r = 0; r < ARRAY_LEN(rowLUT); r++) {
    RowsClose();
    writeByteToI2C(rowLUT[r][0], rowLUT[r][1]);
    delay(delay_before);

    for (size_t c = 0; c < ARRAY_LEN(colLUT); c++) {
      ColClose();
      writeByteToI2C(colLUT[c][0], colLUT[c][1]);
      delay(delay_after);
    }
  }
  RowsClose();
}

void OpenCellRR(int delay_before, int delay_after) {
  for (size_t r = 0; r < ARRAY_LEN(rowLUT); r++) {
    RowsClose();
    writeByteToI2C(rowLUT[r][0], rowLUT[r][1]);
    delay(delay_before);

    for (size_t c = 0; c < ARRAY_LEN(colLUTRR); c++) {
      ColClose();
      writeByteToI2C(colLUTRR[c][0], colLUTRR[c][1]);
      delay(delay_after);
    }
  }
  RowsClose();
}

void OpenCellbyCell() {
  for (size_t r = 0; r < ARRAY_LEN(rowLUT); r++) {
    for (size_t c = 0; c < ARRAY_LEN(colLUT); c++) {
      writeByteToI2C(rowLUT[r][0], rowLUT[r][1]);
      delay(150);
      writeByteToI2C(colLUT[c][0], colLUT[c][1]);
      delay(150);
      RowsClose();
      ColClose();
      delay(150);
    }
  }
  closeAll();
}

/* -------------------- Inputs -------------------- */
void get_inputs_status() {
  bitInOne   = !digitalRead(IN1);
  bitInTwo   = !digitalRead(IN2);
  bitInThree = !digitalRead(IN3);
  start_bit  = digitalRead(StartPin);
        

}

/* -------------------- Setup -------------------- */
void setup() {
  Serial.begin(9600);

  Wire.begin();
  Wire.setClock(100000);

  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);
  pinMode(IN3, INPUT);
  pinMode(StartPin, INPUT_PULLUP);
  pinMode(13, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(StartPin), startBitISR, FALLING);
}

/* -------------------- Loop -------------------- */
void loop() {
  get_inputs_status();

  switch (StateMachine) {
    case CHECK_INPUT:
      if (!bitInOne && !bitInTwo && bitInThree && start_bit)       StateMachine = OPEN_ROWS;
      else if (!bitInOne && bitInTwo && !bitInThree && start_bit)  StateMachine = OPEN_COLUMNS;
      else if (!bitInOne && bitInTwo && bitInThree && start_bit)   StateMachine = OPEN_CELL_FROM_LEFT;
      else if (bitInOne && !bitInTwo && !bitInThree && start_bit)  StateMachine = OPEN_CELL_FROM_RIGHT;
      else if (bitInOne && !bitInTwo && bitInThree && start_bit)   StateMachine = OPEN_CELL_BY_CELL;
      else if (bitInOne && bitInTwo && bitInThree && start_bit)    StateMachine = TEST_ALL;

      break;

    case OPEN_ROWS:
      RowsCheck();
      closeAll();
      delay(1000);
      StateMachine = IDLE;
      break;

    case OPEN_COLUMNS:
      ColumnsCheck();
      closeAll();
      delay(1000);
      StateMachine = IDLE;
      break;

    case OPEN_CELL_FROM_LEFT:
      OpenCellLeft(100, 50);
      closeAll();
      delay(1000);
      StateMachine = IDLE;
      break;

    case OPEN_CELL_FROM_RIGHT:
      OpenCellRR(100, 50);
      closeAll();
      delay(1000);
      StateMachine = IDLE;
      break;

    case OPEN_CELL_BY_CELL:
      OpenCellbyCell();
      closeAll();
      delay(1000);
      StateMachine = IDLE;
      break;

    case TEST_ALL:
      RowsCheck();        closeAll(); delay(5000);
      ColumnsCheck();     closeAll(); delay(5000);
      OpenCellRR(350,150);closeAll(); delay(5000);
      OpenCellLeft(350,150);closeAll();delay(5000);
      OpenCellbyCell();   closeAll(); delay(5000);
      break;

    case IDLE:
      StateMachine = CHECK_INPUT;
      break;
  }
}
