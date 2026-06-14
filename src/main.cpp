#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#define ARRAY_LEN(x) (sizeof(x) / sizeof(x[0]))

/* -------------------- Pins -------------------- */
const int IN1 = 4;
const int IN2 = 7;
const int IN3 = 8;
const int StartPin = 2;

/* -------------------- Timing -------------------- */
// Delay between row activations in RowsCheck (ms)
constexpr int ROW_DELAY_MS    = 2200;
// Minimum ms between valid button presses (debounce)
constexpr unsigned long DEBOUNCE_MS = 200;

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
// ISR only sets a flag — no millis() call here.
// millis() relies on Timer0 interrupt which is disabled inside an ISR,
// so calling it from an ISR returns a stale/frozen value on AVR.
// Debounce is handled safely in loop() where millis() works correctly.
volatile bool startPressed = false;
unsigned long lastPressMs  = 0;   // accessed only in loop(), no volatile needed

void startBitISR() {
  startPressed = true;
}

/* -------------------- Globals -------------------- */
int bitInOne, bitInTwo, bitInThree;

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

/* Number of physical columns (without the OFF sentinel) */
constexpr size_t NUM_COLS = 18;

uint8_t colLUT[][2] = {
  {0x22, 0b11011111},
  {0x22, 0b11101111},
  {0x22, 0b11110111},
  {0x22, 0b01111011},  // NOTE: two zero bits (bit7 + bit2) — activates 2 outputs on 0x22
                       // simultaneously. Verify this matches the physical matrix wiring.
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
  {0x20, 0b11111110}
  // Note: 0xFF (all-off) sentinel removed — use ColClose() explicitly
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
  {0x22, 0b11011111}
  // Note: 0xFF sentinel removed — use ColClose() explicitly
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
    RowsClose();  // ensure previous row is off before activating next
    writeByteToI2C(rowLUTforOPENrowsONLY[i][0],
                   rowLUTforOPENrowsONLY[i][1]);
    delay(i == 8 ? 100 : ROW_DELAY_MS);
  }
  RowsClose();  // ensure all rows off when done
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
    ColClose();  // ensure last column is off before closing the row
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
    ColClose();  // ensure last column is off before closing the row
  }
  RowsClose();
}

void OpenCellbyCell() {
  for (size_t r = 0; r < ARRAY_LEN(rowLUT); r++) {
    for (size_t c = 0; c < ARRAY_LEN(colLUT); c++) {
      // Intentional: row and col are briefly open together so the
      // cell junction can be probed. Close both before moving on.
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
}

/* -------------------- Setup -------------------- */
void setup() {
  Serial.begin(9600);

  Wire.begin();
  Wire.setClock(100000);

  // Use INPUT_PULLUP so floating pins read HIGH (inactive) by default.
  // get_inputs_status() inverts with !, so pulled-up = bit inactive = 0.
  pinMode(IN1, INPUT_PULLUP);
  pinMode(IN2, INPUT_PULLUP);
  pinMode(IN3, INPUT_PULLUP);
  pinMode(StartPin, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  attachInterrupt(digitalPinToInterrupt(StartPin), startBitISR, FALLING);
}

/* -------------------- Loop -------------------- */
// Helper: atomically write a new state
static inline void setState(state s) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { StateMachine = s; }
}

void loop() {
  // Handle start button press (flag set by ISR).
  // Debounce is done here where millis() is reliable (not inside ISR).
  bool pressed;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { pressed = startPressed; }
  if (pressed) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { startPressed = false; }
    if (millis() - lastPressMs >= DEBOUNCE_MS) {
      lastPressMs = millis();
      setState(CHECK_INPUT);
    }
  }

  // Read StateMachine atomically to avoid a race with the ISR.
  // On AVR, enum is 16-bit and reads/writes are not atomic.
  state currentState;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { currentState = StateMachine; }

  get_inputs_status();

  switch (currentState) {
    case CHECK_INPUT:
      if      (!bitInOne && !bitInTwo &&  bitInThree) setState(OPEN_ROWS);
      else if (!bitInOne &&  bitInTwo && !bitInThree) setState(OPEN_COLUMNS);
      else if (!bitInOne &&  bitInTwo &&  bitInThree) setState(OPEN_CELL_FROM_LEFT);
      else if ( bitInOne && !bitInTwo && !bitInThree) setState(OPEN_CELL_FROM_RIGHT);
      else if ( bitInOne && !bitInTwo &&  bitInThree) setState(OPEN_CELL_BY_CELL);
      else if ( bitInOne &&  bitInTwo &&  bitInThree) setState(TEST_ALL);
      break;

    case OPEN_ROWS:
      // Clear any button press queued during the previous idle period
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { startPressed = false; }
      digitalWrite(13, HIGH);
      RowsCheck();
      closeAll();
      delay(1000);
      digitalWrite(13, LOW);
      setState(IDLE);
      break;

    case OPEN_COLUMNS:
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { startPressed = false; }
      digitalWrite(13, HIGH);
      ColumnsCheck();
      closeAll();
      delay(1000);
      digitalWrite(13, LOW);
      setState(IDLE);
      break;

    case OPEN_CELL_FROM_LEFT:
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { startPressed = false; }
      digitalWrite(13, HIGH);
      OpenCellLeft(100, 50);
      closeAll();
      delay(1000);
      digitalWrite(13, LOW);
      setState(IDLE);
      break;

    case OPEN_CELL_FROM_RIGHT:
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { startPressed = false; }
      digitalWrite(13, HIGH);
      OpenCellRR(100, 50);
      closeAll();
      delay(1000);
      digitalWrite(13, LOW);
      setState(IDLE);
      break;

    case OPEN_CELL_BY_CELL:
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { startPressed = false; }
      digitalWrite(13, HIGH);
      OpenCellbyCell();
      closeAll();
      delay(1000);
      digitalWrite(13, LOW);
      setState(IDLE);
      break;

    case TEST_ALL:
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { startPressed = false; }
      digitalWrite(13, HIGH);
      RowsCheck();             closeAll(); delay(5000);
      ColumnsCheck();          closeAll(); delay(5000);
      OpenCellRR(350, 150);   closeAll(); delay(5000);
      OpenCellLeft(350, 150); closeAll(); delay(5000);
      OpenCellbyCell();        closeAll(); delay(5000);
      digitalWrite(13, LOW);
      setState(IDLE);
      break;

    case IDLE:
      setState(CHECK_INPUT);
      break;
  }
}
