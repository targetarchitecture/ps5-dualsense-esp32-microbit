/*
 * PS5 (DualSense) -> ESP32 (Bluepad32) -> I2C Slave -> BBC micro:bit
 *
 * WIRING (3.3V logic both sides):
 *   ESP32 GPIO21 (SDA) -- micro:bit P20 (SDA)
 *   ESP32 GPIO22 (SCL) -- micro:bit P19 (SCL)
 *   ESP32 GND          -- micro:bit GND
 *   External 4.7k pull-ups on SDA and SCL to 3.3V recommended.
 */

#include <Bluepad32.h>
#include <Wire.h>

#define I2C_SLAVE_ADDR      0x42
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22
#define I2C_FREQ_HZ         100000
#define FAILSAFE_TIMEOUT_MS 500
#define STICK_DEADZONE_RAW  16   // ~3% of Bluepad32's -512..512 range
#define RUMBLE_DURATION_MS  200  // Default duration per rumble trigger

// Command bitmasks from micro:bit
#define CMD_RUMBLE 0x01
#define CMD_COLOR  0x02

// High-byte button bitmasks (bits 10..13 in the 16-bit buttons word)
#define BTN_BIT_SELECT  (1 << 10)
#define BTN_BIT_START   (1 << 11)
#define BTN_BIT_SYSTEM  (1 << 12)
#define BTN_BIT_CAPTURE (1 << 13)

// ---- Shared state sent over I2C (10 bytes, packed) ----
struct __attribute__((packed)) PadState {
  uint8_t connected;   // 0 or 1
  uint8_t buttons_lo;  // Bits 0..7  (A, B, X, Y, L1, R1, L2, R2)
  uint8_t buttons_hi;  // Bits 8..13 (ThumbL, ThumbR, Select, Start, System, Capture)
  uint8_t dpad;        // Bit0=Up, Bit1=Down, Bit2=Right, Bit3=Left
  int8_t  leftX;       // -127..127
  int8_t  leftY;       // -127..127
  int8_t  rightX;      // -127..127
  int8_t  rightY;      // -127..127
  uint8_t brake;       // L2: 0..255
  uint8_t throttle;    // R2: 0..255
};
static_assert(sizeof(PadState) == 10, "PadState must be exactly 10 bytes");

// ---- Command frame received FROM the micro:bit (6 bytes, packed) ----
struct __attribute__((packed)) CommandFrame {
  uint8_t cmd;         // Bit0 = apply rumble, Bit1 = apply color
  uint8_t rumbleLeft;  // Strong motor: 0..255
  uint8_t rumbleRight; // Weak motor:   0..255
  uint8_t r;
  uint8_t g;
  uint8_t b;
};
static_assert(sizeof(CommandFrame) == 6, "CommandFrame must be exactly 6 bytes");

ControllerPtr myControllers[BP32_MAX_GAMEPADS] = {nullptr};

// Spinlock protecting shared state between main loop and I2C ISR callbacks
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

PadState sharedPadState = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
CommandFrame pendingCommand = {0, 0, 0, 0, 0, 0};
unsigned long lastUpdateMs = 0;

// Scale Bluepad32's -512..512 stick range to -127..127 with centre deadzone
static inline int8_t scaleAxis(int32_t v) {
  if (v > -STICK_DEADZONE_RAW && v < STICK_DEADZONE_RAW) {
    return 0;
  }
  int32_t scaled = v / 4;
  if (scaled > 127)  return 127;
  if (scaled < -127) return -127;
  return static_cast<int8_t>(scaled);
}

// Scale Bluepad32's 0..1023 trigger range to 0..255
static inline uint8_t scaleTrigger(int32_t v) {
  int32_t scaled = v / 4;
  if (scaled > 255) return 255;
  if (scaled < 0)   return 0;
  return static_cast<uint8_t>(scaled);
}

// Combine standard buttons (bits 0..9) with miscButtons (bits 10..13)
static inline uint16_t packAllButtons(ControllerPtr ctl) {
  uint16_t btns = ctl->buttons() & 0x03FF;
  uint8_t misc  = ctl->miscButtons();

  if (misc & MISC_BUTTON_SELECT)  btns |= BTN_BIT_SELECT;
  if (misc & MISC_BUTTON_START)   btns |= BTN_BIT_START;
  if (misc & MISC_BUTTON_SYSTEM)  btns |= BTN_BIT_SYSTEM;
  if (misc & MISC_BUTTON_CAPTURE) btns |= BTN_BIT_CAPTURE;

  return btns;
}

bool hasActiveController() {
  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected()) {
      return true;
    }
  }
  return false;
}

// Zeroes controls while preserving or clearing connection state atomically
void setFailsafeState(bool stillConnected) {
  PadState zeroed = {
    static_cast<uint8_t>(stillConnected ? 1 : 0),
    0, 0, 0, 0, 0, 0, 0, 0, 0
  };
  portENTER_CRITICAL(&stateMux);
  sharedPadState = zeroed;
  portEXIT_CRITICAL(&stateMux);
}

// ---- I2C Slave Callbacks (ISR Context) ----
void onI2CRequest() {
  PadState snapshot;
  portENTER_CRITICAL_ISR(&stateMux);
  snapshot = sharedPadState;
  portEXIT_CRITICAL_ISR(&stateMux);

  Wire.write(reinterpret_cast<const uint8_t*>(&snapshot), sizeof(PadState));
}

void onI2CReceive(int numBytes) {
  uint8_t buf[sizeof(CommandFrame)];
  size_t idx = 0;

  while (Wire.available()) {
    uint8_t b = Wire.read();
    if (idx < sizeof(CommandFrame)) {
      buf[idx] = b;
    }
    idx++;
  }

  if (idx == sizeof(CommandFrame)) {
    const CommandFrame* rx = reinterpret_cast<const CommandFrame*>(buf);

    // Merge flags so back-to-back Rumble and Color packets don't clobber each other
    portENTER_CRITICAL_ISR(&stateMux);
    if (rx->cmd & CMD_RUMBLE) {
      pendingCommand.cmd |= CMD_RUMBLE;
      pendingCommand.rumbleLeft  = rx->rumbleLeft;
      pendingCommand.rumbleRight = rx->rumbleRight;
    }
    if (rx->cmd & CMD_COLOR) {
      pendingCommand.cmd |= CMD_COLOR;
      pendingCommand.r = rx->r;
      pendingCommand.g = rx->g;
      pendingCommand.b = rx->b;
    }
    portEXIT_CRITICAL_ISR(&stateMux);
  }
}

void applyPendingCommand() {
  CommandFrame cmdCopy;

  portENTER_CRITICAL(&stateMux);
  cmdCopy = pendingCommand;
  pendingCommand.cmd = 0;
  portEXIT_CRITICAL(&stateMux);

  if (cmdCopy.cmd == 0) return;

  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected()) {
      if (cmdCopy.cmd & CMD_RUMBLE) {
        // playDualRumble(delayedStartMs, durationMs, weakMagnitude, strongMagnitude)
        uint16_t duration = (cmdCopy.rumbleLeft == 0 && cmdCopy.rumbleRight == 0) ? 0 : RUMBLE_DURATION_MS;
        ctl->playDualRumble(0, duration, cmdCopy.rumbleRight, cmdCopy.rumbleLeft);
      }
      if (cmdCopy.cmd & CMD_COLOR) {
        ctl->setColorLED(cmdCopy.r, cmdCopy.g, cmdCopy.b);
      }
    }
  }
}

// ---- Bluepad32 Callbacks ----
void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("Controller connected, slot %d\n", i);
      myControllers[i] = ctl;

      ctl->setColorLED(0, 255, 0);
      ctl->setPlayerLEDs(0x04); // Center white player indicator LED on DualSense

      portENTER_CRITICAL(&stateMux);
      sharedPadState.connected = 1;
      portEXIT_CRITICAL(&stateMux);

      lastUpdateMs = millis();
      break;
    }
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("Controller disconnected from slot %d\n", i);
      myControllers[i] = nullptr;
      break;
    }
  }
  setFailsafeState(hasActiveController());
}

void updatePadState(ControllerPtr ctl) {
  uint16_t buttons = packAllButtons(ctl);

  PadState nextState;
  nextState.connected  = 1;
  nextState.buttons_lo = buttons & 0xFF;
  nextState.buttons_hi = (buttons >> 8) & 0xFF;
  nextState.dpad       = ctl->dpad();
  nextState.leftX      = scaleAxis(ctl->axisX());
  nextState.leftY      = scaleAxis(ctl->axisY());
  nextState.rightX     = scaleAxis(ctl->axisRX());
  nextState.rightY     = scaleAxis(ctl->axisRY());
  nextState.brake      = scaleTrigger(ctl->brake());
  nextState.throttle   = scaleTrigger(ctl->throttle());

  portENTER_CRITICAL(&stateMux);
  sharedPadState = nextState;
  portEXIT_CRITICAL(&stateMux);

  lastUpdateMs = millis();
}

void processControllers() {
  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected() && ctl->hasData() && ctl->isGamepad()) {
      updatePadState(ctl);
      break; // Read primary active controller to prevent multi-slot overwrites
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.printf("Bluepad32 firmware: %s\n", BP32.firmwareVersion());

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);

  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);

  // Register callbacks BEFORE Wire.begin() so early master reads are handled cleanly
  Wire.onRequest(onI2CRequest);
  Wire.onReceive(onI2CReceive);
  Wire.begin(static_cast<uint8_t>(I2C_SLAVE_ADDR), I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);

  Serial.printf("I2C slave ready on 0x%02X (SDA=%d, SCL=%d)\n",
                I2C_SLAVE_ADDR, I2C_SDA_PIN, I2C_SCL_PIN);
}

void loop() {
  if (BP32.update()) {
    processControllers();
  }

  // If no HID reports arrive within FAILSAFE_TIMEOUT_MS, zero sticks/triggers/buttons
  // while keeping `connected` true if the Bluetooth link is still up.
  if (millis() - lastUpdateMs > FAILSAFE_TIMEOUT_MS) {
    setFailsafeState(hasActiveController());
    lastUpdateMs = millis();
  }

  applyPendingCommand();

  delay(5); // 200 Hz loop pacing (matches DualSense BT polling better than 10ms)
}