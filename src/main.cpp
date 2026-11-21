#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "pin_config.h"

// Thin diagnostic subclass to expose protected LR11x0 hardware commands
// without enabling global RADIOLIB_GODMODE.
class Core1121Diag : public LR1121 {
  public:
    explicit Core1121Diag(Module* mod) : LR1121(mod) {}

    int16_t diagGetErrors(uint16_t* err) { return this->getErrors(err); }
    int16_t diagClearErrors() { return this->clearErrors(); }
    int16_t diagCalibrate(uint8_t params) { return this->calibrate(params); }
    int16_t diagConfigLfClock(uint8_t setup) { return this->configLfClock(setup); }
    int16_t diagGetVbat(float* vbat) { return this->getVbat(vbat); }

    // Workaround for RadioLib 8.0.0 sign bug in LR11x0::getTemp() (LR11x0_commands.cpp:202).
    // Semtech LR1121 User Manual formula: T(C) = 25 - (1000 / 1.7) * ((raw / 2047) * 1.35 - 0.7295)
    int16_t diagGetTemp(float* temp, uint16_t* rawOut = nullptr) {
      uint8_t buff[2] = { 0 };
      int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GET_TEMP, false, buff, sizeof(buff));
      if (state == RADIOLIB_ERR_NONE) {
        uint16_t raw = ((static_cast<uint16_t>(buff[0]) << 8) | static_cast<uint16_t>(buff[1])) & 0x07FF;
        if (rawOut) *rawOut = raw;
        if (temp) {
          float vSensor = (static_cast<float>(raw) / 2047.0f) * 1.35f;
          *temp = 25.0f - (1000.0f / 1.7f) * (vSensor - 0.7295f);
        }
      }
      return state;
    }

    int16_t diagGetChipEui(uint8_t* eui) { return this->getChipEui(eui); }
    int16_t diagGetSemtechJoinEui(uint8_t* eui) { return this->getSemtechJoinEui(eui); }
    int16_t diagGetRandomNumber(uint32_t* rnd) { return this->getRandomNumber(rnd); }
    int16_t diagGetRssiInst(float* rssi) { return this->getRssiInst(rssi); }
};

static Module radioMod(PIN_LR1121_CS, PIN_LR1121_DIO9, PIN_LR1121_RESET, PIN_LR1121_BUSY);
static Core1121Diag radio(&radioMod);

static volatile bool g_dio9Fired = false;
static volatile uint32_t g_dio9IrqCount = 0;
static bool g_radioReady = false;
static uint32_t g_txCounter = 0;

static void IRAM_ATTR onDio9Irq() {
  g_dio9Fired = true;
  g_dio9IrqCount = g_dio9IrqCount + 1;
}

static void printSeparator(char ch = '=', size_t len = 64) {
  for (size_t i = 0; i < len; i++) {
    Serial.print(ch);
  }
  Serial.println();
}

static void printHexBytes(const uint8_t* buf, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (buf[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(buf[i], HEX);
    if (i + 1 < len) {
      Serial.print(':');
    }
  }
}

static void printErrorFlags(uint16_t err) {
  if (err == 0) {
    Serial.println(F("    Error bits: NONE (0x0000)"));
    return;
  }
  Serial.printf("    Error bits (0x%04X):", err);
  if (err & RADIOLIB_LR11X0_ERROR_STAT_LF_RC_CALIB_ERR)    Serial.print(F(" LF_RC_CALIB"));
  if (err & RADIOLIB_LR11X0_ERROR_STAT_HF_RC_CALIB_ERR)    Serial.print(F(" HF_RC_CALIB"));
  if (err & RADIOLIB_LR11X0_ERROR_STAT_ADC_CALIB_ERR)      Serial.print(F(" ADC_CALIB"));
  if (err & RADIOLIB_LR11X0_ERROR_STAT_PLL_CALIB_ERR)      Serial.print(F(" PLL_CALIB"));
  if (err & RADIOLIB_LR11X0_ERROR_STAT_IMG_CALIB_ERR)      Serial.print(F(" IMG_CALIB"));
  if (err & RADIOLIB_LR11X0_ERROR_STAT_HF_XOSC_START_ERR)  Serial.print(F(" HF_XOSC_START"));
  if (err & RADIOLIB_LR11X0_ERROR_STAT_LF_XOSC_START_ERR)  Serial.print(F(" LF_XOSC_START"));
  if (err & RADIOLIB_LR11X0_ERROR_STAT_PLL_LOCK_ERR)       Serial.print(F(" PLL_LOCK"));
  if (err & RADIOLIB_LR11X0_ERROR_STAT_RX_ADC_OFFSET_ERR)  Serial.print(F(" RX_ADC_OFFSET"));
  Serial.println();
}

static bool sampleSubGhzRssi(uint32_t freqHz, float* avgRssiOut, float* minRssiOut, float* maxRssiOut, bool verbose = false) {
  int16_t state = radio.standby();
  if (state != RADIOLIB_ERR_NONE) {
    if (verbose) Serial.printf("    [ERR] standby() @ %.1f MHz failed: %d (BUSY=%d)\n", freqHz / 1e6, state, digitalRead(PIN_LR1121_BUSY));
    return false;
  }

  state = radio.setFrequency(freqHz);
  if (state != RADIOLIB_ERR_NONE) {
    if (verbose) Serial.printf("    [ERR] setFrequency(%.1f MHz) failed: %d (BUSY=%d)\n", freqHz / 1e6, state, digitalRead(PIN_LR1121_BUSY));
    return false;
  }

  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    if (verbose) Serial.printf("    [ERR] startReceive() @ %.1f MHz failed: %d (BUSY=%d)\n", freqHz / 1e6, state, digitalRead(PIN_LR1121_BUSY));
    return false;
  }
  delay(10);

  constexpr int kSamples = 16;
  float sum = 0.0f;
  float minVal = 100.0f;
  float maxVal = -200.0f;
  int valid = 0;
  int16_t lastRssiErr = RADIOLIB_ERR_NONE;

  for (int i = 0; i < kSamples; i++) {
    float rssi = 0.0f;
    int16_t rState = radio.diagGetRssiInst(&rssi);
    if (rState == RADIOLIB_ERR_NONE) {
      sum += rssi;
      if (rssi < minVal) minVal = rssi;
      if (rssi > maxVal) maxVal = rssi;
      valid++;
    } else {
      lastRssiErr = rState;
    }
    delay(3);
  }

  radio.standby();
  if (valid == 0) {
    if (verbose) Serial.printf("    [ERR] getRssiInst() @ %.1f MHz failed: %d (BUSY=%d)\n", freqHz / 1e6, lastRssiErr, digitalRead(PIN_LR1121_BUSY));
    return false;
  }

  *avgRssiOut = sum / valid;
  *minRssiOut = minVal;
  *maxRssiOut = maxVal;
  return true;
}

static bool test1_ResetAndBusy() {
  Serial.println(F("\n[TEST 1/8] Hardware RESET & BUSY Handshake"));

  pinMode(PIN_LR1121_CS, OUTPUT);
  digitalWrite(PIN_LR1121_CS, HIGH);
  pinMode(PIN_LR1121_BUSY, INPUT);
  pinMode(PIN_LR1121_DIO9, INPUT);
  pinMode(PIN_LR1121_RESET, OUTPUT);

  // Pull RESET low then release high
  digitalWrite(PIN_LR1121_RESET, LOW);
  delay(15);
  int busyDuringReset = digitalRead(PIN_LR1121_BUSY);
  digitalWrite(PIN_LR1121_RESET, HIGH);

  bool sawBusyHigh = (busyDuringReset == HIGH);
  uint32_t t0 = millis();
  while (millis() - t0 < 500) {
    if (digitalRead(PIN_LR1121_BUSY) == HIGH) {
      sawBusyHigh = true;
    } else if (millis() - t0 > 5) {
      break;
    }
    delay(1);
  }

  // Wait for BUSY to settle LOW
  t0 = millis();
  while (digitalRead(PIN_LR1121_BUSY) == HIGH && (millis() - t0 < 500)) {
    delay(1);
  }
  int busyAfter = digitalRead(PIN_LR1121_BUSY);
  uint32_t settleMs = millis() - t0;

  Serial.printf("    BUSY during reset: %d, saw BUSY pulse: %s, BUSY settled: %d (%lu ms)\n",
                busyDuringReset, sawBusyHigh ? "yes" : "no", busyAfter, static_cast<unsigned long>(settleMs));

  if (busyAfter != LOW) {
    Serial.println(F("    [FAIL] BUSY stayed HIGH after reset! Check 3.3V, GND, RESET (GPIO27), BUSY (GPIO26)."));
    return false;
  }

  Serial.println(F("    [PASS] Module reset completed and BUSY is LOW (ready)."));
  return true;
}

static bool test2_SpiAndInit() {
  Serial.println(F("\n[TEST 2/8] SPI Bus & LR1121 Initialization (3.0V TCXO + DCDC + PE4259)"));

  SPI.end();
  SPI.begin(PIN_LR1121_SCK, PIN_LR1121_MISO, PIN_LR1121_MOSI, PIN_LR1121_CS);

  // Core1121-XF uses a 3.0V TCXO (Waveshare lr1121_config.cpp: LR11XX_SYSTEM_TCXO_CTRL_3_0V)
  radio.tcxoVoltage = RadioLibTCXOVoltage_t::Voltage3V0;

  ConfigLoRa_t cfg;
  cfg.frequency = CORE1121_TEST_FREQ_HZ;
  cfg.bandwidth = RADIOLIB_UNIT_KILO(125);
  cfg.spreadingFactor = 7;
  cfg.codingRate = 5;
  cfg.syncWord = RADIOLIB_LORA_SYNC_WORD_PRIVATE;
  cfg.power = CORE1121_SAFE_TX_DBM;
  cfg.preambleLength = 8;

  int16_t state = radio.begin(cfg);
  if (state == RADIOLIB_ERR_UNSUPPORTED_FIRMWARE) {
    Serial.println(F("    [FAIL] SPI responded, but module has non-Transceiver firmware (e.g. LoRaWAN Modem)."));
    Serial.println(F("           Use Waveshare's lr1121_firmware_update to restore Transceiver firmware if needed."));
    return false;
  }
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("    [FAIL] radio.begin() failed with code %d.\n", state);
    if (state == RADIOLIB_ERR_CHIP_NOT_FOUND) {
      Serial.println(F("           Check SPI wiring: CLK=18, MISO=19, MOSI=23, CS=5, RESET=27, BUSY=26, 3V3, GND."));
    }
    return false;
  }

  // Match Waveshare's ~10 ms TCXO startup timeout (300 ticks @ 30.52 us)
  state = radio.setTCXO(RadioLibTCXOVoltage_t::Voltage3V0, 10000);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("    [FAIL] setTCXO(3.0V, 10ms) failed with code %d.\n", state);
    return false;
  }

  // Core1121-XF schematic: L6 inductor populated on DCC_SW -> DCDC supported
  state = radio.setRegulatorDCDC();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("    [FAIL] setRegulatorDCDC() failed with code %d.\n", state);
    return false;
  }

  // Core1121-XF schematic: PE4259 RF switch controlled via DIO5 (RX) & DIO6 (TX/TX_HP)
  radio.setRfSwitchTable(CORE1121_RFSW_PINS, CORE1121_RFSW_TABLE);

  // Core1121-XF schematic: R3=0R populates RFO_HP_LF, R4=NC disconnects RFO_LP_LF -> forceHighPower = true
  state = radio.setOutputPower(CORE1121_SAFE_TX_DBM, true);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("    [FAIL] setOutputPower(%d dBm, HP) failed with code %d.\n", CORE1121_SAFE_TX_DBM, state);
    return false;
  }

  Serial.printf("    Configured: %.3f MHz, LoRa SF7 BW125 CR4/5, TCXO 3.0V, DCDC, HP PA (%d dBm)\n",
                CORE1121_TEST_FREQ_HZ / 1e6, CORE1121_SAFE_TX_DBM);
  Serial.println(F("    [PASS] SPI and LR1121 initialization succeeded."));
  return true;
}

static bool test3_IdentityAndEui() {
  Serial.println(F("\n[TEST 3/8] Chip Identity, Firmware Version & Unique EUI"));

  LR11x0VersionInfo_t info{};
  int16_t state = radio.getVersionInfo(&info);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("    [FAIL] getVersionInfo() failed with code %d.\n", state);
    return false;
  }

  const char* devName = "UNKNOWN";
  if (info.device == RADIOLIB_LR11X0_DEVICE_LR1110)      devName = "LR1110";
  else if (info.device == RADIOLIB_LR11X0_DEVICE_LR1120) devName = "LR1120";
  else if (info.device == RADIOLIB_LR11X0_DEVICE_LR1121) devName = "LR1121";

  Serial.printf("    Device:   %s (0x%02X)\n", devName, info.device);
  Serial.printf("    Hardware: 0x%02X\n", info.hardware);
  Serial.printf("    Firmware: %02X.%02X\n", info.fwMajor, info.fwMinor);

  uint8_t chipEui[8] = {0};
  uint8_t joinEui[8] = {0};
  int16_t euiState = radio.diagGetChipEui(chipEui);
  int16_t joinState = radio.diagGetSemtechJoinEui(joinEui);

  if (euiState == RADIOLIB_ERR_NONE) {
    Serial.print(F("    ChipEUI:  "));
    printHexBytes(chipEui, sizeof(chipEui));
    Serial.println(F("  <-- Record this ID to distinguish Module #1 vs #2"));
  } else {
    Serial.printf("    [WARN] getChipEui() returned %d\n", euiState);
  }

  if (joinState == RADIOLIB_ERR_NONE) {
    Serial.print(F("    JoinEUI:  "));
    printHexBytes(joinEui, sizeof(joinEui));
    Serial.println();
  }

  if (info.device != RADIOLIB_LR11X0_DEVICE_LR1121) {
    Serial.printf("    [FAIL] Expected LR1121 (0x03), got 0x%02X.\n", info.device);
    return false;
  }

  Serial.println(F("    [PASS] Confirmed authentic Semtech LR1121 silicon."));
  return true;
}

static bool test4_PowerAndTemp() {
  Serial.println(F("\n[TEST 4/8] Supply Voltage (VBAT) & On-Chip Temperature"));

  float vbat = 0.0f;
  float temp = 0.0f;
  uint16_t tempRaw = 0;
  int16_t sVbat = radio.diagGetVbat(&vbat);
  int16_t sTemp = radio.diagGetTemp(&temp, &tempRaw);

  if (sVbat != RADIOLIB_ERR_NONE || sTemp != RADIOLIB_ERR_NONE) {
    Serial.printf("    [FAIL] Sensor read failed (vbat=%d, temp=%d).\n", sVbat, sTemp);
    return false;
  }

  Serial.printf("    VBAT: %.3f V (expected 2.80 - 3.70 V)\n", vbat);
  Serial.printf("    Temp: %.2f C (raw=0x%03X, expected -10.0 - 75.0 C)\n", temp, tempRaw);

  bool vbatOk = (vbat >= 2.80f && vbat <= 3.70f);
  bool tempOk = (temp >= -10.0f && temp <= 75.0f);

  if (!vbatOk || !tempOk) {
    Serial.println(F("    [FAIL] VBAT or Temperature out of sane operating range!"));
    return false;
  }

  Serial.println(F("    [PASS] Power rail and internal ADC sensors are healthy."));
  return true;
}

static bool test5_OscillatorsAndCalibration() {
  Serial.println(F("\n[TEST 5/8] Oscillators (32 MHz TCXO + 32.768 kHz XTAL) & Calibration"));

  int16_t state = radio.standby();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("    [FAIL] standby() failed (%d).\n", state);
    return false;
  }

  // Core1121-XF schematic populates Y2 (32.768 kHz crystal on DIO10/DIO11)
  state = radio.diagConfigLfClock(RADIOLIB_LR11X0_LF_BUSY_RELEASE_DISABLED | RADIOLIB_LR11X0_LF_CLK_XOSC);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("    [FAIL] configLfClock(XOSC) failed (%d).\n", state);
    return false;
  }

  radio.diagClearErrors();

  state = radio.diagCalibrate(RADIOLIB_LR11X0_CALIBRATE_ALL);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("    [FAIL] calibrate(ALL) failed (%d).\n", state);
    return false;
  }

  // Image rejection calibration across Core1121-HF Sub-GHz band (850-930 MHz)
  state = radio.calibrateImageRejection(RADIOLIB_UNIT_MEGA(850), RADIOLIB_UNIT_MEGA(930));
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("    [FAIL] calibrateImageRejection(850-930 MHz) failed (%d).\n", state);
    return false;
  }

  uint16_t err = 0xFFFF;
  state = radio.diagGetErrors(&err);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("    [FAIL] getErrors() failed (%d).\n", state);
    return false;
  }

  printErrorFlags(err);

  // Switch LF clock back to internal RC before RX/TX tests because RadioLib's
  // internal setDioIrqParams(irq) maps IRQ2 onto DIO11 when not in GNSS mode.
  radio.diagConfigLfClock(RADIOLIB_LR11X0_LF_BUSY_RELEASE_DISABLED | RADIOLIB_LR11X0_LF_CLK_RC);

  if (err != 0) {
    Serial.println(F("    [FAIL] LR1121 reported hardware/oscillator/calibration error flags!"));
    return false;
  }

  Serial.println(F("    [PASS] Both crystals (32 MHz TCXO & 32.768 kHz XTAL) and all calibrations passed."));
  return true;
}

static bool test6_HardwareTrng() {
  Serial.println(F("\n[TEST 6/8] Hardware True Random Number Generator (TRNG)"));

  uint32_t samples[4] = {0};
  for (int i = 0; i < 4; i++) {
    int16_t state = radio.diagGetRandomNumber(&samples[i]);
    if (state != RADIOLIB_ERR_NONE) {
      Serial.printf("    [FAIL] getRandomNumber() failed at index %d (%d).\n", i, state);
      return false;
    }
  }

  Serial.printf("    Samples: 0x%08lX  0x%08lX  0x%08lX  0x%08lX\n",
                static_cast<unsigned long>(samples[0]),
                static_cast<unsigned long>(samples[1]),
                static_cast<unsigned long>(samples[2]),
                static_cast<unsigned long>(samples[3]));

  bool allSame = (samples[0] == samples[1] && samples[1] == samples[2] && samples[2] == samples[3]);
  bool allZero = (samples[0] == 0 && samples[1] == 0 && samples[2] == 0 && samples[3] == 0);
  if (allSame || allZero) {
    Serial.println(F("    [FAIL] TRNG returned constant/zero values."));
    return false;
  }

  Serial.println(F("    [PASS] Hardware TRNG producing non-repeating entropy."));
  return true;
}

static bool test7_SubGhzRxAndCad() {
  Serial.println(F("\n[TEST 7/8] Sub-GHz RX Front-End, RSSI Noise Floor & LoRa CAD"));

  float avg868 = 0, min868 = 0, max868 = 0;
  float avg915 = 0, min915 = 0, max915 = 0;

  bool ok868 = sampleSubGhzRssi(RADIOLIB_UNIT_MEGA(868), &avg868, &min868, &max868, true);
  bool ok915 = sampleSubGhzRssi(RADIOLIB_UNIT_MEGA(915), &avg915, &min915, &max915, true);

  if (!ok868 || !ok915) {
    uint16_t err = 0;
    if (radio.diagGetErrors(&err) == RADIOLIB_ERR_NONE) {
      printErrorFlags(err);
    }
    Serial.println(F("    [FAIL] Unable to read instantaneous RSSI in RX mode."));
    return false;
  }

  Serial.printf("    868.0 MHz RSSI: avg %.1f dBm (min %.1f, max %.1f)\n", avg868, min868, max868);
  Serial.printf("    915.0 MHz RSSI: avg %.1f dBm (min %.1f, max %.1f)\n", avg915, min915, max915);

  // Restore configured test frequency and test LoRa CAD demodulator
  radio.setFrequency(CORE1121_TEST_FREQ_HZ);
  int16_t cadState = radio.scanChannel();
  const char* cadStr = "ERROR";
  if (cadState == RADIOLIB_CHANNEL_FREE)       cadStr = "CHANNEL_FREE";
  else if (cadState == RADIOLIB_LORA_DETECTED) cadStr = "LORA_DETECTED";
  Serial.printf("    LoRa CAD @ %.3f MHz: %s (code %d)\n", CORE1121_TEST_FREQ_HZ / 1e6, cadStr, cadState);

  bool rssiSane = (avg868 <= -40.0f && avg868 >= -135.0f && avg915 <= -40.0f && avg915 >= -135.0f);
  bool cadOk = (cadState == RADIOLIB_CHANNEL_FREE || cadState == RADIOLIB_LORA_DETECTED);

  if (!rssiSane || !cadOk) {
    Serial.println(F("    [FAIL] RSSI noise floor or LoRa CAD check failed."));
    return false;
  }

  Serial.println(F("    [PASS] Sub-GHz RX front-end (PE4259 RX path) and LoRa demodulator verified."));
  return true;
}

static bool test8_Dio9IrqAndSafeTx() {
  Serial.println(F("\n[TEST 8/8] DIO9 Hardware IRQ & Safe Sub-GHz TX (0 dBm, HP PA)"));
  Serial.println(F("    NOTE: Sub-GHz antenna must be attached to LORA_ANT (ANT2)."));

  int16_t prepState = radio.standby();
  if (prepState != RADIOLIB_ERR_NONE) {
    Serial.printf("    [FAIL] standby() before TX failed: %d (BUSY=%d)\n", prepState, digitalRead(PIN_LR1121_BUSY));
    return false;
  }
  prepState = radio.setFrequency(CORE1121_TEST_FREQ_HZ);
  if (prepState != RADIOLIB_ERR_NONE) {
    Serial.printf("    [FAIL] setFrequency() before TX failed: %d (BUSY=%d)\n", prepState, digitalRead(PIN_LR1121_BUSY));
    return false;
  }
  prepState = radio.setOutputPower(CORE1121_SAFE_TX_DBM, true);
  if (prepState != RADIOLIB_ERR_NONE) {
    Serial.printf("    [FAIL] setOutputPower() before TX failed: %d (BUSY=%d)\n", prepState, digitalRead(PIN_LR1121_BUSY));
    return false;
  }

  const char* payload = "CORE1121-HF-SELF-TEST";
  size_t payloadLen = strlen(payload);
  RadioLibTime_t toaUs = radio.getTimeOnAir(payloadLen);
  uint32_t expectedToaMs = static_cast<uint32_t>((toaUs + 999) / 1000);

  g_dio9Fired = false;
  uint32_t irqBefore = g_dio9IrqCount;
  radio.setIrqAction(onDio9Irq);

  uint32_t tStart = millis();
  int16_t state = radio.startTransmit(reinterpret_cast<const uint8_t*>(payload), payloadLen);
  if (state != RADIOLIB_ERR_NONE) {
    radio.clearIrqAction();
    Serial.printf("    [FAIL] startTransmit() failed with code %d (BUSY=%d).\n", state, digitalRead(PIN_LR1121_BUSY));
    uint16_t err = 0;
    if (radio.diagGetErrors(&err) == RADIOLIB_ERR_NONE) {
      printErrorFlags(err);
    }
    return false;
  }

  uint32_t timeoutMs = expectedToaMs + 500;
  while (!g_dio9Fired && (millis() - tStart < timeoutMs)) {
    delay(1);
  }
  uint32_t elapsedMs = millis() - tStart;

  int16_t finishState = radio.finishTransmit();
  radio.clearIrqAction();

  float vbatAfter = 0.0f;
  uint16_t errAfter = 0xFFFF;
  radio.diagGetVbat(&vbatAfter);
  radio.diagGetErrors(&errAfter);

  Serial.printf("    Expected ToA: ~%lu ms | Measured DIO9 IRQ: %lu ms | IRQ delta: %lu\n",
                static_cast<unsigned long>(expectedToaMs),
                static_cast<unsigned long>(elapsedMs),
                static_cast<unsigned long>(g_dio9IrqCount - irqBefore));
  Serial.printf("    Post-TX VBAT: %.3f V | Errors: 0x%04X | finishTransmit: %d\n",
                vbatAfter, errAfter, finishState);

  if (!g_dio9Fired) {
    Serial.println(F("    [FAIL] TX timed out — DIO9 hardware interrupt did NOT fire!"));
    Serial.printf("           Check DIO9 wiring to GPIO%d.\n", PIN_LR1121_DIO9);
    return false;
  }
  if (finishState != RADIOLIB_ERR_NONE || errAfter != 0) {
    printErrorFlags(errAfter);
    Serial.println(F("    [FAIL] TX completed with error status."));
    return false;
  }

  Serial.println(F("    [PASS] Sub-GHz TX + PE4259 TX path + DIO9 hardware IRQ verified."));
  return true;
}

static void printMenu() {
  Serial.println();
  printSeparator('-');
  Serial.println(F("Interactive Commands (send single character at 115200 baud):"));
  Serial.println(F("  r  - Re-run full 8-step diagnostic suite"));
  Serial.println(F("  s  - Sub-GHz RSSI spectrum sweep (850-930 MHz, 2 MHz step)"));
  Serial.println(F("  w  - 2.4 GHz RX-ONLY RSSI sweep (2400-2484 MHz, no TX)"));
  Serial.println(F("  t  - Send one Sub-GHz LoRa test packet (0 dBm) & check DIO9 IRQ"));
  Serial.println(F("  l  - Continuous Sub-GHz RX listen mode (any key to stop)"));
  Serial.println(F("  h  - Show this command menu"));
  printSeparator('-');
}

static void runFullDiagnostics() {
  printSeparator('=');
  Serial.println(F("  Waveshare Core1121-HF (LR1121) Single-Module Diagnostic Suite"));
  Serial.printf("  ESP32 Pins: CLK=%d MISO=%d MOSI=%d CS=%d RST=%d BUSY=%d DIO9=%d\n",
                PIN_LR1121_SCK, PIN_LR1121_MISO, PIN_LR1121_MOSI, PIN_LR1121_CS,
                PIN_LR1121_RESET, PIN_LR1121_BUSY, PIN_LR1121_DIO9);
  printSeparator('=');

  int passed = 0;
  constexpr int kTotal = 8;

  bool t1 = test1_ResetAndBusy();
  if (t1) passed++;

  bool t2 = test2_SpiAndInit();
  if (t2) passed++;
  g_radioReady = t2;

  if (g_radioReady) {
    if (test3_IdentityAndEui())          passed++;
    if (test4_PowerAndTemp())            passed++;
    if (test5_OscillatorsAndCalibration()) passed++;
    if (test6_HardwareTrng())            passed++;
    if (test7_SubGhzRxAndCad())          passed++;
    if (test8_Dio9IrqAndSafeTx())        passed++;
  } else {
    Serial.println(F("\n[SKIP] Skipping Tests 3-8 because SPI/LR1121 initialization failed."));
  }

  Serial.println();
  printSeparator('=');
  if (passed == kTotal) {
    Serial.printf("  RESULT: %d/%d PASSED -- MODULE IS HEALTHY!\n", passed, kTotal);
  } else {
    Serial.printf("  RESULT: %d/%d PASSED -- CHECK FAILURES ABOVE!\n", passed, kTotal);
  }
  printSeparator('=');
  Serial.println(F("TIP: Unplug USB power before swapping Module #1 and Module #2."));
  printMenu();
}

static void runRssiSweep(uint32_t startMHz, uint32_t endMHz, uint32_t stepMHz, const char* label) {
  if (!g_radioReady) {
    Serial.println(F("[ERR] Radio not initialized. Press 'r' to run diagnostics first."));
    return;
  }

  Serial.printf("\n--- %s (%lu - %lu MHz, RX-only) ---\n",
                label, static_cast<unsigned long>(startMHz), static_cast<unsigned long>(endMHz));

  for (uint32_t mhz = startMHz; mhz <= endMHz; mhz += stepMHz) {
    float avg = 0.0f, minV = 0.0f, maxV = 0.0f;
    if (sampleSubGhzRssi(mhz * 1000000UL, &avg, &minV, &maxV)) {
      // Map -125 dBm .. -45 dBm to 0..40 bar chars
      int barLen = static_cast<int>((avg + 125.0f) * 0.5f);
      if (barLen < 0) barLen = 0;
      if (barLen > 40) barLen = 40;

      Serial.printf("  %4lu MHz | avg %6.1f dBm (peak %6.1f) | ",
                    static_cast<unsigned long>(mhz), avg, maxV);
      for (int i = 0; i < barLen; i++) {
        Serial.print('#');
      }
      Serial.println();
    } else {
      Serial.printf("  %4lu MHz | [READ ERROR]\n", static_cast<unsigned long>(mhz));
    }
  }

  // Always restore Sub-GHz frequency and standby after any sweep (especially 2.4 GHz)
  radio.standby();
  radio.setFrequency(CORE1121_TEST_FREQ_HZ);
  radio.setOutputPower(CORE1121_SAFE_TX_DBM, true);
  Serial.printf("--- Sweep done. Restored %.3f MHz standby. ---\n", CORE1121_TEST_FREQ_HZ / 1e6);
}

static void runSingleTx() {
  if (!g_radioReady) {
    Serial.println(F("[ERR] Radio not initialized. Press 'r' first."));
    return;
  }

  radio.standby();
  radio.setFrequency(CORE1121_TEST_FREQ_HZ);
  radio.setOutputPower(CORE1121_SAFE_TX_DBM, true);

  char msg[48];
  snprintf(msg, sizeof(msg), "CORE1121-PING #%lu", static_cast<unsigned long>(++g_txCounter));

  g_dio9Fired = false;
  radio.setIrqAction(onDio9Irq);
  uint32_t t0 = millis();
  int16_t state = radio.startTransmit(reinterpret_cast<const uint8_t*>(msg), strlen(msg));
  if (state != RADIOLIB_ERR_NONE) {
    radio.clearIrqAction();
    Serial.printf("[TX] startTransmit failed (%d)\n", state);
    return;
  }

  while (!g_dio9Fired && (millis() - t0 < 2000)) {
    delay(1);
  }
  uint32_t dt = millis() - t0;
  int16_t fin = radio.finishTransmit();
  radio.clearIrqAction();

  if (g_dio9Fired && fin == RADIOLIB_ERR_NONE) {
    Serial.printf("[TX] OK: \"%s\" @ %.3f MHz, %d dBm (DIO9 IRQ in %lu ms)\n",
                  msg, CORE1121_TEST_FREQ_HZ / 1e6, CORE1121_SAFE_TX_DBM, static_cast<unsigned long>(dt));
  } else {
    Serial.printf("[TX] FAIL: dio9=%d, finish=%d, elapsed=%lu ms\n",
                  g_dio9Fired ? 1 : 0, fin, static_cast<unsigned long>(dt));
  }
}

static void runRxListen() {
  if (!g_radioReady) {
    Serial.println(F("[ERR] Radio not initialized. Press 'r' first."));
    return;
  }

  radio.standby();
  radio.setFrequency(CORE1121_TEST_FREQ_HZ);

  g_dio9Fired = false;
  radio.setIrqAction(onDio9Irq);
  int16_t state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    radio.clearIrqAction();
    Serial.printf("[RX] startReceive failed (%d)\n", state);
    return;
  }

  Serial.printf("\n[RX] Listening on %.3f MHz (LoRa SF7 BW125)... Send any key to stop.\n",
                CORE1121_TEST_FREQ_HZ / 1e6);

  uint32_t lastRssiPrint = millis();
  while (!Serial.available()) {
    if (g_dio9Fired) {
      g_dio9Fired = false;
      String str;
      int16_t rxState = radio.readData(str);
      if (rxState == RADIOLIB_ERR_NONE) {
        Serial.printf("[RX] Packet: \"%s\" | RSSI: %.1f dBm | SNR: %.1f dB\n",
                      str.c_str(), radio.getRSSI(), radio.getSNR());
      } else {
        Serial.printf("[RX] Packet error (code %d)\n", rxState);
      }
      radio.startReceive();
    }

    if (millis() - lastRssiPrint >= 1000) {
      lastRssiPrint = millis();
      float instRssi = 0.0f;
      if (radio.diagGetRssiInst(&instRssi) == RADIOLIB_ERR_NONE) {
        Serial.printf("[RX] Live RSSI: %.1f dBm\n", instRssi);
      }
    }
    delay(5);
  }

  // Drain trigger key
  while (Serial.available()) {
    Serial.read();
  }
  radio.clearIrqAction();
  radio.standby();
  Serial.println(F("[RX] Stopped listening."));
}

void setup() {
  Serial.begin(115200);
  delay(1500);  // Allow USB-UART monitor to attach cleanly
  runFullDiagnostics();
}

void loop() {
  if (!Serial.available()) {
    delay(10);
    return;
  }

  char cmd = static_cast<char>(Serial.read());
  if (cmd == '\r' || cmd == '\n' || cmd == ' ') {
    return;
  }

  switch (cmd) {
    case 'r':
    case 'R':
      runFullDiagnostics();
      break;
    case 's':
    case 'S':
      runRssiSweep(850, 930, 2, "Sub-GHz RSSI Spectrum Sweep");
      break;
    case 'w':
    case 'W':
      runRssiSweep(2400, 2484, 4, "2.4 GHz RX-ONLY RSSI Sweep (No 2.4G TX)");
      break;
    case 't':
    case 'T':
      runSingleTx();
      break;
    case 'l':
    case 'L':
      runRxListen();
      break;
    case 'h':
    case 'H':
    default:
      printMenu();
      break;
  }
}
