#pragma once

#include <RadioLib.h>

// Classic ESP32 (esp32dev) VSPI + non-strapping control pins
// NOTE: Do NOT use Waveshare's ESP32-S3 pinout (GPIO6/7 are internal flash on esp32dev).
#define PIN_LR1121_SCK    18
#define PIN_LR1121_MISO   19
#define PIN_LR1121_MOSI   23
#define PIN_LR1121_CS     5
#define PIN_LR1121_RESET  27
#define PIN_LR1121_BUSY   26
#define PIN_LR1121_DIO9   25

// Default Sub-GHz test frequency inside Core1121-HF band (850-930 MHz).
// Change to 868000000UL for EU868 or keep 915000000UL for US915/AU915.
#define CORE1121_TEST_FREQ_HZ    915000000UL
#define CORE1121_SAFE_TX_DBM     0

// Core1121-XF PE4259 RF switch mapping (from Core1121_XF_Sch.pdf + Waveshare lr1121_common.c)
// DIO5 (RFSW0_V1) -> PE4259 VDD, DIO6 (RFSW1_V2) -> PE4259 CTRL
static const uint32_t CORE1121_RFSW_PINS[] = {
  RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6,
  RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC
};

static const Module::RfSwitchMode_t CORE1121_RFSW_TABLE[] = {
  // mode                   DIO5   DIO6
  { LR11x0::MODE_STBY,    { LOW,   LOW  } },
  { LR11x0::MODE_RX,      { HIGH,  LOW  } },
  { LR11x0::MODE_TX,      { LOW,   HIGH } },
  { LR11x0::MODE_TX_HP,   { LOW,   HIGH } },
  { LR11x0::MODE_TX_HF,   { LOW,   LOW  } },
  { LR11x0::MODE_GNSS,    { LOW,   LOW  } },
  { LR11x0::MODE_WIFI,    { LOW,   LOW  } },
  END_OF_MODE_TABLE,
};
