#pragma once

#include "wled.h"
#include "esp-knx-ip.h"

#ifndef USERMOD_ID_KNX_IP
#define USERMOD_ID_KNX_IP 0xA902
#endif

class KnxIpUsermod : public Usermod {
public:
  // --- Config values (editable via JSON/UI) ---
  bool  enabled = true;
  char  individualAddr[16] = "1.1.100";

  // Inbound GAs (commands from KNX -> WLED)
  char  gaInPower[16]   = "1/0/1";   // DPT 1.001 (switch)
  char  gaInBri[16]     = "1/0/2";   // DPT 5.001 (0..100%)
  char  gaInR[16]       = "1/1/1";   // DPT 5.010 (0..255)
  char  gaInG[16]       = "1/1/2";   // DPT 5.010 (0..255)
  char  gaInB[16]       = "1/1/3";   // DPT 5.010 (0..255)
  char  gaInW[16]       = "1/1/4";   // DPT 5.010 (0..255)
  char  gaInCct[16]     = "1/1/5";   // DPT 7.600 (Kelvin)
  char  gaInWW[16]      = "1/1/6";   // DPT 5.010 (0..255)
  char  gaInCW[16]      = "1/1/7";   // DPT 5.010 (0..255)
  char  gaInFx[16]      = "1/2/1";   // DPT 5.xxx (0..255)
  char  gaInPreset[16]  = "1/2/2";   // DPT 5.xxx (0..255)

  // Outbound GAs (state feedback WLED -> KNX)
  char  gaOutPower[16]  = "2/0/1";    // DPT 1.001
  char  gaOutBri[16]    = "2/0/2";    // DPT 5.001 (0..100%)
  char  gaOutR[16]      = "2/1/1";    // DPT 5.010 (0..255)
  char  gaOutG[16]      = "2/1/2";    // DPT 5.010 (0..255)
  char  gaOutB[16]      = "2/1/3";    // DPT 5.010 (0..255)
  char  gaOutW[16]      = "2/1/4";    // DPT 5.010 (0..255)
  char  gaOutCct[16]    = "2/1/5";    // DPT 7.600 (Kelvin)
  char  gaOutWW[16]     = "2/1/6";    // DPT 5.010 (0..255)
  char  gaOutCW[16]     = "2/1/7";    // DPT 5.010 (0..255)
  char  gaOutFx[16]     = "2/2/1";    // DPT 5.xxx
  char  gaOutPreset[16] = "2/2/2";    // DPT 5.xxx (0..255)

  // TX coalescing
  uint16_t txRateLimitMs = 200;

  // Periodic state publish (optional)
  bool     periodicEnabled    = false;
  uint32_t periodicIntervalMs = 10000; // 10s

  // CCT mapping range (Kelvin) — configurable
  uint16_t kelvinMin = 2700;
  uint16_t kelvinMax = 6500;

  // --- Usermod API ---
  void setup();
  void loop();
  void addToConfig(JsonObject& root);
  bool readFromConfig(JsonObject& root);
  uint16_t getId() { return USERMOD_ID_KNX_IP; }
  const char* getName() { return "KNX_IP"; }

  void appendConfigData(Print& uiScript) override;  // enable UI hook

private:
  // --- TX coalescing flags/timer ---
  unsigned long _nextTxAt = 0;
  bool _pendingTxPower = false, _pendingTxBri = false, _pendingTxFx = false;
  uint32_t _lastPeriodicMs = 0;   // last time we scheduled a periodic publish
  
  // --- GA caches ---
  uint16_t GA_IN_W   = 0, GA_IN_CCT = 0, GA_IN_WW = 0, GA_IN_CW = 0;
  uint16_t GA_OUT_W  = 0, GA_OUT_CCT= 0, GA_OUT_WW = 0, GA_OUT_CW = 0;

  // Track last preset value we set (used for OUT if configured)
  uint8_t _lastPreset = 0;
  uint8_t  LAST_R = 0, LAST_G = 0, LAST_B = 0;
  uint8_t  LAST_W = 0;     // 0..255
  uint8_t  LAST_CCT = 127; // 0=warm, 255=cold

  // Outbound state
  void publishState();
  void scheduleStatePublish(bool pwr, bool bri, bool fx);

  // handlers (KNX -> WLED)
  void onKnxPower(bool on);
  void onKnxBrightness(uint8_t pct); // 0..100
  void onKnxRGB(uint8_t r, uint8_t g, uint8_t b);
  void onKnxEffect(uint8_t fxIndex);
  void onKnxPreset(uint8_t preset);
  void onKnxWhite(uint8_t v);      // 0..255
  void onKnxCct(uint16_t kelvin);  // Kelvin (DPT 7.600)
  void onKnxWW(uint8_t v);         // 0..255
  void onKnxCW(uint8_t v);         // 0..255

  // helper to apply current LAST_W/LAST_CCT to the active color
  void applyWhiteAndCct();

  // mapping helpers
  uint8_t  kelvinToCct255(uint16_t k) const;
  uint16_t cct255ToKelvin(uint8_t cct) const;

  // Change-tracking to publish on GUI updates (debounced)
  uint8_t  _lastSentBri = 255;
  bool     _lastSentOn  = true;
  uint32_t _lastUiSendMs = 0;
  uint8_t  _lastFxSent      = 0xFF;   // last effect index we published
  int16_t  _lastPresetSent  = -1;     // last preset number we published
  const uint16_t _minUiSendIntervalMs = 300;  // debounce window
};
