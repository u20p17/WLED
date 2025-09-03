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
  char  gaInPower[16]   = "1/0/1";   // DPT 1.001
  char  gaInBri[16]     = "1/0/2";   // DPT 5.001 (0..100%)
  char  gaInR[16]       = "1/1/1";   // DPT 5.001 (0..100%) per channel
  char  gaInG[16]       = "1/1/2";   // DPT 5.001 (0..100%) per channel
  char  gaInB[16]       = "1/1/3";   // DPT 5.001 (0..100%) per channel
  char  gaInW[16]       = "1/1/4";   // White level (DPT 5.001 0..100%)
  char  gaInCct[16]     = "1/1/5";   // CCT mix (0=warm .. 100=cold) DPT 5.001
  char  gaInWW[16]      = "1/1/6";   // Optional: Warm white level  (DPT 5.001)
  char  gaInCW[16]      = "1/1/7";   // Optional: Cold white level  (DPT 5.001)
  char  gaInFx[16]      = "1/2/1";   // DPT 5.xxx (0..255)
  char  gaInPreset[16]  = "1/2/2";   // DPT 5.xxx (WLED preset)
  
  // Outbound GAs (state feedback WLED -> KNX)
  char  gaOutPower[16]  = "2/0/1";    // DPT 1.001
  char  gaOutBri[16]    = "2/0/2";    // DPT 5.001
  char  gaOutR[16]      = "2/1/1";    // DPT 5.001 (0..100%) per channel
  char  gaOutG[16]      = "2/1/2";    // DPT 5.001 (0..100%) per channel
  char  gaOutB[16]      = "2/1/3";    // DPT 5.001 (0..100%) per channel
  char  gaOutW[16]      = "2/1/4";    // White level (DPT 5.001)
  char  gaOutCct[16]    = "2/1/5";    // CCT mix (DPT 5.001)
  char  gaOutWW[16]     = "2/1/6";    // Optional: derived warm white (DPT 5.001)
  char  gaOutCW[16]     = "2/1/7";    // Optional: derived cold white (DPT 5.001)
  char  gaOutFx[16]     = "2/2/1";    // DPT 5.xxx
  char  gaOutPreset[16] = "2/2/2";    // DPT 5.xxx (0..255) – last applied/known preset

  // TX coalescing
  uint16_t txRateLimitMs = 200;

  // --- Usermod API ---
  void setup();
  void loop();
  void addToConfig(JsonObject& root);
  bool readFromConfig(JsonObject& root);
  uint16_t getId() { return USERMOD_ID_KNX_IP; }
  const char* getName() { return "KNX_IP"; }

private:
  // --- TX coalescing flags/timer ---
  unsigned long _nextTxAt = 0;
  bool _pendingTxPower = false, _pendingTxBri = false, _pendingTxFx = false;

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
  void onKnxWhite(uint8_t pct);
  void onKnxCct(uint8_t pct);
  void onKnxWW(uint8_t pct);
  void onKnxCW(uint8_t pct);

  // helper to apply current LAST_W/LAST_CCT to the active color
  void applyWhiteAndCct();
};
