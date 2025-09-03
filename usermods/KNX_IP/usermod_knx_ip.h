#pragma once

#include "wled.h"
#include "esp-knx-ip.h"

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
  char  gaInFx[16]      = "1/2/1";   // DPT 5.xxx (0..255)
  char  gaInPreset[16]  = "1/2/2";   // DPT 5.xxx (WLED preset)

  // Outbound GAs (state feedback WLED -> KNX)
  char  gaOutPower[16]  = "2/0/1";    // DPT 1.001
  char  gaOutBri[16]    = "2/0/2";    // DPT 5.001
  char  gaOutFx[16]     = "2/2/1";    // DPT 5.xxx
  char  gaOutR[16]       = "";        // DPT 5.001 (0..100%) per channel
  char  gaOutG[16]       = "";        // DPT 5.001 (0..100%) per channel
  char  gaOutB[16]       = "";        // DPT 5.001 (0..100%) per channel
  char  gaOutPreset[16]  = "";        // DPT 5.xxx (0..255) – last applied/known preset
  
  // TX coalescing
  uint16_t txRateLimitMs = 200;

  // --- Usermod API ---
  void setup();
  void loop();
  void addToConfig(JsonObject& root);
  bool readFromConfig(JsonObject& root);
  uint16_t getId() { return USERMOD_ID_RESERVED; }   // change to a free ID later
  const char* getName() { return "KNX_IP"; }

private:
  // --- TX coalescing flags/timer ---
  unsigned long _nextTxAt = 0;
  bool _pendingTxPower = false, _pendingTxBri = false, _pendingTxFx = false;

  // Track last preset value we set (used for OUT if configured)
  uint8_t _lastPreset = 0;

  // Outbound state
  void publishState();
  void scheduleStatePublish(bool pwr, bool bri, bool fx);

  // handlers (KNX -> WLED)
  void onKnxPower(bool on);
  void onKnxBrightness(uint8_t pct); // 0..100
  void onKnxRGB(uint8_t r, uint8_t g, uint8_t b);
  void onKnxEffect(uint8_t fxIndex);
  void onKnxPreset(uint8_t preset);
};
