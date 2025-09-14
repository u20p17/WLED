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
  char  gaInH[16]       = "1/1/8";   // DPT 5.003 (Hue 0..360° scaled to 0..255)
  char  gaInS[16]       = "1/1/9";   // DPT 5.001 (0..100% -> 0..255)
  char  gaInV[16]       = "1/1/10";  // DPT 5.001 (0..100% -> 0..255)
  char  gaInFx[16]      = "1/1/11";  // DPT 5.xxx (0..255)
  char  gaInPreset[16]  = "1/1/12";  // DPT 5.xxx (0..255)
  char  gaInRGB[16]     = "1/1/13";  // DPST-232-600, 3 bytes [R,G,B]
  char  gaInHSV[16]     = "1/1/14";  // DPST-232-600, 3 bytes [H_byte,S_byte,V_byte]
  char  gaInRGBW[16]    = "1/1/15";  // DPST-251-600, 6 bytes [R,G,B,W,ext1,ext2]

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
  char  gaOutH[16]      = "2/1/8";    // DPT 5.003 (Hue)
  char  gaOutS[16]      = "2/1/9";    // DPT 5.001 (S)
  char  gaOutV[16]      = "2/1/10";   // DPT 5.001 (V)
  char  gaOutFx[16]     = "2/1/11";   // DPT 5.xxx
  char  gaOutPreset[16] = "2/1/12";   // DPT 5.xxx (0..255)
  char  gaOutRGB[16]    = "2/1/13";   // DPST-232-600, 3 bytes
  char  gaOutHSV[16]    = "2/1/14";   // DPST-232-600, 3 bytes
  char  gaOutRGBW[16]   = "2/1/15";   // DPST-251-600, 6 bytes

  // TX coalescing
  uint16_t txRateLimitMs = 200;

  // Periodic state publish (optional)
  bool     periodicEnabled    = false;
  uint32_t periodicIntervalMs = 10000; // 10s

  // CCT mapping range (Kelvin) — configurable
  uint16_t kelvinMin = 2700;
  uint16_t kelvinMax = 6500;

  // in class KnxIpUsermod private/public fields (config)
  bool     commEnhance        = true;   // Tasmota-style enhancement enable
  uint8_t  commResends        = 3;      // how many total sends
  uint16_t commResendGapMs    = 0;      // gap between repeats
  uint16_t commRxDedupMs      = 700;    // duplicate window


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
  uint16_t GA_IN_RGB = 0, GA_IN_HSV = 0, GA_IN_RGBW = 0;
  uint16_t GA_IN_H = 0, GA_IN_S = 0, GA_IN_V = 0;
  uint16_t GA_OUT_RGB = 0, GA_OUT_HSV = 0, GA_OUT_RGBW = 0;
  uint16_t GA_OUT_H = 0, GA_OUT_S = 0, GA_OUT_V = 0;

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
  void onKnxRGBW(uint8_t r, uint8_t g, uint8_t b, uint8_t w);
  void onKnxHSV(float hDeg, float s01, float v01);
  void onKnxH(float hDeg);
  void onKnxS(float s01);
  void onKnxV(float v01);

  // helper to apply current LAST_W/LAST_CCT to the active color
  void applyWhiteAndCct();
  static void rgbToHsv(uint8_t r, uint8_t g, uint8_t b, float& hDeg, float& s01, float& v01);
  static void hsvToRgb(float hDeg, float s01, float v01, uint8_t& r, uint8_t& g, uint8_t& b);
  static inline uint8_t hueDegToByte(float hDeg) { while (hDeg<0) hDeg+=360.f; while (hDeg>=360) hDeg-=360.f; return (uint8_t)roundf(hDeg * 255.f / 360.f); }
  static inline float   byteToHueDeg(uint8_t hb) { return (hb * 360.f) / 255.f; }
  static inline uint8_t pct01ToByte(float p)     { if (p<0) p=0; if (p>1) p=1; return (uint8_t)roundf(p*255.f); }
  static inline float   byteToPct01(uint8_t b)   { return b / 255.f; }

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
