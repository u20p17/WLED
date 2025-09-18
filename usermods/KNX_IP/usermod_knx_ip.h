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
  char  gaInTime[16]    = "1/7/1";   // DPT 10.001, TimeOfDay (3 bytes)
  char  gaInDate[16]    = "1/7/2";   // DPT 11.001, Date (3 bytes)
  char  gaInDateTime[16]= "1/7/3";   // DPT 19.001, DateTime (8 bytes)
  // NEW: Relative adjustment inbound GAs using DPT 3.007 (4-bit step+direction, "Dimmen Relativ")
  // each telegram encodes one relative step (no continuous ramp maintained here).
  // Mapping (bit3=direction 0=decrease 1=increase, bits2..0 step code):
  // 0=STOP (ignored), 1=100, 2=50, 3=25, 4=12, 5=6, 6=3, 7=1 (% of full scale for brightness, scaled for channels)
  char  gaInBriRel[16]  = "1/0/3";   // DPT 3.007 relative brightness
  char  gaInRRel[16]    = "1/1/16";  // DPT 3.007 relative Red
  char  gaInGRel[16]    = "1/1/17";  // DPT 3.007 relative Green
  char  gaInBRel[16]    = "1/1/18";  // DPT 3.007 relative Blue
  char  gaInWRel[16]    = "1/1/19";  // DPT 3.007 relative White
  char  gaInWWRel[16]   = "1/1/20";  // DPT 3.007 relative Warm component (translated to W+CCT)
  char  gaInCWRel[16]   = "1/1/21";  // DPT 3.007 relative Cold component (translated to W+CCT)
  char  gaInHRel[16]    = "1/1/22";  // DPT 3.007 relative Hue (treated as step degrees)
  char  gaInSRel[16]    = "1/1/23";  // DPT 3.007 relative Sat
  char  gaInVRel[16]    = "1/1/24";  // DPT 3.007 relative Value
  char  gaInFxRel[16]   = "1/1/25";  // DPT 3.007 relative Effect index
  char  gaInRGBRel[16]  = "1/1/26";  // DPST-232-600 relative RGB (3 bytes: R,G,B each DPT3 low nibble)
  char  gaInHSVRel[16]  = "1/1/27";  // DPST-232-600 relative HSV (3 bytes: H,S,V each DPT3 low nibble)
  char  gaInRGBWRel[16] = "1/1/28";  // DPST-251-600 relative RGBW (4 or 6 bytes: R,G,B,W DPT3)

  // Outbound GAs (state feedback WLED -> KNX)
  char  gaOutPower[16]   = "2/0/1";    // DPT 1.001
  char  gaOutBri[16]     = "2/0/2";    // DPT 5.001 (0..100%)
  char  gaOutR[16]       = "2/1/1";    // DPT 5.010 (0..255)
  char  gaOutG[16]       = "2/1/2";    // DPT 5.010 (0..255)
  char  gaOutB[16]       = "2/1/3";    // DPT 5.010 (0..255)
  char  gaOutW[16]       = "2/1/4";    // DPT 5.010 (0..255)
  char  gaOutCct[16]     = "2/1/5";    // DPT 7.600 (Kelvin)
  char  gaOutWW[16]      = "2/1/6";    // DPT 5.010 (0..255)
  char  gaOutCW[16]      = "2/1/7";    // DPT 5.010 (0..255)
  char  gaOutH[16]       = "2/1/8";    // DPT 5.003 (Hue)
  char  gaOutS[16]       = "2/1/9";    // DPT 5.001 (S)
  char  gaOutV[16]       = "2/1/10";   // DPT 5.001 (V)
  char  gaOutFx[16]      = "2/1/11";   // DPT 5.xxx
  char  gaOutPreset[16]  = "2/1/12";   // DPT 5.xxx (0..255)
  char  gaOutRGB[16]     = "2/1/13";   // DPST-232-600, 3 bytes
  char  gaOutHSV[16]     = "2/1/14";   // DPST-232-600, 3 bytes
  char  gaOutRGBW[16]    = "2/1/15";   // DPST-251-600, 6 bytes
  char  gaOutIntTemp[16] = "2/2/1";   // DPST-14-68 (4-byte float °C)
  char  gaOutTemp[16]    = "2/2/2";   // DPST-14-68 (4-byte float °C) - classic Temperature usermod  
  char  gaOutIntTempAlarm[16] = "2/2/3";   // ESP internal temp alarm GA (1-bit Alarm)
  char  gaOutTempAlarm[16]    = "2/2/4";   // Dallas temp alarm GA (1-bit Alarm)

  // --- Alarm (DPST-1-5) configuration (°C thresholds) ---
  float intTempAlarmMaxC      = 80.0f;      // trip threshold for ESP internal (°C)
  float dallasTempAlarmMaxC   = 80.0f;      // trip threshold for Dallas (°C)
  float tempAlarmHystC        = 1.0f;       // hysteresis (°C) to clear the alarm


  // TX coalescing
  uint16_t txRateLimitMs = 200;

  // Periodic state publish (optional)
  bool     periodicEnabled    = false;
  uint32_t periodicIntervalMs = 10000; // 10s

  // CCT mapping range (Kelvin) — configurable
  uint16_t kelvinMin = 2700;
  uint16_t kelvinMax = 6500;

  // in class KnxIpUsermod private/public fields (config)
  bool     commEnhance        = false;   // Tasmota-style enhancement enable
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

  // --- Validation helpers (callable before saving config) ---
  static bool validateGroupAddressString(const char* s);  // "x/y/z" within KNX 3-level limits
  static bool validateIndividualAddressString(const char* s); // "a.b.c" within area/line/device limits

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
  uint16_t GA_IN_PWR = 0, GA_IN_BRI = 0, GA_IN_R = 0, GA_IN_G = 0;
  uint16_t GA_IN_B = 0, GA_IN_FX = 0, GA_IN_PRESET = 0, GA_IN_PRE = 0;
  uint16_t GA_IN_TIME = 0, GA_IN_DATE = 0, GA_IN_DATETIME = 0;
  uint16_t GA_IN_BRI_REL = 0, GA_IN_R_REL = 0, GA_IN_G_REL = 0, GA_IN_B_REL = 0;
  uint16_t GA_IN_W_REL = 0, GA_IN_WW_REL = 0, GA_IN_CW_REL = 0;
  uint16_t GA_IN_H_REL = 0, GA_IN_S_REL = 0, GA_IN_V_REL = 0, GA_IN_FX_REL = 0;
  uint16_t GA_IN_RGB_REL = 0, GA_IN_HSV_REL = 0, GA_IN_RGBW_REL = 0;

  uint16_t GA_OUT_RGB = 0, GA_OUT_HSV = 0, GA_OUT_RGBW = 0;
  uint16_t GA_OUT_H = 0, GA_OUT_S = 0, GA_OUT_V = 0;
  uint16_t GA_OUT_INT_TEMP = 0, GA_OUT_TEMP = 0;
  uint16_t GA_OUT_INT_TEMP_ALARM = 0, GA_OUT_TEMP_ALARM = 0;
  uint16_t GA_OUT_PWR = 0, GA_OUT_BRI = 0, GA_OUT_R = 0, GA_OUT_G = 0;
  uint16_t GA_OUT_B = 0, GA_OUT_FX = 0, GA_OUT_PRESET = 0, GA_OUT_PRE = 0;


  // Track last preset value we set (used for OUT if configured)
  uint8_t _lastPreset = 0;
  uint8_t  LAST_R = 0, LAST_G = 0, LAST_B = 0;
  uint8_t  LAST_W = 0;     // 0..255
  uint8_t  LAST_CCT = 127; // 0=warm, 255=cold

  // Last alarm states to send only on change
  bool lastIntTempAlarmState     = false;
  bool lastDallasTempAlarmState  = false;

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
  void onKnxTime_10_001(const uint8_t* p, uint8_t len);     // expects 3 bytes -> time_of_day_t
  void onKnxDate_11_001(const uint8_t* p, uint8_t len);     // expects 3 bytes -> date_t
  void onKnxDateTime_19_001(const uint8_t* p, uint8_t len); // expects 8 bytes
  void onKnxBrightnessRel(uint8_t dpt3);
  void onKnxColorRel(uint8_t channel, uint8_t dpt3); // channel: 0=R 1=G 2=B 3=W
  void onKnxWhiteRel(uint8_t dpt3);
  void onKnxWWRel(uint8_t dpt3);
  void onKnxCWRel(uint8_t dpt3);
  void onKnxHueRel(uint8_t dpt3);
  void onKnxSatRel(uint8_t dpt3);
  void onKnxValRel(uint8_t dpt3);
  void onKnxEffectRel(uint8_t dpt3);
  void onKnxRGBRel(uint8_t rCtl, uint8_t gCtl, uint8_t bCtl);
  void onKnxHSVRel(uint8_t hCtl, uint8_t sCtl, uint8_t vCtl);
  void onKnxRGBWRel(uint8_t rCtl, uint8_t gCtl, uint8_t bCtl, uint8_t wCtl);

  void evalAndPublishTempAlarm(uint16_t ga, float tempC, float maxC, bool& lastState, const char* tag);

  // System clock
  void setSystemClockYMDHMS(int year, int month, int day, int hour, int minute, int second);
  void setSystemClockYMDHMS_withDST(int year, int month, int day, int hour, int minute, int second, int isDst /* -1 auto, 0 standard, 1 DST */);

  // helper to apply current LAST_W/LAST_CCT to the active color
  void applyWhiteAndCct();
  static void rgbToHsv(uint8_t r, uint8_t g, uint8_t b, float& hDeg, float& s01, float& v01);
  static void hsvToRgb(float hDeg, float s01, float v01, uint8_t& r, uint8_t& g, uint8_t& b);
  static inline uint8_t hueDegToByte(float hDeg) { while (hDeg<0) hDeg+=360.f; while (hDeg>=360) hDeg-=360.f; return (uint8_t)roundf(hDeg * 255.f / 360.f); }
  static inline float   byteToHueDeg(uint8_t hb) { return (hb * 360.f) / 255.f; }
  static inline uint8_t pct01ToByte(float p)     { if (p<0) p=0; if (p>1) p=1; return (uint8_t)roundf(p*255.f); }
  static inline float   byteToPct01(uint8_t b)   { return b / 255.f;   }
  
  bool readEspInternalTempC(float& outC) const;   // Internal_temperature_v2 only
  bool readDallasTempC(float& outC) const;        // DS18B20 usermod only
  void publishTemperatureOnce();                  // send each to its own GA

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
