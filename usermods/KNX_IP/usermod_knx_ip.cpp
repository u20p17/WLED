#include "usermod_knx_ip.h"
#include "wled.h"   // access to global 'strip' and segments
#include "DPT.h"
#include <sys/time.h>

#if defined(ESP32)
  #include <esp_sntp.h>
#endif

// WLED core globals from ntp.cpp
extern unsigned long ntpLastSyncTime;

extern "C" {
  // Dallas usermod may (optionally) provide this. If not, we'll just skip Dallas.
  float __attribute__((weak)) wled_get_temperature_c();
}

enum class LedProfile : uint8_t { MONO = 1, CCT, RGB, RGBW, RGBCCT };
static LedProfile g_ledProfile = LedProfile::RGB;

// Map segment light-capabilities to profile.
// lc bit0=RGB (1), bit1=White (2), bit2=CCT (4)
static LedProfile detectLedProfileFromSegments() {
  uint8_t lc = 0;
  const uint16_t n = strip.getSegmentsNum();
  for (uint16_t i = 0; i < n; i++) {
    lc |= strip.getSegment(i).getLightCapabilities();
  }
  if (lc == 0 && n > 0) lc = strip.getMainSegment().getLightCapabilities(); // fallback

  const bool hasRGB = lc & 0x01;
  const bool hasW   = lc & 0x02;
  const bool hasCCT = lc & 0x04;

  if (hasRGB && hasCCT) return LedProfile::RGBCCT;
  if (hasRGB && hasW)   return LedProfile::RGBW;
  if (hasRGB)           return LedProfile::RGB;
  if (hasCCT)           return LedProfile::CCT;
  if (hasW)             return LedProfile::MONO;
  return LedProfile::MONO;
}

// Track last-sent preset so we only transmit GA_OUT_PRE when it changes
static uint8_t s_lastPresetSent = 0xFF;

static void getCurrentRGBW(uint8_t &r, uint8_t &g, uint8_t &b, uint8_t &w) {
  uint32_t c = SEGCOLOR(0); // primary color slot
  r = R(c); g = G(c); b = B(c); w = W(c);
}

// single global KNX instance is declared in esp-knx-ip.{h,cpp}
extern KnxIpCore KNX;

// ------------- helpers -------------
static inline uint8_t clamp100(uint8_t v){ return (v>100)?100:v; }
static inline uint8_t pct_to_0_255(uint8_t pct) { return (uint8_t)((pct * 255u + 50u) / 100u); }
static inline uint8_t to_pct_0_100(uint8_t v0_255) { return (uint8_t)((v0_255 * 100u + 127u) / 255u); }
static inline uint8_t clamp8i(int v){ return (uint8_t) (v<0?0:(v>255?255:v)); }

// Parse "x/y/z" -> GA (returns 0 if invalid). Strict unsigned parsing with range enforcement.
static uint16_t parseGA(const char* s) {
  if (!s || !*s) return 0;
  uint32_t a=0,b=0,c=0; const char* p = s;
  auto parseUInt = [&](uint32_t& out)->bool {
    if (*p < '0' || *p > '9') return false; // must start with digit
    uint32_t v=0;
    while (*p >= '0' && *p <= '9') {
      v = v*10u + (uint32_t)(*p - '0');
      if (v > 1000u) return false; // simple overflow guard
      p++;
    }
    out = v; return true;
  };
  if (!parseUInt(a) || *p!='/') return 0; ++p;
  if (!parseUInt(b) || *p!='/') return 0; ++p;
  if (!parseUInt(c) || *p!='\0') return 0;
  if (a>31u || b>7u || c>255u) return 0;  // KNX 3-level GA limits
  return knxMakeGroupAddress((uint8_t)a,(uint8_t)b,(uint8_t)c);
}

// Parse individual address "x.x.x" -> 16-bit PA: area(4) | line(4) | dev(8). Returns 0 if invalid.
static uint16_t parsePA(const char* s) {
  if (!s || !*s) return 0;
  uint32_t area=0,line=0,dev=0; const char* p=s;
  auto parseUInt = [&](uint32_t& out)->bool {
    if (*p < '0' || *p > '9') return false;
    uint32_t v=0;
    while (*p>='0' && *p<='9') {
      v = v*10u + (uint32_t)(*p-'0');
      if (v > 1000u) return false; // guard
      p++;
    }
    out=v; return true;
  };
  if (!parseUInt(area) || *p!='.') return 0; ++p;
  if (!parseUInt(line) || *p!='.') return 0; ++p;
  if (!parseUInt(dev)  || *p!='\0') return 0;
  if (area>15u || line>15u || dev>255u) return 0; // strict
  return (uint16_t)((area & 0x0F) << 12) | (uint16_t)((line & 0x0F) << 8) | (uint16_t)(dev & 0xFF);
}

// Public validation wrappers (reuse parsing logic without logging)
bool KnxIpUsermod::validateGroupAddressString(const char* s) {
  return parseGA(s) != 0;
}
bool KnxIpUsermod::validateIndividualAddressString(const char* s) {
  return parsePA(s) != 0;
}

// ---- Small helper registration shims to reduce lambda repetition ----
// Registers a 1-byte inbound object (if ga!=0) and wires a simple callback taking the first byte.
static void register1ByteHandler(uint16_t ga, DptMain dpt, std::function<void(uint8_t)> cb) {
  if (!ga) return;
  KNX.addGroupObject(ga, dpt, false, true);
  KNX.onGroup(ga, [cb](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){
    if (svc == KnxService::GroupValue_Write && p && len >= 1) cb(p[0]);
  });
}

// Registers a multi-byte inbound object (if ga!=0) and wires a callback taking pointer to payload (len already checked).
static void registerMultiHandler(uint16_t ga, DptMain dpt, uint8_t minLen, std::function<void(const uint8_t*)> cb) {
  if (!ga) return;
  KNX.addGroupObject(ga, dpt, false, true);
  KNX.onGroup(ga, [cb,minLen](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){
    if (svc == KnxService::GroupValue_Write && p && len >= minLen) cb(p);
  });
}

// --- CCT mapping helpers ---
uint8_t KnxIpUsermod::kelvinToCct255(uint16_t k) const {
  uint16_t kmin = kelvinMin;
  uint16_t kmax = kelvinMax;
  if (kmin > kmax) { uint16_t t=kmin; kmin=kmax; kmax=t; }
  if (k <= kmin) return 0;
  if (k >= kmax) return 255;
  uint32_t span = (uint32_t)kmax - (uint32_t)kmin;
  uint32_t pos  = (uint32_t)k  - (uint32_t)kmin;
  return (uint8_t)((pos * 255u + (span/2)) / span);
}

uint16_t KnxIpUsermod::cct255ToKelvin(uint8_t cct) const {
  uint16_t kmin = kelvinMin;
  uint16_t kmax = kelvinMax;
  if (kmin > kmax) { uint16_t t=kmin; kmin=kmax; kmax=t; }
  uint32_t span = (uint32_t)kmax - (uint32_t)kmin;
  return (uint16_t)(kmin + (uint32_t)cct * span / 255u);
}

// ------------- KNX→WLED handlers -------------
void KnxIpUsermod::onKnxPower(bool on) {
  if (on) {
    if (bri == 0) {
      bri = (briLast > 0) ? briLast : 128;
    }
  } else {
    briLast = bri;
    bri = 0;
  }
  stateUpdated(CALL_MODE_DIRECT_CHANGE);
  scheduleStatePublish(true, true, true);
}

void KnxIpUsermod::onKnxBrightness(uint8_t pct) {
  pct = clamp100(pct);
  bri = pct_to_0_255(pct);
  if (bri > 0) strip.setBrightness(bri);
  stateUpdated(CALL_MODE_DIRECT_CHANGE);
  scheduleStatePublish(true, true, true);
}

void KnxIpUsermod::onKnxRGB(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t cr,cg,cb,cw; getCurrentRGBW(cr,cg,cb,cw);
  strip.setColor(0, r, g, b, cw); // preserve existing white
  colorUpdated(CALL_MODE_DIRECT_CHANGE);
  scheduleStatePublish(true, true, false);
}

void KnxIpUsermod::onKnxEffect(uint8_t fxIndex) {
  strip.setMode(0, fxIndex);
  stateUpdated(CALL_MODE_DIRECT_CHANGE);
  scheduleStatePublish(false, false, true);
}

// ---------------- Relative handlers (DPT 3.007) -----------------
static uint8_t knx_step_pct(uint8_t stepCode) {
  switch(stepCode) {
    case 1: return 100; case 2: return 50; case 3: return 25; case 4: return 12; case 5: return 6; case 6: return 3; case 7: return 1; default: return 0; }
}

static int16_t knx_step_delta(uint8_t nibble, uint16_t maxVal) {
  if (nibble == 0) return 0; // stop
  bool inc = (nibble & 0x8) != 0; // bit3
  uint8_t sc = nibble & 0x7;
  uint8_t pct = knx_step_pct(sc);
  uint16_t mag = (uint32_t)maxVal * pct / 100U;
  if (mag == 0) mag = 1;
  return inc ? (int16_t)mag : -(int16_t)mag;
}

void KnxIpUsermod::onKnxBrightnessRel(uint8_t dpt3) {
  int16_t d = knx_step_delta(dpt3 & 0x0F, 255);
  if (!d) return;
  int val = (int)bri + d;
  if (val < 0) val = 0; else if (val > 255) val = 255;
  bri = (uint8_t)val;
  if (bri > 0) strip.setBrightness(bri);
  stateUpdated(CALL_MODE_DIRECT_CHANGE);
  scheduleStatePublish(true, true, true);
}

void KnxIpUsermod::onKnxColorRel(uint8_t channel, uint8_t dpt3) {
  int16_t d = knx_step_delta(dpt3 & 0x0F, 255);
  if (!d) return;
  uint8_t r,g,b,w; getCurrentRGBW(r,g,b,w);
  uint8_t* tgt = nullptr;
  switch(channel){case 0: tgt=&r; break; case 1: tgt=&g; break; case 2: tgt=&b; break; case 3: tgt=&w; break;}
  if (!tgt) return;
  int nv = (int)(*tgt) + d; if (nv<0) nv=0; else if (nv>255) nv=255; *tgt=(uint8_t)nv;
  strip.setColor(0,r,g,b,w);
  colorUpdated(CALL_MODE_DIRECT_CHANGE);
  scheduleStatePublish(true,true,false);
}

void KnxIpUsermod::onKnxWhiteRel(uint8_t dpt3){ onKnxColorRel(3,dpt3); }

void KnxIpUsermod::onKnxWWRel(uint8_t dpt3) {
  int16_t d = knx_step_delta(dpt3 & 0x0F, 255);
  if (!d) return;
  Segment& seg = strip.getSegment(0);
  uint8_t cct = seg.cct;
  uint8_t r,g,b,w; getCurrentRGBW(r,g,b,w);
  uint16_t ww = (uint16_t)w * (255 - cct) / 255;
  int nww = (int)ww + d; if (nww<0) nww=0; else if (nww>255) nww=255;
  uint16_t cw = (uint16_t)w * cct / 255; // unchanged
  uint16_t sum = (uint16_t)nww + cw;
  uint8_t newW = sum>255?255:(uint8_t)sum;
  uint8_t newCct = (sum==0)?cct:(uint8_t)((cw*255u + sum/2)/sum);
  strip.setColor(0,r,g,b,newW);
  seg.cct = newCct;
  colorUpdated(CALL_MODE_DIRECT_CHANGE);
  scheduleStatePublish(true,true,false);
}

void KnxIpUsermod::onKnxCWRel(uint8_t dpt3) {
  int16_t d = knx_step_delta(dpt3 & 0x0F, 255);
  if (!d) return;
  Segment& seg = strip.getSegment(0);
  uint8_t cct = seg.cct;
  uint8_t r,g,b,w; getCurrentRGBW(r,g,b,w);
  uint16_t cw = (uint16_t)w * cct / 255;
  int ncw = (int)cw + d; if (ncw<0) ncw=0; else if (ncw>255) ncw=255;
  uint16_t ww = (uint16_t)w * (255 - cct) / 255;
  uint16_t sum = (uint16_t)ncw + ww;
  uint8_t newW = sum>255?255:(uint8_t)sum;
  uint8_t newCct = (sum==0)?cct:(uint8_t)((ncw*255u + sum/2)/sum);
  strip.setColor(0,r,g,b,newW);
  seg.cct = newCct;
  colorUpdated(CALL_MODE_DIRECT_CHANGE);
  scheduleStatePublish(true,true,false);
}

void KnxIpUsermod::onKnxHueRel(uint8_t dpt3) {
  int16_t d = knx_step_delta(dpt3 & 0x0F, 30); // max 30-degree jump for 100%
  if (!d) return;
  uint8_t r,g,b,w; getCurrentRGBW(r,g,b,w);
  float h,s,v; rgbToHsv(r,g,b,h,s,v);
  h += (float)d; while(h<0) h+=360.f; while(h>=360.f) h-=360.f; applyHSV(h,s,v,true);
}

void KnxIpUsermod::onKnxSatRel(uint8_t dpt3) {
  int16_t d = knx_step_delta(dpt3 & 0x0F, 255);
  if (!d) return;
  uint8_t r,g,b,w; getCurrentRGBW(r,g,b,w);
  float h,s,v; rgbToHsv(r,g,b,h,s,v);
  s += (float)d / 255.f; if (s<0) s=0; if (s>1) s=1; applyHSV(h,s,v,true);
}

void KnxIpUsermod::onKnxValRel(uint8_t dpt3) {
  int16_t d = knx_step_delta(dpt3 & 0x0F, 255);
  if (!d) return;
  uint8_t r,g,b,w; getCurrentRGBW(r,g,b,w);
  float h,s,v; rgbToHsv(r,g,b,h,s,v);
  v += (float)d / 255.f; if (v<0) v=0; if (v>1) v=1; applyHSV(h,s,v,true);
}

void KnxIpUsermod::onKnxEffectRel(uint8_t dpt3) {
  int16_t d = knx_step_delta(dpt3 & 0x0F, 10); if(!d) return; int count = strip.getModeCount(); int cur = (int)effectCurrent + d; if (cur<0) cur=0; else if (cur>=count) cur=count-1; onKnxEffect((uint8_t)cur);
}

// ---- Composite relative handlers (multi-byte, each byte low nibble = DPT 3.007) ----
// RGB relative: 3 bytes [Rctl, Gctl, Bctl]
void KnxIpUsermod::onKnxRGBRel(uint8_t rCtl, uint8_t gCtl, uint8_t bCtl) {
  int16_t dr = knx_step_delta(rCtl & 0x0F, 255);
  int16_t dg = knx_step_delta(gCtl & 0x0F, 255);
  int16_t db = knx_step_delta(bCtl & 0x0F, 255);
  if (!dr && !dg && !db) return; //  nothing changed
  uint8_t r,g,b,w; getCurrentRGBW(r,g,b,w);
  if (dr) { int v = (int)r + dr; if (v<0) v=0; else if (v>255) v=255; r=(uint8_t)v; }
  if (dg) { int v = (int)g + dg; if (v<0) v=0; else if (v>255) v=255; g=(uint8_t)v; }
  if (db) { int v = (int)b + db; if (v<0) v=0; else if (v>255) v=255; b=(uint8_t)v; }
  strip.setColor(0,r,g,b,w);
  colorUpdated(CALL_MODE_DIRECT_CHANGE);
  scheduleStatePublish(true,true,false);
}

// HSV relative: 3 bytes [Hctl, Sctl, Vctl]
void KnxIpUsermod::onKnxHSVRel(uint8_t hCtl, uint8_t sCtl, uint8_t vCtl) {
  int16_t dh = knx_step_delta(hCtl & 0x0F, 30);   // same 30° max jump used in single Hue rel
  int16_t ds = knx_step_delta(sCtl & 0x0F, 255);  // work in 0..255 domain then scale
  int16_t dv = knx_step_delta(vCtl & 0x0F, 255);
  if (!dh && !ds && !dv) return;
  uint8_t r,g,b,w; getCurrentRGBW(r,g,b,w);
  float h,s,v; rgbToHsv(r,g,b,h,s,v);
  if (dh) { h += (float)dh; while (h < 0) h += 360.f; while (h >= 360.f) h -= 360.f; }
  if (ds) { s += (float)ds / 255.f; if (s < 0) s = 0; if (s > 1) s = 1; }
  if (dv) { v += (float)dv / 255.f; if (v < 0) v = 0; if (v > 1) v = 1; }
  applyHSV(h,s,v,true); // preserves existing white and publishes
}

// RGBW relative: 4 (or 6) bytes [Rctl,Gctl,Bctl,Wctl,(ext...)]
void KnxIpUsermod::onKnxRGBWRel(uint8_t rCtl, uint8_t gCtl, uint8_t bCtl, uint8_t wCtl) {
  int16_t dr = knx_step_delta(rCtl & 0x0F, 255);
  int16_t dg = knx_step_delta(gCtl & 0x0F, 255);
  int16_t db = knx_step_delta(bCtl & 0x0F, 255);
  int16_t dw = knx_step_delta(wCtl & 0x0F, 255);
  if (!dr && !dg && !db && !dw) return; // nothing changed
  uint8_t r,g,b,w; getCurrentRGBW(r,g,b,w);
  if (dr) { int v = (int)r + dr; if (v<0) v=0; else if (v>255) v=255; r=(uint8_t)v; }
  if (dg) { int v = (int)g + dg; if (v<0) v=0; else if (v>255) v=255; g=(uint8_t)v; }
  if (db) { int v = (int)b + db; if (v<0) v=0; else if (v>255) v=255; b=(uint8_t)v; }
  if (dw) { int v = (int)w + dw; if (v<0) v=0; else if (v>255) v=255; w=(uint8_t)v; }
  strip.setColor(0,r,g,b,w);
  colorUpdated(CALL_MODE_DIRECT_CHANGE);
  scheduleStatePublish(true,true,false);
}

void KnxIpUsermod::onKnxPreset(uint8_t preset) {
  _lastPreset = preset;    // remember for status OUT
  applyPreset(preset);
  scheduleStatePublish(true, true, true);
}

void KnxIpUsermod::onKnxWhite(uint8_t v) {            // 0..255 (DPT 5.010)
  uint8_t r,g,b,w; getCurrentRGBW(r,g,b,w);
  strip.setColor(0, r, g, b, v);
  colorUpdated(CALL_MODE_DIRECT_CHANGE);
  scheduleStatePublish(true, true, false);
}

void KnxIpUsermod::onKnxCct(uint16_t kelvin) {        // Kelvin (DPT 7.600)
  Segment& seg = strip.getSegment(0);
  seg.cct = kelvinToCct255(kelvin);
  stateUpdated(CALL_MODE_DIRECT_CHANGE);
  scheduleStatePublish(true, true, false);
}

void KnxIpUsermod::applyWhiteAndCct() {
  uint8_t r,g,b,w; getCurrentRGBW(r,g,b,w);
  strip.setColor(0, r, g, b, w); // W is already part of color; CCT is separate
  // CCT is stored on the segment
  colorUpdated(CALL_MODE_DIRECT_CHANGE);
}

void KnxIpUsermod::onKnxWW(uint8_t v) {               // 0..255 (DPT 5.010)
  const Segment& seg = strip.getSegment(0);
  const uint8_t wLive = W(seg.colors[0]);
  const uint8_t cctLive = seg.cct;

  uint8_t cw = (uint8_t)(((uint16_t)wLive * cctLive + 127u) / 255u);
  uint16_t sum = (uint16_t)v + (uint16_t)cw;

  uint8_t newW   = (sum > 255) ? 255 : (uint8_t)sum;
  uint8_t newCct = (sum == 0) ? cctLive : (uint8_t)((cw * 255u + sum/2) / sum);

  // apply
  uint8_t r,g,b,_w; getCurrentRGBW(r,g,b,_w);
  strip.setColor(0, r, g, b, newW);
  strip.getSegment(0).cct = newCct;

  colorUpdated(CALL_MODE_DIRECT_CHANGE);
  scheduleStatePublish(true, true, false);
}

void KnxIpUsermod::onKnxCW(uint8_t v) {               // 0..255 (DPT 5.010)
  const Segment& seg = strip.getSegment(0);
  const uint8_t wLive = W(seg.colors[0]);
  const uint8_t cctLive = seg.cct;

  uint8_t ww = (uint8_t)(((uint16_t)wLive * (255u - cctLive) + 127u) / 255u);
  uint16_t sum = (uint16_t)ww + (uint16_t)v;

  uint8_t newW   = (sum > 255) ? 255 : (uint8_t)sum;
  uint8_t newCct = (sum == 0) ? cctLive : (uint8_t)((v * 255u + sum/2) / sum);

  // apply
  uint8_t r,g,b,_w; getCurrentRGBW(r,g,b,_w);
  strip.setColor(0, r, g, b, newW);
  strip.getSegment(0).cct = newCct;

  colorUpdated(CALL_MODE_DIRECT_CHANGE);
  scheduleStatePublish(true, true, false);
}

void KnxIpUsermod::onKnxRGBW(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  strip.setColor(0, r, g, b, w);
  colorUpdated(CALL_MODE_DIRECT_CHANGE);
  scheduleStatePublish(true, true, false);
}

void KnxIpUsermod::hsvToRgb(float h, float s, float v, uint8_t& r, uint8_t& g, uint8_t& b) {
  if (s <= 0.f) { r=g=b=clamp8i((int)roundf(v*255.f)); return; }
  while (h < 0.f) h += 360.f; while (h >= 360.f) h -= 360.f;
  float c = v * s;
  float x = c * (1.f - fabsf(fmodf(h/60.f, 2.f) - 1.f));
  float m = v - c;
  float rf=0,gf=0,bf=0;
  if (h < 60)      { rf=c; gf=x; bf=0; }
  else if (h <120) { rf=x; gf=c; bf=0; }
  else if (h <180) { rf=0; gf=c; bf=x; }
  else if (h <240) { rf=0; gf=x; bf=c; }
  else if (h <300) { rf=x; gf=0; bf=c; }
  else             { rf=c; gf=0; bf=x; }
  r = clamp8i((int)roundf((rf+m)*255.f));
  g = clamp8i((int)roundf((gf+m)*255.f));
  b = clamp8i((int)roundf((bf+m)*255.f));
}

void KnxIpUsermod::rgbToHsv(uint8_t r, uint8_t g, uint8_t b, float& h, float& s, float& v) {
  float rf = r/255.f, gf = g/255.f, bf = b/255.f;
  float cmax = fmaxf(rf, fmaxf(gf, bf));
  float cmin = fminf(rf, fminf(gf, bf));
  float delta = cmax - cmin;
  // Hue
  if (delta == 0.f) h = 0.f;
  else if (cmax == rf) h = 60.f * fmodf(((gf - bf)/delta), 6.f);
  else if (cmax == gf) h = 60.f * (((bf - rf)/delta) + 2.f);
  else                 h = 60.f * (((rf - gf)/delta) + 4.f);
  if (h < 0.f) h += 360.f;
  // Saturation
  s = (cmax == 0.f) ? 0.f : (delta / cmax);
  // Value
  v = cmax;
}

void KnxIpUsermod::applyHSV(float hDeg, float s01, float v01, bool preserveWhite) {
  uint8_t r,g,b; hsvToRgb(hDeg, s01, v01, r, g, b);
  uint8_t cr,cg,cb,cw; getCurrentRGBW(cr,cg,cb,cw);
  if (!preserveWhite) cw = 0; // optional future use; currently always true
  strip.setColor(0, r, g, b, cw);
  colorUpdated(CALL_MODE_DIRECT_CHANGE);
  scheduleStatePublish(true, true, false);
}

void KnxIpUsermod::onKnxH(float hDeg) {
  uint8_t r,g,b,w; getCurrentRGBW(r,g,b,w);
  float ch,cs,cv; rgbToHsv(r,g,b,ch,cs,cv);
  applyHSV(hDeg, cs, cv, true);
}

void KnxIpUsermod::onKnxS(float s01) {
  uint8_t r,g,b,w; getCurrentRGBW(r,g,b,w);
  float ch,cs,cv; rgbToHsv(r,g,b,ch,cs,cv);
  applyHSV(ch, s01, cv, true);
}

void KnxIpUsermod::onKnxV(float v01) {
  uint8_t r,g,b,w; getCurrentRGBW(r,g,b,w);
  float ch,cs,cv; rgbToHsv(r,g,b,ch,cs,cv);
  applyHSV(ch, cs, v01, true);
}

bool KnxIpUsermod::readEspInternalTempC(float& outC) const {
#if defined(ESP8266) || defined(CONFIG_IDF_TARGET_ESP32S2)
  Serial.printf("ESP-int: not supported on this chip\n");
  return false;
#else
  // ESP32 / ESP32S3 / ESP32C3: direct internal sensor read
  float v = temperatureRead();                 // degrees C (approximate)
  if (isnan(v) || v < -40.0f || v > 150.0f) {  // quick sanity
    Serial.printf("ESP-internal Temp: invalid (%.1f)\n", v);
    return false;
  }
  // match the other usermod’s rounding (0.1 °C)
  v = roundf(v * 10.0f) / 10.0f;
  Serial.printf("ESP-internal Temp: OK (%.1f °C)\n", v);
  outC = v;
  return true;
#endif
}

bool KnxIpUsermod::readDallasTempC(float& outC) const {
  if (&wled_get_temperature_c != nullptr) {
    float v = wled_get_temperature_c();
    if (!isnan(v)) {
      Serial.printf("Dallas Temp probe: OK (%.1f °C)\n", v);
      outC = v;
      return true;
    }
    Serial.printf("Dallas Temp probe: NaN\n");
  } else {
    Serial.printf("Dallas Temp probe: symbol missing\n");
  }
  return false;
}

void KnxIpUsermod::publishTemperatureOnce() {
  if (!KNX.running()) {
    Serial.printf("skip: KNX not running\n");
    return;
  }

  // ESP32 internal -> GA_OUT_INT_TEMP
  if (GA_OUT_INT_TEMP) {
    float tEsp;
    if (readEspInternalTempC(tEsp)) {
      uint8_t buf[4];
      KnxIpCore::pack4ByteFloat(tEsp, buf);
      bool ok = KNX.groupValueWrite(GA_OUT_INT_TEMP, buf, 4);
      Serial.printf("TX ESP internal Temp: %.1f °C (%s)\n", tEsp, ok ? "OK" : "FAIL");
      evalAndPublishTempAlarm(GA_OUT_INT_TEMP_ALARM, tEsp, intTempAlarmMaxC, lastIntTempAlarmState, "ESP-int");
    } else {
      Serial.printf("skip ESP internal Temp: no value\n");
    }
  }
  

  // Dallas DS18B20 -> GA_OUT_TEMP
  if (GA_OUT_TEMP) {
    float tDallas;
    if (readDallasTempC(tDallas)) {
      uint8_t buf[4];
      KnxIpCore::pack4ByteFloat(tDallas, buf);
      bool ok = KNX.groupValueWrite(GA_OUT_TEMP, buf, 4);
      Serial.printf("TX Dallas Temp: %.1f °C (%s)\n", tDallas, ok ? "OK" : "FAIL");
      evalAndPublishTempAlarm(GA_OUT_TEMP_ALARM, tDallas, dallasTempAlarmMaxC, lastDallasTempAlarmState, "Dallas");
    } else {
      Serial.printf("skip Dallas Temp: no value\n");
    }
  }

  if (!GA_OUT_INT_TEMP && !GA_OUT_TEMP) {
    Serial.printf("skip: no temperature GAs configured\n");
  }
}

void KnxIpUsermod::evalAndPublishTempAlarm(uint16_t ga, float tempC, float maxC, bool& lastState,const char* tag) {
  if (!ga) return; // not configured

  // Disabled threshold? Allow negative to mean "off"
  if (!(maxC > -100.0f)) {
    // Optional: clear if previously set
    if (lastState) {
      uint8_t b0 = 0;
      KNX.groupValueWrite(ga, &b0, 1);
      lastState = false;
      Serial.printf("[KNX-UM][TEMP] %s alarm DISABLED -> send 0 to 0x%04X\n", tag, ga);
    }
    return;
  }

  bool trip   = (tempC >= maxC);
  bool clear  = (tempC <= (maxC - tempAlarmHystC));

  bool newState = lastState;
  if (!lastState && trip)       newState = true;   // 0 -> 1 (trip)
  else if (lastState && clear)  newState = false;  // 1 -> 0 (clear)

  if (newState != lastState) {
    uint8_t bit = newState ? 1 : 0;    // DPST-1-5: 1 = alarm active
    bool ok = KNX.groupValueWrite(ga, &bit, 1);    // core packs 1-bit correctly
    Serial.printf("[KNX-UM][TEMP] %s alarm %s @ %.2f°C (thr=%.2f°C, hyst=%.2f) -> GA 0x%04X (%s)\n",
                  tag, newState ? "ON" : "OFF", tempC, maxC, tempAlarmHystC, ga, ok?"OK":"FAIL");
    lastState = newState;
  }
  else {
    Serial.printf("[KNX-UM][TEMP] %s alarm unchanged (%s) @ %.2f°C "
                  "(thr=%.2f°C, hyst=%.2f)\n",
                  tag, lastState ? "ON" : "OFF",
                  tempC, maxC, tempAlarmHystC);
  }
}

void KnxIpUsermod::setSystemClockYMDHMS(int year, int month, int day, int hour, int minute, int second) {
  // KNX 0..99 -> 2000..2099 (adjust if you prefer a different epoch)
  if (year < 100) year += 2000;

  struct tm t{};
  t.tm_year = year - 1900; // years since 1900
  t.tm_mon  = month - 1;   // 0..11
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min  = minute;
  t.tm_sec  = second;
  t.tm_isdst = -1;      // <— let mktime() figure out DST from the TZ rules

  #if defined(ESP32)
    sntp_stop();                    
  #endif

  time_t epoch = mktime(&t); // local time
  if (epoch == (time_t)-1) {
    Serial.printf("[KNX-UM][TIME] mktime() failed for %04d-%02d-%02d %02d:%02d:%02d\n",
                  year, month, day, hour, minute, second);
    return;
  }
  // Set system clock (both POSIX ways)
  struct timeval tv{ .tv_sec = epoch, .tv_usec = 0 };
  settimeofday(&tv, nullptr);

  tzset();

#if defined(ESP32)
  sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
#endif
ntpLastSyncTime = (unsigned long)epoch;

Serial.printf("[KNX-UM][TIME] Clock set to %04d-%02d-%02d %02d:%02d:%02d (local)\n",
                year, month, day, hour, minute, second);

time_t chk = time(nullptr);
struct tm cur{}; localtime_r(&chk, &cur);
Serial.printf("[KNX-UM][TIME] Clock set -> %04d-%02d-%02d %02d:%02d:%02d (local, read-back)\n",
                cur.tm_year + 1900, cur.tm_mon + 1, cur.tm_mday,
                cur.tm_hour, cur.tm_min, cur.tm_sec);
}

void KnxIpUsermod::setSystemClockYMDHMS_withDST(int year, int month, int day, int hour, int minute, int second, int isDst /* -1 auto, 0 standard, 1 DST */){
  if (year < 100) year += 2000;

  struct tm t{};
  t.tm_year  = year - 1900;
  t.tm_mon   = month - 1;
  t.tm_mday  = day;
  t.tm_hour  = hour;
  t.tm_min   = minute;
  t.tm_sec   = second;
  t.tm_isdst = isDst;      // <— use the hint from DPT19 if provided

  #if defined(ESP32)
    sntp_stop(); 
  #endif

  time_t epoch = mktime(&t);
  if (epoch == (time_t)-1) {
    Serial.printf("[KNX-UM][TIME] mktime() failed (DST=%d)\n", isDst);
    return;
  }
  struct timeval tv{ .tv_sec = epoch, .tv_usec = 0 };
  settimeofday(&tv, nullptr);

  tzset();

#if defined(ESP32)
  sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
#endif
  ntpLastSyncTime = (unsigned long)epoch;

  time_t chk = time(nullptr);
  struct tm cur{}; localtime_r(&chk, &cur);
  Serial.printf("[KNX-UM][TIME] DPT19 set -> %04d-%02d-%02d %02d:%02d:%02d (local, read-back, DST=%d)\n",
                cur.tm_year + 1900, cur.tm_mon + 1, cur.tm_mday,
                cur.tm_hour, cur.tm_min, cur.tm_sec, cur.tm_isdst);
}

static inline void applyKnxWallClock(int year, int month, int day, int hour, int minute, int second, int isDst /* -1 auto, 0 std, 1 DST */, bool stopSntp /*true to prevent override*/) {
  if (year < 100) year += 2000;

  struct tm t{};
  t.tm_year  = year - 1900;
  t.tm_mon   = month - 1;
  t.tm_mday  = day;
  t.tm_hour  = hour;
  t.tm_min   = minute;
  t.tm_sec   = second;
  t.tm_isdst = isDst;   // for DPT19 use summerTime?1:0; for 10/11 use -1

  // Convert local wall time -> UTC epoch using WLED’s configured TZ/DST
  time_t epoch = mktime(&t);
  if (epoch == (time_t)-1) {
    Serial.printf("[KNX-UM][TIME] mktime() failed for %04d-%02d-%02d %02d:%02d:%02d (isDst=%d)\n",
                  year, month, day, hour, minute, second, isDst);
    return;
  }

#if defined(ESP32)
  if (stopSntp) sntp_stop();   // optional: keep KNX authoritative
#endif

  // Hand off to WLED core (updates everything the Info page uses)
  setTimeFromAPI((uint32_t)epoch);

  // Debug: read back local time
  time_t chk = time(nullptr);
  struct tm cur{}; localtime_r(&chk, &cur);
  Serial.printf("[KNX-UM][TIME] KNX->API set: %04d-%02d-%02d %02d:%02d:%02d (local, read-back)\n",
                cur.tm_year + 1900, cur.tm_mon + 1, cur.tm_mday,
                cur.tm_hour, cur.tm_min, cur.tm_sec);
}

static void dumpBytesHexLocal(const uint8_t* p, uint8_t len) {
  if (!p || !len) return;
  Serial.print("[KNX-UM][TIME] Raw: ");
  for (uint8_t i = 0; i < len; i++) {
    if (p[i] < 16) Serial.print('0');
    Serial.print(p[i], HEX);
    Serial.print(i + 1 < len ? ' ' : ' ');
  }
  Serial.println();
}

void KnxIpUsermod::onKnxTime_10_001(const uint8_t* p, uint8_t len) {
  if (!p || len < 3) return;
  dumpBytesHexLocal(p, len);

  const int hour   =  p[0] & 0x1F;  // 5 bits
  const int minute =  p[1] & 0x3F;  // 6 bits
  const int second =  p[2] & 0x3F;  // 6 bits

  // Get current date to merge with
  time_t nowUtc = time(nullptr);
  struct tm curLocal{}; localtime_r(&nowUtc, &curLocal);

  struct tm t{};
  t.tm_year  = curLocal.tm_year;                 // keep current date
  t.tm_mon   = curLocal.tm_mon;
  t.tm_mday  = curLocal.tm_mday;
  t.tm_hour  = hour;
  t.tm_min   = minute;
  t.tm_sec   = second;
  t.tm_isdst = -1;                               // let libc apply TZ/DST rules

  time_t epoch = mktime(&t);                     // interpret as *local* wall clock -> UTC epoch
  if (epoch == (time_t)-1) {
    Serial.printf("[KNX-UM][TIME] DPT10 mktime() failed for %02d:%02d:%02d\n", hour, minute, second);
    return;
  }

#if defined(ESP32)
  sntp_stop(); // keep KNX authoritative
#endif
  setTimeFromAPI((uint32_t)epoch);
#if defined(ESP32)
  sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
#endif

  struct tm rb{}; localtime_r(&epoch, &rb);
  Serial.printf("[KNX-UM][TIME] DPT10 set -> %04d-%02d-%02d %02d:%02d:%02d (local)\n",
                rb.tm_year + 1900, rb.tm_mon + 1, rb.tm_mday, rb.tm_hour, rb.tm_min, rb.tm_sec);
}

void KnxIpUsermod::onKnxDate_11_001(const uint8_t* p, uint8_t len) {
  if (!p || len < 3) return;
  dumpBytesHexLocal(p, len);

  const int day   = p[0] & 0x1F;     // 1..31, mask is defensive
  const int month = p[1] & 0x0F ? p[1] : p[1]; // keep as-is; input 1..12
  int year        = p[2];             // 0..99
  year = (year < 100) ? (2000 + year) : year;

  // Keep current time-of-day
  time_t nowUtc = time(nullptr);
  struct tm curLocal{}; localtime_r(&nowUtc, &curLocal);

  struct tm t{};
  t.tm_year  = year - 1900;
  t.tm_mon   = (month - 1);
  t.tm_mday  = day;
  t.tm_hour  = curLocal.tm_hour;
  t.tm_min   = curLocal.tm_min;
  t.tm_sec   = curLocal.tm_sec;
  t.tm_isdst = -1;                     // let libc apply TZ/DST rules

  time_t epoch = mktime(&t);           // local wall clock -> UTC epoch
  if (epoch == (time_t)-1) {
    Serial.printf("[KNX-UM][TIME] DPT11 mktime() failed for %04d-%02d-%02d\n", year, month, day);
    return;
  }

#if defined(ESP32)
  sntp_stop();
#endif
  setTimeFromAPI((uint32_t)epoch);
#if defined(ESP32)
  sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
#endif

  struct tm rb{}; localtime_r(&epoch, &rb);
  Serial.printf("[KNX-UM][TIME] DPT11 set -> %04d-%02d-%02d %02d:%02d:%02d (local)\n",
                rb.tm_year + 1900, rb.tm_mon + 1, rb.tm_mday, rb.tm_hour, rb.tm_min, rb.tm_sec);
}

void KnxIpUsermod::onKnxDateTime_19_001(const uint8_t* p, uint8_t len) {
  if (!p || len < 8) return;
  dumpBytesHexLocal(p, len);

  const uint8_t year8  = p[0];
  const uint8_t month  = p[1];
  const uint8_t day    = p[2];
  /*const uint8_t wday = p[3];*/      // weekday not used for setting
  const uint8_t hourB  = p[4];
  const uint8_t minB   = p[5];
  const uint8_t secB   = p[6];
  const uint8_t flags  = p[7];

  const bool invalidDate = (flags & 0x08) != 0;
  const bool invalidTime = (flags & 0x04) != 0;
  const bool summerTime  = (flags & 0x10) != 0;

  if (invalidDate || invalidTime) {
    Serial.printf("[KNX-UM][TIME] DPT19 invalid flags=0x%02X -> ignore\n", flags);
    return;
  }

  int year = (year8 < 100) ? (2000 + year8) : year8;
  const int hour   = hourB & 0x1F;   // 0..23
  const int minute = minB  & 0x3F;   // 0..59
  const int second = secB  & 0x3F;   // 0..59

  // Build local wall time. Use the DPT flag to steer DST explicitly.
  struct tm t{};
  t.tm_year  = year - 1900;
  t.tm_mon   = month - 1;
  t.tm_mday  = day;
  t.tm_hour  = hour;
  t.tm_min   = minute;
  t.tm_sec   = second;
  t.tm_isdst = summerTime ? 1 : 0;   // explicit per DPT19

  time_t epoch = mktime(&t);         // local wall clock -> UTC epoch
  if (epoch == (time_t)-1) {
    Serial.printf("[KNX-UM][TIME] DPT19 mktime() failed for %04d-%02d-%02d %02d:%02d:%02d (DST=%d)\n",
                  year, month, day, hour, minute, second, (int)summerTime);
    return;
  }

#if defined(ESP32)
  sntp_stop();
#endif
  setTimeFromAPI((uint32_t)epoch);
#if defined(ESP32)
  sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
#endif

  struct tm rb{}; localtime_r(&epoch, &rb);
  Serial.printf("[KNX-UM][TIME] DPT19 set -> %04d-%02d-%02d %02d:%02d:%02d (local, DST=%d, flags=0x%02X)\n",
                rb.tm_year + 1900, rb.tm_mon + 1, rb.tm_mday,
                rb.tm_hour, rb.tm_min, rb.tm_sec, (int)summerTime, flags);
}

// -------------------- Usermod API --------------------
void KnxIpUsermod::setup() {
  if (!enabled) return;

  // --- Early validation of PA and GA strings (mirrors readFromConfig pre-validation) ---
  if (*individualAddr && !KnxIpUsermod::validateIndividualAddressString(individualAddr)) {
    Serial.printf("[KNX-UM][WARN] Invalid individual address '%s' at startup -> reverting to 1.1.100\n", individualAddr);
    strlcpy(individualAddr, "1.1.100", sizeof(individualAddr));
  }
  auto validateOrClear = [](char* s, const char* tag){
    if (*s && !KnxIpUsermod::validateGroupAddressString(s)) {
      Serial.printf("[KNX-UM][WARN] Invalid GA '%s' (%s) at startup -> disabled\n", s, tag);
      s[0] = 0;
    }
  };
  validateOrClear(gaInPower,  "gaInPower");
  validateOrClear(gaInBri,    "gaInBri");
  validateOrClear(gaInR,      "gaInR");
  validateOrClear(gaInG,      "gaInG");
  validateOrClear(gaInB,      "gaInB");
  validateOrClear(gaInW,      "gaInW");
  validateOrClear(gaInCct,    "gaInCct");
  validateOrClear(gaInWW,     "gaInWW");
  validateOrClear(gaInCW,     "gaInCW");
  validateOrClear(gaInH,      "gaInH");
  validateOrClear(gaInS,      "gaInS");
  validateOrClear(gaInV,      "gaInV");
  validateOrClear(gaInFx,     "gaInFx");
  validateOrClear(gaInPreset, "gaInPreset");
  validateOrClear(gaInRGB,    "gaInRGB");
  validateOrClear(gaInHSV,    "gaInHSV");
  validateOrClear(gaInRGBW,   "gaInRGBW");
  validateOrClear(gaInTime,   "gaInTime");
  validateOrClear(gaInDate,   "gaInDate");
  validateOrClear(gaInDateTime, "gaInDateTime");
  validateOrClear(gaInBriRel, "gaInBriRel");
  validateOrClear(gaInRRel,   "gaInRRel");
  validateOrClear(gaInGRel,   "gaInGRel");
  validateOrClear(gaInBRel,   "gaInBRel");
  validateOrClear(gaInWRel,   "gaInWRel");
  validateOrClear(gaInWWRel,  "gaInWWRel");
  validateOrClear(gaInCWRel,  "gaInCWRel");
  validateOrClear(gaInHRel,   "gaInHRel");
  validateOrClear(gaInSRel,   "gaInSRel");
  validateOrClear(gaInVRel,   "gaInVRel");
  validateOrClear(gaInFxRel,  "gaInFxRel");
  validateOrClear(gaInRGBRel, "gaInRGBRel");
  validateOrClear(gaInHSVRel, "gaInHSVRel");
  validateOrClear(gaInRGBWRel,"gaInRGBWRel");
  validateOrClear(gaOutPower, "gaOutPower");
  validateOrClear(gaOutBri,   "gaOutBri");
  validateOrClear(gaOutR,     "gaOutR");
  validateOrClear(gaOutG,     "gaOutG");
  validateOrClear(gaOutB,     "gaOutB");
  validateOrClear(gaOutW,     "gaOutW");
  validateOrClear(gaOutCct,   "gaOutCct");
  validateOrClear(gaOutWW,    "gaOutWW");
  validateOrClear(gaOutCW,    "gaOutCW");
  validateOrClear(gaOutH,     "gaOutH");
  validateOrClear(gaOutS,     "gaOutS");
  validateOrClear(gaOutV,     "gaOutV");
  validateOrClear(gaOutFx,    "gaOutFx");
  validateOrClear(gaOutPreset,"gaOutPreset");
  validateOrClear(gaOutRGB,   "gaOutRGB");
  validateOrClear(gaOutHSV,   "gaOutHSV");
  validateOrClear(gaOutRGBW,  "gaOutRGBW");
  validateOrClear(gaOutIntTemp,      "gaOutIntTemp");
  validateOrClear(gaOutTemp,         "gaOutTemp");
  validateOrClear(gaOutIntTempAlarm, "gaOutIntTempAlarm");
  validateOrClear(gaOutTempAlarm,    "gaOutTempAlarm");

  // Parse & set PA
  if (uint16_t pa = parsePA(individualAddr)) {
    KNX.setIndividualAddress(pa);
    Serial.printf("[KNX-UM] PA set to %u.%u.%u (0x%04X)\n",
                  (unsigned)((pa>>12)&0x0F), (unsigned)((pa>>8)&0x0F), (unsigned)(pa&0xFF), pa);
  } else {
    Serial.printf("[KNX-UM][WARN] Invalid individual address '%s' -> using previous/not set\n", individualAddr);
   
  }

  // Apply Communication Enhancement to the core
  KNX.setCommunicationEnhancement(commEnhance, commResends, commResendGapMs, commRxDedupMs);
  Serial.printf("[KNX-UM] CommEnhance %s (resends=%u gapMs=%u dedupMs=%u)\n",
                commEnhance?"ON":"OFF", commResends, commResendGapMs, commRxDedupMs);

  // Parse IN GA strings (always)
  GA_IN_PWR  = parseGA(gaInPower);
  GA_IN_BRI  = parseGA(gaInBri);
  GA_IN_R    = parseGA(gaInR);
  GA_IN_G    = parseGA(gaInG);
  GA_IN_B    = parseGA(gaInB);
  GA_IN_FX   = parseGA(gaInFx);
  GA_IN_PRE  = parseGA(gaInPreset);
  if (!GA_IN_PWR   && *gaInPower)  Serial.printf("[KNX-UM][WARN] Invalid GA in power '%s'\n", gaInPower);
  if (!GA_IN_BRI   && *gaInBri)    Serial.printf("[KNX-UM][WARN] Invalid GA in bri '%s'\n", gaInBri);
  if (!GA_IN_R     && *gaInR)      Serial.printf("[KNX-UM][WARN] Invalid GA in r '%s'\n", gaInR);
  if (!GA_IN_G     && *gaInG)      Serial.printf("[KNX-UM][WARN] Invalid GA in g '%s'\n", gaInG);
  if (!GA_IN_B     && *gaInB)      Serial.printf("[KNX-UM][WARN] Invalid GA in b '%s'\n", gaInB);
  if (!GA_IN_FX    && *gaInFx)     Serial.printf("[KNX-UM][WARN] Invalid GA in fx '%s'\n", gaInFx);
  if (!GA_IN_PRE   && *gaInPreset) Serial.printf("[KNX-UM][WARN] Invalid GA in preset '%s'\n", gaInPreset);
  Serial.printf("[KNX-UM] IN  pwr=0x%04X bri=0x%04X R=0x%04X G=0x%04X B=0x%04X fx=0x%04X pre=0x%04X\n",
                GA_IN_PWR, GA_IN_BRI, GA_IN_R, GA_IN_G, GA_IN_B, GA_IN_FX, GA_IN_PRE);

  GA_IN_RGB  = parseGA(gaInRGB);
  GA_IN_HSV  = parseGA(gaInHSV);
  GA_IN_RGBW = parseGA(gaInRGBW);
  GA_IN_RGB_REL  = parseGA(gaInRGBRel);
  GA_IN_HSV_REL  = parseGA(gaInHSVRel);
  GA_IN_RGBW_REL = parseGA(gaInRGBWRel);
  GA_IN_H    = parseGA(gaInH);
  GA_IN_S    = parseGA(gaInS);
  GA_IN_V    = parseGA(gaInV);
  GA_IN_TIME     = parseGA(gaInTime);
  GA_IN_DATE     = parseGA(gaInDate);
  GA_IN_DATETIME = parseGA(gaInDateTime);
  GA_IN_BRI_REL = parseGA(gaInBriRel);
  GA_IN_R_REL   = parseGA(gaInRRel);
  GA_IN_G_REL   = parseGA(gaInGRel);
  GA_IN_B_REL   = parseGA(gaInBRel);
  GA_IN_W_REL   = parseGA(gaInWRel);
  GA_IN_WW_REL  = parseGA(gaInWWRel);
  GA_IN_CW_REL  = parseGA(gaInCWRel);
  GA_IN_H_REL   = parseGA(gaInHRel);
  GA_IN_S_REL   = parseGA(gaInSRel);
  GA_IN_V_REL   = parseGA(gaInVRel);
  GA_IN_FX_REL  = parseGA(gaInFxRel);

  // --- LED capability detect & gate inputs ---
  g_ledProfile = detectLedProfileFromSegments();
  bool allowRGB = (g_ledProfile == LedProfile::RGB || g_ledProfile == LedProfile::RGBW || g_ledProfile == LedProfile::RGBCCT);
  bool allowW   = (g_ledProfile == LedProfile::RGBW || g_ledProfile == LedProfile::MONO);
  bool allowCCT = (g_ledProfile == LedProfile::CCT || g_ledProfile == LedProfile::RGBCCT);
  
  Serial.printf("[KNX-UM] LED profile: %s (RGB=%d, W=%d, CCT=%d)\n",
    (g_ledProfile==LedProfile::MONO?"MONO":
     g_ledProfile==LedProfile::CCT?"CCT":
     g_ledProfile==LedProfile::RGB?"RGB":
     g_ledProfile==LedProfile::RGBW?"RGBW":"RGBCCT"),
    (int)allowRGB,(int)allowW,(int)allowCCT);

  if (!allowRGB) { GA_IN_R = GA_IN_G = GA_IN_B = GA_IN_R_REL = GA_IN_G_REL = GA_IN_B_REL = GA_IN_RGB = 
                    GA_IN_HSV = GA_IN_H = GA_IN_S = GA_IN_V = GA_IN_H_REL = GA_IN_S_REL = GA_IN_V_REL = 
                    GA_IN_RGB_REL = GA_IN_HSV_REL = 0;  }
  if (!allowW)   { GA_IN_W = GA_IN_W_REL = 0; }
  if (!allowCCT) { GA_IN_CCT = GA_IN_WW = GA_IN_CW = GA_IN_WW_REL = GA_IN_CW_REL = 0; }
  if (!(g_ledProfile == LedProfile::RGBW || g_ledProfile == LedProfile::RGBCCT)) { GA_IN_RGBW = GA_IN_RGBW_REL = 0; }

  // Optional additional inputs registered separately (will be skipped if masked above)
  GA_IN_W   = parseGA(gaInW);
  if (GA_IN_W) {
    register1ByteHandler(GA_IN_W, DptMain::DPT_5xx, [this](uint8_t v){ onKnxWhite(v); });
  }

  GA_IN_CCT = parseGA(gaInCct);
  if (GA_IN_CCT) {
    registerMultiHandler(GA_IN_CCT, DptMain::DPT_7xx, 2, [this](const uint8_t* p){
      uint16_t kelvin = ((uint16_t)p[0] << 8) | (uint16_t)p[1];
      onKnxCct(kelvin);
    });
  }

  // Optional direct WW/CW level inputs
  GA_IN_WW = parseGA(gaInWW);
  if (GA_IN_WW) {
    register1ByteHandler(GA_IN_WW, DptMain::DPT_5xx, [this](uint8_t v){ onKnxWW(v); });
  }

  GA_IN_CW = parseGA(gaInCW);
  if (GA_IN_CW) {
    register1ByteHandler(GA_IN_CW, DptMain::DPT_5xx, [this](uint8_t v){ onKnxCW(v); });
  }

  if (GA_IN_RGB) {
    registerMultiHandler(GA_IN_RGB, DptMain::DPT_232xx, 3, [this](const uint8_t* p){ onKnxRGB(p[0],p[1],p[2]); });
  }
  if (GA_IN_HSV) {
    registerMultiHandler(GA_IN_HSV, DptMain::DPT_232xx, 3, [this](const uint8_t* p){
      float h = byteToHueDeg(p[0]);
      float s = byteToPct01(p[1]);
      float v = byteToPct01(p[2]);
      applyHSV(h,s,v,true);
    });
  }
  if (GA_IN_RGBW) {
    registerMultiHandler(GA_IN_RGBW, DptMain::DPT_251xx, 4, [this](const uint8_t* p){ onKnxRGBW(p[0],p[1],p[2],p[3]); });
  }
  if (GA_IN_RGB_REL) {
    registerMultiHandler(GA_IN_RGB_REL, DptMain::DPT_232xx, 3, [this](const uint8_t* p){ onKnxRGBRel(p[0],p[1],p[2]); });
  }
  if (GA_IN_HSV_REL) {
    registerMultiHandler(GA_IN_HSV_REL, DptMain::DPT_232xx, 3, [this](const uint8_t* p){ onKnxHSVRel(p[0],p[1],p[2]); });
  }
  if (GA_IN_RGBW_REL) {
    registerMultiHandler(GA_IN_RGBW_REL, DptMain::DPT_251xx, 4, [this](const uint8_t* p){ onKnxRGBWRel(p[0],p[1],p[2],p[3]); });
  }
  if (GA_IN_H) {
    register1ByteHandler(GA_IN_H, DptMain::DPT_5xx, [this](uint8_t v){ onKnxH(byteToHueDeg(v)); });
  }
  if (GA_IN_S) {
    register1ByteHandler(GA_IN_S, DptMain::DPT_5xx, [this](uint8_t v){ onKnxS(byteToPct01(v)); });
  }
  if (GA_IN_V) {
    register1ByteHandler(GA_IN_V, DptMain::DPT_5xx, [this](uint8_t v){ onKnxV(byteToPct01(v)); });
  }
  if (GA_IN_BRI_REL) { KNX.addGroupObject(GA_IN_BRI_REL, DptMain::DPT_3xx, false, true); KNX.onGroup(GA_IN_BRI_REL, [this](uint16_t, DptMain, KnxService s, const uint8_t* p, uint8_t l){ if (s==KnxService::GroupValue_Write && l>=1) onKnxBrightnessRel(p[0]); }); }
  if (GA_IN_R_REL)   { KNX.addGroupObject(GA_IN_R_REL,   DptMain::DPT_3xx, false, true); KNX.onGroup(GA_IN_R_REL,   [this](uint16_t, DptMain, KnxService s, const uint8_t* p, uint8_t l){ if (s==KnxService::GroupValue_Write && l>=1) onKnxColorRel(0,p[0]); }); }
  if (GA_IN_G_REL)   { KNX.addGroupObject(GA_IN_G_REL,   DptMain::DPT_3xx, false, true); KNX.onGroup(GA_IN_G_REL,   [this](uint16_t, DptMain, KnxService s, const uint8_t* p, uint8_t l){ if (s==KnxService::GroupValue_Write && l>=1) onKnxColorRel(1,p[0]); }); }
  if (GA_IN_B_REL)   { KNX.addGroupObject(GA_IN_B_REL,   DptMain::DPT_3xx, false, true); KNX.onGroup(GA_IN_B_REL,   [this](uint16_t, DptMain, KnxService s, const uint8_t* p, uint8_t l){ if (s==KnxService::GroupValue_Write && l>=1) onKnxColorRel(2,p[0]); }); }
  if (GA_IN_W_REL)   { KNX.addGroupObject(GA_IN_W_REL,   DptMain::DPT_3xx, false, true); KNX.onGroup(GA_IN_W_REL,   [this](uint16_t, DptMain, KnxService s, const uint8_t* p, uint8_t l){ if (s==KnxService::GroupValue_Write && l>=1) onKnxWhiteRel(p[0]); }); }
  if (GA_IN_WW_REL)  { KNX.addGroupObject(GA_IN_WW_REL,  DptMain::DPT_3xx, false, true); KNX.onGroup(GA_IN_WW_REL,  [this](uint16_t, DptMain, KnxService s, const uint8_t* p, uint8_t l){ if (s==KnxService::GroupValue_Write && l>=1) onKnxWWRel(p[0]); }); }
  if (GA_IN_CW_REL)  { KNX.addGroupObject(GA_IN_CW_REL,  DptMain::DPT_3xx, false, true); KNX.onGroup(GA_IN_CW_REL,  [this](uint16_t, DptMain, KnxService s, const uint8_t* p, uint8_t l){ if (s==KnxService::GroupValue_Write && l>=1) onKnxCWRel(p[0]); }); }
  if (GA_IN_H_REL)   { KNX.addGroupObject(GA_IN_H_REL,   DptMain::DPT_3xx, false, true); KNX.onGroup(GA_IN_H_REL,   [this](uint16_t, DptMain, KnxService s, const uint8_t* p, uint8_t l){ if (s==KnxService::GroupValue_Write && l>=1) onKnxHueRel(p[0]); }); }
  if (GA_IN_S_REL)   { KNX.addGroupObject(GA_IN_S_REL,   DptMain::DPT_3xx, false, true); KNX.onGroup(GA_IN_S_REL,   [this](uint16_t, DptMain, KnxService s, const uint8_t* p, uint8_t l){ if (s==KnxService::GroupValue_Write && l>=1) onKnxSatRel(p[0]); }); }
  if (GA_IN_V_REL)   { KNX.addGroupObject(GA_IN_V_REL,   DptMain::DPT_3xx, false, true); KNX.onGroup(GA_IN_V_REL,   [this](uint16_t, DptMain, KnxService s, const uint8_t* p, uint8_t l){ if (s==KnxService::GroupValue_Write && l>=1) onKnxValRel(p[0]); }); }
  if (GA_IN_FX_REL)  { KNX.addGroupObject(GA_IN_FX_REL,  DptMain::DPT_3xx, false, true); KNX.onGroup(GA_IN_FX_REL,  [this](uint16_t, DptMain, KnxService s, const uint8_t* p, uint8_t l){ if (s==KnxService::GroupValue_Write && l>=1) onKnxEffectRel(p[0]); }); }
  // DPT 10.001 TimeOfDay (3 bytes)
  if (GA_IN_TIME) {
    KNX.addGroupObject(GA_IN_TIME, DptMain::DPT_10xx, /*tx=*/false, /*rx=*/true);
    KNX.onGroup(GA_IN_TIME, [this](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write && p && len >= 3) onKnxTime_10_001(p, len);
    });
  }
  // DPT 11.001 Date (3 bytes)
  if (GA_IN_DATE) {
    KNX.addGroupObject(GA_IN_DATE, DptMain::DPT_11xx, false, true);
    KNX.onGroup(GA_IN_DATE, [this](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write && p && len >= 3) onKnxDate_11_001(p, len);
    });
  }
  // DPT 19.001 DateTime (8 bytes)
  if (GA_IN_DATETIME) {
    KNX.addGroupObject(GA_IN_DATETIME, DptMain::DPT_19xx, false, true);
    KNX.onGroup(GA_IN_DATETIME, [this](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write && p && len >= 8) onKnxDateTime_19_001(p, len);
    });
  }

  // Parse OUT GA strings
  GA_OUT_PWR = parseGA(gaOutPower);
  GA_OUT_BRI = parseGA(gaOutBri);
  GA_OUT_FX  = parseGA(gaOutFx);
  GA_OUT_R   = parseGA(gaOutR);
  GA_OUT_G   = parseGA(gaOutG);
  GA_OUT_B   = parseGA(gaOutB);
  GA_OUT_PRE = parseGA(gaOutPreset);
  GA_OUT_W   = parseGA(gaOutW);
  GA_OUT_CCT = parseGA(gaOutCct);
  GA_OUT_WW  = parseGA(gaOutWW);
  GA_OUT_CW  = parseGA(gaOutCW);
  GA_OUT_RGB  = parseGA(gaOutRGB);
  GA_OUT_HSV  = parseGA(gaOutHSV);
  GA_OUT_RGBW = parseGA(gaOutRGBW);
  GA_OUT_H    = parseGA(gaOutH);
  GA_OUT_S    = parseGA(gaOutS);
  GA_OUT_V    = parseGA(gaOutV);
  GA_OUT_INT_TEMP = parseGA(gaOutIntTemp);
  GA_OUT_TEMP     = parseGA(gaOutTemp);
  GA_OUT_INT_TEMP_ALARM = parseGA(gaOutIntTempAlarm);
  GA_OUT_TEMP_ALARM     = parseGA(gaOutTempAlarm);

  Serial.printf("[KNX-UM] OUT pwr=0x%04X bri=0x%04X R=0x%04X G=0x%04X B=0x%04X W=0x%04X CCT=0x%04X WW=0x%04X CW=0x%04X fx=0x%04X pre=0x%04X H=%04X S=%04X V=%04X\n",
                GA_OUT_PWR, GA_OUT_BRI, GA_OUT_R, GA_OUT_G, GA_OUT_B, GA_OUT_W, GA_OUT_CCT, GA_OUT_WW, GA_OUT_CW, GA_OUT_FX, GA_OUT_PRE, GA_OUT_H, GA_OUT_S, GA_OUT_V);

  // --- Gate outputs by LED capability ---
  if (!(g_ledProfile == LedProfile::RGB || g_ledProfile == LedProfile::RGBW || g_ledProfile == LedProfile::RGBCCT)) {
    GA_OUT_R = GA_OUT_G = GA_OUT_B = 0;
  }
  if (!(g_ledProfile == LedProfile::RGBW)) {
    GA_OUT_W = 0;
  }
  if (!(g_ledProfile == LedProfile::CCT || g_ledProfile == LedProfile::RGBCCT)) {
    GA_OUT_CCT = GA_OUT_WW = GA_OUT_CW = 0;
  }
  if (!(g_ledProfile == LedProfile::RGB || g_ledProfile == LedProfile::RGBW || g_ledProfile == LedProfile::RGBCCT)) {
    GA_OUT_RGB = GA_OUT_HSV = GA_OUT_H = GA_OUT_S = GA_OUT_V = 0;
  }
  if (!(g_ledProfile == LedProfile::RGBW || g_ledProfile == LedProfile::RGBCCT)) {
    GA_OUT_RGBW = 0;
  }

  // Register inbound objects and callbacks
  if (GA_IN_PWR) {
    KNX.addGroupObject(GA_IN_PWR, DptMain::DPT_1xx, false, true);
    KNX.onGroup(GA_IN_PWR, [this](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write && p && len>=1) onKnxPower((p[0] & 0x01)!=0);
    });
  }

  if (GA_IN_BRI) {
    register1ByteHandler(GA_IN_BRI, DptMain::DPT_5xx, [this](uint8_t v){ onKnxBrightness(v); });
  }

  // RGB inputs (masked to 0 if unsupported)
  if (GA_IN_R) {
    register1ByteHandler(GA_IN_R, DptMain::DPT_5xx, [this](uint8_t r){
      uint8_t cr,cg,cb,cw; getCurrentRGBW(cr,cg,cb,cw); onKnxRGB(r,cg,cb);
    });
  }
  if (GA_IN_G) {
    register1ByteHandler(GA_IN_G, DptMain::DPT_5xx, [this](uint8_t g){
      uint8_t cr,cg,cb,cw; getCurrentRGBW(cr,cg,cb,cw); onKnxRGB(cr,g,cb);
    });
  }
  if (GA_IN_B) {
    register1ByteHandler(GA_IN_B, DptMain::DPT_5xx, [this](uint8_t b){
      uint8_t cr,cg,cb,cw; getCurrentRGBW(cr,cg,cb,cw); onKnxRGB(cr,cg,b);
    });
  }

  if (GA_IN_FX) {
    register1ByteHandler(GA_IN_FX, DptMain::DPT_5xx, [this](uint8_t v){ onKnxEffect(v); });
  }

  if (GA_IN_PRE) {
    KNX.addGroupObject(GA_IN_PRE, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_PRE, [this](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){ if (svc==KnxService::GroupValue_Write && p && len>=1) onKnxPreset(p[0]); });
  }

  // Outbound/status objects
  if (GA_OUT_PWR) KNX.addGroupObject(GA_OUT_PWR, DptMain::DPT_1xx, /*tx=*/true, /*rx=*/false);
  if (GA_OUT_BRI) KNX.addGroupObject(GA_OUT_BRI, DptMain::DPT_5xx, true, false);
  if (GA_OUT_FX)  KNX.addGroupObject(GA_OUT_FX,  DptMain::DPT_5xx, true, false);
  if (GA_OUT_R)   KNX.addGroupObject(GA_OUT_R,   DptMain::DPT_5xx, true, false);
  if (GA_OUT_G)   KNX.addGroupObject(GA_OUT_G,   DptMain::DPT_5xx, true, false);
  if (GA_OUT_B)   KNX.addGroupObject(GA_OUT_B,   DptMain::DPT_5xx, true, false);
  if (GA_OUT_PRE) KNX.addGroupObject(GA_OUT_PRE, DptMain::DPT_5xx, true, false);
  if (GA_OUT_W)   KNX.addGroupObject(GA_OUT_W,   DptMain::DPT_5xx, true, false);
  if (GA_OUT_CCT) KNX.addGroupObject(GA_OUT_CCT, DptMain::DPT_7xx, true, false);
  if (GA_OUT_WW)  KNX.addGroupObject(GA_OUT_WW,  DptMain::DPT_5xx, true, false);
  if (GA_OUT_CW)  KNX.addGroupObject(GA_OUT_CW,  DptMain::DPT_5xx, true, false);
  if (GA_OUT_RGB)  KNX.addGroupObject(GA_OUT_RGB,  DptMain::DPT_232xx, true, false);
  if (GA_OUT_HSV)  KNX.addGroupObject(GA_OUT_HSV,  DptMain::DPT_232xx, true, false);
  if (GA_OUT_RGBW) KNX.addGroupObject(GA_OUT_RGBW, DptMain::DPT_251xx, true, false);
  if (GA_OUT_H)    KNX.addGroupObject(GA_OUT_H,    DptMain::DPT_5xx, true, false);
  if (GA_OUT_S)    KNX.addGroupObject(GA_OUT_S,    DptMain::DPT_5xx, true, false);
  if (GA_OUT_V)    KNX.addGroupObject(GA_OUT_V,    DptMain::DPT_5xx, true, false);
  if (GA_OUT_INT_TEMP) KNX.addGroupObject(GA_OUT_INT_TEMP, DptMain::DPT_14xx, true, false);
  if (GA_OUT_TEMP)     KNX.addGroupObject(GA_OUT_TEMP,     DptMain::DPT_14xx, true, false);
  if (GA_OUT_INT_TEMP_ALARM) KNX.addGroupObject(GA_OUT_INT_TEMP_ALARM, DptMain::DPT_1xx, /*tx=*/true, /*rx=*/false);
  if (GA_OUT_TEMP_ALARM)     KNX.addGroupObject(GA_OUT_TEMP_ALARM,     DptMain::DPT_1xx, /*tx=*/true, /*rx=*/false);

  Serial.printf("[KNX-UM] OUT intTemp=0x%04X temp=0x%04X\n", GA_OUT_INT_TEMP, GA_OUT_TEMP);
  Serial.printf("[KNX-UM] OUT intTempAlarm=0x%04X tempAlarm=0x%04X (thr: %.1f/%.1f °C, hyst=%.1f)\n",
  
    GA_OUT_INT_TEMP_ALARM, GA_OUT_TEMP_ALARM, intTempAlarmMaxC, dallasTempAlarmMaxC, tempAlarmHystC);
  // Start KNX
  IPAddress ip = WiFi.localIP();
  if (!ip || ip.toString() == String("0.0.0.0")) {
    Serial.println("[KNX-UM] WiFi connected but no IP yet, deferring KNX.begin().");
    return;
  }

#ifdef ARDUINO_ARCH_ESP32
  WiFi.setSleep(false);     // modem-sleep off helps multicast reliability
#endif

  bool ok = KNX.begin();
  Serial.printf("[KNX-UM] KNX.begin() -> %s (localIP=%s)\n", ok ? "OK" : "FAILED", ip.toString().c_str());
}

void KnxIpUsermod::publishState() {
  // If nothing is pending and no OUT GAs are configured, bail early
  const bool anyPending  = _pendingTxPower || _pendingTxBri || _pendingTxFx;
  const bool anyColorOut = (GA_OUT_R || GA_OUT_G || GA_OUT_B || GA_OUT_W || GA_OUT_CCT || GA_OUT_WW || GA_OUT_CW);

  // Current (live) state snapshot
  const Segment& seg = strip.getSegment(0);
  const uint32_t c   = seg.colors[0];
  const uint8_t  r   = R(c), g = G(c), b = B(c), w = W(c);
  const uint8_t  cct = seg.cct; // 0..255 (0=warm .. 255=cold)

  // Compute which OUT objects actually changed since the last published snapshot
  const bool chR   = GA_OUT_R   && (r   != LAST_R);
  const bool chG   = GA_OUT_G   && (g   != LAST_G);
  const bool chB   = GA_OUT_B   && (b   != LAST_B);
  const bool chW   = GA_OUT_W   && (w   != LAST_W);
  const bool chCct = GA_OUT_CCT && (cct != LAST_CCT);

  // Derived warm/cold components from W and CCT (compare to last)
  const uint8_t ww     = (uint8_t)((uint16_t)w * (255u - cct) / 255u);
  const uint8_t cw     = (uint8_t)((uint16_t)w * cct / 255u);
  const uint8_t lastWw = (uint8_t)((uint16_t)LAST_W * (255u - LAST_CCT) / 255u);
  const uint8_t lastCw = (uint8_t)((uint16_t)LAST_W * LAST_CCT / 255u);
  const bool chWW = GA_OUT_WW && (ww != lastWw);
  const bool chCW = GA_OUT_CW && (cw != lastCw);

  const bool anyColorChanged = chR || chG || chB || chW || chCct || chWW || chCW;

  if (!anyPending && !anyColorOut && !GA_OUT_PRE) return;
  if (!anyPending && !anyColorChanged && !GA_OUT_PRE) return; // nothing to send

  // Base state
  const bool    pwr     = (bri > 0);
  const uint8_t pct     = (uint8_t)((bri * 100u + 127u) / 255u);   // 0..100 (DPT 5.001)
  const uint8_t fxIndex = effectCurrent;                           // 0..255

  // Pending (coalesced) telegrams
  if (_pendingTxPower && GA_OUT_PWR) KNX.write1Bit(GA_OUT_PWR, pwr);   // DPT 1.001
  if (_pendingTxBri   && GA_OUT_BRI) KNX.writeScaling(GA_OUT_BRI, pct); // DPT 5.001
  if (_pendingTxFx    && GA_OUT_FX)  {
    uint8_t v = fxIndex;                                              // DPT 5.xxx raw
    KNX.groupValueWrite(GA_OUT_FX, &v, 1);
  }

  // Colors / White / CCT / WW / CW — only if changed
  if (anyColorOut) {
    if (chR)   { uint8_t v=r;   KNX.groupValueWrite(GA_OUT_R,   &v, 1); }
    if (chG)   { uint8_t v=g;   KNX.groupValueWrite(GA_OUT_G,   &v, 1); }
    if (chB)   { uint8_t v=b;   KNX.groupValueWrite(GA_OUT_B,   &v, 1); }
    if (chW)   { uint8_t v=w;   KNX.groupValueWrite(GA_OUT_W,   &v, 1); }
    if (chCct) {
      uint16_t kelvin = cct255ToKelvin(cct);                          // DPT 7.600
      uint8_t payload[2] = { (uint8_t)(kelvin >> 8), (uint8_t)(kelvin & 0xFF) }; // big-endian
      KNX.groupValueWrite(GA_OUT_CCT, payload, 2);
    }
    if (chWW)  { uint8_t v=ww;  KNX.groupValueWrite(GA_OUT_WW,  &v, 1); }
    if (chCW)  { uint8_t v=cw;  KNX.groupValueWrite(GA_OUT_CW,  &v, 1); }
  }

  if (anyColorChanged) {
    // Snapshot again (already have r,g,b,w,cct)
    // 1) RGB (DPST-232-600) 3 bytes
    if (GA_OUT_RGB) {
      uint8_t rgb[3] = { r, g, b };
      KNX.groupValueWrite(GA_OUT_RGB, rgb, 3);
    }
    // 2) HSV (DPST-232-600) 3 bytes: [H_byte, S_byte, V_byte]
    float hDeg, s01, v01; rgbToHsv(r,g,b,hDeg,s01,v01);
    if (GA_OUT_HSV) {
      uint8_t hsv[3] = { hueDegToByte(hDeg), pct01ToByte(s01), pct01ToByte(v01) };
      KNX.groupValueWrite(GA_OUT_HSV, hsv, 3);
    }
    // 3) RGBW (DPST-251-600) 6 bytes: [R,G,B,W,0,0]
    if (GA_OUT_RGBW) {
      uint8_t rgbw[6] = { r, g, b, w, 0x00, 0x00 };
      KNX.groupValueWrite(GA_OUT_RGBW, rgbw, 6);
    }
    // 4) Individual H/S/V (1 byte each)
    if (GA_OUT_H) { uint8_t hb = hueDegToByte(hDeg); KNX.groupValueWrite(GA_OUT_H, &hb, 1); }
    if (GA_OUT_S) { uint8_t sb = pct01ToByte(s01);  KNX.groupValueWrite(GA_OUT_S, &sb, 1); }
    if (GA_OUT_V) { uint8_t vb = pct01ToByte(v01);  KNX.groupValueWrite(GA_OUT_V, &vb, 1); }
  }

// Preset index (if configured) – send only if it actually changed
  if (GA_OUT_PRE) {
    if (s_lastPresetSent != _lastPreset) {
      uint8_t p = _lastPreset; // last applied via onKnxPreset()
      KNX.groupValueWrite(GA_OUT_PRE, &p, 1);
      s_lastPresetSent = _lastPreset;
    }
  }

  // Publish temperature if configured and available
  publishTemperatureOnce();

  // Update “last published” snapshot for color/CCT
  LAST_R = r; LAST_G = g; LAST_B = b; LAST_W = w; LAST_CCT = cct;

  // Clear pending flags after publish
  _pendingTxPower = _pendingTxBri = _pendingTxFx = false;
}

void KnxIpUsermod::loop() {
  if (!enabled) return;
// --- Detect LED capability (lc) change at runtime and rebuild GA mapping immediately ---
static uint8_t       s_lastLc = 0xFF;
static unsigned long s_lcChangedAt = 0;

// OR light-capabilities across all segments (robust for multi-bus setups)
uint8_t lcNow = 0;
const uint16_t segCount = strip.getSegmentsNum();
for (uint16_t i = 0; i < segCount; i++) {
  lcNow |= strip.getSegment(i).getLightCapabilities(); // bit0=RGB, bit1=W, bit2=CCT
}

if (lcNow != s_lastLc) {
  s_lastLc = lcNow;
  s_lcChangedAt = millis();
  Serial.printf("[KNX-UM] LED capabilities changed (lc=0x%02X). Pending rebuild...\n", lcNow);
}

// Debounce (avoid thrashing while user edits LED settings in the UI)
if (s_lcChangedAt && (millis() - s_lcChangedAt >= 300)) {
  s_lcChangedAt = 0;

  LedProfile newProf = detectLedProfileFromSegments();
  if (newProf != g_ledProfile) {
    Serial.printf("[KNX-UM] LED profile changed %s -> %s. Re-registering KNX GAs now.\n",
      (g_ledProfile==LedProfile::MONO?"MONO":
       g_ledProfile==LedProfile::CCT?"CCT":
       g_ledProfile==LedProfile::RGB?"RGB":
       g_ledProfile==LedProfile::RGBW?"RGBW":"RGBCCT"),
      (newProf==LedProfile::MONO?"MONO":
       newProf==LedProfile::CCT?"CCT":
       newProf==LedProfile::RGB?"RGB":
       newProf==LedProfile::RGBW?"RGBW":"RGBCCT"));

    // Full rebuild: drop socket + GA registry; setup() will detect and re-register
    KNX.end();
    KNX.clearRegistrations();
    g_ledProfile = newProf;   // update hint; setup() re-detects and gates GAs
    setup();

    // Optional: primer so routers/ETS learn us immediately
    if (KNX.running()) {
      const uint16_t primer = knxMakeGroupAddress(0,0,1);
      KNX.groupValueRead(primer);
    }
  }
}


  // If KNX could not start in setup() due to missing IP, retry once we have one.
  static bool knxStartedLogged = false;
  if (!knxStartedLogged) {
    if (WiFi.status() == WL_CONNECTED) {
      IPAddress ip = WiFi.localIP();
      if (ip && ip.toString() != String("0.0.0.0")) {
        Serial.println("[KNX-UM] WiFi ready (got IP). Retrying KNX.begin()...");
        bool ok = KNX.begin();
        Serial.printf("[KNX-UM] KNX.begin() -> %s (localIP=%s)\n",
                      ok ? "OK" : "FAILED",
                      ip.toString().c_str());
        if (ok) {
          publishState(); // send immediate Power + Dim so we have TX to see in Wireshark
        }        
        if (ok) knxStartedLogged = true;
      } else {
        Serial.printf("[KNX-UM] WiFi connected, waiting for IP... (status=%d)\n", (int)WiFi.status());
      }
    } else {
      Serial.printf("[KNX-UM] WiFi not connected yet (status=%d).\n", (int)WiFi.status());
    }
  }

  // Detect Wi-Fi IP changes and refresh IGMP membership without tearing socket down
  static IPAddress _lastIpForKnx;
  if (KNX.running()) {
    IPAddress cur = WiFi.localIP();
    if (cur && cur.toString() != String("0.0.0.0")) {
      if (_lastIpForKnx != cur) {
        Serial.printf("[KNX-UM] WiFi IP changed %s -> %s, refreshing KNX multicast membership...\n",
                      _lastIpForKnx.toString().c_str(), cur.toString().c_str());
        if (!KNX.rejoinMulticast()) {
          // Fallback: hard restart the KNX socket if rejoin fails
          KNX.end();
          KNX.begin();
        }
        _lastIpForKnx = cur;
      }
    }
  }

  KNX.loop();

  // Publish immediately when GUI changed brightness/power/CCT/RGBW (debounced)
  bool curOn = (bri > 0);
  const Segment& seg = strip.getSegment(0);
  uint8_t curCct = seg.cct; // 0..255 (0=warm .. 255=cold)
  const uint32_t c = seg.colors[0];
  uint8_t r = R(c), g = G(c), b = B(c), w = W(c);

  bool briOrPwrChanged = (curOn != _lastSentOn) || (bri != _lastSentBri);
  bool cctChanged      = (curCct != LAST_CCT);
  bool rgbwChanged     = (r != LAST_R) || (g != LAST_G) || (b != LAST_B) || (w != LAST_W);

  uint32_t now = millis();

  // Optional extra throttle during effects to avoid chatter
  uint16_t minInterval = _minUiSendIntervalMs;
  if (effectCurrent != 0) minInterval = 1000;  // be gentler while an effect runs

  if ((briOrPwrChanged || cctChanged || rgbwChanged) && (now - _lastUiSendMs >= minInterval)) {
    _lastUiSendMs = now;

    // If brightness/power changed, make sure those GAs get sent even if no color changed
    if (_lastSentOn != curOn) {
      _pendingTxPower = true;   // just power on toggle
    } 
    if (bri != _lastSentBri){
      _pendingTxBri = true;     // brightness changed
    }

    // Debug trace so you can see what triggered the TX
    Serial.printf("[KNX-UM] GUI change -> publish (%s%s%s) bri %u→%u, on %u→%u, cct %u→%u, "
                "RGBW %u,%u,%u,%u → %u,%u,%u,%u\n",
                briOrPwrChanged ? "bri/pwr " : "",
                cctChanged      ? "cct "     : "",
                rgbwChanged     ? "rgbw "    : "",
                _lastSentBri, bri, _lastSentOn, curOn, LAST_CCT, curCct,
                LAST_R, LAST_G, LAST_B, LAST_W, r, g, b, w);
    publishState();  // coalesced sender (respects pending flags + sends CCT/white if configured)

  // Power/brightness debounce cache (color/CCT snapshots are updated in publishState)
  _lastSentOn  = curOn;
  _lastSentBri = bri;
  }

  // --- Effect / Preset change detection (publish only those) ---
  static uint8_t lastFxSent = 0xFF;
  uint8_t fxLive = effectCurrent;                       // WLED global: current effect index (byte)
  if (GA_OUT_FX && fxLive != lastFxSent) {
    _pendingTxFx = true;                                // only FX pending
    Serial.printf("[KNX-UM] GUI change -> publish (fx %u)\n", fxLive);
    publishState();                                     // sends GA_OUT_FX only
    lastFxSent = fxLive;
  }

  static int lastPresetSent = -1;                       // -1 sentinel; currentPreset is byte
  int preLive = (int)currentPreset;                     // WLED global: 0 none, >0 active (byte)
  if (GA_OUT_PRE && preLive != lastPresetSent) {
    _lastPreset = (uint8_t)constrain(preLive, 0, 255);  // keep existing _lastPreset semantics
    Serial.printf("[KNX-UM] GUI change -> publish (preset %d)\n", preLive);
    publishState();                                     // publishState() gates PRE to "changed-only"
    lastPresetSent = preLive;
  }

  // Optional periodic state publish (coalesced into normal TX path)
  if (periodicEnabled) {
    uint32_t now2 = millis();
    if (now2 - _lastPeriodicMs >= periodicIntervalMs) {
      _lastPeriodicMs = now2;
      // Schedule power + brightness + effect; colors are sent if GA_OUT_* are configured
      scheduleStatePublish(true, true, true);
    }
  }


  if (_nextTxAt && millis() >= _nextTxAt) {
    _nextTxAt = 0;
    Serial.println("[KNX-UM] publishState() due.");
    publishState();
  }
}

void KnxIpUsermod::scheduleStatePublish(bool pwr, bool bri_, bool fx) {
  _pendingTxPower |= pwr;
  _pendingTxBri   |= bri_;
  _pendingTxFx    |= fx;
  unsigned long now = millis();
  if (_nextTxAt == 0 || now > _nextTxAt) _nextTxAt = now + txRateLimitMs;
}

void KnxIpUsermod::addToConfig(JsonObject& root) {
  JsonObject top = root.createNestedObject("KNX_IP");
  top["enabled"] = enabled;
  top["individual_address"] = individualAddr;
  top["tx_rate_limit_ms"] = txRateLimitMs;
  top["periodic_enabled"] = periodicEnabled;
  top["periodic_interval_ms"] = periodicIntervalMs;
  top["cct_kelvin_min"] = kelvinMin;
  top["cct_kelvin_max"] = kelvinMax;
  top["communication_enhancement"]      = commEnhance;
  top["communication_resends"]      = commResends;
  top["communication_resend_gap"]= commResendGapMs;
  top["communication_rx_dedup"]  = commRxDedupMs;
  top["Internal Temperature Alarm"]   = intTempAlarmMaxC;
  top["Temperature Sensor Alarm"]     = dallasTempAlarmMaxC;
  top["Temperature Alarm Hysteresis"] = tempAlarmHystC;


  JsonObject gIn  = top.createNestedObject("GA in");
  gIn["power"]  = gaInPower;
  gIn["bri"]    = gaInBri;
  gIn["r"]      = gaInR;
  gIn["g"]      = gaInG;
  gIn["b"]      = gaInB;
  gIn["w"]      = gaInW;
  gIn["cct"]    = gaInCct;
  gIn["ww"]     = gaInWW;
  gIn["cw"]     = gaInCW;
  gIn["h"]      = gaInH;       // DPT 5.003
  gIn["s"]      = gaInS;       // DPT 5.001
  gIn["v"]      = gaInV;       // DPT 5.001
  gIn["fx"]     = gaInFx;
  gIn["preset"] = gaInPreset;
  gIn["rgb"]    = gaInRGB;     // DPST-232-600 (3B)
  gIn["hsv"]    = gaInHSV;     // DPST-232-600 (3B)
  gIn["rgbw"]   = gaInRGBW;    // DPST-251-600 (6B)
  gIn["time"]     = gaInTime;
  gIn["date"]     = gaInDate;
  gIn["datetime"] = gaInDateTime;
  gIn["bri_rel"] = gaInBriRel; // DPT 3.007 step/direction
  gIn["r_rel"]   = gaInRRel;   // DPT 3.007
  gIn["g_rel"]   = gaInGRel;   // DPT 3.007
  gIn["b_rel"]   = gaInBRel;   // DPT 3.007
  gIn["w_rel"]   = gaInWRel;   // DPT 3.007
  gIn["ww_rel"]  = gaInWWRel;  // DPT 3.007
  gIn["cw_rel"]  = gaInCWRel;  // DPT 3.007
  gIn["h_rel"]   = gaInHRel;   // DPT 3.007
  gIn["s_rel"]   = gaInSRel;   // DPT 3.007
  gIn["v_rel"]   = gaInVRel;   // DPT 3.007
  gIn["fx_rel"]  = gaInFxRel;  // DPT 3.007
  gIn["rgb_rel"] = gaInRGBRel;  // DPST-232-600 (3B, each byte low nibble DPT3)
  gIn["hsv_rel"] = gaInHSVRel;  // DPST-232-600 (3B, each byte low nibble DPT3)
  gIn["rgbw_rel"] = gaInRGBWRel; // DPST-251-600 (4+ bytes, first 4 used)


  JsonObject gOut = top.createNestedObject("GA out");
  gOut["power"]  = gaOutPower;
  gOut["bri"]    = gaOutBri;
  gOut["r"]      = gaOutR;
  gOut["g"]      = gaOutG;
  gOut["b"]      = gaOutB;
  gOut["w"]      = gaOutW;
  gOut["cct"]    = gaOutCct;
  gOut["ww"]     = gaOutWW;
  gOut["cw"]     = gaOutCW;
  gOut["h"]      = gaOutH;       // DPT 5.003
  gOut["s"]      = gaOutS;       // DPT 5.001
  gOut["v"]      = gaOutV;       // DPT 5.001
  gOut["fx"]     = gaOutFx;
  gOut["preset"] = gaOutPreset;
  gOut["rgb"]    = gaOutRGB;     // DPST-232-600 (3B)
  gOut["hsv"]    = gaOutHSV;     // DPST-232-600 (3B)
  gOut["rgbw"]   = gaOutRGBW;    // DPST-251-600 (6B)
  gOut["Internal_Temperature"]  = gaOutIntTemp;
  gOut["Temperature_Sensor"]    = gaOutTemp;
  gOut["Internal_Temperature_Alarm"]  = gaOutIntTempAlarm;
  gOut["Temperature_Sensor_Alarm"]    = gaOutTempAlarm;
}

bool KnxIpUsermod::readFromConfig(JsonObject& root) {
  JsonObject top = root["KNX_IP"];
  if (top.isNull()) {
    top = root["KNX-IP"];       // legacy
    if (top.isNull()) return false;
  }

  enabled = top["enabled"] | enabled;
  strlcpy(individualAddr, top["individual_address"] | individualAddr, sizeof(individualAddr));
  kelvinMin = top["cct_kelvin_min"] | kelvinMin;
  kelvinMax = top["cct_kelvin_max"] | kelvinMax;
  periodicEnabled    = top["periodic_enabled"]     | periodicEnabled;
  periodicIntervalMs = top["periodic_interval_ms"] | periodicIntervalMs;
  commEnhance       = top["communication_enhancement"]    | commEnhance;
  commResends       = top["communication_resends"]        | commResends;
  commResendGapMs   = top["communication_resend_gap"]  | commResendGapMs;
  commRxDedupMs     = top["communication_rx_dedup"]    | commRxDedupMs;


  // accept either "GA in"/"GA out" (what we save) or "in"/"out"
  JsonObject gIn  = top["GA in"];  if (gIn.isNull())  gIn  = top["in"];
  JsonObject gOut = top["GA out"]; if (gOut.isNull()) gOut = top["out"];

  if (!gIn.isNull()) {
    strlcpy(gaInPower,  gIn["power"]  | gaInPower,  sizeof(gaInPower));
    strlcpy(gaInBri,    gIn["bri"]    | gaInBri,    sizeof(gaInBri));
    strlcpy(gaInR,      gIn["r"]      | gaInR,      sizeof(gaInR));
    strlcpy(gaInG,      gIn["g"]      | gaInG,      sizeof(gaInG));
    strlcpy(gaInB,      gIn["b"]      | gaInB,      sizeof(gaInB));
    strlcpy(gaInW,      gIn["w"]      | gaInW,      sizeof(gaInW));
    strlcpy(gaInCct,    gIn["cct"]    | gaInCct,    sizeof(gaInCct));
    strlcpy(gaInWW,     gIn["ww"]     | gaInWW,     sizeof(gaInWW));
    strlcpy(gaInCW,     gIn["cw"]     | gaInCW,     sizeof(gaInCW));
    strlcpy(gaInH,      gIn["h"]      | gaInH,      sizeof(gaInH));
    strlcpy(gaInS,      gIn["s"]      | gaInS,      sizeof(gaInS));
    strlcpy(gaInV,      gIn["v"]      | gaInV,      sizeof(gaInV));
    strlcpy(gaInFx,     gIn["fx"]     | gaInFx,     sizeof(gaInFx));
    strlcpy(gaInPreset, gIn["preset"] | gaInPreset, sizeof(gaInPreset));
    strlcpy(gaInRGB,    gIn["rgb"]    | gaInRGB,    sizeof(gaInRGB));
    strlcpy(gaInHSV,    gIn["hsv"]    | gaInHSV,    sizeof(gaInHSV));
    strlcpy(gaInRGBW,   gIn["rgbw"]   | gaInRGBW,   sizeof(gaInRGBW));
    strlcpy(gaInTime,     gIn["time"]     | gaInTime,     sizeof(gaInTime));
    strlcpy(gaInDate,     gIn["date"]     | gaInDate,     sizeof(gaInDate));
    strlcpy(gaInDateTime, gIn["datetime"] | gaInDateTime, sizeof(gaInDateTime));
  strlcpy(gaInBriRel, gIn["bri_rel"] | gaInBriRel, sizeof(gaInBriRel)); // DPT 3.007
  strlcpy(gaInRRel,   gIn["r_rel"]   | gaInRRel,   sizeof(gaInRRel));   // DPT 3.007
  strlcpy(gaInGRel,   gIn["g_rel"]   | gaInGRel,   sizeof(gaInGRel));   // DPT 3.007
  strlcpy(gaInBRel,   gIn["b_rel"]   | gaInBRel,   sizeof(gaInBRel));   // DPT 3.007
  strlcpy(gaInWRel,   gIn["w_rel"]   | gaInWRel,   sizeof(gaInWRel));   // DPT 3.007
  strlcpy(gaInWWRel,  gIn["ww_rel"]  | gaInWWRel,  sizeof(gaInWWRel));  // DPT 3.007
  strlcpy(gaInCWRel,  gIn["cw_rel"]  | gaInCWRel,  sizeof(gaInCWRel));  // DPT 3.007
  strlcpy(gaInHRel,   gIn["h_rel"]   | gaInHRel,   sizeof(gaInHRel));   // DPT 3.007
  strlcpy(gaInSRel,   gIn["s_rel"]   | gaInSRel,   sizeof(gaInSRel));   // DPT 3.007
  strlcpy(gaInVRel,   gIn["v_rel"]   | gaInVRel,   sizeof(gaInVRel));   // DPT 3.007
  strlcpy(gaInFxRel,  gIn["fx_rel"]  | gaInFxRel,  sizeof(gaInFxRel));  // DPT 3.007
  strlcpy(gaInRGBRel,  gIn["rgb_rel"]  | gaInRGBRel,  sizeof(gaInRGBRel));  // composite rel
  strlcpy(gaInHSVRel,  gIn["hsv_rel"]  | gaInHSVRel,  sizeof(gaInHSVRel));  // composite rel
  strlcpy(gaInRGBWRel, gIn["rgbw_rel"] | gaInRGBWRel, sizeof(gaInRGBWRel)); // composite rel
  }

  // --- Pre-validate GA / PA strings (clear invalid to prevent repeated parse warnings) ---
  // Validate individual address (personal address / PA)
  if (*individualAddr && !KnxIpUsermod::validateIndividualAddressString(individualAddr)) {
    Serial.printf("[KNX-UM][WARN] Invalid individual address '%s' in config -> reverting to default 1.1.100\n", individualAddr);
    strlcpy(individualAddr, "1.1.100", sizeof(individualAddr));
  }

  auto validateOrClear = [](char* s, const char* tag){
    if (*s && !KnxIpUsermod::validateGroupAddressString(s)) {
      Serial.printf("[KNX-UM][WARN] Invalid GA '%s' (%s) -> disabled\n", s, tag);
      s[0] = 0; // disable
    }
  };

  // Inbound GAs
  validateOrClear(gaInPower, "gaInPower");
  validateOrClear(gaInBri,   "gaInBri");
  validateOrClear(gaInR,     "gaInR");
  validateOrClear(gaInG,     "gaInG");
  validateOrClear(gaInB,     "gaInB");
  validateOrClear(gaInW,     "gaInW");
  validateOrClear(gaInCct,   "gaInCct");
  validateOrClear(gaInWW,    "gaInWW");
  validateOrClear(gaInCW,    "gaInCW");
  validateOrClear(gaInH,     "gaInH");
  validateOrClear(gaInS,     "gaInS");
  validateOrClear(gaInV,     "gaInV");
  validateOrClear(gaInFx,    "gaInFx");
  validateOrClear(gaInPreset,"gaInPreset");
  validateOrClear(gaInRGB,   "gaInRGB");
  validateOrClear(gaInHSV,   "gaInHSV");
  validateOrClear(gaInRGBW,  "gaInRGBW");
  validateOrClear(gaInTime,  "gaInTime");
  validateOrClear(gaInDate,  "gaInDate");
  validateOrClear(gaInDateTime, "gaInDateTime");
  validateOrClear(gaInBriRel, "gaInBriRel");
  validateOrClear(gaInRRel,   "gaInRRel");
  validateOrClear(gaInGRel,   "gaInGRel");
  validateOrClear(gaInBRel,   "gaInBRel");
  validateOrClear(gaInWRel,   "gaInWRel");
  validateOrClear(gaInWWRel,  "gaInWWRel");
  validateOrClear(gaInCWRel,  "gaInCWRel");
  validateOrClear(gaInHRel,   "gaInHRel");
  validateOrClear(gaInSRel,   "gaInSRel");
  validateOrClear(gaInVRel,   "gaInVRel");
  validateOrClear(gaInFxRel,  "gaInFxRel");
  validateOrClear(gaInRGBRel, "gaInRGBRel");
  validateOrClear(gaInHSVRel, "gaInHSVRel");
  validateOrClear(gaInRGBWRel,"gaInRGBWRel");

  // Outbound GAs
  if (!gOut.isNull()) {
    validateOrClear(gaOutPower, "gaOutPower");
    validateOrClear(gaOutBri,   "gaOutBri");
    validateOrClear(gaOutR,     "gaOutR");
    validateOrClear(gaOutG,     "gaOutG");
    validateOrClear(gaOutB,     "gaOutB");
    validateOrClear(gaOutW,     "gaOutW");
    validateOrClear(gaOutCct,   "gaOutCct");
    validateOrClear(gaOutWW,    "gaOutWW");
    validateOrClear(gaOutCW,    "gaOutCW");
    validateOrClear(gaOutH,     "gaOutH");
    validateOrClear(gaOutS,     "gaOutS");
    validateOrClear(gaOutV,     "gaOutV");
    validateOrClear(gaOutFx,    "gaOutFx");
    validateOrClear(gaOutPreset,"gaOutPreset");
    validateOrClear(gaOutRGB,   "gaOutRGB");
    validateOrClear(gaOutHSV,   "gaOutHSV");
    validateOrClear(gaOutRGBW,  "gaOutRGBW");
    validateOrClear(gaOutIntTemp,      "gaOutIntTemp");
    validateOrClear(gaOutTemp,         "gaOutTemp");
    validateOrClear(gaOutIntTempAlarm, "gaOutIntTempAlarm");
    validateOrClear(gaOutTempAlarm,    "gaOutTempAlarm");
  }

  if (!gOut.isNull()) {
    strlcpy(gaOutPower,  gOut["power"]  | gaOutPower,  sizeof(gaOutPower));
    strlcpy(gaOutBri,    gOut["bri"]    | gaOutBri,    sizeof(gaOutBri));
    strlcpy(gaOutR,      gOut["r"]      | gaOutR,      sizeof(gaOutR));
    strlcpy(gaOutG,      gOut["g"]      | gaOutG,      sizeof(gaOutG));
    strlcpy(gaOutB,      gOut["b"]      | gaOutB,      sizeof(gaOutB));
    strlcpy(gaOutW,      gOut["w"]      | gaOutW,      sizeof(gaOutW));
    strlcpy(gaOutCct,    gOut["cct"]    | gaOutCct,    sizeof(gaOutCct));
    strlcpy(gaOutWW,     gOut["ww"]     | gaOutWW,     sizeof(gaOutWW));
    strlcpy(gaOutCW,     gOut["cw"]     | gaOutCW,     sizeof(gaOutCW));
    strlcpy(gaOutH,      gOut["h"]      | gaOutH,      sizeof(gaOutH));
    strlcpy(gaOutS,      gOut["s"]      | gaOutS,      sizeof(gaOutS));
    strlcpy(gaOutV,      gOut["v"]      | gaOutV,      sizeof(gaOutV));
    strlcpy(gaOutFx,     gOut["fx"]     | gaOutFx,     sizeof(gaOutFx));
    strlcpy(gaOutPreset, gOut["preset"] | gaOutPreset, sizeof(gaOutPreset));
    strlcpy(gaOutRGB,    gOut["rgb"]    | gaOutRGB,    sizeof(gaOutRGB));
    strlcpy(gaOutHSV,    gOut["hsv"]    | gaOutHSV,    sizeof(gaOutHSV));
    strlcpy(gaOutRGBW,   gOut["rgbw"]   | gaOutRGBW,   sizeof(gaOutRGBW));
    strlcpy(gaOutIntTemp, gOut["Internal_Temperature"] | gaOutIntTemp, sizeof(gaOutIntTemp));
    strlcpy(gaOutTemp,    gOut["Temperature_Sensor"]    | gaOutTemp,    sizeof(gaOutTemp));
    strlcpy(gaOutIntTempAlarm, gOut["Internal_Temperature_Alarm"] | gaOutIntTempAlarm, sizeof(gaOutIntTempAlarm));
    strlcpy(gaOutTempAlarm,    gOut["Temperature_Sensor_Alarm"]   | gaOutTempAlarm,    sizeof(gaOutTempAlarm));
  }

   txRateLimitMs = top["tx_rate_limit_ms"] | txRateLimitMs;
   intTempAlarmMaxC    = top["Internal Temperature Alarm"]   | intTempAlarmMaxC;
   dallasTempAlarmMaxC = top["Temperature Sensor Alarm"]     | dallasTempAlarmMaxC;
   tempAlarmHystC      = top["Temperature Alarm Hysteresis"] | tempAlarmHystC;

  // --- decide if a rebuild is needed (compare new parsed GAs vs current cache) ---
  // keep snapshots of previous caches before we overwrite them
  const uint16_t PREV_IN_PWR = GA_IN_PWR, PREV_IN_BRI = GA_IN_BRI, PREV_IN_R = GA_IN_R,
                 PREV_IN_G = GA_IN_G, PREV_IN_B = GA_IN_B, PREV_IN_W = GA_IN_W,
                 PREV_IN_CCT = GA_IN_CCT, PREV_IN_WW = GA_IN_WW, PREV_IN_CW = GA_IN_CW,
                 PREV_IN_FX = GA_IN_FX, PREV_IN_PRE = GA_IN_PRE, PREV_IN_RGB = GA_IN_RGB,
                 PREV_IN_HSV = GA_IN_HSV, PREV_IN_RGBW = GA_IN_RGBW, PREV_IN_H = GA_IN_H,
                 PREV_IN_S = GA_IN_S, PREV_IN_V = GA_IN_V, PREV_IN_BRI_REL = GA_IN_BRI_REL, 
                 PREV_IN_R_REL = GA_IN_R_REL, PREV_IN_G_REL = GA_IN_G_REL, PREV_IN_B_REL = GA_IN_B_REL,
                 PREV_IN_W_REL = GA_IN_W_REL, PREV_IN_WW_REL = GA_IN_WW_REL, PREV_IN_CW_REL = GA_IN_CW_REL,
                 PREV_IN_H_REL = GA_IN_H_REL, PREV_IN_S_REL = GA_IN_S_REL, PREV_IN_V_REL = GA_IN_V_REL, 
                 PREV_IN_FX_REL = GA_IN_FX_REL, PREV_IN_RGB_REL = GA_IN_RGB_REL, PREV_IN_HSV_REL = GA_IN_HSV_REL,
                 PREV_IN_RGBW_REL = GA_IN_RGBW_REL;

  const uint16_t PREV_OUT_PWR = GA_OUT_PWR, PREV_OUT_BRI = GA_OUT_BRI, PREV_OUT_R = GA_OUT_R,
                 PREV_OUT_G = GA_OUT_G, PREV_OUT_B = GA_OUT_B, PREV_OUT_W = GA_OUT_W,
                 PREV_OUT_CCT = GA_OUT_CCT, PREV_OUT_WW = GA_OUT_WW, PREV_OUT_CW = GA_OUT_CW,
                 PREV_OUT_FX = GA_OUT_FX, PREV_OUT_PRE = GA_OUT_PRE, PREV_OUT_RGB = GA_OUT_RGB,
                 PREV_OUT_HSV = GA_OUT_HSV, PREV_OUT_RGBW = GA_OUT_RGBW, PREV_OUT_H = GA_OUT_H,
                 PREV_OUT_S = GA_OUT_S, PREV_OUT_V = GA_OUT_V;

  // parse the just-loaded GA strings into NEW values (locals)
  const uint16_t NEW_IN_PWR = parseGA(gaInPower);
  const uint16_t NEW_IN_BRI = parseGA(gaInBri);
  const uint16_t NEW_IN_R   = parseGA(gaInR);
  const uint16_t NEW_IN_G   = parseGA(gaInG);
  const uint16_t NEW_IN_B   = parseGA(gaInB);
  const uint16_t NEW_IN_W   = parseGA(gaInW);
  const uint16_t NEW_IN_CCT = parseGA(gaInCct);
  const uint16_t NEW_IN_WW  = parseGA(gaInWW);
  const uint16_t NEW_IN_CW  = parseGA(gaInCW);
  const uint16_t NEW_IN_H   = parseGA(gaInH);
  const uint16_t NEW_IN_S   = parseGA(gaInS);
  const uint16_t NEW_IN_V   = parseGA(gaInV);
  const uint16_t NEW_IN_FX  = parseGA(gaInFx);
  const uint16_t NEW_IN_PRE = parseGA(gaInPreset);
  const uint16_t NEW_IN_RGB = parseGA(gaInRGB);
  const uint16_t NEW_IN_HSV = parseGA(gaInHSV);
  const uint16_t NEW_IN_RGBW = parseGA(gaInRGBW);
  const uint16_t NEW_IN_RGB_REL = parseGA(gaInRGBRel);
  const uint16_t NEW_IN_HSV_REL = parseGA(gaInHSVRel);
  const uint16_t NEW_IN_RGBW_REL = parseGA(gaInRGBWRel);
  const uint16_t NEW_IN_BRI_REL = parseGA(gaInBriRel);
  const uint16_t NEW_IN_R_REL   = parseGA(gaInRRel);
  const uint16_t NEW_IN_G_REL   = parseGA(gaInGRel);
  const uint16_t NEW_IN_B_REL   = parseGA(gaInBRel);
  const uint16_t NEW_IN_W_REL   = parseGA(gaInWRel);
  const uint16_t NEW_IN_WW_REL  = parseGA(gaInWWRel);
  const uint16_t NEW_IN_CW_REL  = parseGA(gaInCWRel);
  const uint16_t NEW_IN_H_REL   = parseGA(gaInHRel);
  const uint16_t NEW_IN_S_REL   = parseGA(gaInSRel);
  const uint16_t NEW_IN_V_REL   = parseGA(gaInVRel);
  const uint16_t NEW_IN_FX_REL  = parseGA(gaInFxRel);

  const uint16_t NEW_OUT_PWR = parseGA(gaOutPower);
  const uint16_t NEW_OUT_BRI = parseGA(gaOutBri);
  const uint16_t NEW_OUT_R   = parseGA(gaOutR);
  const uint16_t NEW_OUT_G   = parseGA(gaOutG);
  const uint16_t NEW_OUT_B   = parseGA(gaOutB);
  const uint16_t NEW_OUT_W   = parseGA(gaOutW);
  const uint16_t NEW_OUT_CCT = parseGA(gaOutCct);
  const uint16_t NEW_OUT_WW  = parseGA(gaOutWW);
  const uint16_t NEW_OUT_CW  = parseGA(gaOutCW);
  const uint16_t NEW_OUT_H   = parseGA(gaOutH);
  const uint16_t NEW_OUT_S   = parseGA(gaOutS);
  const uint16_t NEW_OUT_V   = parseGA(gaOutV);
  const uint16_t NEW_OUT_FX  = parseGA(gaOutFx);
  const uint16_t NEW_OUT_PRE = parseGA(gaOutPreset);
  const uint16_t NEW_OUT_RGB = parseGA(gaOutRGB);
  const uint16_t NEW_OUT_HSV = parseGA(gaOutHSV);
  const uint16_t NEW_OUT_RGBW = parseGA(gaOutRGBW);

  // compute rebuild need (any GA mapping changed OR KNX just got enabled)
  bool anyGAChanged =
      (NEW_IN_PWR != PREV_IN_PWR) || (NEW_IN_BRI != PREV_IN_BRI) || (NEW_IN_R != PREV_IN_R) ||
      (NEW_IN_G  != PREV_IN_G ) || (NEW_IN_B   != PREV_IN_B ) || (NEW_IN_W  != PREV_IN_W ) ||
      (NEW_IN_CCT!= PREV_IN_CCT) || (NEW_IN_WW != PREV_IN_WW) || (NEW_IN_CW != PREV_IN_CW) ||
      (NEW_IN_FX != PREV_IN_FX) || (NEW_IN_PRE != PREV_IN_PRE) ||
      (NEW_OUT_PWR!=PREV_OUT_PWR)||(NEW_OUT_BRI!=PREV_OUT_BRI)||(NEW_OUT_R!=PREV_OUT_R) ||
      (NEW_OUT_G != PREV_OUT_G ) || (NEW_OUT_B  != PREV_OUT_B ) || (NEW_OUT_W != PREV_OUT_W) ||
      (NEW_OUT_CCT!=PREV_OUT_CCT)||(NEW_OUT_WW != PREV_OUT_WW)||(NEW_OUT_CW!=PREV_OUT_CW) ||
      (NEW_OUT_FX != PREV_OUT_FX) || (NEW_OUT_PRE!=PREV_OUT_PRE) ||
      (NEW_IN_RGB != PREV_IN_RGB) || (NEW_IN_HSV != PREV_IN_HSV) || (NEW_IN_RGBW != PREV_IN_RGBW) ||
      (NEW_OUT_RGB != PREV_OUT_RGB) || (NEW_OUT_HSV != PREV_OUT_HSV) || (NEW_OUT_RGBW != PREV_OUT_RGBW) ||
      (NEW_IN_H != PREV_IN_H) || (NEW_IN_S != PREV_IN_S) || (NEW_IN_V != PREV_IN_V) ||
      (NEW_OUT_H != PREV_OUT_H) || (NEW_OUT_S != PREV_OUT_S) || (NEW_OUT_V != PREV_OUT_V) ||
      (NEW_IN_BRI_REL != PREV_IN_BRI_REL) || (NEW_IN_R_REL != PREV_IN_R_REL) || (NEW_IN_G_REL != PREV_IN_G_REL) || (NEW_IN_B_REL != PREV_IN_B_REL) ||
      (NEW_IN_W_REL != PREV_IN_W_REL) || (NEW_IN_WW_REL != PREV_IN_WW_REL) || (NEW_IN_CW_REL != PREV_IN_CW_REL) ||
  (NEW_IN_H_REL != PREV_IN_H_REL) || (NEW_IN_S_REL != PREV_IN_S_REL) || (NEW_IN_V_REL != PREV_IN_V_REL) || (NEW_IN_FX_REL != PREV_IN_FX_REL) ||
  (NEW_IN_RGB_REL != PREV_IN_RGB_REL) || (NEW_IN_HSV_REL != PREV_IN_HSV_REL) || (NEW_IN_RGBW_REL != PREV_IN_RGBW_REL);

  bool prevEnabled = KNX.running();  // current runtime state is a good proxy here

  // now update the global caches with the NEW values
  GA_IN_PWR  = NEW_IN_PWR;  GA_IN_BRI = NEW_IN_BRI; GA_IN_R = NEW_IN_R; GA_IN_G = NEW_IN_G;
  GA_IN_B    = NEW_IN_B;    GA_IN_W   = NEW_IN_W;   GA_IN_CCT = NEW_IN_CCT; GA_IN_WW = NEW_IN_WW;
  GA_IN_CW   = NEW_IN_CW;   GA_IN_FX  = NEW_IN_FX;  GA_IN_PRE = NEW_IN_PRE; GA_IN_H = NEW_IN_H;
  GA_IN_S    = NEW_IN_S;    GA_IN_V   = NEW_IN_V;   GA_IN_RGB = NEW_IN_RGB; GA_IN_HSV = NEW_IN_HSV; GA_IN_RGBW = NEW_IN_RGBW; 
  GA_IN_BRI_REL = NEW_IN_BRI_REL; GA_IN_R_REL = NEW_IN_R_REL; GA_IN_G_REL = NEW_IN_G_REL; GA_IN_B_REL = NEW_IN_B_REL;
  GA_IN_W_REL = NEW_IN_W_REL; GA_IN_WW_REL = NEW_IN_WW_REL; GA_IN_CW_REL = NEW_IN_CW_REL;
  GA_IN_H_REL = NEW_IN_H_REL; GA_IN_S_REL = NEW_IN_S_REL; GA_IN_V_REL = NEW_IN_V_REL; GA_IN_FX_REL = NEW_IN_FX_REL;
  GA_IN_RGB_REL = NEW_IN_RGB_REL; GA_IN_HSV_REL = NEW_IN_HSV_REL; GA_IN_RGBW_REL = NEW_IN_RGBW_REL;

  GA_OUT_PWR = NEW_OUT_PWR; GA_OUT_BRI = NEW_OUT_BRI; GA_OUT_R = NEW_OUT_R; GA_OUT_G = NEW_OUT_G;
  GA_OUT_B   = NEW_OUT_B;   GA_OUT_W   = NEW_OUT_W;   GA_OUT_CCT = NEW_OUT_CCT; GA_OUT_WW = NEW_OUT_WW;
  GA_OUT_CW  = NEW_OUT_CW;  GA_OUT_FX  = NEW_OUT_FX;  GA_OUT_PRE = NEW_OUT_PRE; GA_OUT_H = NEW_OUT_H;
  GA_OUT_S   = NEW_OUT_S;   GA_OUT_V   = NEW_OUT_V;   GA_OUT_RGB = NEW_OUT_RGB; GA_OUT_HSV = NEW_OUT_HSV; GA_OUT_RGBW = NEW_OUT_RGBW;

  // Re-apply enhancement to running core (safe to do regardless)
  KNX.setCommunicationEnhancement(commEnhance, commResends, commResendGapMs, commRxDedupMs);  // :contentReference[oaicite:0]{index=0}

  // ---- ENABLED/OFF handling ----
  if (!enabled) {
    // GUI disabled → leave multicast + free socket if running
    if (KNX.running()) {
      Serial.println("[KNX-UM] KNX disabled via GUI → shutting down.");
      KNX.end();                                                   // leaves group and closes socket :contentReference[oaicite:1]{index=1}
    }
    return true;
  }

  // If PA string is valid, update it without forcing a rebuild
  if (uint16_t pa = parsePA(individualAddr)) {                     // :contentReference[oaicite:2]{index=2}
    KNX.setIndividualAddress(pa);
    Serial.printf("[KNX-UM] PA set to %u.%u.%u (0x%04X)\n",
                  (unsigned)((pa>>12)&0x0F), (unsigned)((pa>>8)&0x0F), (unsigned)(pa&0xFF), pa);
  } else {
    Serial.printf("[KNX-UM][WARN] Invalid individual address '%s' (unchanged)\n", individualAddr);
  }

  // ---- rebuild vs. tweak ----
  const bool rebuildNeeded = anyGAChanged || !prevEnabled;

  if (rebuildNeeded) {
    Serial.println("[KNX-UM] Rebuild KNX registrations & socket (GA map changed or first enable).");
    KNX.end();
    KNX.clearRegistrations();                                      // drop old GA registry
    setup();                                                       // re-register + begin() if Wi-Fi up :contentReference[oaicite:3]{index=3}
    KNX.setCommunicationEnhancement(commEnhance, commResends, commResendGapMs, commRxDedupMs);

    if (KNX.running()) {
      // Optional: send a primer read so routers learn our presence
      uint16_t primer = knxMakeGroupAddress(0,0,1);
      KNX.groupValueRead(primer);
    }
    // Push state so KNX sees the new mapping
    scheduleStatePublish(true, true, true);
  } else {
    // Lightweight path: keep socket, just refresh IGMP and tweak runtime
    if (KNX.running()) {
      if (!KNX.rejoinMulticast()) {                                // refresh IP_ADD_MEMBERSHIP + MULTICAST_IF :contentReference[oaicite:4]{index=4}
        KNX.end();
        KNX.begin();
      }
      // Optional primer
      uint16_t primer = knxMakeGroupAddress(0,0,1);
      KNX.groupValueRead(primer);
    } else {
      // Enabled but not running yet (e.g., Wi-Fi not ready) → try to start
      KNX.begin();                                                 // joins multicast, sets TTL/LOOP/IF :contentReference[oaicite:5]{index=5}
    }
  }
  return true;
}

void KnxIpUsermod::appendConfigData(Print& uiScript)
{
  // Section shortcuts like in AudioReactive
  uiScript.print(F("ux='KNX_IP';"));
  uiScript.print(F("uxIn = ux+':GA in';"));
  uiScript.print(F("uxOut= ux+':GA out';"));

  // Compact layout (shorter inputs + aligned rows) – same idea you liked
  uiScript.print(F(
    "(()=>{const css=`"
      "#knxip-card .knx-grid{display:grid;grid-template-columns:180px 1fr;gap:6px 12px;align-items:center}"
      "#knxip-card .knx-row{display:contents}"
      "#knxip-card input[type=text],#knxip-card input[type=number]{max-width:200px}"
      "#knxip-card .unit{margin-left:6px;opacity:.7;font-weight:400}"
    "`;let st=document.createElement('style');st.textContent=css;document.head.appendChild(st);})();"
  ));

  // Add unit labels to the right of inputs (index 1 = the actual field), mirroring addInfo() usage in AudioReactive
  // ---- GA in ----
  uiScript.print(F("addInfo(uxIn+':power',1,' [-] (DPT 1.001)');"));
  uiScript.print(F("addInfo(uxIn+':bri',1,' [0..100] (DPT 5.001)');"));
  uiScript.print(F("addInfo(uxIn+':r',1,' [0..255] (DPT 5.010)');"));
  uiScript.print(F("addInfo(uxIn+':g',1,' [0..255] (DPT 5.010)');"));
  uiScript.print(F("addInfo(uxIn+':b',1,' [0..255] (DPT 5.010)');"));
  uiScript.print(F("addInfo(uxIn+':w',1,' [0..255] (DPT 5.010)');"));
  uiScript.print(F("addInfo(uxIn+':cct',1,' [Kelvin] (DPT 7.600)');"));
  uiScript.print(F("addInfo(uxIn+':ww',1,' [0..255] (DPT 5.010)');"));
  uiScript.print(F("addInfo(uxIn+':cw',1,' [0..255] (DPT 5.010)');"));
  uiScript.print(F("addInfo(uxIn+':h',1,' [0..255] (DPT 5.003)');"));
  uiScript.print(F("addInfo(uxIn+':s',1,' [0..100] (DPT 5.001)');")); // Saturation scaling (%)
  uiScript.print(F("addInfo(uxIn+':v',1,' [0..100] (DPT 5.001)');")); // Value scaling (%)
  uiScript.print(F("addInfo(uxIn+':fx',1,' [0..255] (DPT 5.010)');"));
  uiScript.print(F("addInfo(uxIn+':preset',1,' [0..255] (DPT 5.010)');"));
  uiScript.print(F("addInfo(uxIn+':rgb',1,' [0..255] (DPST-232-600)');"));
  uiScript.print(F("addInfo(uxIn+':rgbw',1,' [0..255] (DPST-251-600)');"));
  uiScript.print(F("addInfo(uxIn+':hsv',1,' [0..255] (DPST-232-600)');"));
  uiScript.print(F("addInfo(uxIn+':time',1,' [TimeOfDay,3Bytes](DPT 10.001)');"));
  uiScript.print(F("addInfo(uxIn+':date',1,' [Date,3Bytes] (DPT 11.001)');"));
  uiScript.print(F("addInfo(uxIn+':datetime',1,' [DateTime,8Bytes] (DPT 19.001)');"));
  uiScript.print(F("addInfo(uxIn+':bri_rel',1,' [step dir] (DPT 3.007)');"));
  uiScript.print(F("addInfo(uxIn+':r_rel',1,' [step dir] (DPT 3.007)');"));
  uiScript.print(F("addInfo(uxIn+':g_rel',1,' [step dir] (DPT 3.007)');"));
  uiScript.print(F("addInfo(uxIn+':b_rel',1,' [step dir] (DPT 3.007)');"));
  uiScript.print(F("addInfo(uxIn+':w_rel',1,' [step dir] (DPT 3.007)');"));
  uiScript.print(F("addInfo(uxIn+':ww_rel',1,' [step dir] (DPT 3.007)');"));
  uiScript.print(F("addInfo(uxIn+':cw_rel',1,' [step dir] (DPT 3.007)');"));
  uiScript.print(F("addInfo(uxIn+':h_rel',1,' [step hue] (DPT 3.007)');"));
  uiScript.print(F("addInfo(uxIn+':s_rel',1,' [step sat] (DPT 3.007)');"));
  uiScript.print(F("addInfo(uxIn+':v_rel',1,' [step val] (DPT 3.007)');"));
  uiScript.print(F("addInfo(uxIn+':fx_rel',1,' [step fx] (DPT 3.007)');"));
  uiScript.print(F("addInfo(uxIn+':rgb_rel',1,' [R,G,B,3Bytes] (DPT 3.007)');"));
  uiScript.print(F("addInfo(uxIn+':hsv_rel',1,' [H,S,V,3Bytes] (DPT 3.007)');"));
  uiScript.print(F("addInfo(uxIn+':rgbw_rel',1,' [R,G,B,W,4Bytes] (DPT 3.007)');"));

  // ---- GA out ----
  uiScript.print(F("addInfo(uxOut+':power',1,' [-]  (DPT 1.001)');"));
  uiScript.print(F("addInfo(uxOut+':bri',1,' [0..100] (DPT 5.001)');"));
  uiScript.print(F("addInfo(uxOut+':r',1,' [0..255] (DPT 5.010)');"));
  uiScript.print(F("addInfo(uxOut+':g',1,' [0..255] (DPT 5.010)');"));
  uiScript.print(F("addInfo(uxOut+':b',1,' [0..255] (DPT 5.010)');"));
  uiScript.print(F("addInfo(uxOut+':w',1,' [0..255] (DPT 5.010)');"));
  uiScript.print(F("addInfo(uxOut+':cct',1,' [Kelvin] (DPT 7.600)');"));
  uiScript.print(F("addInfo(uxOut+':ww',1,' [0..255] (DPT 5.010)');"));
  uiScript.print(F("addInfo(uxOut+':cw',1,' [0..255] (DPT 5.010)');"));
  uiScript.print(F("addInfo(uxOut+':h',1,' [0..255] (DPT 5.003)');"));
  uiScript.print(F("addInfo(uxOut+':s',1,' [0..100] (DPT 5.001)');"));
  uiScript.print(F("addInfo(uxOut+':v',1,' [0..100] (DPT 5.001)');"));
  uiScript.print(F("addInfo(uxOut+':fx',1,' [0..255] (DPT 5.010)');"));
  uiScript.print(F("addInfo(uxOut+':preset',1,' [0..255] (DPT 5.010)');"));
  uiScript.print(F("addInfo(uxOut+':rgb',1,' [0..255] (DPST-232-600)');"));
  uiScript.print(F("addInfo(uxOut+':rgbw',1,' [0..255] (DPST-251-600)');"));
  uiScript.print(F("addInfo(uxOut+':hsv',1,' [0..255] (DPST-232-600)');"));
  uiScript.print(F("addInfo(uxOut+':Internal_Temperature',1,' [°C] (DPST-14-68)');"));
  uiScript.print(F("addInfo(uxOut+':Temperature_Sensor',1,' [°C] (DPST-14-68)');"));
  uiScript.print(F("addInfo(uxOut+':Temperature_Sensor_Alarm',1,' [°C] (DPST-1-5)');"));
  uiScript.print(F("addInfo(uxOut+':Internal_Temperature_Alarm',1,' [°C] (DPST-1-5)');"));
  

  uiScript.print(F("addInfo(ux+':tx_rate_limit_ms',1,' [ms]');"));

  // ---- CCT range at top-level ----
  uiScript.print(F("addInfo(ux+':cct_kelvin_min',1,' [K]');"));
  uiScript.print(F("addInfo(ux+':cct_kelvin_max',1,' [K]');"));
  // Periodic interval units
  uiScript.print(F("addInfo(ux+':periodic_enabled',1,' [-]');"));
  uiScript.print(F("addInfo(ux+':periodic_interval_ms',1,' [ms]');"));

  uiScript.print(F("addInfo(ux+':communication_enhancement',1,' [-]');"));
  uiScript.print(F("addInfo(ux+':communication_resends',1,' [-]');"));
  uiScript.print(F("addInfo(ux+':communication_resend_gap',1,' [ms]');"));
  uiScript.print(F("addInfo(ux+':communication_rx_dedup',1,' [ms]');"));

  uiScript.print(F("addInfo(ux+':Internal Temperature Alarm',1,' [°C]');"));
  uiScript.print(F("addInfo(ux+':Temperature Sensor Alarm',1,' [°C]');"));
  uiScript.print(F("addInfo(ux+':Temperature Alarm Hysteresis',1,' [°C]');"));


  // Tag the KNX card so CSS only scopes there
  uiScript.print(F(
    "(()=>{const card=[...document.querySelectorAll('.um')]"
    ".find(c=>{const h=c.querySelector('h3');return h&&h.textContent.trim()==='KNX_IP';});"
    "if(card) card.id='knxip-card';})();"
  ));
}