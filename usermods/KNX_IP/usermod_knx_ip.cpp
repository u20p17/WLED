#include "usermod_knx_ip.h"

// ---- Cached parsed GAs (uint16_t) ----
namespace {
  uint16_t GA_IN_PWR  = 0;
  uint16_t GA_IN_BRI  = 0;
  uint16_t GA_IN_R    = 0;
  uint16_t GA_IN_G    = 0;
  uint16_t GA_IN_B    = 0;
  uint16_t GA_IN_FX   = 0;
  uint16_t GA_IN_PRE  = 0;

  uint16_t GA_OUT_PWR = 0;
  uint16_t GA_OUT_BRI = 0;
  uint16_t GA_OUT_FX  = 0;
  uint16_t GA_OUT_R   = 0;
  uint16_t GA_OUT_G   = 0;
  uint16_t GA_OUT_B   = 0;
  uint16_t GA_OUT_PRE = 0;
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

// Parse "x/y/z" -> GA (returns 0 if invalid)
static uint16_t parseGA(const char* s) {
  if (!s || !*s) return 0;
  int a=0,b=0,c=0; const char* p = s;
  auto parseUInt = [&](int& out)->bool {
    int v=0; bool any=false;
    while (*p >= '0' && *p <= '9') { v = v*10 + (*p-'0'); p++; any=true; }
    out=v; return any;
  };
  if (!parseUInt(a) || *p!='/') return 0; ++p;
  if (!parseUInt(b) || *p!='/') return 0; ++p;
  if (!parseUInt(c) || *p!='\0') return 0;
  if (a<0||a>31 || b<0||b>7 || c<0||c>255) return 0;
  return knxMakeGroupAddress((uint8_t)a,(uint8_t)b,(uint8_t)c);
}

// Parse individual address "x.x.x" -> 16-bit PA: area(4) | line(4) | dev(8)
static uint16_t parsePA(const char* s) {
  if (!s || !*s) return 0;
  int area=0,line=0,dev=0; const char* p=s;
  auto parseUInt = [&](int& out)->bool {
    int v=0; bool any=false;
    while (*p>='0' && *p<='9') { v=v*10+(*p-'0'); p++; any=true; }
    out=v; return any;
  };
  if (!parseUInt(area) || *p!='.') return 0; ++p;
  if (!parseUInt(line) || *p!='.') return 0; ++p;
  if (!parseUInt(dev)  || *p!='\0') return 0;
  if (area<0||area>15) area=1;
  if (line<0||line>15) line=1;
  if (dev<0 ||dev>255) dev =100;
  return (uint16_t)((area & 0x0F) << 12) | (uint16_t)((line & 0x0F) << 8) | (uint16_t)(dev & 0xFF);
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

// -------------------- Usermod API --------------------
void KnxIpUsermod::setup() {
  if (!enabled) return;

  // Parse & set PA
  if (uint16_t pa = parsePA(individualAddr)) {
    KNX.setIndividualAddress(pa);
    Serial.printf("[KNX-UM] PA set to %u.%u.%u (0x%04X)\n",
                  (unsigned)((pa>>12)&0x0F), (unsigned)((pa>>8)&0x0F), (unsigned)(pa&0xFF), pa);
  } else {
    Serial.println("[KNX-UM] WARNING: Invalid individual address string; PA not set.");
   
  }

  // Apply Communication Enhancement to the core
  KNX.setCommunicationEnhancement(commEnhance, commResends, commResendGapMs, commRxDedupMs);
  Serial.printf("[KNX-UM] CommEnhance %s (resends=%u gapMs=%u dedupMs=%u)\n",
                commEnhance ? "ON" : "OFF", commResends, commResendGapMs, commRxDedupMs);


  // Parse GA strings once
  GA_IN_PWR  = parseGA(gaInPower);
  GA_IN_BRI  = parseGA(gaInBri);
  GA_IN_R    = parseGA(gaInR);
  GA_IN_G    = parseGA(gaInG);
  GA_IN_B    = parseGA(gaInB);
  GA_IN_FX   = parseGA(gaInFx);
  GA_IN_PRE  = parseGA(gaInPreset);
  Serial.printf("[KNX-UM] IN  pwr=0x%04X bri=0x%04X R=0x%04X G=0x%04X B=0x%04X fx=0x%04X pre=0x%04X\n",
                GA_IN_PWR, GA_IN_BRI, GA_IN_R, GA_IN_G, GA_IN_B, GA_IN_FX, GA_IN_PRE);
 

  GA_IN_W   = parseGA(gaInW);
  if (GA_IN_W) {
    KNX.addGroupObject(GA_IN_W, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_W, [this](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write && len >= 1) onKnxWhite(p[0]); // raw 0..255
    });
  }

  GA_IN_CCT = parseGA(gaInCct);
  if (GA_IN_CCT) {
    KNX.addGroupObject(GA_IN_CCT, DptMain::DPT_7xx, false, true);
    KNX.onGroup(GA_IN_CCT, [this](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write && len >= 2) {
        uint16_t k = ((uint16_t)p[0] << 8) | (uint16_t)p[1]; // big-endian
        onKnxCct(k);
      }
    });
  }

  // Optional direct WW/CW level inputs
  GA_IN_WW = parseGA(gaInWW);
  if (GA_IN_WW) {
    KNX.addGroupObject(GA_IN_WW, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_WW, [this](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write && len >= 1) onKnxWW(p[0]); // raw 0..255
    });
  }

  GA_IN_CW = parseGA(gaInCW);
  if (GA_IN_CW) {
    KNX.addGroupObject(GA_IN_CW, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_CW, [this](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write && len >= 1) onKnxCW(p[0]); // raw 0..255
    });
  }

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
  Serial.printf("[KNX-UM] OUT pwr=0x%04X bri=0x%04X R=0x%04X G=0x%04X B=0x%04X W=0x%04X CCT=0x%04X WW=0x%04X CW=0x%04X fx=0x%04X pre=0x%04X\n",
                GA_OUT_PWR, GA_OUT_BRI, GA_OUT_R, GA_OUT_G, GA_OUT_B, GA_OUT_W, GA_OUT_CCT, GA_OUT_WW, GA_OUT_CW, GA_OUT_FX, GA_OUT_PRE);
 

  // Register inbound objects and callbacks
  if (GA_IN_PWR) {
    KNX.addGroupObject(GA_IN_PWR, DptMain::DPT_1xx, /*tx=*/false, /*rx=*/true);
    KNX.onGroup(GA_IN_PWR, [this](uint16_t /*ga*/, DptMain /*dpt*/, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write) onKnxPower(KnxIpCore::unpack1Bit(p, len));
      else if (svc == KnxService::GroupValue_Read) {
        uint8_t resp = KnxIpCore::pack1Bit(bri > 0);
        KNX.groupValueResponse(GA_IN_PWR, &resp, 1);
      }
    });
  }

  if (GA_IN_BRI) {
    KNX.addGroupObject(GA_IN_BRI, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_BRI, [this](uint16_t /*ga*/, DptMain /*dpt*/, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write) onKnxBrightness(KnxIpCore::unpackScaling(p, len));
      else if (svc == KnxService::GroupValue_Read) {
        uint8_t resp = KnxIpCore::packScaling(to_pct_0_100(bri));
        KNX.groupValueResponse(GA_IN_BRI, &resp, 1);
      }
    });
  }

  // R/G/B channels (DPT 5.010 raw 0..255)
  // R
  if (GA_IN_R) {
    KNX.addGroupObject(GA_IN_R, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_R, [this](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write && len >= 1) {
        uint8_t r = p[0];
        uint8_t cr,cg,cb,cw; getCurrentRGBW(cr,cg,cb,cw);
        onKnxRGB(r, cg, cb);
      }
    });
  }
  // G
  if (GA_IN_G) {
    KNX.addGroupObject(GA_IN_G, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_G, [this](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write && len >= 1) {
        uint8_t g = p[0];
        uint8_t cr,cg,cb,cw; getCurrentRGBW(cr,cg,cb,cw);
        onKnxRGB(cr, g, cb);
      }
    });
  }
  // B
  if (GA_IN_B) {
    KNX.addGroupObject(GA_IN_B, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_B, [this](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write && len >= 1) {
        uint8_t b = p[0];
        uint8_t cr,cg,cb,cw; getCurrentRGBW(cr,cg,cb,cw);
        onKnxRGB(cr, cg, b);
      }
    });
  }

  // Effect index (0..255) and Preset (0..250 typical)
  if (GA_IN_FX) {
    KNX.addGroupObject(GA_IN_FX, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_FX, [this](uint16_t /*ga*/, DptMain /*dpt*/, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write) onKnxEffect(p && len ? p[0] : 0);
    });
  }
  if (GA_IN_PRE) {
    KNX.addGroupObject(GA_IN_PRE, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_PRE, [this](uint16_t /*ga*/, DptMain /*dpt*/, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write) onKnxPreset(p && len ? p[0] : 0);
    });
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
  Serial.printf("[KNX-UM] KNX.begin() -> %s (localIP=%s)\n",
                ok ? "OK" : "FAILED",
                ip.toString().c_str());
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

// Preset index (if configured) – send only if it actually changed
  if (GA_OUT_PRE) {
    if (s_lastPresetSent != _lastPreset) {
      uint8_t p = _lastPreset; // last applied via onKnxPreset()
      KNX.groupValueWrite(GA_OUT_PRE, &p, 1);
      s_lastPresetSent = _lastPreset;
    }
  }

  // Update “last published” snapshot for color/CCT
  LAST_R = r; LAST_G = g; LAST_B = b; LAST_W = w; LAST_CCT = cct;

  // Clear pending flags after publish
  _pendingTxPower = _pendingTxBri = _pendingTxFx = false;
}

void KnxIpUsermod::loop() {
  if (!enabled) return;

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
  top["individual_addr"] = individualAddr;
  top["tx_rate_limit_ms"] = txRateLimitMs;
  top["periodic_enabled"] = periodicEnabled;
  top["periodic_interval_ms"] = periodicIntervalMs;
  top["cct_kelvin_min"] = kelvinMin;
  top["cct_kelvin_max"] = kelvinMax;
  top["comm_enhance"]      = commEnhance;
  top["comm_resends"]      = commResends;
  top["comm_resend_gap_ms"]= commResendGapMs;
  top["comm_rx_dedup_ms"]  = commRxDedupMs;


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
  gIn["fx"]     = gaInFx;
  gIn["preset"] = gaInPreset;

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
  gOut["fx"]     = gaOutFx;
  gOut["preset"] = gaOutPreset;
}

bool KnxIpUsermod::readFromConfig(JsonObject& root) {
  JsonObject top = root["KNX_IP"];
  if (top.isNull()) {
    top = root["KNX-IP"];       // legacy
    if (top.isNull()) return false;
  }

  enabled = top["enabled"] | enabled;
  strlcpy(individualAddr, top["individual_addr"] | individualAddr, sizeof(individualAddr));
  kelvinMin = top["cct_kelvin_min"] | kelvinMin;
  kelvinMax = top["cct_kelvin_max"] | kelvinMax;
  periodicEnabled    = top["periodic_enabled"]     | periodicEnabled;
  periodicIntervalMs = top["periodic_interval_ms"] | periodicIntervalMs;
  commEnhance       = top["comm_enhance"]        | commEnhance;
  commResends       = top["comm_resends"]        | commResends;
  commResendGapMs   = top["comm_resend_gap_ms"]  | commResendGapMs;
  commRxDedupMs     = top["comm_rx_dedup_ms"]    | commRxDedupMs;


  // 🔧 accept either "GA in"/"GA out" (what we save) or "in"/"out"
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
    strlcpy(gaInFx,     gIn["fx"]     | gaInFx,     sizeof(gaInFx));
    strlcpy(gaInPreset, gIn["preset"] | gaInPreset, sizeof(gaInPreset));
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
    strlcpy(gaOutFx,     gOut["fx"]     | gaOutFx,     sizeof(gaOutFx));
    strlcpy(gaOutPreset, gOut["preset"] | gaOutPreset, sizeof(gaOutPreset));
  }

   txRateLimitMs = top["tx_rate_limit_ms"] | txRateLimitMs;

  // --- decide if a rebuild is needed (compare new parsed GAs vs current cache) ---
  // keep snapshots of previous caches before we overwrite them
  const uint16_t PREV_IN_PWR = GA_IN_PWR, PREV_IN_BRI = GA_IN_BRI, PREV_IN_R = GA_IN_R,
                 PREV_IN_G = GA_IN_G, PREV_IN_B = GA_IN_B, PREV_IN_W = GA_IN_W,
                 PREV_IN_CCT = GA_IN_CCT, PREV_IN_WW = GA_IN_WW, PREV_IN_CW = GA_IN_CW,
                 PREV_IN_FX = GA_IN_FX, PREV_IN_PRE = GA_IN_PRE;

  const uint16_t PREV_OUT_PWR = GA_OUT_PWR, PREV_OUT_BRI = GA_OUT_BRI, PREV_OUT_R = GA_OUT_R,
                 PREV_OUT_G = GA_OUT_G, PREV_OUT_B = GA_OUT_B, PREV_OUT_W = GA_OUT_W,
                 PREV_OUT_CCT = GA_OUT_CCT, PREV_OUT_WW = GA_OUT_WW, PREV_OUT_CW = GA_OUT_CW,
                 PREV_OUT_FX = GA_OUT_FX, PREV_OUT_PRE = GA_OUT_PRE;

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
  const uint16_t NEW_IN_FX  = parseGA(gaInFx);
  const uint16_t NEW_IN_PRE = parseGA(gaInPreset);

  const uint16_t NEW_OUT_PWR = parseGA(gaOutPower);
  const uint16_t NEW_OUT_BRI = parseGA(gaOutBri);
  const uint16_t NEW_OUT_R   = parseGA(gaOutR);
  const uint16_t NEW_OUT_G   = parseGA(gaOutG);
  const uint16_t NEW_OUT_B   = parseGA(gaOutB);
  const uint16_t NEW_OUT_W   = parseGA(gaOutW);
  const uint16_t NEW_OUT_CCT = parseGA(gaOutCct);
  const uint16_t NEW_OUT_WW  = parseGA(gaOutWW);
  const uint16_t NEW_OUT_CW  = parseGA(gaOutCW);
  const uint16_t NEW_OUT_FX  = parseGA(gaOutFx);
  const uint16_t NEW_OUT_PRE = parseGA(gaOutPreset);

  // compute rebuild need (any GA mapping changed OR KNX just got enabled)
  bool anyGAChanged =
      (NEW_IN_PWR != PREV_IN_PWR) || (NEW_IN_BRI != PREV_IN_BRI) || (NEW_IN_R != PREV_IN_R) ||
      (NEW_IN_G  != PREV_IN_G ) || (NEW_IN_B   != PREV_IN_B ) || (NEW_IN_W  != PREV_IN_W ) ||
      (NEW_IN_CCT!= PREV_IN_CCT) || (NEW_IN_WW != PREV_IN_WW) || (NEW_IN_CW != PREV_IN_CW) ||
      (NEW_IN_FX != PREV_IN_FX) || (NEW_IN_PRE != PREV_IN_PRE) ||
      (NEW_OUT_PWR!=PREV_OUT_PWR)||(NEW_OUT_BRI!=PREV_OUT_BRI)||(NEW_OUT_R!=PREV_OUT_R) ||
      (NEW_OUT_G != PREV_OUT_G ) || (NEW_OUT_B  != PREV_OUT_B ) || (NEW_OUT_W != PREV_OUT_W) ||
      (NEW_OUT_CCT!=PREV_OUT_CCT)||(NEW_OUT_WW != PREV_OUT_WW)||(NEW_OUT_CW!=PREV_OUT_CW) ||
      (NEW_OUT_FX != PREV_OUT_FX) || (NEW_OUT_PRE!=PREV_OUT_PRE);

  bool prevEnabled = KNX.running();  // current runtime state is a good proxy here

  // now update the global caches with the NEW values
  GA_IN_PWR  = NEW_IN_PWR;  GA_IN_BRI = NEW_IN_BRI; GA_IN_R = NEW_IN_R; GA_IN_G = NEW_IN_G;
  GA_IN_B    = NEW_IN_B;    GA_IN_W   = NEW_IN_W;   GA_IN_CCT = NEW_IN_CCT; GA_IN_WW = NEW_IN_WW;
  GA_IN_CW   = NEW_IN_CW;   GA_IN_FX  = NEW_IN_FX;  GA_IN_PRE = NEW_IN_PRE;

  GA_OUT_PWR = NEW_OUT_PWR; GA_OUT_BRI = NEW_OUT_BRI; GA_OUT_R = NEW_OUT_R; GA_OUT_G = NEW_OUT_G;
  GA_OUT_B   = NEW_OUT_B;   GA_OUT_W   = NEW_OUT_W;   GA_OUT_CCT = NEW_OUT_CCT; GA_OUT_WW = NEW_OUT_WW;
  GA_OUT_CW  = NEW_OUT_CW;  GA_OUT_FX  = NEW_OUT_FX;  GA_OUT_PRE = NEW_OUT_PRE;

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
    Serial.println("[KNX-UM] WARNING: Invalid individual address string; PA not changed.");
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
  uiScript.print(F("addInfo(uxIn+':power',1,' [-]');"));
  uiScript.print(F("addInfo(uxIn+':bri',1,' [0..100]');"));
  uiScript.print(F("addInfo(uxIn+':r',1,' [0..255]');"));
  uiScript.print(F("addInfo(uxIn+':g',1,' [0..255]');"));
  uiScript.print(F("addInfo(uxIn+':b',1,' [0..255]');"));
  uiScript.print(F("addInfo(uxIn+':w',1,' [0..255]');"));
  uiScript.print(F("addInfo(uxIn+':cct',1,' [Kelvin]');"));
  uiScript.print(F("addInfo(uxIn+':ww',1,' [0..255]');"));
  uiScript.print(F("addInfo(uxIn+':cw',1,' [0..255]');"));
  uiScript.print(F("addInfo(uxIn+':fx',1,' [0..255]');"));
  uiScript.print(F("addInfo(uxIn+':preset',1,' [0..255]');"));

  // ---- GA out ----
  uiScript.print(F("addInfo(uxOut+':power',1,' [-]');"));
  uiScript.print(F("addInfo(uxOut+':bri',1,' [0..100]');"));
  uiScript.print(F("addInfo(uxOut+':r',1,' [0..255]');"));
  uiScript.print(F("addInfo(uxOut+':g',1,' [0..255]');"));
  uiScript.print(F("addInfo(uxOut+':b',1,' [0..255]');"));
  uiScript.print(F("addInfo(uxOut+':w',1,' [0..255]');"));
  uiScript.print(F("addInfo(uxOut+':cct',1,' [Kelvin]');"));
  uiScript.print(F("addInfo(uxOut+':ww',1,' [0..255]');"));
  uiScript.print(F("addInfo(uxOut+':cw',1,' [0..255]');"));
  uiScript.print(F("addInfo(uxOut+':fx',1,' [0..255]');"));
  uiScript.print(F("addInfo(uxOut+':preset',1,' [0..255]');"));

  uiScript.print(F("addInfo(ux+':tx_rate_limit_ms',1,' [ms]');"));

  // ---- CCT range at top-level ----
  uiScript.print(F("addInfo(ux+':cct_kelvin_min',1,' [K]');"));
  uiScript.print(F("addInfo(ux+':cct_kelvin_max',1,' [K]');"));
  // Periodic interval units
  uiScript.print(F("addInfo(ux+':periodic_enabled',1,' [-]');"));
  uiScript.print(F("addInfo(ux+':periodic_interval_ms',1,' [ms]');"));

  // Tag the KNX card so CSS only scopes there
  uiScript.print(F(
    "(()=>{const card=[...document.querySelectorAll('.um')]"
    ".find(c=>{const h=c.querySelector('h3');return h&&h.textContent.trim()==='KNX_IP';});"
    "if(card) card.id='knxip-card';})();"
  ));
}
