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

  // Track last known RGB so per-channel writes can preserve other components
  uint8_t LAST_R = 255, LAST_G = 255, LAST_B = 255;
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
  LAST_R = r; LAST_G = g; LAST_B = b;
  strip.setColor(0, r, g, b); // slot 0
  colorUpdated(CALL_MODE_DIRECT_CHANGE);
  // RGB "out" is optional; see note below.
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

// ------------- Coalesced outbound state -------------
void KnxIpUsermod::scheduleStatePublish(bool pwr, bool bri_, bool fx) {
  _pendingTxPower |= pwr;
  _pendingTxBri   |= bri_;
  _pendingTxFx    |= fx;
  unsigned long now = millis();
  if (_nextTxAt == 0 || now > _nextTxAt) _nextTxAt = now + txRateLimitMs;
}

void KnxIpUsermod::publishState() {
  if (!_pendingTxPower && !_pendingTxBri && !_pendingTxFx
      && !(gaOutR[0] || gaOutG[0] || gaOutB[0]) && !gaOutPreset[0]) return;

  const bool    pwr     = (bri > 0);
  const uint8_t pct     = (uint8_t)((bri * 100u + 127u) / 255u);
  const uint8_t fxIndex = effectCurrent;

  // Existing OUTs
  if (_pendingTxPower && GA_OUT_PWR) KNX.write1Bit(GA_OUT_PWR, pwr);
  if (_pendingTxBri   && GA_OUT_BRI) KNX.writeScaling(GA_OUT_BRI, pct);
  if (_pendingTxFx    && GA_OUT_FX) {
    uint8_t v = fxIndex;
    KNX.groupValueWrite(GA_OUT_FX, &v, 1);
  }

  // NEW: publish RGB (as 0..100% via DPT 5.001)
  if (gaOutR[0] || gaOutG[0] || gaOutB[0]) {
    // Read current segment 0 color (primary color slot 0)
    const Segment& seg = strip.getSegment(0);
    uint32_t c = seg.colors[0];
    uint8_t r = (c >> 16) & 0xFF;
    uint8_t g = (c >>  8) & 0xFF;
    uint8_t b =  c        & 0xFF;

    if (GA_OUT_R) {
      uint8_t rpct = (uint8_t)((r * 100u + 127u) / 255u);
      KNX.writeScaling(GA_OUT_R, rpct);
    }
    if (GA_OUT_G) {
      uint8_t gpct = (uint8_t)((g * 100u + 127u) / 255u);
      KNX.writeScaling(GA_OUT_G, gpct);
    }
    if (GA_OUT_B) {
      uint8_t bpct = (uint8_t)((b * 100u + 127u) / 255u);
      KNX.writeScaling(GA_OUT_B, bpct);
    }
  }

  // NEW: publish last-applied preset (raw 0..255)
  if (GA_OUT_PRE) {
    uint8_t p = _lastPreset;
    KNX.groupValueWrite(GA_OUT_PRE, &p, 1);
  }

  _pendingTxPower = _pendingTxBri = _pendingTxFx = false;
}

// -------------------- Usermod API --------------------
void KnxIpUsermod::setup() {
  if (!enabled) return;

  // Parse & set PA
  if (uint16_t pa = parsePA(individualAddr)) {
    KNX.setIndividualAddress(pa);
  }

  // Parse GA strings once
  GA_IN_PWR  = parseGA(gaInPower);
  GA_IN_BRI  = parseGA(gaInBri);
  GA_IN_R    = parseGA(gaInR);
  GA_IN_G    = parseGA(gaInG);
  GA_IN_B    = parseGA(gaInB);
  GA_IN_FX   = parseGA(gaInFx);
  GA_IN_PRE  = parseGA(gaInPreset);

  GA_OUT_PWR = parseGA(gaOutPower);
  GA_OUT_BRI = parseGA(gaOutBri);
  GA_OUT_FX  = parseGA(gaOutFx);
  GA_OUT_R   = parseGA(gaOutR);
  GA_OUT_G   = parseGA(gaOutG);
  GA_OUT_B   = parseGA(gaOutB);
  GA_OUT_PRE = parseGA(gaOutPreset);

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

  // R/G/B channels (DPT 5.001 percent 0..100)
  if (GA_IN_R) {
    KNX.addGroupObject(GA_IN_R, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_R, [this](uint16_t /*ga*/, DptMain /*dpt*/, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write) {
        uint8_t pct = KnxIpCore::unpackScaling(p, len);
        onKnxRGB(pct_to_0_255(clamp100(pct)), LAST_G, LAST_B);
      }
    });
  }
  if (GA_IN_G) {
    KNX.addGroupObject(GA_IN_G, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_G, [this](uint16_t /*ga*/, DptMain /*dpt*/, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write) {
        uint8_t pct = KnxIpCore::unpackScaling(p, len);
        onKnxRGB(LAST_R, pct_to_0_255(clamp100(pct)), LAST_B);
      }
    });
  }
  if (GA_IN_B) {
    KNX.addGroupObject(GA_IN_B, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_B, [this](uint16_t /*ga*/, DptMain /*dpt*/, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write) {
        uint8_t pct = KnxIpCore::unpackScaling(p, len);
        onKnxRGB(LAST_R, LAST_G, pct_to_0_255(clamp100(pct)));
      }
    });
  }

  // Effect index (0..255) and Preset (0..250 typical)
  if (GA_IN_FX) {
    KNX.addGroupObject(GA_IN_FX, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_FX, [this](uint16_t /*ga*/, DptMain /*dpt*/, KnxService svc, const uint8_t* p, uint8_t /*len*/){
      if (svc == KnxService::GroupValue_Write) onKnxEffect(p ? p[0] : 0);
    });
  }
  if (GA_IN_PRE) {
    KNX.addGroupObject(GA_IN_PRE, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_PRE, [this](uint16_t /*ga*/, DptMain /*dpt*/, KnxService svc, const uint8_t* p, uint8_t /*len*/){
      if (svc == KnxService::GroupValue_Write) onKnxPreset(p ? p[0] : 0);
    });
  }

  // Outbound/status objects
  if (GA_OUT_PWR) KNX.addGroupObject(GA_OUT_PWR, DptMain::DPT_1xx, /*tx=*/true, /*rx=*/false);
  if (GA_OUT_BRI) KNX.addGroupObject(GA_OUT_BRI, DptMain::DPT_5xx, true, false);
  if (GA_OUT_FX)  KNX.addGroupObject(GA_OUT_FX,  DptMain::DPT_5xx, true, false);

  // Start KNX
  KNX.begin();

  // Optional: send initial status
  // scheduleStatePublish(true, true, true);
}

void KnxIpUsermod::loop() {
  if (!enabled) return;

  KNX.loop();

  if (_nextTxAt && millis() >= _nextTxAt) {
    _nextTxAt = 0;
    publishState();
  }
}

void KnxIpUsermod::addToConfig(JsonObject& root) {
  JsonObject top = root.createNestedObject("KNX_IP");
  top["enabled"] = enabled;
  top["individual_addr"] = individualAddr;

  JsonObject gIn  = top.createNestedObject("in");
  JsonObject gOut = top.createNestedObject("out");

  gIn["power"]  = gaInPower;
  gIn["bri"]    = gaInBri;
  gIn["r"]      = gaInR;
  gIn["g"]      = gaInG;
  gIn["b"]      = gaInB;
  gIn["fx"]     = gaInFx;
  gIn["preset"] = gaInPreset;

  gOut["power"]  = gaOutPower;
  gOut["bri"]    = gaOutBri;
  gOut["fx"]     = gaOutFx;
  gOut["r"]      = gaOutR;
  gOut["g"]      = gaOutG;
  gOut["b"]      = gaOutB;
  gOut["preset"] = gaOutPreset;

  top["tx_rate_limit_ms"] = txRateLimitMs;
}

bool KnxIpUsermod::readFromConfig(JsonObject& root) {
  JsonObject top = root["KNX_IP"];
  if (top.isNull()) {
    // also accept legacy key "KNX-IP"
    top = root["KNX-IP"];
    if (top.isNull()) return false;
  }

  enabled = top["enabled"] | enabled;
  strlcpy(individualAddr, top["individual_addr"] | individualAddr, sizeof(individualAddr));

  JsonObject gIn  = top["in"];
  JsonObject gOut = top["out"];

  if (!gIn.isNull()) {
    strlcpy(gaInPower,  gIn["power"]  | gaInPower,  sizeof(gaInPower));
    strlcpy(gaInBri,    gIn["bri"]    | gaInBri,    sizeof(gaInBri));
    strlcpy(gaInR,      gIn["r"]      | gaInR,      sizeof(gaInR));
    strlcpy(gaInG,      gIn["g"]      | gaInG,      sizeof(gaInG));
    strlcpy(gaInB,      gIn["b"]      | gaInB,      sizeof(gaInB));
    strlcpy(gaInFx,     gIn["fx"]     | gaInFx,     sizeof(gaInFx));
    strlcpy(gaInPreset, gIn["preset"] | gaInPreset, sizeof(gaInPreset));
  }

  if (!gOut.isNull()) {
    strlcpy(gaOutPower,  gOut["power"]  | gaOutPower,  sizeof(gaOutPower));
    strlcpy(gaOutBri,    gOut["bri"]    | gaOutBri,    sizeof(gaOutBri));
    strlcpy(gaOutFx,     gOut["fx"]     | gaOutFx,     sizeof(gaOutFx));
    strlcpy(gaOutR,      gOut["r"]      | gaOutR,      sizeof(gaOutR));
    strlcpy(gaOutG,      gOut["g"]      | gaOutG,      sizeof(gaOutG));
    strlcpy(gaOutB,      gOut["b"]      | gaOutB,      sizeof(gaOutB));
    strlcpy(gaOutPreset, gOut["preset"] | gaOutPreset, sizeof(gaOutPreset));
  }

  txRateLimitMs = top["tx_rate_limit_ms"] | txRateLimitMs;

  // refresh GA cache from new config
  GA_IN_PWR  = parseGA(gaInPower);
  GA_IN_BRI  = parseGA(gaInBri);
  GA_IN_R    = parseGA(gaInR);
  GA_IN_G    = parseGA(gaInG);
  GA_IN_B    = parseGA(gaInB);
  GA_IN_FX   = parseGA(gaInFx);
  GA_IN_PRE  = parseGA(gaInPreset);
  GA_OUT_PWR = parseGA(gaOutPower);
  GA_OUT_BRI = parseGA(gaOutBri);
  GA_OUT_FX  = parseGA(gaOutFx);

  return true;
}
