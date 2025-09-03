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

  // (no anonymous LAST_* variables here)
}

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

// ------------- Coalesced outbound state -------------
void KnxIpUsermod::scheduleStatePublish(bool pwr, bool bri_, bool fx) {
  _pendingTxPower |= pwr;
  _pendingTxBri   |= bri_;
  _pendingTxFx    |= fx;
  unsigned long now = millis();
  if (_nextTxAt == 0 || now > _nextTxAt) _nextTxAt = now + txRateLimitMs;
}

void KnxIpUsermod::publishState() {
  // If nothing is pending and no OUT GAs are configured, bail early
  const bool anyPending  = _pendingTxPower || _pendingTxBri || _pendingTxFx;
  const bool anyColorOut = (GA_OUT_R || GA_OUT_G || GA_OUT_B || GA_OUT_W || GA_OUT_CCT || GA_OUT_WW || GA_OUT_CW);
  if (!anyPending && !anyColorOut && !GA_OUT_PRE) return;

  // Base state
  const bool    pwr     = (bri > 0);
  const uint8_t pct     = (uint8_t)((bri * 100u + 127u) / 255u);   // 0..100
  const uint8_t fxIndex = effectCurrent;                           // 0..255

  // Pending (coalesced) telegrams
  if (_pendingTxPower && GA_OUT_PWR) KNX.write1Bit(GA_OUT_PWR, pwr);
  if (_pendingTxBri   && GA_OUT_BRI) KNX.writeScaling(GA_OUT_BRI, pct);
  if (_pendingTxFx    && GA_OUT_FX)  {
    uint8_t v = fxIndex;
    KNX.groupValueWrite(GA_OUT_FX, &v, 1); // DPT 5.xxx
  }

  // Color + white + CCT + derived WW/CW (publish when configured)
  if (anyColorOut) {
    const Segment& seg = strip.getSegment(0);
    const uint32_t c   = seg.colors[0];
    const uint8_t  r   = R(c), g = G(c), b = B(c), w = W(c);
    const uint8_t  cct = seg.cct; // 0..255 (0=warm .. 255=cold)

    if (GA_OUT_R)   KNX.writeScaling(GA_OUT_R,   (uint8_t)((r * 100u + 127u) / 255u));
    if (GA_OUT_G)   KNX.writeScaling(GA_OUT_G,   (uint8_t)((g * 100u + 127u) / 255u));
    if (GA_OUT_B)   KNX.writeScaling(GA_OUT_B,   (uint8_t)((b * 100u + 127u) / 255u));
    if (GA_OUT_W)   KNX.writeScaling(GA_OUT_W,   (uint8_t)((w * 100u + 127u) / 255u));

    if (GA_OUT_CCT) KNX.writeScaling(GA_OUT_CCT, (uint8_t)((cct * 100u + 127u) / 255u));

    if (GA_OUT_WW || GA_OUT_CW) {
      // Derive warm/cold components from W and CCT
      const uint16_t ww = (uint16_t)((uint16_t)w * (255u - cct) / 255u);
      const uint16_t cw = (uint16_t)((uint16_t)w * cct / 255u);
      if (GA_OUT_WW) KNX.writeScaling(GA_OUT_WW, (uint8_t)((ww * 100u + 127u) / 255u));
      if (GA_OUT_CW) KNX.writeScaling(GA_OUT_CW, (uint8_t)((cw * 100u + 127u) / 255u));
    }
  }

  // Preset index (if configured)
  if (GA_OUT_PRE) {
    uint8_t p = _lastPreset; // last applied via onKnxPreset()
    KNX.groupValueWrite(GA_OUT_PRE, &p, 1);
  }

  // Clear pending flags after publish
  _pendingTxPower = _pendingTxBri = _pendingTxFx = false;
}

void KnxIpUsermod::onKnxWhite(uint8_t pct) {            // 0..100
  if (pct > 100) pct = 100;
  LAST_W = (uint8_t)((pct * 255u + 50u) / 100u);
  uint8_t r,g,b,w; getCurrentRGBW(r,g,b,w);
  strip.setColor(0, r, g, b, LAST_W);
  colorUpdated(CALL_MODE_DIRECT_CHANGE);
  scheduleStatePublish(true, true, false);
}

void KnxIpUsermod::onKnxCct(uint8_t pct) {              // 0..100
  if (pct > 100) pct = 100;
  LAST_CCT = (uint8_t)((pct * 255u + 50u) / 100u);
  // Apply CCT to current segment
  Segment& seg = strip.getSegment(0);
  // Prefer method if available; otherwise assign (WLED 0.13+ keeps seg.cct public).
  seg.cct = LAST_CCT;
  stateUpdated(CALL_MODE_DIRECT_CHANGE);
  scheduleStatePublish(true, true, false);
}

void KnxIpUsermod::applyWhiteAndCct() {
  // Apply LAST_W and LAST_CCT to segment 0
  uint8_t r,g,b,w; getCurrentRGBW(r,g,b,w);
  strip.setColor(0, r, g, b, LAST_W);
  strip.getSegment(0).cct = LAST_CCT;
  colorUpdated(CALL_MODE_DIRECT_CHANGE);
}

void KnxIpUsermod::onKnxWW(uint8_t pct) {
  if (pct > 100) pct = 100;
  uint8_t ww = (uint8_t)((pct * 255u + 50u) / 100u);

  // read live W & CCT
  const Segment& seg = strip.getSegment(0);
  uint8_t wLive = W(seg.colors[0]);
  uint8_t cctLive = seg.cct;

  // derive current CW from live values
  uint8_t cw = (uint8_t)(( (uint16_t)wLive * cctLive + 127u) / 255u);
  uint16_t sum = (uint16_t)ww + (uint16_t)cw;

  LAST_W   = (sum > 255) ? 255 : (uint8_t)sum;
  LAST_CCT = (sum == 0) ? cctLive : (uint8_t)((cw * 255u + sum/2) / sum);

  applyWhiteAndCct();
  scheduleStatePublish(true, true, false);
}

void KnxIpUsermod::onKnxCW(uint8_t pct) {
  if (pct > 100) pct = 100;
  uint8_t cw = (uint8_t)((pct * 255u + 50u) / 100u);

  const Segment& seg = strip.getSegment(0);
  uint8_t wLive = W(seg.colors[0]);
  uint8_t cctLive = seg.cct;

  uint8_t ww = (uint8_t)(( (uint16_t)wLive * (255u - cctLive) + 127u) / 255u);
  uint16_t sum = (uint16_t)ww + (uint16_t)cw;

  LAST_W   = (sum > 255) ? 255 : (uint8_t)sum;
  LAST_CCT = (sum == 0) ? cctLive : (uint8_t)((cw * 255u + sum/2) / sum);

  applyWhiteAndCct();
  scheduleStatePublish(true, true, false);
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

  GA_IN_W   = parseGA(gaInW);
  if (GA_IN_W) {
    KNX.addGroupObject(GA_IN_W, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_W, [this](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write) onKnxWhite(KnxIpCore::unpackScaling(p, len));
    });
  }

  GA_IN_CCT = parseGA(gaInCct);
  if (GA_IN_CCT) {
    KNX.addGroupObject(GA_IN_CCT, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_CCT, [this](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write) onKnxCct(KnxIpCore::unpackScaling(p, len));
    });
  }

  // Optional direct WW/CW level inputs
  GA_IN_WW = parseGA(gaInWW);
  if (GA_IN_WW) {
    KNX.addGroupObject(GA_IN_WW, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_WW, [this](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write) onKnxWW(KnxIpCore::unpackScaling(p, len));
    });
  }

  GA_IN_CW = parseGA(gaInCW);
  if (GA_IN_CW) {
    KNX.addGroupObject(GA_IN_CW, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_CW, [this](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write) onKnxCW(KnxIpCore::unpackScaling(p, len));
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
  // R
  if (GA_IN_R) {
    KNX.addGroupObject(GA_IN_R, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_R, [this](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write) {
        uint8_t pct = KnxIpCore::unpackScaling(p, len);
        uint8_t r,g,b,w; getCurrentRGBW(r,g,b,w);
        onKnxRGB(pct_to_0_255(clamp100(pct)), g, b);
      }
    });
  }
  // G
  if (GA_IN_G) {
    KNX.addGroupObject(GA_IN_G, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_G, [this](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write) {
        uint8_t pct = KnxIpCore::unpackScaling(p, len);
        uint8_t r,g,b,w; getCurrentRGBW(r,g,b,w);
        onKnxRGB(r, pct_to_0_255(clamp100(pct)), b);
      }
    });
  }
  // B
  if (GA_IN_B) {
    KNX.addGroupObject(GA_IN_B, DptMain::DPT_5xx, false, true);
    KNX.onGroup(GA_IN_B, [this](uint16_t, DptMain, KnxService svc, const uint8_t* p, uint8_t len){
      if (svc == KnxService::GroupValue_Write) {
        uint8_t pct = KnxIpCore::unpackScaling(p, len);
        uint8_t r,g,b,w; getCurrentRGBW(r,g,b,w);
        onKnxRGB(r, g, pct_to_0_255(clamp100(pct)));
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
  if (GA_OUT_R)   KNX.addGroupObject(GA_OUT_R,   DptMain::DPT_5xx, true, false);
  if (GA_OUT_G)   KNX.addGroupObject(GA_OUT_G,   DptMain::DPT_5xx, true, false);
  if (GA_OUT_B)   KNX.addGroupObject(GA_OUT_B,   DptMain::DPT_5xx, true, false);
  if (GA_OUT_PRE) KNX.addGroupObject(GA_OUT_PRE, DptMain::DPT_5xx, true, false);
  if (GA_OUT_W)   KNX.addGroupObject(GA_OUT_W,   DptMain::DPT_5xx, true, false);
  if (GA_OUT_CCT) KNX.addGroupObject(GA_OUT_CCT, DptMain::DPT_5xx, true, false);
  if (GA_OUT_WW)  KNX.addGroupObject(GA_OUT_WW,  DptMain::DPT_5xx, true, false);
  if (GA_OUT_CW)  KNX.addGroupObject(GA_OUT_CW,  DptMain::DPT_5xx, true, false);
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
  gIn["w"]   = gaInW;
  gIn["cct"] = gaInCct;
  gIn["ww"]  = gaInWW;    // optional
  gIn["cw"]  = gaInCW;    // optional

  gOut["power"]  = gaOutPower;
  gOut["bri"]    = gaOutBri;
  gOut["fx"]     = gaOutFx;
  gOut["r"]      = gaOutR;
  gOut["g"]      = gaOutG;
  gOut["b"]      = gaOutB;
  gOut["preset"] = gaOutPreset;
  gOut["w"]   = gaOutW;
  gOut["cct"] = gaOutCct;
  gOut["ww"]  = gaOutWW;  // optional
  gOut["cw"]  = gaOutCW;  // optional

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
    strlcpy(gaInW,      gIn["w"]      | gaInW,      sizeof(gaInW));
    strlcpy(gaInCct,    gIn["cct"]    | gaInCct,    sizeof(gaInCct));
    strlcpy(gaInWW,     gIn["ww"]     | gaInWW,     sizeof(gaInWW));
    strlcpy(gaInCW,     gIn["cw"]     | gaInCW,     sizeof(gaInCW));
  }

  if (!gOut.isNull()) {
    strlcpy(gaOutPower,  gOut["power"]  | gaOutPower,  sizeof(gaOutPower));
    strlcpy(gaOutBri,    gOut["bri"]    | gaOutBri,    sizeof(gaOutBri));
    strlcpy(gaOutFx,     gOut["fx"]     | gaOutFx,     sizeof(gaOutFx));
    strlcpy(gaOutR,      gOut["r"]      | gaOutR,      sizeof(gaOutR));
    strlcpy(gaOutG,      gOut["g"]      | gaOutG,      sizeof(gaOutG));
    strlcpy(gaOutB,      gOut["b"]      | gaOutB,      sizeof(gaOutB));
    strlcpy(gaOutPreset, gOut["preset"] | gaOutPreset, sizeof(gaOutPreset));
    strlcpy(gaOutW,      gOut["w"]      | gaOutW,      sizeof(gaOutW));
    strlcpy(gaOutCct,    gOut["cct"]    | gaOutCct,    sizeof(gaOutCct));
    strlcpy(gaOutWW,     gOut["ww"]     | gaOutWW,     sizeof(gaOutWW));
    strlcpy(gaOutCW,     gOut["cw"]     | gaOutCW,     sizeof(gaOutCW));
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
  GA_IN_W   = parseGA(gaInW);
  GA_IN_CCT = parseGA(gaInCct);
  GA_IN_WW  = parseGA(gaInWW);
  GA_IN_CW  = parseGA(gaInCW);

  GA_OUT_PWR = parseGA(gaOutPower);
  GA_OUT_BRI = parseGA(gaOutBri);
  GA_OUT_R   = parseGA(gaOutR);
  GA_OUT_G   = parseGA(gaOutG);
  GA_OUT_B   = parseGA(gaOutB);
  GA_OUT_FX  = parseGA(gaOutFx);
  GA_OUT_PRE = parseGA(gaOutPreset);
  GA_OUT_W    = parseGA(gaOutW);
  GA_OUT_CCT  = parseGA(gaOutCct);
  GA_OUT_WW   = parseGA(gaOutWW);
  GA_OUT_CW   = parseGA(gaOutCW);

  return true;
}