#include "usermod_knx_ip.h"

// single global KNX instance is defined in esp-knx-ip.cpp
extern KnxIpCore KNX;

void KnxIpUsermod::setup() {
  if (WiFi.status() == WL_CONNECTED && !_started) {
    KNX.setIndividualAddress(pa);
    KNX.addGroupObject(_gaPower(), DptMain::DPT_1xx, /*tx*/true, /*rx*/true);
    KNX.addGroupObject(_gaDim(),   DptMain::DPT_5xx, /*tx*/true, /*rx*/true);

    KNX.onGroup(_gaPower(), [this](uint16_t ga, DptMain dpt, KnxService svc, const uint8_t* p, uint8_t l){
      _onKnx(ga, dpt, svc, p, l);
    });
    KNX.onGroup(_gaDim(), [this](uint16_t ga, DptMain dpt, KnxService svc, const uint8_t* p, uint8_t l){
      _onKnx(ga, dpt, svc, p, l);
    });

    if (KNX.begin()) {
      _started = true;
      _publishState();
    }
  }
}

void KnxIpUsermod::loop() {
  if (_started) {
    KNX.loop();
    if (enablePublish && millis() - _lastPubMs > 10000UL) _publishState();
  } else if (WiFi.status() == WL_CONNECTED) {
    setup();
  }
}

void KnxIpUsermod::_onKnx(uint16_t ga, DptMain, KnxService svc, const uint8_t* payload, uint8_t len) {
  if (svc == KnxService::GroupValue_Read) {
    if (ga == _gaPower()) {
      uint8_t bit = KnxIpCore::pack1Bit((bool)bri);
      KNX.groupValueResponse(_gaPower(), &bit, 1);
    } else if (ga == _gaDim()) {
      uint8_t pct = map(bri, 0, 255, 0, 100);
      uint8_t enc = KnxIpCore::packScaling(pct);
      KNX.groupValueResponse(_gaDim(), &enc, 1);
    }
    return;
  }

  if (svc == KnxService::GroupValue_Write) {
    if (ga == _gaPower()) {
      bool on = KnxIpCore::unpack1Bit(payload, len);
      bri = on ? (bri == 0 ? 128 : bri) : 0;
      effectCurrent = 0;
      colorUpdated(CALL_MODE_DIRECT_CHANGE);
    } else if (ga == _gaDim()) {
      uint8_t pct = KnxIpCore::unpackScaling(payload, len);
      bri = map(pct, 0, 100, 0, 255);
      colorUpdated(CALL_MODE_DIRECT_CHANGE);
    }
    if (enablePublish) _publishState();
  }
}

void KnxIpUsermod::_publishState() {
  _lastPubMs = millis();
  uint8_t bit = KnxIpCore::pack1Bit((bool)bri);
  KNX.groupValueWrite(_gaPower(), &bit, 1);

  uint8_t pct = map(bri, 0, 255, 0, 100);
  uint8_t enc = KnxIpCore::packScaling(pct);
  KNX.groupValueWrite(_gaDim(), &enc, 1);
}

void KnxIpUsermod::addToConfig(JsonObject &root) {
  JsonObject j = root.createNestedObject("KNX-IP");
  j["pa"]            = pa;
  j["gaPowerMain"]   = gaPowerMain;
  j["gaPowerMid"]    = gaPowerMid;
  j["gaPowerSub"]    = gaPowerSub;
  j["gaDimMain"]     = gaDimMain;
  j["gaDimMid"]      = gaDimMid;
  j["gaDimSub"]      = gaDimSub;
  j["enablePublish"] = enablePublish;
}

bool KnxIpUsermod::readFromConfig(JsonObject &root) {
  JsonObject j = root["KNX-IP"];
  if (j.isNull()) return false;

  pa            = j["pa"]            | pa;
  gaPowerMain   = j["gaPowerMain"]   | gaPowerMain;
  gaPowerMid    = j["gaPowerMid"]    | gaPowerMid;
  gaPowerSub    = j["gaPowerSub"]    | gaPowerSub;
  gaDimMain     = j["gaDimMain"]     | gaDimMain;
  gaDimMid      = j["gaDimMid"]      | gaDimMid;
  gaDimSub      = j["gaDimSub"]      | gaDimSub;
  enablePublish = j["enablePublish"] | enablePublish;
  return true;
}
