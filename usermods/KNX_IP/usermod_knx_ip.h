#pragma once
#include "wled.h"
#include "esp-knx-ip.h"

#ifndef USERMOD_ID_KNX_IP
#define USERMOD_ID_KNX_IP 0xA902   // pick any free value (or use USERMOD_ID_UNSPECIFIED)
#endif

class KnxIpUsermod : public Usermod {
public:
  // Defaults; can be overridden via JSON config
  uint8_t  gaPowerMain   = 1, gaPowerMid   = 1, gaPowerSub   = 10;
  uint8_t  gaDimMain     = 1, gaDimMid     = 1, gaDimSub     = 11;
  uint16_t pa            = 0x1101;
  bool     enablePublish = true;

  void setup() override;
  void loop() override;

  void addToConfig(JsonObject &root) override;
  bool readFromConfig(JsonObject &root) override;

  uint16_t getId() override { return USERMOD_ID_KNX_IP; }  // or USERMOD_ID_UNSPECIFIED
  const char* getName() { return "KNX-IP"; }               // no 'override' needed in some WLED versions

private:
  bool _started = false;
  uint32_t _lastPubMs = 0;

  uint16_t _gaPower() const { return knxMakeGroupAddress(gaPowerMain, gaPowerMid, gaPowerSub); }
  uint16_t _gaDim()   const { return knxMakeGroupAddress(gaDimMain,   gaDimMid,   gaDimSub);   }

  void _onKnx(uint16_t ga, DptMain dpt, KnxService svc, const uint8_t* payload, uint8_t len);
  void _publishState();
};
