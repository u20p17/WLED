#pragma once
/*
 * ESP-KNX-IP (ESP32-only, no webserver)
 * Minimal KNXnet/IP core for embedding into apps like WLED.
 *
 * Features:
 *  - KNXnet/IP over UDP multicast (224.0.23.12:3671)
 *  - Send GroupValueWrite / GroupValueRead telegrams
 *  - Register per-GA callbacks for incoming GroupValueWrite/Read
 *  - No runtime web configuration, no storage by default
 *
 * Configuration ownership:
 *  - The host app (e.g., WLED usermod) should set the individual
 *    address and manage group objects itself.
 *
 * License: same as original project license or your fork's choice.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <map>
#include <functional>

#ifndef KNX_ENABLE_RUNTIME_CONFIG
#define KNX_ENABLE_RUNTIME_CONFIG 0   // 0 = no runtime config/storage (stubs only)
#endif

// ===== KNX/IP defaults =====
#ifndef KNX_IP_MULTICAST_A
#define KNX_IP_MULTICAST_A 224
#endif
#ifndef KNX_IP_MULTICAST_B
#define KNX_IP_MULTICAST_B 0
#endif
#ifndef KNX_IP_MULTICAST_C
#define KNX_IP_MULTICAST_C 23
#endif
#ifndef KNX_IP_MULTICAST_D
#define KNX_IP_MULTICAST_D 12
#endif

#ifndef KNX_IP_UDP_PORT
#define KNX_IP_UDP_PORT 3671
#endif

// Common service types (cEMI / APCI)
enum class KnxService : uint8_t {
  GroupValue_Read   = 0x00,  // APCI 0x00
  GroupValue_Response = 0x01,// APCI 0x01
  GroupValue_Write  = 0x02,  // APCI 0x02
};

// DPT enums you can extend in your app
enum class DptMain : uint16_t {
  DPT_1xx  = 1,   // 1 bit
  DPT_5xx  = 5,   // 8 bit (0..255 / 0..100%)
  DPT_9xx  = 9,   // 2-byte float
  // add more as needed
};

// A small description for a KNX group object we care about
struct KnxGroupObject {
  uint16_t ga;              // group address (main/middle/sub packed as 0xMMMM)
  DptMain  dpt;             // primary DPT family
  bool     transmit;        // we may send on this GA
  bool     receive;         // we listen to this GA
};

// Callback signature for incoming telegrams
// write= true  => GroupValueWrite
// write= false => GroupValueRead (or Response depending on service)
using KnxGroupCallback = std::function<void(uint16_t ga,
                                            DptMain dpt,
                                            KnxService service,
                                            const uint8_t* payload,
                                            uint8_t len)>;

// ===== Helper: GA packing/unpacking =====
// GA is often represented as main/middle/sub (x/y/z). We store as 16-bit:
// (main << 11) | (middle << 8) | sub (EIS style)
inline constexpr uint16_t knxMakeGroupAddress(uint8_t main, uint8_t middle, uint8_t sub)
{
  return (uint16_t(main & 0x1F) << 11) | (uint16_t(middle & 0x07) << 8) | (uint16_t(sub));
}

inline constexpr uint8_t knxGaMain(uint16_t ga)   { return (ga >> 11) & 0x1F; }
inline constexpr uint8_t knxGaMiddle(uint16_t ga) { return (ga >> 8) & 0x07; }
inline constexpr uint8_t knxGaSub(uint16_t ga)    { return ga & 0xFF; }

// ===== Core class =====
class KnxIpCore {
public:
  KnxIpCore();
  ~KnxIpCore() = default;

  // Initialize UDP multicast receiver/sender
  // You must have WiFi connected already.
  bool begin();

  // Poll for incoming KNXnet/IP frames; call frequently from loop()
  void loop();

  // Optional: stop UDP
  void end();

  // Individual address (PA), 0x0000 means "unspecified" (we don't enforce PA in IP mode,
  // but user code may want to keep it for consistency).
  void     setIndividualAddress(uint16_t pa) { _pa = pa; }
  uint16_t individualAddress() const         { return _pa; }

  // Register which GA(s) the app is interested in (receive/send).
  // Call before begin() or anytime (thread-safety assumed from Arduino loop).
  void addGroupObject(uint16_t ga, DptMain dpt, bool transmit, bool receive);

  // Callbacks for incoming GroupValue* services to a specific GA.
  void onGroup(uint16_t ga, KnxGroupCallback cb);

  // High-level send helpers (payload is raw KNX data; you can use packers below)
  bool groupValueWrite(uint16_t ga, const uint8_t* data, uint8_t len);
  bool groupValueRead(uint16_t ga);

  // Convenience overloads for common DPTs (encode for you)
  bool write1Bit(uint16_t ga, bool value);                 // DPT 1.xxx
  bool writeScaling(uint16_t ga, uint8_t pct0_100);        // DPT 5.001 (0..100%)
  bool write2ByteFloat(uint16_t ga, float value);          // DPT 9.xxx (e.g., 9.001 temperature)

  // Optional: send a GroupValueResponse (rarely needed at app level)
  bool groupValueResponse(uint16_t ga, const uint8_t* data, uint8_t len);

  // Stats
  uint32_t rxPackets() const { return _rxPackets; }
  uint32_t txPackets() const { return _txPackets; }
  uint32_t rxErrors()  const { return _rxErrors;  }
  uint32_t txErrors()  const { return _txErrors;  }

  // Low-level access (advanced): send raw cEMI payload (already composed)
  bool sendCemiToGroup(uint16_t ga, KnxService svc, const uint8_t* asdu, uint8_t asduLen);

    // Pack helpers for common DPTs (implemented in esp-knx-ip-conversion.cpp)
  static uint8_t  pack1Bit(bool v);
  static uint8_t  packScaling(uint8_t pct);              // 0..100
  static void     pack2ByteFloat(float value, uint8_t out[2]); // KNX 2-byte float (DPT9)

  // Unpack helpers for common DPTs (implemented in esp-knx-ip-conversion.cpp)
  static bool     unpack1Bit(const uint8_t* p, uint8_t len);
  static uint8_t  unpackScaling(const uint8_t* p, uint8_t len);
  static float    unpack2ByteFloat(const uint8_t* p, uint8_t len);

private:
  // Internal dispatch
  void _handleIncoming(const uint8_t* buf, int len);
  bool _composeAndSendApci(uint16_t ga, KnxService svc, const uint8_t* asdu, uint8_t asduLen);

private:
  WiFiUDP     _udp;
  IPAddress   _maddr;
  uint16_t    _pa;  // physical address (optional / informational)
  bool        _running;

  // bookkeeping
  uint32_t    _rxPackets;
  uint32_t    _txPackets;
  uint32_t    _rxErrors;
  uint32_t    _txErrors;

  // which group objects we care about
  std::map<uint16_t, KnxGroupObject> _gos;
  // callbacks per GA
  std::map<uint16_t, KnxGroupCallback> _callbacks;
};

// ===== Implementation details (small inlines) =====

inline KnxIpCore::KnxIpCore()
: _maddr(KNX_IP_MULTICAST_A, KNX_IP_MULTICAST_B, KNX_IP_MULTICAST_C, KNX_IP_MULTICAST_D)
, _pa(0)
, _running(false)
, _rxPackets(0)
, _txPackets(0)
, _rxErrors(0)
, _txErrors(0)
{}

inline void KnxIpCore::addGroupObject(uint16_t ga, DptMain dpt, bool transmit, bool receive) {
  _gos[ga] = KnxGroupObject{ga, dpt, transmit, receive};
}

inline void KnxIpCore::onGroup(uint16_t ga, KnxGroupCallback cb) {
  _callbacks[ga] = cb;
}

inline bool KnxIpCore::write1Bit(uint16_t ga, bool value) {
  uint8_t v = pack1Bit(value);
  // 1-bit goes into APCI last 6 bits when len==1; we send as 1 byte ASDU for simplicity
  return sendCemiToGroup(ga, KnxService::GroupValue_Write, &v, 1);
}

inline bool KnxIpCore::writeScaling(uint16_t ga, uint8_t pct0_100) {
  uint8_t v = packScaling(pct0_100);
  return sendCemiToGroup(ga, KnxService::GroupValue_Write, &v, 1);
}

inline bool KnxIpCore::write2ByteFloat(uint16_t ga, float value) {
  uint8_t v[2];
  pack2ByteFloat(value, v);
  return sendCemiToGroup(ga, KnxService::GroupValue_Write, v, 2);
}

// KNX DPT 9.xxx 2-byte float (EIS5) helpers are implemented in
// `esp-knx-ip-conversion.cpp` to keep heavy conversion logic out of the
// header and avoid duplicate definitions when the header is included in
// multiple translation units.

// ===== Runtime config stubs (no storage/UI) =====
using config_id_t = uint16_t;

#if KNX_ENABLE_RUNTIME_CONFIG == 0
// Keep symbols so existing app code compiles, but do nothing.
inline void        knx_config_begin() {}
inline void        knx_config_end() {}
inline config_id_t knx_config_register_ga(const char* /*name*/) { return 0; }
inline config_id_t knx_config_register_int(const char* /*name*/, int /*default_val*/) { return 0; }
inline uint16_t    knx_config_get_ga(config_id_t /*id*/) { return 0; }
inline int         knx_config_get_int(config_id_t /*id*/) { return 0; }
inline void        knx_load() {}
inline void        knx_save() {}
#else
// (If you later enable runtime config, declare the real functions here
//  and implement them in a .cpp using Preferences/NVS.)
#endif

// ===== Global instance convenience (optional) =====
extern KnxIpCore KNX;

