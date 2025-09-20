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
#include <lwip/sockets.h>
#include <sys/socket.h>
#include <lwip/igmp.h>
#include <fcntl.h>
#include <errno.h>
#include <map>
#include <functional>
#include <array>
#include <unordered_map>

#ifndef KNX_DEBUG
#define KNX_DEBUG 1   // 1=enable Serial logs, 0=disable
#endif
#if KNX_DEBUG
#  define KNX_LOG(fmt, ...) do { Serial.printf("[KNX] " fmt "\n", ##__VA_ARGS__); } while(0)
#else
#  define KNX_LOG(fmt, ...) do {} while(0)
#endif

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
// Primary DPT family identifiers (coarse). Only DPT_1xx is treated specially (1-bit embedding).
// Others are semantic sugar so user code can register the correct family for clarity.
enum class DptMain : uint16_t {
  DPT_1xx    = 1,    // 1 bit (on/off etc.)
  DPT_2xx    = 2,    // (unused here; 2-bit controlled)
  DPT_3xx    = 3,    // (unused here; 4-bit dimming steps)
  DPT_5xx    = 5,    // 8 bit unsigned (0..255 / scaling 0..100%)
  DPT_6xx    = 6,    // 8 bit signed (two's complement) - relative dim/color deltas
  DPT_7xx    = 7,    // 16 bit unsigned (e.g. Kelvin)
  DPT_8xx    = 8,    // 16 bit signed (not currently used)
  DPT_9xx    = 9,    // 2-byte float (EIS5)
  DPT_10xx   = 10,   // TimeOfDay (3 bytes)
  DPT_11xx   = 11,   // Date (3 bytes)
  DPT_12xx   = 12,   // 32 bit unsigned (not used yet)
  DPT_13xx   = 13,   // 32 bit signed (not used yet)
  DPT_14xx   = 14,   // 4-byte float (IEEE 754)
  DPT_19xx   = 19,   // DateTime (8 bytes)
  // Extended application specific composites used here:
  DPT_232xx  = 232,  // 3-byte RGB / HSV style (DPST-232-600)
  DPT_251xx  = 251,  // 6-byte RGBW (DPST-251-600)
};

// ---- KNX DPT 3.* (4-bit step/direction) helper ----
// KNX standard defines the 4-bit control field layout:
// bit3: direction (0 = decrease, 1 = increase)
// bits2..0: step code (0 = stop, 1..7 = relative step / speed). The quantitative
// magnitude or ramp speed associated with codes 1..7 is NOT fixed by the core
// standard and is left to the receiving actuator. Therefore the core only
// decodes/encodes structure; higher layers decide on percentage or ramp mapping.
struct KnxDpt3Step {
  bool    increase;  // true => increase, false => decrease
  uint8_t step;      // 0=STOP, 1..7=step code
  bool isStop() const { return step == 0; }
};

static inline KnxDpt3Step knxDecodeDpt3(uint8_t raw) {
  return KnxDpt3Step{ (raw & 0x08)!=0, (uint8_t)(raw & 0x07) };
}

static inline uint8_t knxEncodeDpt3(const KnxDpt3Step& v) {
  return (uint8_t)((v.increase ? 0x08 : 0x00) | (v.step & 0x07));
}

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

  // Re-apply IGMP membership and TX iface without tearing the socket down.
  bool rejoinMulticast();

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
  static void     pack4ByteFloat(float value, uint8_t out[4]); // KNX DPT14 (IEEE 754, big-endian)

  // Unpack helpers for common DPTs (implemented in esp-knx-ip-conversion.cpp)
  static bool     unpack1Bit(const uint8_t* p, uint8_t len);
  static uint8_t  unpackScaling(const uint8_t* p, uint8_t len);
  static float    unpack2ByteFloat(const uint8_t* p, uint8_t len);
  static float    unpack4ByteFloat(const uint8_t* p, uint8_t len);

  bool running() const { return _running; }
  void clearRegistrations();
  bool _sendSearchResponse(bool extended, const uint8_t* req, int reqLen);

   void setCommunicationEnhancement(bool enable, uint8_t count = 3, uint16_t gapMs = 0, uint16_t dedupMs = 700) {
    _enhanced = enable;
    _enhancedSendCount = (count < 1) ? 1 : count;
    _enhancedGapMs = gapMs;
    _rxDedupWindowMs = dedupMs;
  }


private:
  // Internal dispatch
  void _handleIncoming(const uint8_t* buf, int len);
  bool _composeAndSendApci(uint16_t ga, KnxService svc, const uint8_t* asdu, uint8_t asduLen);

   // ===== Enhancement state =====
  bool     _enhanced            = false;
  uint8_t  _enhancedSendCount   = 1;      // 1 = no repetition
  uint16_t _enhancedGapMs       = 0;      // spacing between repeats (0 = back-to-back)
  uint16_t _rxDedupWindowMs     = 700;    // like Tasmota doc

  // Seen-packet signature cache (tiny ring buffer)
  struct RxSig { uint32_t sig; uint32_t ts; };
  static constexpr size_t RXSIG_SLOTS = 8;
  std::array<RxSig, RXSIG_SLOTS> _rxSeen{{}};
  uint8_t _rxSeenIdx = 0;

  // Per-GA last 1-bit value/time (for toggle throttling)
  struct RxBitState { uint32_t ts; uint8_t v; };
  std::unordered_map<uint16_t, RxBitState> _rxBitCache;

  // quick 32-bit signature mixer for RX dedup
  static inline uint32_t _mix32(uint32_t x) {
    x ^= x >> 16; x *= 0x7feb352d; x ^= x >> 15; x *= 0x846ca68b; x ^= x >> 16; return x;
  }

  int         _sock;      // raw UDP socket
  IPAddress   _maddr;     // multicast group address
  struct sockaddr_in _mcastAddr; // cached multicast sockaddr
  in_addr _lastIfAddr;        // last interface address used for IGMP
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
: _sock(-1)
, _maddr(KNX_IP_MULTICAST_A, KNX_IP_MULTICAST_B, KNX_IP_MULTICAST_C, KNX_IP_MULTICAST_D)
, _pa(0)
, _running(false)
, _rxPackets(0)
, _txPackets(0)
, _rxErrors(0)
, _txErrors(0)
{
  memset(&_mcastAddr, 0, sizeof(_mcastAddr));
  _lastIfAddr = {};
}

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

