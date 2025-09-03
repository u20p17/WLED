#include "esp-knx-ip.h"

// ======== Tunables / constants ========
static constexpr uint8_t  KNX_PROTOCOL_VERSION = 0x10;   // KNXnet/IP proto version
static constexpr uint16_t KNX_SVC_ROUTING_IND  = 0x0530; // Routing Indication (rx)
static constexpr uint16_t KNX_SVC_ROUTING_REQ  = 0x0531; // Routing Request    (tx)
static constexpr uint8_t  CEMI_LDATA_IND       = 0x29;   // cEMI L_Data.ind (rx)
static constexpr uint8_t  CEMI_LDATA_REQ       = 0x11;   // cEMI L_Data.req (tx)

// cEMI Control fields defaults:
//  - Standard frame, no repeat suppression, priority=Low
//  - Group Address, hop count 6 (typical), ACK disabled
static constexpr uint8_t  CEMI_CTRL1_DEFAULT   = 0x00;   // std frame, low prio
static constexpr uint8_t  CEMI_CTRL2_GROUP_HC6 = 0xE0;   // 1xxxxxxx (group) + hop=6

// KNX UDP RX buffer (routing frames are small; 512 is plenty)
static constexpr size_t   UDP_BUF_SIZE = 512;


// ======== Global instance as declared in header ========
KnxIpCore KNX;


// ======== Begin / End / Loop ========
bool KnxIpCore::begin() {
  if (_running) return true;
  if (WiFi.status() != WL_CONNECTED) return false;

  // Join KNX multicast
  if (!_udp.beginMulticast(_maddr, KNX_IP_UDP_PORT)) {
    _rxErrors++;
    return false;
  }
  _running = true;
  return true;
}

void KnxIpCore::end() {
  if (!_running) return;
  _udp.stop();
  _running = false;
}

void KnxIpCore::loop() {
  if (!_running) return;

  uint8_t buf[UDP_BUF_SIZE];
  int len = _udp.parsePacket();
  if (len <= 0) return;

  if (len > (int)sizeof(buf)) {
    // drain
    while (_udp.available()) (void)_udp.read();
    _rxErrors++;
    return;
  }

  int got = _udp.read(buf, len);
  if (got <= 0) return;

  _handleIncoming(buf, got);
}


// ======== Public TX API ========
bool KnxIpCore::groupValueWrite(uint16_t ga, const uint8_t* data, uint8_t len) {
  return sendCemiToGroup(ga, KnxService::GroupValue_Write, data, len);
}

bool KnxIpCore::groupValueRead(uint16_t ga) {
  // No ASDU for read
  return sendCemiToGroup(ga, KnxService::GroupValue_Read, nullptr, 0);
}

bool KnxIpCore::groupValueResponse(uint16_t ga, const uint8_t* data, uint8_t len) {
  return sendCemiToGroup(ga, KnxService::GroupValue_Response, data, len);
}


// ======== Low-level send ========
bool KnxIpCore::sendCemiToGroup(uint16_t ga, KnxService svc, const uint8_t* asdu, uint8_t asduLen) {
  if (!_running) return false;

  // Build cEMI L_Data.req
  // cEMI format (for L_Data.req / L_Data.ind):
  // [0]  msgCode
  // [1]  addInfoLen
  // [2]  ctrl1
  // [3]  ctrl2
  // [4]  src addr (hi)
  // [5]  src addr (lo)
  // [6]  dst addr (hi)
  // [7]  dst addr (lo)
  // [8]  data length (ASDU length in bytes)
  // [9..] TPDU (starts with 2 bytes: TPCI/APCI + data)
  //
  // For GroupValue*:
  //  APCI is 4 bits: bits [1:0] of TPDU[0] and bits [7:6] of TPDU[1]
  //  We send unnumbered data group: TPDU[0] low 2 bits = 0b00 (so TPDU[0]=0)
  //
  //  If ASDU is 1 byte for 1-bit DPT:
  //    - Put the data bit (bit0) into TPDU[1] low bit, along with APCI high bits.
  //    - No further ASDU bytes follow.
  //
  //  For multi-byte ASDU:
  //    - TPDU[1] holds only APCI high bits (data bits = 0)
  //    - Then append 'asduLen' bytes.

  uint8_t cemi[64];
  uint8_t idx = 0;

  const uint8_t msgCode   = CEMI_LDATA_REQ;
  const uint8_t addInfo   = 0x00;
  const uint8_t ctrl1     = CEMI_CTRL1_DEFAULT;
  const uint8_t ctrl2     = CEMI_CTRL2_GROUP_HC6;
  const uint16_t srcAddr  = _pa; // if 0, still acceptable in IP routing context

  // Compose TPDU header:
  uint8_t tpdu0 = 0x00; // unnumbered data group (low 2 bits = 0)
  uint8_t tpdu1 = 0x00;

  uint8_t apci2bits = 0; // top 2 bits of TPDU[1]
  switch (svc) {
    case KnxService::GroupValue_Read:     apci2bits = 0b00; break; // APCI 0x0
    case KnxService::GroupValue_Response: apci2bits = 0b01; break; // APCI 0x1
    case KnxService::GroupValue_Write:    apci2bits = 0b10; break; // APCI 0x2
  }
  tpdu1 = (uint8_t)(apci2bits << 6);

  bool oneBit = (asduLen == 1);

  // Start cEMI
  cemi[idx++] = msgCode;
  cemi[idx++] = addInfo;
  cemi[idx++] = ctrl1;
  cemi[idx++] = ctrl2;
  cemi[idx++] = (uint8_t)(srcAddr >> 8);
  cemi[idx++] = (uint8_t)(srcAddr & 0xFF);
  cemi[idx++] = (uint8_t)(ga >> 8);
  cemi[idx++] = (uint8_t)(ga & 0xFF);

  // data length = TPDU payload (bytes after TPDU[1]) + (potentially embedded 1bit in TPDU[1])
  // KNX "data length" is the number of bytes in the APDU (TPDU starting after the length byte).
  // We always include TPDU[0] and TPDU[1] inside that length.
  uint8_t apduLenBytes = 2 + (oneBit ? 0 : asduLen);
  cemi[idx++] = apduLenBytes;

  // TPDU
  cemi[idx++] = tpdu0;
  if (oneBit && asdu) {
    uint8_t bit = (asdu[0] & 0x01);
    cemi[idx++] = (uint8_t)(tpdu1 | bit);
  } else {
    cemi[idx++] = tpdu1;
    for (uint8_t i = 0; i < asduLen; ++i) cemi[idx++] = asdu[i];
  }

  // Now wrap cEMI inside KNXnet/IP Routing Request
  uint8_t frame[96];
  uint8_t p = 0;

  // KNXnet/IP header (6 bytes)
  //  - header size (0x06)
  //  - protocol version (0x10)
  //  - service type (hi, lo) = 0x0531 (Routing Request)
  //  - total length (hi, lo) = 6 + cEMI length
  uint16_t totalLen = 6 + idx;
  frame[p++] = 0x06;
  frame[p++] = KNX_PROTOCOL_VERSION;
  frame[p++] = (uint8_t)(KNX_SVC_ROUTING_REQ >> 8);
  frame[p++] = (uint8_t)(KNX_SVC_ROUTING_REQ & 0xFF);
  frame[p++] = (uint8_t)(totalLen >> 8);
  frame[p++] = (uint8_t)(totalLen & 0xFF);

  // cEMI payload
  for (uint8_t i = 0; i < idx; ++i) frame[p++] = cemi[i];

  // Send
  // Some ESP32 WiFiUDP variants don't implement beginPacketMulticast;
  // sending to the multicast address via beginPacket works instead.
  int sent = _udp.beginPacket(_maddr, KNX_IP_UDP_PORT);
  if (sent == 0) { _txErrors++; return false; }
  sent = _udp.write(frame, p);
  bool ok = _udp.endPacket();

  if (!ok || sent != p) { _txErrors++; return false; }
  _txPackets++;
  return true;
}


// ======== Internal RX path ========
void KnxIpCore::_handleIncoming(const uint8_t* buf, int len) {
  // Validate KNXnet/IP header (min 6 bytes)
  if (len < 6) { _rxErrors++; return; }

  uint8_t  headerSize  = buf[0];
  uint8_t  proto       = buf[1];
  uint16_t svc         = (uint16_t(buf[2]) << 8) | buf[3];
  uint16_t totalLen    = (uint16_t(buf[4]) << 8) | buf[5];

  if (headerSize != 0x06 || proto != KNX_PROTOCOL_VERSION) { _rxErrors++; return; }
  if (totalLen != len) {
    // Some stacks pad UDP; be lenient if totalLen <= len
    if (totalLen > len) { _rxErrors++; return; }
  }

  // Routing Indication frames carry cEMI
  if (svc != KNX_SVC_ROUTING_IND) {
    // ignore other services (search response, tunneling, etc.)
    return;
  }

  // cEMI starts after 6 bytes
  if (len < 6 + 10) { // minimal cEMI header size check
    _rxErrors++; return;
  }
  const uint8_t* cemi = buf + 6;
  int cemiLen = len - 6;

  uint8_t msgCode = cemi[0];
  uint8_t addInfo = cemi[1];
  (void)addInfo; // we don't use additional info (0 usually)

  if (msgCode != CEMI_LDATA_IND && msgCode != CEMI_LDATA_REQ) {
    // We only care about L_Data.ind/req on routing
    return;
  }

  if (cemiLen < 10) { _rxErrors++; return; }

  uint8_t ctrl1 = cemi[2];
  uint8_t ctrl2 = cemi[3];
  (void)ctrl1; (void)ctrl2;

  uint16_t src  = (uint16_t(cemi[4]) << 8) | cemi[5];
  uint16_t dst  = (uint16_t(cemi[6]) << 8) | cemi[7];

  uint8_t apduLen = cemi[8]; // includes TPDU[0] and TPDU[1] and any data bytes

  // TPDU starts at cemi[9]
  if (cemiLen < 9 + apduLen) { _rxErrors++; return; }
  const uint8_t* tpdu = cemi + 9;
  if (apduLen < 2) { _rxErrors++; return; } // must have at least TPDU[0] + TPDU[1]

  // Destination type: group if ctrl2 bit7 == 1
  bool isGroup = (ctrl2 & 0x80) != 0;
  if (!isGroup) {
    // Not a group address -> ignore for our API
    return;
  }

  // Extract APCI (4 bits): [TPDU0:1:0]<<2 | [TPDU1:7:6]
  uint8_t apci4 = ((tpdu[0] & 0x03) << 2) | ((tpdu[1] & 0xC0) >> 6);
  KnxService svcDetected;
  switch (apci4) {
    case 0x0: svcDetected = KnxService::GroupValue_Read; break;
    case 0x1: svcDetected = KnxService::GroupValue_Response; break;
    case 0x2: svcDetected = KnxService::GroupValue_Write; break;
    default:
      // other APCIs (A/B writing, memory read/write, etc.) not handled here
      return;
  }

  // Determine ASDU bytes (after the two TPDU header bytes)
  const uint8_t* asdu = nullptr;
  uint8_t asduLen = 0;

  if (svcDetected == KnxService::GroupValue_Write && apduLen == 2) {
    // 1-bit write, data in TPDU[1] bit0
    static uint8_t one;
    one = (tpdu[1] & 0x01);
    asdu = &one;
    asduLen = 1;
  } else {
    // multi-byte payload: TPDU[2..]
    if (apduLen > 2) {
      asdu = tpdu + 2;
      asduLen = apduLen - 2;
    } else {
      asdu = nullptr;
      asduLen = 0;
    }
  }

  // If we have a registered group object & callback, dispatch
  auto itGo = _gos.find(dst);
  if (itGo != _gos.end()) {
    DptMain dpt = itGo->second.dpt;
    auto itCb = _callbacks.find(dst);
    if (itCb != _callbacks.end() && itCb->second) {
      itCb->second(dst, dpt, svcDetected, asdu, asduLen);
    }
  }

  (void)src; // available if you want to use/forward it
  _rxPackets++;
}


// ======== Compose and send APCI ========
bool KnxIpCore::_composeAndSendApci(uint16_t ga, KnxService svc, const uint8_t* asdu, uint8_t asduLen) {
  return sendCemiToGroup(ga, svc, asdu, asduLen);
}


// ======== (Optional) convenience wrappers used by header ========
bool KnxIpCore::sendCemiToGroup(uint16_t ga, KnxService svc, const uint8_t* asdu, uint8_t asduLen); // forward declared in header; implemented above.

