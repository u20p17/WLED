#include "esp-knx-ip.h"

// ======== Tunables / constants ========
static constexpr uint8_t  KNX_PROTOCOL_VERSION = 0x10;   // KNXnet/IP proto version
static constexpr uint16_t KNX_SVC_ROUTING_IND  = 0x0530; // Routing Indication (rx)
static constexpr uint8_t  CEMI_LDATA_IND       = 0x29;   // cEMI L_Data.ind (rx)


// cEMI Control fields defaults:
//  - Standard frame, no repeat suppression, priority=Low
//  - Group Address, hop count 6 (typical), ACK disabled
static constexpr uint8_t  CEMI_CTRL1_DEFAULT   = 0xBC;   // std frame, low prio
static constexpr uint8_t  CEMI_CTRL2_GROUP_HC6 = 0xE0;   // 1xxxxxxx (group) + hop=6

// KNX UDP RX buffer (routing frames are small; 512 is plenty)
static constexpr size_t   UDP_BUF_SIZE = 512;


// ======== Global instance as declared in header ========
KnxIpCore KNX;

// small local hex-dump helper for debugging (first up to 96 bytes)
static void knx_dump_hex(const char* tag, const uint8_t* data, size_t len) {
  if (!data || !len) { return; }
  const size_t maxDump = len < 96 ? len : 96;
  Serial.printf("[KNX] %s (%u bytes): ", tag, (unsigned)len);
  for (size_t i = 0; i < maxDump; ++i) {
    Serial.printf("%02X", data[i]);
    if (i + 1 < maxDump) Serial.print(' ');
  }
  if (maxDump < len) Serial.print(" ...");
  Serial.print('\n');
}


// ======== Begin / End / Loop ========
bool KnxIpCore::begin() {
  if (_running) return true;
  if (WiFi.status() != WL_CONNECTED) {
    KNX_LOG("begin(): WiFi not connected (status=%d).", (int)WiFi.status());
    return false;
  }

  // Create UDP socket
  _sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (_sock < 0) { _rxErrors++; KNX_LOG("begin(): socket() failed errno=%d", errno); return false; }

  // Allow address reuse
  int yes = 1;
  (void)::setsockopt(_sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  // Bind to INADDR_ANY:3671
  struct sockaddr_in local; memset(&local, 0, sizeof(local));
  local.sin_family = AF_INET;
  local.sin_port   = htons(KNX_IP_UDP_PORT);
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  if (::bind(_sock, (struct sockaddr*)&local, sizeof(local)) < 0) {
    _rxErrors++; KNX_LOG("begin(): bind() failed errno=%d", errno); ::close(_sock); _sock=-1; return false; }

in_addr maddr{};        maddr.s_addr = inet_addr("224.0.23.12");        // KNX group
in_addr ifaddr{};       ifaddr.s_addr = inet_addr(WiFi.localIP().toString().c_str());

// Join multicast group on all interfaces (or use ifaddr here if you prefer)
ip_mreq mreq{}; 
mreq.imr_multiaddr = maddr;
mreq.imr_interface.s_addr = htonl(INADDR_ANY);
if (::setsockopt(_sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
  _rxErrors++; KNX_LOG("begin(): IP_ADD_MEMBERSHIP failed errno=%d", errno);
}

// TTL=1 and LOOP=1 are fine…
uint8_t ttl = 1;  (void)::setsockopt(_sock, IPPROTO_IP, IP_MULTICAST_TTL,  &ttl,  sizeof(ttl));
uint8_t loop = 1; (void)::setsockopt(_sock, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));

// Pin outgoing multicast to the STA interface
  if (::setsockopt(_sock, IPPROTO_IP, IP_MULTICAST_IF, &ifaddr, sizeof(ifaddr)) < 0) {
    KNX_LOG("begin(): IP_MULTICAST_IF failed errno=%d", errno);
  }
// Prepare sockaddr for send()
memset(&_mcastAddr, 0, sizeof(_mcastAddr));
_mcastAddr.sin_family = AF_INET;
_mcastAddr.sin_port   = htons(KNX_IP_UDP_PORT);
_mcastAddr.sin_addr   = maddr;

  // Non-blocking
  fcntl(_sock, F_SETFL, O_NONBLOCK);

  KNX_LOG("begin(): joined %u.%u.%u.%u:%u (sock=%d)",
          _maddr[0], _maddr[1], _maddr[2], _maddr[3],
          (unsigned)KNX_IP_UDP_PORT, _sock);_running = true;
  return true;
}

void KnxIpCore::end() {
  if (!_running) return;
  if (_sock >= 0) { ::close(_sock); _sock = -1; }
  _running = false;
}

void KnxIpCore::loop() {
  if (!_running) return;

  uint8_t buf[UDP_BUF_SIZE];
  struct sockaddr_in from; socklen_t flen = sizeof(from);
  int len = ::recvfrom(_sock, (char*)buf, sizeof(buf), 0, (struct sockaddr*)&from, &flen);
  if (len <= 0) return; // EWOULDBLOCK
  // debug
  knx_dump_hex("RX packet", buf, len);
  _handleIncoming(buf, len);
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
  if (!_running) {
    KNX_LOG("TX: not running, drop.");
    return false;
  }

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

  const uint8_t msgCode   = CEMI_LDATA_IND;
  const uint8_t addInfo   = 0x00;
  const uint8_t ctrl1     = CEMI_CTRL1_DEFAULT;
  const uint8_t ctrl2     = CEMI_CTRL2_GROUP_HC6;
  const uint16_t srcAddr  = _pa; // if 0, still acceptable in IP routing context

  KNX_LOG("TX: GA=%u/%u/%u (0x%04X) svc=%u asduLen=%u srcPA=%u.%u.%u",
          knxGaMain(ga), knxGaMiddle(ga), knxGaSub(ga), ga, (unsigned)svc, (unsigned)asduLen,
          (unsigned)((_pa>>12)&0x0F), (unsigned)((_pa>>8)&0x0F), (unsigned)(_pa&0xFF));

  bool oneBit = false;
  auto it = _gos.find(ga);
  if (it != _gos.end()) {
    oneBit = (it->second.dpt == DptMain::DPT_1xx);
  } else {
    // Fallback if GA wasn't registered: assume NOT 1-bit
    oneBit = false;
  }

  if (svc == KnxService::GroupValue_Read) {
  asdu = nullptr;
  asduLen = 0;
  // oneBit stays as determined; it won’t matter because no data follows.
  }

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
  uint8_t apduLenMinusTpdu0 = 1 + (oneBit ? 0 : asduLen);
  cemi[idx++] = apduLenMinusTpdu0;

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
  //  - service type (hi, lo) = 0x0530 (Routing Request)
  //  - total length (hi, lo) = 6 + cEMI length
  uint16_t totalLen = 6 + idx;
  frame[p++] = 0x06;
  frame[p++] = KNX_PROTOCOL_VERSION;
  frame[p++] = (uint8_t)(KNX_SVC_ROUTING_IND >> 8);
  frame[p++] = (uint8_t)(KNX_SVC_ROUTING_IND & 0xFF);
  frame[p++] = (uint8_t)(totalLen >> 8);
  frame[p++] = (uint8_t)(totalLen & 0xFF);

  // cEMI payload
  for (uint8_t i = 0; i < idx; ++i) frame[p++] = cemi[i];

  // debug: dump the whole KNXnet/IP Routing Request we're about to send
  knx_dump_hex("TX frame", frame, p);

  // ===== Send to 224.0.23.12:3671 =====
  ssize_t sent = ::sendto(_sock, frame, p, 0, (struct sockaddr*)&_mcastAddr, sizeof(_mcastAddr));
  if (sent != p) {
    _txErrors++;
    KNX_LOG("TX: sendto()=%d/%d errno=%d txErrors=%u", (int)sent, p, errno, (unsigned)_txErrors);
    return false;
  }
  KNX_LOG("TX: sent %d bytes (txPackets=%u)", p, (unsigned)(_txPackets+1));
  _txPackets++;
  return true;
}


// ======== Internal RX path ========
void KnxIpCore::_handleIncoming(const uint8_t* buf, int len) {
  // Validate KNXnet/IP header (min 6 bytes)
  if (len < 6) { _rxErrors++; KNX_LOG("RX: too short (%d).", len); return; }

  uint8_t  headerSize  = buf[0];
  uint8_t  proto       = buf[1];
  uint16_t svc         = (uint16_t(buf[2]) << 8) | buf[3];
  uint16_t totalLen    = (uint16_t(buf[4]) << 8) | buf[5];

if (headerSize != 0x06 || proto != KNX_PROTOCOL_VERSION) {
    _rxErrors++; KNX_LOG("RX: bad header: size=0x%02X proto=0x%02X.", headerSize, proto); return;
  }
  if (totalLen != len) {
    // Some stacks pad UDP; be lenient if totalLen <= len
    if (totalLen > len) { _rxErrors++; KNX_LOG("RX: totalLen(%u)>len(%d).", (unsigned)totalLen, len); return; }
  }

  // Routing Indication frames carry cEMI
  if (svc != KNX_SVC_ROUTING_IND) {
    // ignore other services (search response, tunneling, etc.)
    KNX_LOG("RX: ignore svc=0x%04X (not Routing_Ind).", (unsigned)svc);
    return;
  }

  // cEMI starts after 6 bytes
  if (len < 6 + 10) { // minimal cEMI header size check
    _rxErrors++; KNX_LOG("RX: cEMI too short (len=%d).", len); return;
  }
  const uint8_t* cemi = buf + 6;
  int cemiLen = len - 6;

  uint8_t msgCode = cemi[0];
  uint8_t addInfo = cemi[1];
  (void)addInfo; // we don't use additional info (0 usually)

  if (msgCode != CEMI_LDATA_IND ) {
    // We only care about L_Data.ind/req on routing
    KNX_LOG("RX: msgCode=0x%02X not L_Data.(ind|req).", msgCode);
    return;
  }

  if (cemiLen < 10) { _rxErrors++; KNX_LOG("RX: cEMI header truncated (cemiLen=%d).", cemiLen); return; }

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
    KNX_LOG("RX: dst=0x%04X not group (ctrl2=0x%02X).", dst, ctrl2);
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
      KNX_LOG("RX: APCI=0x%X not handled.", apci4);
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
  
  KNX_LOG("RX: src=%u.%u.%u dst=%u/%u/%u (0x%04X) apduLen=%u svc=%u lenASDU=%u",
          (unsigned)((src>>12)&0x0F), (unsigned)((src>>8)&0x0F), (unsigned)(src&0xFF),
          knxGaMain(dst), knxGaMiddle(dst), knxGaSub(dst), dst, (unsigned)apduLen, (unsigned)svcDetected, (unsigned)asduLen);

  auto itGo = _gos.find(dst);
  if (itGo != _gos.end()) {
    DptMain dpt = itGo->second.dpt;
    auto itCb = _callbacks.find(dst);
    if (itCb != _callbacks.end() && itCb->second) {
      itCb->second(dst, dpt, svcDetected, asdu, asduLen);
      KNX_LOG("RX: dispatched to GA 0x%04X callback.", dst);
    }
  } else {
    KNX_LOG("RX: no registered GA for 0x%04X.", dst);
  }

  (void)src; // available if you want to use/forward it
  _rxPackets++;
  KNX_LOG("RX: done (rxPackets=%u).", (unsigned)_rxPackets);
}


// ======== Compose and send APCI ========
bool KnxIpCore::_composeAndSendApci(uint16_t ga, KnxService svc, const uint8_t* asdu, uint8_t asduLen) {
  return sendCemiToGroup(ga, svc, asdu, asduLen);
}

void KnxIpCore::clearRegistrations() {
  _gos.clear();
  _callbacks.clear();
}


// ======== (Optional) convenience wrappers used by header ========
// (end of file)

