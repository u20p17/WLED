#include "esp-knx-ip.h"
#ifndef UNIT_TEST
#include "src/dependencies/network/Network.h"
#endif

// ======== UDP Debug Support ========
#if KNX_UDP_DEBUG && !defined(UNIT_TEST)
#include <WiFiUdp.h>
static WiFiUDP debugUdp;
static IPAddress debugTarget;
static bool debugUdpInit = false;

static void initUdpDebug() {
  if (debugUdpInit) return;
  debugTarget = IPAddress(255, 255, 255, 255); // Broadcast initially
  debugUdp.begin(0); // Any available port
  debugUdpInit = true;
}

void sendUdpDebug(const char* fmt, ...) {
  if (!debugUdpInit) initUdpDebug();
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  
  debugUdp.beginPacket(debugTarget, 5140); // Port 5140 for debug
  debugUdp.print(buf);
  debugUdp.endPacket();
}
#else
void sendUdpDebug(const char* fmt, ...) { (void)fmt; }
#endif

// ======== Network abstraction for testing ========
class KnxNetworkInterface {
public:
#ifndef UNIT_TEST
    static IPAddress localIP() { return Network.localIP(); }
    static void localMAC(uint8_t* mac) { Network.localMAC(mac); }
    static bool isConnected() { return Network.isConnected(); }
#else
    static IPAddress localIP() { return WiFi.localIP(); }
    static void localMAC(uint8_t* mac) { WiFi.macAddress(mac); }
    static bool isConnected() { return WiFi.status() == WL_CONNECTED; }
#endif
};

// ======== Tunables / constants ========
static constexpr uint8_t  KNX_PROTOCOL_VERSION    = 0x10;   // KNXnet/IP proto version
static constexpr uint16_t KNX_SVC_ROUTING_IND     = 0x0530; // Routing Indication (rx)
static constexpr uint8_t  CEMI_LDATA_IND          = 0x29;   // cEMI L_Data.ind (rx)
static constexpr uint16_t KNX_SVC_SEARCH_REQ      = 0x0201; // SearchRequest
static constexpr uint16_t KNX_SVC_SEARCH_RES      = 0x0202; // SearchResponse
static constexpr uint16_t KNX_SVC_SEARCH_REQ_EXT  = 0x020B; // SearchRequestExtended
static constexpr uint16_t KNX_SVC_SEARCH_RES_EXT  = 0x020C; // SearchResponseExtended


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
  sendUdpDebug("[KNX] KnxIpCore::begin() called");
  if (_running) {
    sendUdpDebug("[KNX] Already running, returning true");
    return true;
  }
  if (!KnxNetworkInterface::isConnected()) {
    sendUdpDebug("[KNX] ERROR: Network not connected");
    KNX_LOG("begin(): Network not connected.");
    return false;
  }

  IPAddress localIP = KnxNetworkInterface::localIP();
  sendUdpDebug("[KNX] Local IP: %s", localIP.toString().c_str());

  // Create UDP socket
  _sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (_sock < 0) { 
    _rxErrors++; 
    sendUdpDebug("[KNX] ERROR: socket() failed errno=%d", errno);
    KNX_LOG("begin(): socket() failed errno=%d", errno); 
    return false; 
  }
  sendUdpDebug("[KNX] Socket created: %d", _sock);

  // Allow address reuse
  int yes = 1;
  (void)::setsockopt(_sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  // Bind to INADDR_ANY:3671
  struct sockaddr_in local; memset(&local, 0, sizeof(local));
  local.sin_family = AF_INET;
  local.sin_port   = htons(KNX_IP_UDP_PORT);
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  if (::bind(_sock, (struct sockaddr*)&local, sizeof(local)) < 0) {
    _rxErrors++; 
    sendUdpDebug("[KNX] ERROR: bind() failed errno=%d", errno);
    KNX_LOG("begin(): bind() failed errno=%d", errno); 
    ::close(_sock); _sock=-1; return false; 
  }
  sendUdpDebug("[KNX] Socket bound to port %d", KNX_IP_UDP_PORT);

in_addr maddr{};        maddr.s_addr = inet_addr("224.0.23.12");        // KNX group
in_addr ifaddr{};       ifaddr.s_addr = inet_addr(KnxNetworkInterface::localIP().toString().c_str());
_lastIfAddr = ifaddr;

sendUdpDebug("[KNX] Joining multicast 224.0.23.12 on interface %s", KnxNetworkInterface::localIP().toString().c_str());

// Join multicast group on all interfaces (or use ifaddr here if you prefer)
ip_mreq mreq{}; 
mreq.imr_multiaddr = maddr;
mreq.imr_interface = ifaddr;
if (::setsockopt(_sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
  _rxErrors++; 
  sendUdpDebug("[KNX] ERROR: IP_ADD_MEMBERSHIP failed errno=%d", errno);
  KNX_LOG("begin(): IP_ADD_MEMBERSHIP failed errno=%d", errno);
} else {
  sendUdpDebug("[KNX] Successfully joined multicast group");
}

// TTL=1 and LOOP=1 are fine…
uint8_t ttl = 1;  (void)::setsockopt(_sock, IPPROTO_IP, IP_MULTICAST_TTL,  &ttl,  sizeof(ttl));
uint8_t loop = 1; (void)::setsockopt(_sock, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));

// Pin outgoing multicast to the STA interface
  if (::setsockopt(_sock, IPPROTO_IP, IP_MULTICAST_IF, &ifaddr, sizeof(ifaddr)) < 0) {
    sendUdpDebug("[KNX] ERROR: IP_MULTICAST_IF failed errno=%d", errno);
    KNX_LOG("begin(): IP_MULTICAST_IF failed errno=%d", errno);
  } else {
    sendUdpDebug("[KNX] Multicast interface set successfully");
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
          (unsigned)KNX_IP_UDP_PORT, _sock);
  
  _running = true;
  sendUdpDebug("[KNX] KnxIpCore::begin() completed successfully, running=%d", _running);
  return true;
}

void KnxIpCore::end() {
  if (!_running) return;
  if (_sock >= 0) { ::close(_sock); _sock = -1; }
  _running = false;
}

bool KnxIpCore::rejoinMulticast() {
  if (!_running || _sock < 0) {
    KNX_LOG("rejoinMulticast(): core not running.");
    return false;
  }

  in_addr maddr{};  inet_aton("224.0.23.12", &maddr);
  in_addr ifaddr{}; inet_aton(KnxNetworkInterface::localIP().toString().c_str(), &ifaddr);

  // If interface changed, drop old membership first to avoid stale IGMP state
  if (_lastIfAddr.s_addr != 0 && _lastIfAddr.s_addr != ifaddr.s_addr) {
    ip_mreq drop{};
    drop.imr_multiaddr = maddr;
    drop.imr_interface = _lastIfAddr;
    (void)::setsockopt(_sock, IPPROTO_IP, IP_DROP_MEMBERSHIP, &drop, sizeof(drop));
  }

  // (Re)join multicast group on current interface
  ip_mreq mreq{};
  mreq.imr_multiaddr = maddr;
  mreq.imr_interface = ifaddr;
  if (::setsockopt(_sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
    KNX_LOG("rejoinMulticast(): IP_ADD_MEMBERSHIP failed errno=%d", errno);
    return false;
  }

  // Re-apply multicast socket options that some stacks reset
  uint8_t ttl = 1;  (void)::setsockopt(_sock, IPPROTO_IP, IP_MULTICAST_TTL,  &ttl,  sizeof(ttl));
  uint8_t loop = 1; (void)::setsockopt(_sock, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));
 

  // Pin outgoing multicast to Wi-Fi STA again (some drivers reset on config changes)
  if (::setsockopt(_sock, IPPROTO_IP, IP_MULTICAST_IF, &ifaddr, sizeof(ifaddr)) < 0) {
    KNX_LOG("rejoinMulticast(): IP_MULTICAST_IF failed errno=%d", errno);
    // not fatal – continue
  }
  _lastIfAddr = ifaddr;

  KNX_LOG("rejoinMulticast(): refreshed membership for 224.0.23.12 on %s",
          KnxNetworkInterface::localIP().toString().c_str());
  return true;
}

void KnxIpCore::loop() {
  if (!_running) return;

  uint8_t buf[UDP_BUF_SIZE];
  struct sockaddr_in from; socklen_t flen = sizeof(from);
  int len = ::recvfrom(_sock, (char*)buf, sizeof(buf), 0, (struct sockaddr*)&from, &flen);
  if (len <= 0) return; // EWOULDBLOCK
  
  sendUdpDebug("[KNX] Received packet: %d bytes from %d.%d.%d.%d", 
    len, 
    (int)((ntohl(from.sin_addr.s_addr) >> 24) & 0xFF),
    (int)((ntohl(from.sin_addr.s_addr) >> 16) & 0xFF), 
    (int)((ntohl(from.sin_addr.s_addr) >> 8) & 0xFF),
    (int)(ntohl(from.sin_addr.s_addr) & 0xFF));
  
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

  // ---- decide true 1-bit telegram by DPT, not by len ----
  bool oneBit = false;
  auto it = _gos.find(ga);
  if (it != _gos.end()) {
    oneBit = (it->second.dpt == DptMain::DPT_1xx);
  } else {
    // if GA unknown, be conservative: treat as multi-byte (no embedded bit)
    oneBit = false;
  }

  // Read has no ASDU
  if (svc == KnxService::GroupValue_Read) {
    asdu = nullptr;
    asduLen = 0;
  }

  // ---- build cEMI L_Data.ind ----
  uint8_t cemi[64];
  uint8_t idx = 0;

  const uint8_t msgCode = CEMI_LDATA_IND;
  const uint8_t addInfo = 0x00;

  cemi[idx++] = msgCode;
  cemi[idx++] = addInfo;
  cemi[idx++] = CEMI_CTRL1_DEFAULT;   // 0xBC
  cemi[idx++] = CEMI_CTRL2_GROUP_HC6; // 0xE0
  cemi[idx++] = (uint8_t)(_pa >> 8);
  cemi[idx++] = (uint8_t)(_pa & 0xFF);
  cemi[idx++] = (uint8_t)(ga >> 8);
  cemi[idx++] = (uint8_t)(ga & 0xFF);

  // TPDU header
  const uint8_t tpdu0 = 0x00; // UDT group
  uint8_t tpdu1 = 0x00;
  switch (svc) {
    case KnxService::GroupValue_Read:     tpdu1 = (0b00 << 6); break;
    case KnxService::GroupValue_Response: tpdu1 = (0b01 << 6); break;
    case KnxService::GroupValue_Write:    tpdu1 = (0b10 << 6); break;
  }

  // APDU bytes (incl. TPDU0 + TPDU1 + payload); cEMI "length" is APDU-1
  const uint8_t apduBytes = oneBit ? 2 : (uint8_t)(2 + asduLen);
  cemi[idx++] = (uint8_t)(apduBytes - 1);

  // Write TPDU and payload
  cemi[idx++] = tpdu0;
  if (oneBit && asdu) {
    // embed the bit into TPDU[1] LSB (only for DPT_1xx)
    const uint8_t bit = (asdu[0] & 0x01);
    cemi[idx++] = (uint8_t)(tpdu1 | bit);
  } else {
    // multi-byte payload (or read/unknown GA)
    cemi[idx++] = tpdu1;
    for (uint8_t i = 0; i < asduLen; ++i) cemi[idx++] = asdu[i];
  }

  // ---- KNXnet/IP routing wrapper ----
  uint8_t frame[96]; uint8_t p = 0;
  const uint16_t totalLen = (uint16_t)(6 + idx);

  frame[p++] = 0x06;                               // header size
  frame[p++] = KNX_PROTOCOL_VERSION;               // 0x10
  frame[p++] = (uint8_t)(KNX_SVC_ROUTING_IND >> 8);
  frame[p++] = (uint8_t)(KNX_SVC_ROUTING_IND & 0xFF);
  frame[p++] = (uint8_t)(totalLen >> 8);
  frame[p++] = (uint8_t)(totalLen & 0xFF);

  for (uint8_t i = 0; i < idx; ++i) frame[p++] = cemi[i];

  // debug (optional)
  knx_dump_hex("TX frame", frame, p);

  // ---- send to 224.0.23.12:3671 ----
  // Tasmota-style redundancy: send the same frame multiple times if enabled
  const uint8_t repeats = _enhanced ? _enhancedSendCount : 1;
  for (uint8_t i = 0; i < repeats; ++i) {
    ssize_t sent = ::sendto(_sock, frame, p, 0, (struct sockaddr*)&_mcastAddr, sizeof(_mcastAddr));
    if (sent != p) {
      _txErrors++;
      KNX_LOG("TX: sendto()=%d/%d errno=%d txErrors=%u", (int)sent, p, errno, (unsigned)_txErrors);
      return false;
    }
    _txPackets++;
    KNX_LOG("TX: sent %d bytes rpt=%u/%u (txPackets=%u)", p, (unsigned)(i+1), (unsigned)repeats, (unsigned)_txPackets);
    if (_enhancedGapMs) delay(_enhancedGapMs);
  }
  return true;
}

// ======== Internal RX path ========
bool KnxIpCore::_sendSearchResponse(bool extended, const uint8_t* req, int reqLen) {
  // --- Ziel bestimmen: Unicast zurück an den in der Anfrage angegebenen HPAI ---
  // Für 0x0201/0x020B steht ab Offset 6 die HPAI (Len=8, IPv4/UDP, IP, Port)
  struct sockaddr_in to{}; memset(&to, 0, sizeof(to));
  to.sin_family = AF_INET;
  if (reqLen >= 14 && req[0]==0x06 && req[1]==KNX_PROTOCOL_VERSION && req[6]==0x08 && req[7]==0x01) {
    // IP (8..11), Port (12..13)
    uint32_t ip;   memcpy(&ip,   &req[8],  4);
    uint16_t port; memcpy(&port, &req[12], 2);
    to.sin_addr.s_addr = ip;
    to.sin_port        = *(uint16_t*)&req[12]; // Netzwerk-Byteordnung belassen
  } else {
    // Fallback (sollte nicht nötig sein)
    to.sin_addr.s_addr = inet_addr("255.255.255.255");
    to.sin_port        = htons(KNX_IP_UDP_PORT);
  }

  // --- HPAI (Control Endpoint) unseres Geräts ---
  uint8_t hpai[8];
  hpai[0] = 0x08; hpai[1] = 0x01; // len, IPv4/UDP
  IPAddress ip = KnxNetworkInterface::localIP();
  hpai[2] = ip[0]; hpai[3] = ip[1]; hpai[4] = ip[2]; hpai[5] = ip[3];
  hpai[6] = (uint8_t)(KNX_IP_UDP_PORT >> 8);
  hpai[7] = (uint8_t)(KNX_IP_UDP_PORT & 0xFF);

  // --- DIB Device Info (Typ 0x01, fixe Länge 0x36 für Einfachheit) ---
  uint8_t dib_dev[0x36]; memset(dib_dev, 0, sizeof(dib_dev));
  dib_dev[0] = 0x36; dib_dev[1] = 0x01; // len, type
  dib_dev[2] = 0x20; // Medium IP
  dib_dev[3] = 0x00; // Device status OK
  // PA (falls gesetzt, sonst 0)
  dib_dev[4] = (uint8_t)(_pa >> 8);
  dib_dev[5] = (uint8_t)(_pa & 0xFF);
  // Project/Installation ID:
  dib_dev[6] = 0x00; dib_dev[7] = 0x00;
  // Seriennummer (MAC)
  uint8_t mac[6]; KnxNetworkInterface::localMAC(mac);
  memcpy(&dib_dev[8], mac, 6);
  // Multicast 224.0.23.12
  dib_dev[14] = 224; dib_dev[15] = 0; dib_dev[16] = 23; dib_dev[17] = 12;
  // MAC again
  memcpy(&dib_dev[18], mac, 6);
  // Friendly Name (max 30B inkl. 0)
  extern char serverDescription[33]; // kommt aus WLED core
  const char* fallbackName = "WLED KNX";
  const char* name = (serverDescription[0] != '\0') ? serverDescription : fallbackName;
  strncpy((char*)&dib_dev[24], name, 30);
  dib_dev[24 + 29] = '\0'; // sicherstellen, dass immer terminiert
  // (Rest bleibt 0 als Padding)

  // --- DIB Supported Service Families (Typ 0x02) ---
  // Minimal: Core v1, Routing v1
  uint8_t dib_svc[10]; memset(dib_svc, 0, sizeof(dib_svc));
  dib_svc[0] = 0x0A; dib_svc[1] = 0x02;
  dib_svc[2] = 0x02; dib_svc[3] = 0x01; // Core v1
  dib_svc[4] = 0x05; dib_svc[5] = 0x01; // Routing v1
  // Optional: kein Tunneling (lassen wir weg / v0)

  // (Optional) Bei Extended Request könnte man zusätzliche DIBs anfügen.
  // Für routing-only genügt die gleiche Antwort.

  // --- KNXnet/IP Header ---
  // 06 10 02 02/0C <len>
  const uint16_t svc = extended ? KNX_SVC_SEARCH_RES_EXT : KNX_SVC_SEARCH_RES;
  const size_t payloadLen = 6 + sizeof(hpai) + sizeof(dib_dev) + sizeof(dib_svc);
  uint8_t* pkt = (uint8_t*)malloc(payloadLen);
  if (!pkt) { KNX_LOG("SearchResponse: OOM"); return false; }

  size_t p = 0;
  pkt[p++] = 0x06; pkt[p++] = KNX_PROTOCOL_VERSION;
  pkt[p++] = (uint8_t)(svc >> 8);
  pkt[p++] = (uint8_t)(svc & 0xFF);
  pkt[p++] = (uint8_t)(payloadLen >> 8);
  pkt[p++] = (uint8_t)(payloadLen & 0xFF);

  memcpy(&pkt[p], hpai, sizeof(hpai));       p += sizeof(hpai);
  memcpy(&pkt[p], dib_dev, sizeof(dib_dev)); p += sizeof(dib_dev);
  memcpy(&pkt[p], dib_svc, sizeof(dib_svc)); p += sizeof(dib_svc);

  // --- Senden ---
  ssize_t sent = ::sendto(_sock, pkt, payloadLen, 0, (struct sockaddr*)&to, sizeof(to));
  bool ok = (sent == (ssize_t)payloadLen);
  if (!ok) _txErrors++;
  else     _txPackets++;
  KNX_LOG("TX: %s (%u bytes) to %s:%u",
          extended ? "SearchResponseExtended" : "SearchResponse",
          (unsigned)payloadLen,
          ip.toString().c_str(), KNX_IP_UDP_PORT);
  free(pkt);
  return ok;
}

void KnxIpCore::_handleIncoming(const uint8_t* buf, int len) {
  // Validate KNXnet/IP header (min 6 bytes)
  if (len < 6) { _rxErrors++; KNX_LOG("RX: too short (%d).", len); return; }

  const uint8_t  headerSize = buf[0];
  const uint8_t  proto      = buf[1];
  const uint16_t svc        = (uint16_t(buf[2]) << 8) | buf[3];
  const uint16_t totalLen   = (uint16_t(buf[4]) << 8) | buf[5];

  if (headerSize != 0x06 || proto != KNX_PROTOCOL_VERSION) {
    _rxErrors++; KNX_LOG("RX: bad header: size=0x%02X proto=0x%02X.", headerSize, proto); return;
  }
  if (totalLen != len) {
    if (totalLen > len) { _rxErrors++; KNX_LOG("RX: totalLen(%u)>len(%d).", (unsigned)totalLen, len); return; }
  }

  // Accept SearchRequest(Extended) and Routing Indication
  if (svc == KNX_SVC_SEARCH_REQ || svc == KNX_SVC_SEARCH_REQ_EXT) {
    _sendSearchResponse(svc == KNX_SVC_SEARCH_REQ_EXT, buf, len);
    return;
  }

  if (svc != KNX_SVC_ROUTING_IND) {
    KNX_LOG("RX: ignore svc=0x%04X (not Routing_Ind).", (unsigned)svc);
    return;
  }


  if (len < 6 + 10) { _rxErrors++; KNX_LOG("RX: cEMI too short (len=%d).", len); return; }

  const uint8_t* cemi = buf + 6;
  const int      cemiLen = len - 6;

  const uint8_t msgCode = cemi[0];
  const uint8_t addInfo = cemi[1]; (void)addInfo;

  if (msgCode != CEMI_LDATA_IND) {
    KNX_LOG("RX: msgCode=0x%02X not L_Data.ind.", msgCode);
    return;
  }
  if (cemiLen < 10) { _rxErrors++; KNX_LOG("RX: cEMI header truncated (cemiLen=%d).", cemiLen); return; }

  const uint8_t ctrl1 = cemi[2];
  const uint8_t ctrl2 = cemi[3];
  (void)ctrl1;

  const uint16_t src = (uint16_t(cemi[4]) << 8) | cemi[5];
  const uint16_t dst = (uint16_t(cemi[6]) << 8) | cemi[7];

  // cEMI [8] = APDU length minus 1
  const uint8_t apduLenMinus1 = cemi[8];
  const uint8_t apduBytes     = uint8_t(apduLenMinus1 + 1); // TPDU0+TPDU1+payload

  // TPDU starts at cemi[9]
  if (cemiLen < 9 + apduBytes) { _rxErrors++; KNX_LOG("RX: TPDU truncated (need %u, have %d).", (unsigned)apduBytes, cemiLen - 9); return; }
  const uint8_t* tpdu = cemi + 9;
  if (apduBytes < 2) { _rxErrors++; KNX_LOG("RX: APDU < 2 bytes."); return; }

  // Must be group address (ctrl2 bit7)
  const bool isGroup = (ctrl2 & 0x80) != 0;
  if (!isGroup) {
    KNX_LOG("RX: dst=0x%04X not group (ctrl2=0x%02X).", dst, ctrl2);
    return;
  }

  // Drop our own multicast (mirrored GA protection)
  if (src == _pa && _pa != 0) {
    KNX_LOG("RX: own frame (src=%u.%u.%u) ignored.",
      (unsigned)((src>>12)&0x0F), (unsigned)((src>>8)&0x0F), (unsigned)(src&0xFF));
    return;
  }

  // ---------- APCI & service ----------
  const uint8_t apci4 = uint8_t(((tpdu[0] & 0x03) << 2) | ((tpdu[1] & 0xC0) >> 6));
  KnxService svcDetected;
  switch (apci4) {
    case 0x0: svcDetected = KnxService::GroupValue_Read;     break;
    case 0x1: svcDetected = KnxService::GroupValue_Response; break;
    case 0x2: svcDetected = KnxService::GroupValue_Write;    break;
    default:
      KNX_LOG("RX: APCI=0x%X not handled.", apci4);
      return;
  }

  // ---------- ASDU extraction ----------
  const uint8_t* asdu  = nullptr;
  uint8_t        asduLen = 0;

  if (svcDetected == KnxService::GroupValue_Write && apduBytes == 2) {
    static uint8_t one;
    one = (tpdu[1] & 0x01);
    asdu = &one; asduLen = 1;
  } else if (apduBytes > 2) {
    asdu = tpdu + 2; asduLen = uint8_t(apduBytes - 2);
  }

  KNX_LOG("RX: src=%u.%u.%u dst=%u/%u/%u (0x%04X) apduBytes=%u svc=%u lenASDU=%u",
          (unsigned)((src>>12)&0x0F), (unsigned)((src>>8)&0x0F), (unsigned)(src&0xFF),
          knxGaMain(dst), knxGaMiddle(dst), knxGaSub(dst), dst,
          (unsigned)apduBytes, (unsigned)svcDetected, (unsigned)asduLen);

  // ---------- Communication enhancement: RX de-dup + toggle throttle ----------
  if (_enhanced) {
    const uint32_t now = millis();

    // Build a small signature across src, dst, apci, and up to 4 bytes of payload
    uint32_t sig = ((uint32_t)src << 16) ^ (uint32_t)dst ^ ((uint32_t)apci4 << 28);
    if (asdu && asduLen) {
      uint32_t d = 0; memcpy(&d, asdu, (asduLen >= 4 ? 4 : asduLen));
      sig ^= _mix32(d + ((uint32_t)asduLen << 24));
    }
    sig = _mix32(sig);

    // Deduplicate within window
    for (size_t i = 0; i < _rxSeen.size(); ++i) {
      const RxSig &r = _rxSeen[i];
      if (r.sig == sig && (now - r.ts) <= _rxDedupWindowMs) {
        KNX_LOG("RX: duplicate suppressed (sig=0x%08X, %ums)", (unsigned)sig, (unsigned)(now - r.ts));
        return;
      }
    }
    _rxSeen[_rxSeenIdx] = RxSig{sig, now};
    _rxSeenIdx = (_rxSeenIdx + 1) % _rxSeen.size();

    // Toggle throttling for 1-bit writes (ignore second toggle <1s)
    if (svcDetected == KnxService::GroupValue_Write && asdu && asduLen == 1) {
      const uint8_t bit = (asdu[0] & 0x01);
      auto &st = _rxBitCache[dst];
      if (st.ts != 0 && (now - st.ts) < 1000 && st.v != bit) {
        KNX_LOG("RX: toggle throttled on GA 0x%04X (%u->%u in %u ms)", dst, st.v, bit, (unsigned)(now - st.ts));
        return;
      }
      st.v = bit; st.ts = now;
    }
  }

  // ---------- Dispatch ----------
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

