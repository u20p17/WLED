// esp-knx-ip-conversion.cpp
// ESP32-only DPT conversion helpers for ESP-KNX-IP (no web / no storage)

#include "esp-knx-ip.h"
#include <math.h>

// ===== DPT 1.xxx (1 bit) =====
uint8_t KnxIpCore::pack1Bit(bool v) {
  return v ? 0x01 : 0x00;
}

bool KnxIpCore::unpack1Bit(const uint8_t* p, uint8_t len) {
  if (!p || len < 1) return false;
  return (p[0] & 0x01) != 0;
}

// ===== DPT 5.001 (Scaling 0..100%) =====
uint8_t KnxIpCore::packScaling(uint8_t pct) {
  if (pct > 100) pct = 100;
  // 0..100% → 0..255 with rounding
  return (uint8_t)(((uint16_t)pct * 255u + 50u) / 100u);
}

uint8_t KnxIpCore::unpackScaling(const uint8_t* p, uint8_t len) {
  if (!p || len < 1) return 0;
  uint8_t raw = p[0]; // 0..255
  // 0..255 → 0..100% with rounding
  return (uint8_t)(((uint16_t)raw * 100u + 127u) / 255u);
}

// DPT 9.xxx (2-byte float, EIS5) implementation
// Format: S EEEE MMMMMMMMMMM (1,4,11)
// Value = 0.01 * M * 2^E   (M is signed 11-bit)

void KnxIpCore::pack2ByteFloat(float value, uint8_t out[2]) {
  if (!out) return;
  if (isnan(value) || isinf(value)) { out[0] = 0; out[1] = 0; return; }

  int sign = (value < 0.f) ? 1 : 0;
  float v = fabsf(value);

  // Convert to mantissa with 0.01 resolution
  int mant = (int)lroundf(v / 0.01f);
  int exp = 0;

  // Normalize so mant fits into 11 bits (signed)
  while (mant > 0x07FF) { mant >>= 1; ++exp; }
  if (exp > 0x0F) { // out of range, clamp
    exp = 0x0F;
    mant = 0x07FF;
  }

  // Apply sign to 11-bit mantissa
  if (sign) mant = (-mant) & 0x07FF;

  uint16_t raw = (uint16_t(sign) << 15) | (uint16_t(exp & 0x0F) << 11) | uint16_t(mant & 0x07FF);
  out[0] = (raw >> 8) & 0xFF;
  out[1] = (raw) & 0xFF;
}

float KnxIpCore::unpack2ByteFloat(const uint8_t* p, uint8_t len) {
  if (!p || len < 2) return 0.f;

  uint16_t raw = (uint16_t(p[0]) << 8) | uint16_t(p[1]);

  int sign = (raw & 0x8000) ? -1 : 1;
  int exp  = (raw >> 11) & 0x0F;
  int mant = (raw & 0x07FF);

  // sign-extend 11-bit mantissa
  if (mant & 0x0400) mant |= 0xF800;

  float val = float(mant) * powf(2.0f, float(exp)) * 0.01f;
  return (float)sign * val;
}

void KnxIpCore::pack4ByteFloat(float value, uint8_t out[4]) {
  union { float f; uint32_t u; } v;
  v.f = value;                     // IEEE-754 on ESP32
  out[0] = (uint8_t)(v.u >> 24);   // big-endian
  out[1] = (uint8_t)(v.u >> 16);
  out[2] = (uint8_t)(v.u >> 8);
  out[3] = (uint8_t)(v.u);
}

float KnxIpCore::unpack4ByteFloat(const uint8_t* p, uint8_t len) {
  if (!p || len < 4) return NAN;
  union { float f; uint32_t u; } v;
  v.u = (uint32_t(p[0])<<24) | (uint32_t(p[1])<<16) | (uint32_t(p[2])<<8) | uint32_t(p[3]);
  return v.f;
}