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
  // For safety clamp to 0..100; KNX 5.001 usually expects 0..100 domain.
  if (pct > 100) pct = 100;
  return pct;
}

uint8_t KnxIpCore::unpackScaling(const uint8_t* p, uint8_t len) {
  if (!p || len < 1) return 0;
  uint8_t v = p[0];
  if (v > 100) v = 100;
  return v;
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