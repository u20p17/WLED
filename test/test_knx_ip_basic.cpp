#ifdef UNIT_TEST
#include <unity.h>
#include <stdint.h>
#include <stdio.h>
#include <cstdlib>
#include <cmath>

extern "C" {
  uint16_t knx_test_parseGA(const char* s);
  uint16_t knx_test_parsePA(const char* s);
  uint8_t  knx_test_step_pct(uint8_t sc);
  int16_t  knx_test_step_delta(uint8_t nibble, uint16_t maxVal);
  void     knx_test_rgbToHsv(uint8_t r,uint8_t g,uint8_t b,float& h,float& s,float& v);
  void     knx_test_hsvToRgb(float h,float s,float v,uint8_t& r,uint8_t& g,uint8_t& b);
  void     knx_test_white_split(uint8_t w,uint8_t cct,int16_t delta,int adjustWarm,uint8_t* outW,uint8_t* outCct);
  void     knx_test_rgb_rel(uint8_t r,uint8_t g,uint8_t b,uint8_t rCtl,uint8_t gCtl,uint8_t bCtl,uint8_t* or_,uint8_t* og_,uint8_t* ob_);
  void     knx_test_hsv_rel(uint8_t r,uint8_t g,uint8_t b,uint8_t hCtl,uint8_t sCtl,uint8_t vCtl,uint8_t* or_,uint8_t* og_,uint8_t* ob_);
  void     knx_test_rgbw_rel(uint8_t r,uint8_t g,uint8_t b,uint8_t w,uint8_t rCtl,uint8_t gCtl,uint8_t bCtl,uint8_t wCtl,uint8_t* or_,uint8_t* og_,uint8_t* ob_,uint8_t* ow_);
}

void test_parseGA_valid() {
  TEST_ASSERT_EQUAL_UINT16( (1U<<11)|(2U<<8)|3U, knx_test_parseGA("1/2/3") );
  TEST_ASSERT_EQUAL_UINT16( (31U<<11)|(7U<<8)|255U, knx_test_parseGA("31/7/255") );
}

void test_parseGA_invalid() {
  TEST_ASSERT_EQUAL_UINT16(0, knx_test_parseGA(""));
  TEST_ASSERT_EQUAL_UINT16(0, knx_test_parseGA("a/b/c"));
  TEST_ASSERT_EQUAL_UINT16(0, knx_test_parseGA("32/1/1")); // main out of range
  TEST_ASSERT_EQUAL_UINT16(0, knx_test_parseGA("1/8/1"));  // middle out of range
  TEST_ASSERT_EQUAL_UINT16(0, knx_test_parseGA("1/1/256")); // sub out of range
  TEST_ASSERT_EQUAL_UINT16(0, knx_test_parseGA("1/1")); // missing field
}

void test_step_pct_mapping() {
  TEST_ASSERT_EQUAL_UINT8(100, knx_test_step_pct(1));
  TEST_ASSERT_EQUAL_UINT8(50,  knx_test_step_pct(2));
  TEST_ASSERT_EQUAL_UINT8(25,  knx_test_step_pct(3));
  TEST_ASSERT_EQUAL_UINT8(12,  knx_test_step_pct(4));
  TEST_ASSERT_EQUAL_UINT8(6,   knx_test_step_pct(5));
  TEST_ASSERT_EQUAL_UINT8(3,   knx_test_step_pct(6));
  TEST_ASSERT_EQUAL_UINT8(1,   knx_test_step_pct(7));
  TEST_ASSERT_EQUAL_UINT8(0,   knx_test_step_pct(0)); // stop / invalid scale code
}

void test_step_delta_inc_dec() {
  // nibble: bit3=dir (1=up), low3 bits scale code
  // Up 100%
  TEST_ASSERT_GREATER_THAN(0, knx_test_step_delta(0x8 | 1, 255));
  // Down 100%
  TEST_ASSERT_LESS_THAN(0, knx_test_step_delta(0x0 | 1, 255));
  // Stop yields 0
  TEST_ASSERT_EQUAL_INT16(0, knx_test_step_delta(0x0, 255));
}

void test_hsv_roundtrip_primary_colors() {
  const uint8_t primaries[3][3] = { {255,0,0}, {0,255,0}, {0,0,255} };
  for (int i=0;i<3;i++) {
    float h,s,v; uint8_t r2,g2,b2; 
    knx_test_rgbToHsv(primaries[i][0], primaries[i][1], primaries[i][2], h,s,v);
    knx_test_hsvToRgb(h,s,v,r2,g2,b2);
  TEST_ASSERT_LESS_OR_EQUAL_UINT8(1, (uint8_t)std::abs((int)primaries[i][0] - (int)r2));
  TEST_ASSERT_LESS_OR_EQUAL_UINT8(1, (uint8_t)std::abs((int)primaries[i][1] - (int)g2));
  TEST_ASSERT_LESS_OR_EQUAL_UINT8(1, (uint8_t)std::abs((int)primaries[i][2] - (int)b2));
  }
}

void test_white_split_increase_warm() {
  uint8_t outW,outCct; knx_test_white_split(100,128,20,1,&outW,&outCct);
  TEST_ASSERT_TRUE(outW >= 100); // warm increase should not reduce total
  TEST_ASSERT_TRUE(outCct <= 200); // CCT may shift slightly toward warm (lower or near original)
}

void test_white_split_decrease_cold() {
  uint8_t outW,outCct; knx_test_white_split(150,200,-30,0,&outW,&outCct);
  TEST_ASSERT_TRUE(outW <= 150);
  // if total not zero, CCT should not exceed 255
  if(outW>0) TEST_ASSERT_TRUE(outCct <= 255);
}

void test_white_split_zero_no_negative() {
  uint8_t outW,outCct; knx_test_white_split(0,180,-10,1,&outW,&outCct);
  TEST_ASSERT_EQUAL_UINT8(0,outW);
  TEST_ASSERT_EQUAL_UINT8(180,outCct); // ratio preserved when all off
}

void test_rgb_composite_rel() {
  uint8_t r,g,b; knx_test_rgb_rel(100,150,200, 0x9,0x0,0x2, &r,&g,&b); // +stop-
  TEST_ASSERT_TRUE(r>100);
  TEST_ASSERT_EQUAL_UINT8(150,g);
  TEST_ASSERT_TRUE(b<200);
}

void test_hsv_composite_rel() {
  uint8_t r,g,b; knx_test_hsv_rel(255,0,0, 0x1,0x6,0x6, &r,&g,&b); // -h, -s, -v
  // Expect red still dominant, but brightness reduced a bit and some desaturation may introduce small g/b.
  TEST_ASSERT_TRUE(r >= g && r >= b);           // red channel remains max
  TEST_ASSERT_TRUE(r <= 255);                   // still within range
  TEST_ASSERT_TRUE(r < 255 || (g==0 && b==0));  // if still full red, no g/b introduced
}

void test_rgbw_composite_rel() {
  uint8_t r,g,b,w; knx_test_rgbw_rel(10,20,30,40, 0x9,0x9,0x2,0x6, &r,&g,&b,&w); // ++--
  TEST_ASSERT_TRUE(r>10 && g>20 && b<30 && w<40);
}

// Boundary & no-op guard tests
void test_rgb_rel_increase_clamp_at_255() {
  uint8_t r,g,b; knx_test_rgb_rel(250,0,0, 0x9,0x0,0x0, &r,&g,&b); // +100% on red
  TEST_ASSERT_EQUAL_UINT8(255,r);
  TEST_ASSERT_EQUAL_UINT8(0,g);
  TEST_ASSERT_EQUAL_UINT8(0,b);
}

void test_rgb_rel_decrease_clamp_at_0() {
  uint8_t r,g,b; knx_test_rgb_rel(5,10,15, 0x1,0x0,0x0, &r,&g,&b); // -100% on red
  TEST_ASSERT_EQUAL_UINT8(0,r);
  TEST_ASSERT_EQUAL_UINT8(10,g);
  TEST_ASSERT_EQUAL_UINT8(15,b);
}

void test_rgb_rel_all_stop_noop() {
  uint8_t r,g,b; knx_test_rgb_rel(100,150,200, 0x0,0x0,0x0, &r,&g,&b); // all stop
  TEST_ASSERT_EQUAL_UINT8(100,r);
  TEST_ASSERT_EQUAL_UINT8(150,g);
  TEST_ASSERT_EQUAL_UINT8(200,b);
}

void test_white_split_overflow_clamp() {
  uint8_t outW,outCct; knx_test_white_split(250,128,40,1,&outW,&outCct); // push warm beyond 255 total
  TEST_ASSERT_EQUAL_UINT8(255,outW);
}

void test_step_delta_minimum_one() {
  // Small maxVal with 1% scale must still yield magnitude 1
  TEST_ASSERT_EQUAL_INT16(1, knx_test_step_delta(0x8 | 7, 5));  // +1%
  TEST_ASSERT_EQUAL_INT16(-1, knx_test_step_delta(0x0 | 7, 5)); // -1%
}

// Hue wrap tests
void test_hsv_rel_hue_wrap_negative() {
  // Start at hue 5, subtract 30 -> wrap to 335
  uint8_t r0,g0,b0; knx_test_hsvToRgb(5.f,1.f,1.f,r0,g0,b0);
  uint8_t r1,g1,b1; knx_test_hsv_rel(r0,g0,b0, 0x1,0x0,0x0, &r1,&g1,&b1); // -100% hue (30 deg)
  float h,s,v; knx_test_rgbToHsv(r1,g1,b1,h,s,v);
  TEST_ASSERT_TRUE(h >= 330.f && h < 360.f);
  TEST_ASSERT_TRUE(s > 0.90f);
  TEST_ASSERT_TRUE(v > 0.90f);
}

void test_hsv_rel_hue_wrap_positive() {
  // Start at hue 355, add 30 -> wrap to 25
  uint8_t r0,g0,b0; knx_test_hsvToRgb(355.f,1.f,1.f,r0,g0,b0);
  uint8_t r1,g1,b1; knx_test_hsv_rel(r0,g0,b0, 0x9,0x0,0x0, &r1,&g1,&b1); // +100% hue (30 deg)
  float h,s,v; knx_test_rgbToHsv(r1,g1,b1,h,s,v);
  TEST_ASSERT_TRUE(h >= 20.f && h <= 30.f);
  TEST_ASSERT_TRUE(s > 0.90f);
  TEST_ASSERT_TRUE(v > 0.90f);
}

void test_hsv_rel_sv_clamp() {
  // Start with mid HSV, drive S and V upward beyond bounds
  uint8_t r0,g0,b0; knx_test_hsvToRgb(120.f,0.8f,0.9f,r0,g0,b0); // greenish
  uint8_t r1,g1,b1; knx_test_hsv_rel(r0,g0,b0, 0x0, 0x9, 0x9, &r1,&g1,&b1); // +S +V
  float h,s,v; knx_test_rgbToHsv(r1,g1,b1,h,s,v);
  TEST_ASSERT_TRUE(s <= 1.0001f); // saturated at bound
  TEST_ASSERT_TRUE(v <= 1.0001f); // saturated at bound
  TEST_ASSERT_TRUE(h >= 110.f && h <= 130.f); // hue preserved
}

void test_hsv_rel_multi_hue_wrap() {
  // Apply four +30° steps (~120° shift)
  uint8_t r,g,b; knx_test_hsvToRgb(45.f,1.f,1.f,r,g,b);
  for(int i=0;i<4;i++) {
    uint8_t nr,ng,nb; knx_test_hsv_rel(r,g,b, 0x9,0x0,0x0, &nr,&ng,&nb); r=nr; g=ng; b=nb; }
  float h,s,v; knx_test_rgbToHsv(r,g,b,h,s,v);
  // Expected hue ~45+120=165
  TEST_ASSERT_TRUE(h >= 150.f && h <= 180.f);
  TEST_ASSERT_TRUE(s > 0.85f && v > 0.85f);
}

void test_hsv_rel_all_stop_noop() {
  uint8_t r0,g0,b0; knx_test_hsvToRgb(200.f,0.5f,0.6f,r0,g0,b0);
  uint8_t r1,g1,b1; knx_test_hsv_rel(r0,g0,b0, 0x0,0x0,0x0, &r1,&g1,&b1);
  TEST_ASSERT_EQUAL_UINT8(r0,r1);
  TEST_ASSERT_EQUAL_UINT8(g0,g1);
  TEST_ASSERT_EQUAL_UINT8(b0,b1);
}

void test_rgbw_rel_all_stop_noop() {
  uint8_t r,g,b,w; knx_test_rgbw_rel(12,34,56,78, 0x0,0x0,0x0,0x0, &r,&g,&b,&w);
  TEST_ASSERT_EQUAL_UINT8(12,r);
  TEST_ASSERT_EQUAL_UINT8(34,g);
  TEST_ASSERT_EQUAL_UINT8(56,b);
  TEST_ASSERT_EQUAL_UINT8(78,w);
}

// Additional edge tests
void test_hsv_rel_sv_negative_clamp() {
  // Start with low S and V then apply negative steps
  uint8_t r0,g0,b0; knx_test_hsvToRgb(300.f,0.05f,0.06f,r0,g0,b0);
  // Use -100% codes (dir bit 0, scale 1)
  uint8_t r1,g1,b1; knx_test_hsv_rel(r0,g0,b0, 0x0, 0x1, 0x1, &r1,&g1,&b1);
  float h,s,v; knx_test_rgbToHsv(r1,g1,b1,h,s,v);
  // Ensure both decreased (not necessarily to zero due to fixed step size) and remain non-negative
  float s0=0.05f, v0=0.06f;
  TEST_ASSERT_TRUE(s <= s0 && s >= 0.f);
  TEST_ASSERT_TRUE(v <= v0 && v >= 0.f);
  // When saturation drops to zero hue becomes undefined; just assert range 0..360
  TEST_ASSERT_TRUE(h >= 0.f && h <= 360.f);
}

void test_hsv_rel_multi_cycle_hue_identity() {
  // 12 * +30° = 360° should return near original hue
  const float startHue = 77.f;
  uint8_t r,g,b; knx_test_hsvToRgb(startHue,1.f,1.f,r,g,b);
  for(int i=0;i<12;i++){ uint8_t nr,ng,nb; knx_test_hsv_rel(r,g,b, 0x9,0x0,0x0, &nr,&ng,&nb); r=nr; g=ng; b=nb; }
  float h,s,v; knx_test_rgbToHsv(r,g,b,h,s,v);
  // Allow small numeric drift
  TEST_ASSERT_TRUE(h >= startHue-3.f && h <= startHue+3.f);
  TEST_ASSERT_TRUE(s > 0.90f && v > 0.90f);
}

void test_hue_min_step_delta() {
  // With maxVal=30 and scale code 7 (1%) expect magnitude 1
  TEST_ASSERT_EQUAL_INT16(1, knx_test_step_delta(0x8 | 7, 30));
  TEST_ASSERT_EQUAL_INT16(-1, knx_test_step_delta(0x0 | 7, 30));
}

#ifndef ARDUINO
int main() {
  UNITY_BEGIN();
  RUN_TEST(test_parseGA_valid);
  RUN_TEST(test_parseGA_invalid);
  RUN_TEST(test_step_pct_mapping);
  RUN_TEST(test_step_delta_inc_dec);
  RUN_TEST(test_hsv_roundtrip_primary_colors);
  RUN_TEST(test_white_split_increase_warm);
  RUN_TEST(test_white_split_decrease_cold);
  RUN_TEST(test_white_split_zero_no_negative);
  RUN_TEST(test_rgb_composite_rel);
  RUN_TEST(test_hsv_composite_rel);
  RUN_TEST(test_rgbw_composite_rel);
  RUN_TEST(test_rgb_rel_increase_clamp_at_255);
  RUN_TEST(test_rgb_rel_decrease_clamp_at_0);
  RUN_TEST(test_rgb_rel_all_stop_noop);
  RUN_TEST(test_white_split_overflow_clamp);
  RUN_TEST(test_step_delta_minimum_one);
  RUN_TEST(test_hsv_rel_hue_wrap_negative);
  RUN_TEST(test_hsv_rel_hue_wrap_positive);
  RUN_TEST(test_hsv_rel_sv_clamp);
  RUN_TEST(test_hsv_rel_multi_hue_wrap);
  RUN_TEST(test_hsv_rel_all_stop_noop);
  RUN_TEST(test_rgbw_rel_all_stop_noop);
  RUN_TEST(test_hsv_rel_sv_negative_clamp);
  RUN_TEST(test_hsv_rel_multi_cycle_hue_identity);
  RUN_TEST(test_hue_min_step_delta);
  return UNITY_END();
}
#endif
#endif // UNIT_TEST
