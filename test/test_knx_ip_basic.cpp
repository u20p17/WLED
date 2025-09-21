#ifdef UNIT_TEST
#include <unity.h>
#include <stdint.h>
#include <stdio.h>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <cstring>

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
  uint8_t  knx_test_clamp100(uint8_t v);
  uint8_t  knx_test_pct_to_0_255(uint8_t pct);
  uint8_t  knx_test_to_pct_0_100(uint8_t v0_255);
  uint8_t  knx_test_kelvin_to_cct255(uint16_t k);
  uint16_t knx_test_cct255_to_kelvin(uint8_t cct);
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

// ===== Brightness conversion tests (DPT 5.001 percentage scaling) =====
void test_brightness_dpt5001_percent_scaling() {
  // Test the core DPT 5.001 percentage to brightness conversion
  // Formula: (pct * 255 + 50) / 100
  
  // Edge cases
  TEST_ASSERT_EQUAL_UINT8(0, knx_test_pct_to_0_255(0));     // 0% -> 0
  TEST_ASSERT_EQUAL_UINT8(255, knx_test_pct_to_0_255(100)); // 100% -> 255
  
  // 25% case that was the original issue
  TEST_ASSERT_EQUAL_UINT8(64, knx_test_pct_to_0_255(25));   // 25% -> 64 (not 163!)
  
  // Additional test cases with proper rounding
  TEST_ASSERT_EQUAL_UINT8(13, knx_test_pct_to_0_255(5));    // 5% -> 13
  TEST_ASSERT_EQUAL_UINT8(26, knx_test_pct_to_0_255(10));   // 10% -> 26
  TEST_ASSERT_EQUAL_UINT8(128, knx_test_pct_to_0_255(50));  // 50% -> 128
  TEST_ASSERT_EQUAL_UINT8(191, knx_test_pct_to_0_255(75));  // 75% -> 191
  TEST_ASSERT_EQUAL_UINT8(230, knx_test_pct_to_0_255(90));  // 90% -> 230
}

void test_brightness_over_100_clamped() {
  // Test clamp100 function - values over 100 should be clamped to 100
  TEST_ASSERT_EQUAL_UINT8(100, knx_test_clamp100(150));
  TEST_ASSERT_EQUAL_UINT8(100, knx_test_clamp100(255));
  TEST_ASSERT_EQUAL_UINT8(75, knx_test_clamp100(75));   // values <= 100 unchanged
  TEST_ASSERT_EQUAL_UINT8(0, knx_test_clamp100(0));
  
  // Test the complete conversion chain: clamp then convert
  // If someone sends 150 as DPT 5.001, it should be treated as 100%
  uint8_t clamped = knx_test_clamp100(150);
  uint8_t brightness = knx_test_pct_to_0_255(clamped);
  TEST_ASSERT_EQUAL_UINT8(255, brightness);
}

void test_brightness_roundtrip_conversion() {
  // Test round-trip: brightness -> percentage -> brightness
  TEST_ASSERT_EQUAL_UINT8(0, knx_test_to_pct_0_100(0));     // 0 -> 0% -> 0
  TEST_ASSERT_EQUAL_UINT8(100, knx_test_to_pct_0_100(255)); // 255 -> 100% -> 255
  
  // Test some intermediate values
  uint8_t pct25 = knx_test_to_pct_0_100(64);  // 64 -> should be ~25%
  TEST_ASSERT_EQUAL_UINT8(25, pct25);
  
  uint8_t pct50 = knx_test_to_pct_0_100(128); // 128 -> should be ~50%
  TEST_ASSERT_EQUAL_UINT8(50, pct50);
}

// ===== CCT conversion tests (DPT 7.600 Kelvin <-> 0-255 CCT) =====
void test_cct_kelvin_to_255_conversion() {
  // Test standard CCT range conversion (2700K-6500K -> 0-255)
  TEST_ASSERT_EQUAL_UINT8(0, knx_test_kelvin_to_cct255(2700));     // Min Kelvin -> 0 (warm)
  TEST_ASSERT_EQUAL_UINT8(255, knx_test_kelvin_to_cct255(6500));   // Max Kelvin -> 255 (cold)
  
  // Test middle value (allow 1 unit tolerance for rounding)
  uint8_t mid_cct = knx_test_kelvin_to_cct255(4600);
  TEST_ASSERT_TRUE(mid_cct >= 127 && mid_cct <= 128);              // Middle -> ~127-128
  
  // Test boundary conditions
  TEST_ASSERT_EQUAL_UINT8(0, knx_test_kelvin_to_cct255(2000));     // Below min -> 0
  TEST_ASSERT_EQUAL_UINT8(255, knx_test_kelvin_to_cct255(7000));   // Above max -> 255
  
  // Test common values (with tolerance)
  uint8_t quarter_cct = knx_test_kelvin_to_cct255(3650);
  TEST_ASSERT_TRUE(quarter_cct >= 63 && quarter_cct <= 65);        // Quarter point -> ~64 ± 1
  
  uint8_t three_quarter_cct = knx_test_kelvin_to_cct255(5550);
  TEST_ASSERT_TRUE(three_quarter_cct >= 190 && three_quarter_cct <= 192); // Three-quarter point -> ~191 ± 1
}

void test_cct_255_to_kelvin_conversion() {
  // Test reverse conversion (0-255 -> 2700K-6500K)
  TEST_ASSERT_EQUAL_UINT16(2700, knx_test_cct255_to_kelvin(0));    // 0 -> 2700K (warm)
  TEST_ASSERT_EQUAL_UINT16(6500, knx_test_cct255_to_kelvin(255));  // 255 -> 6500K (cold)
  
  // Test middle value
  uint16_t mid_kelvin = knx_test_cct255_to_kelvin(127);
  TEST_ASSERT_TRUE(mid_kelvin >= 4580 && mid_kelvin <= 4620);      // ~4600K ± tolerance
  
  // Test quarter and three-quarter points
  uint16_t quarter_kelvin = knx_test_cct255_to_kelvin(64);
  TEST_ASSERT_TRUE(quarter_kelvin >= 3620 && quarter_kelvin <= 3680); // ~3650K ± tolerance
  
  uint16_t three_quarter_kelvin = knx_test_cct255_to_kelvin(191);
  TEST_ASSERT_TRUE(three_quarter_kelvin >= 5520 && three_quarter_kelvin <= 5580); // ~5550K ± tolerance
}

void test_cct_roundtrip_conversion() {
  // Test round-trip conversion: Kelvin -> CCT255 -> Kelvin
  uint16_t test_kelvins[] = {2700, 3000, 4000, 5000, 6000, 6500};
  
  for (size_t i = 0; i < sizeof(test_kelvins)/sizeof(test_kelvins[0]); i++) {
    uint16_t original = test_kelvins[i];
    uint8_t cct = knx_test_kelvin_to_cct255(original);
    uint16_t converted_back = knx_test_cct255_to_kelvin(cct);
    
    // Allow small tolerance due to rounding
    int16_t diff = (int16_t)converted_back - (int16_t)original;
    TEST_ASSERT_TRUE(abs(diff) <= 20); // Within 20K tolerance
  }
}

// ===== Individual Address (PA) parsing tests =====
void test_parsePA_valid() {
  // Test valid PA format "area.line.device" -> (area<<12)|(line<<8)|device
  TEST_ASSERT_EQUAL_UINT16((1U<<12)|(2U<<8)|3U, knx_test_parsePA("1.2.3"));
  TEST_ASSERT_EQUAL_UINT16((15U<<12)|(15U<<8)|255U, knx_test_parsePA("15.15.255"));
  TEST_ASSERT_EQUAL_UINT16((0U<<12)|(0U<<8)|1U, knx_test_parsePA("0.0.1"));
}

void test_parsePA_invalid() {
  // Test invalid PA formats
  TEST_ASSERT_EQUAL_UINT16(0, knx_test_parsePA(""));
  TEST_ASSERT_EQUAL_UINT16(0, knx_test_parsePA("a.b.c"));
  TEST_ASSERT_EQUAL_UINT16(0, knx_test_parsePA("16.1.1")); // area out of range (>15)
  TEST_ASSERT_EQUAL_UINT16(0, knx_test_parsePA("1.16.1")); // line out of range (>15)
  TEST_ASSERT_EQUAL_UINT16(0, knx_test_parsePA("1.1.256")); // device out of range (>255)
  TEST_ASSERT_EQUAL_UINT16(0, knx_test_parsePA("1.1")); // missing field
  TEST_ASSERT_EQUAL_UINT16(0, knx_test_parsePA("1/1/1")); // wrong separator
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

// ===== SearchRequest / SearchResponse pure builder tests =====
// We replicate the production _sendSearchResponse layout logic in a pure function.
// This avoids Arduino/WiFi dependencies while validating packet structure.
static void build_search_response_pure(bool extended,
                                       const uint8_t req[], int reqLen,
                                       const uint8_t localIp[4], const uint8_t mac[6],
                                       std::vector<uint8_t>& out) {
  (void)req; (void)reqLen; // For now we trust the caller passes a minimally valid request.
  const uint16_t svc = extended ? 0x020C : 0x0202; // KNX_SVC_SEARCH_RES(_EXT)
  // Construct HPAI
  uint8_t hpai[8]; hpai[0]=0x08; hpai[1]=0x01; // len, IPv4/UDP
  hpai[2]=localIp[0]; hpai[3]=localIp[1]; hpai[4]=localIp[2]; hpai[5]=localIp[3];
  hpai[6]= (uint8_t)(3671 >> 8); hpai[7]= (uint8_t)(3671 & 0xFF);
  // Device Info DIB (fixed 0x36)
  uint8_t dib_dev[0x36]; memset(dib_dev,0,sizeof(dib_dev));
  dib_dev[0]=0x36; dib_dev[1]=0x01; dib_dev[2]=0x20; dib_dev[3]=0x00; // len,type, medium, status
  // PA left 0
  // Project/Installation ID zero
  memcpy(&dib_dev[8], mac, 6);                   // serial (MAC)
  dib_dev[14]=224; dib_dev[15]=0; dib_dev[16]=23; dib_dev[17]=12; // multicast
  memcpy(&dib_dev[18], mac, 6);                  // MAC again
  const char* name = "WLED KNX";                // simplified name
  strncpy((char*)&dib_dev[24], name, 30);
  dib_dev[24+29]='\0';
  // Supported Service Families DIB (0x0A)
  uint8_t dib_svc[10]; memset(dib_svc,0,sizeof(dib_svc));
  dib_svc[0]=0x0A; dib_svc[1]=0x02; // len, type
  dib_svc[2]=0x02; dib_svc[3]=0x01; // Core v1
  dib_svc[4]=0x05; dib_svc[5]=0x01; // Routing v1
  // Compose full packet
  const size_t payloadLen = 6 + sizeof(hpai) + sizeof(dib_dev) + sizeof(dib_svc);
  out.resize(payloadLen);
  size_t p=0;
  out[p++]=0x06; out[p++]=0x10;
  out[p++]= (uint8_t)(svc >> 8); out[p++]= (uint8_t)(svc & 0xFF);
  out[p++]= (uint8_t)(payloadLen >> 8); out[p++]= (uint8_t)(payloadLen & 0xFF);
  memcpy(&out[p], hpai, sizeof(hpai)); p+=sizeof(hpai);
  memcpy(&out[p], dib_dev, sizeof(dib_dev)); p+=sizeof(dib_dev);
  memcpy(&out[p], dib_svc, sizeof(dib_svc)); p+=sizeof(dib_svc);
}

static void common_assert_search_response(const std::vector<uint8_t>& pkt,
                                          bool extended,
                                          const uint8_t localIp[4], const uint8_t mac[6]) {
  TEST_ASSERT_TRUE(pkt.size() > 6);
  TEST_ASSERT_EQUAL_UINT8(0x06, pkt[0]);
  TEST_ASSERT_EQUAL_UINT8(0x10, pkt[1]);
  const uint16_t svc = (uint16_t(pkt[2])<<8) | pkt[3];
  TEST_ASSERT_EQUAL_UINT16(extended?0x020C:0x0202, svc);
  const uint16_t declaredLen = (uint16_t(pkt[4])<<8) | pkt[5];
  TEST_ASSERT_EQUAL_UINT16(pkt.size(), declaredLen);
  // HPAI at offset 6
  TEST_ASSERT_EQUAL_UINT8(0x08, pkt[6]);
  TEST_ASSERT_EQUAL_UINT8(0x01, pkt[7]);
  TEST_ASSERT_EQUAL_UINT8(localIp[0], pkt[8]);
  TEST_ASSERT_EQUAL_UINT8(localIp[1], pkt[9]);
  TEST_ASSERT_EQUAL_UINT8(localIp[2], pkt[10]);
  TEST_ASSERT_EQUAL_UINT8(localIp[3], pkt[11]);
  TEST_ASSERT_EQUAL_UINT16(3671, (uint16_t(pkt[12])<<8)|pkt[13]);
  // Device info DIB starts at 14
  TEST_ASSERT_EQUAL_UINT8(0x36, pkt[14]);
  TEST_ASSERT_EQUAL_UINT8(0x01, pkt[15]);
  TEST_ASSERT_EQUAL_UINT8(0x20, pkt[16]); // medium
  // Serial MAC at 22..27? (offset 14 + 8 = 22)
  for(int i=0;i<6;i++) TEST_ASSERT_EQUAL_UINT8(mac[i], pkt[22+i]);
  // Multicast 224.0.23.12 at offset 14+14=28
  TEST_ASSERT_EQUAL_UINT8(224, pkt[28]);
  TEST_ASSERT_EQUAL_UINT8(0,   pkt[29]);
  TEST_ASSERT_EQUAL_UINT8(23,  pkt[30]);
  TEST_ASSERT_EQUAL_UINT8(12,  pkt[31]);
  // Supported Service Families DIB starts after 0x36 bytes: 14 + 0x36 = 68
  const size_t svcOff = 14 + 0x36;
  TEST_ASSERT_TRUE(pkt.size() >= svcOff + 10);
  TEST_ASSERT_EQUAL_UINT8(0x0A, pkt[svcOff]);
  TEST_ASSERT_EQUAL_UINT8(0x02, pkt[svcOff+1]);
  TEST_ASSERT_EQUAL_UINT8(0x02, pkt[svcOff+2]);
  TEST_ASSERT_EQUAL_UINT8(0x01, pkt[svcOff+3]);
  TEST_ASSERT_EQUAL_UINT8(0x05, pkt[svcOff+4]);
  TEST_ASSERT_EQUAL_UINT8(0x01, pkt[svcOff+5]);
}

void test_search_response_standard() {
  uint8_t req[14] = {0x06,0x10,0x02,0x01,0x00,0x0E,0x08,0x01,192,168,0,121,0xD1,0xC2};
  uint8_t ip[4] = {192,168,0,50};
  uint8_t mac[6] = {0xAA,0xBB,0xCC,0x11,0x22,0x33};
  std::vector<uint8_t> pkt;
  build_search_response_pure(false, req, sizeof(req), ip, mac, pkt);
  common_assert_search_response(pkt,false,ip,mac);
}

void test_search_response_extended() {
  uint8_t req[22] = {0x06,0x10,0x02,0x0B,0x00,0x16,0x08,0x01,192,168,0,121,0xD1,0xC2,0x08,0x04,1,2,3,4,5,6};
  uint8_t ip[4] = {10,1,2,3};
  uint8_t mac[6] = {0xDE,0xAD,0xBE,0xEF,0x00,0x01};
  std::vector<uint8_t> pkt;
  build_search_response_pure(true, req, sizeof(req), ip, mac, pkt);
  common_assert_search_response(pkt,true,ip,mac);
}

#ifndef ARDUINO
int main() {
  UNITY_BEGIN();
  RUN_TEST(test_parseGA_valid);
  RUN_TEST(test_parseGA_invalid);
  RUN_TEST(test_parsePA_valid);
  RUN_TEST(test_parsePA_invalid);
  RUN_TEST(test_brightness_dpt5001_percent_scaling);
  RUN_TEST(test_brightness_over_100_clamped);
  RUN_TEST(test_brightness_roundtrip_conversion);
  RUN_TEST(test_cct_kelvin_to_255_conversion);
  RUN_TEST(test_cct_255_to_kelvin_conversion);
  RUN_TEST(test_cct_roundtrip_conversion);
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
  RUN_TEST(test_search_response_standard);
  RUN_TEST(test_search_response_extended);
  return UNITY_END();
}
#endif
#endif // UNIT_TEST
