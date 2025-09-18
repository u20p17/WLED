#ifdef UNIT_TEST
// Standalone pure implementations of KNX test helpers so we can build without full WLED core.
// Only algorithms under test are reproduced; NO side effects, NO globals.

#include <stdint.h>
#include <stddef.h>
#include <math.h>

// ---- parse group address "a/b/c" ----
static uint16_t _parseGA_impl(const char* s) {
  if (!s || !*s) return 0;
  uint32_t a=0,b=0,c=0; const char* p = s;
  auto parseUInt=[&](uint32_t& out){ if (*p<'0'||*p>'9') return false; uint32_t v=0; while(*p>='0'&&*p<='9'){ v = v*10u + (uint32_t)(*p-'0'); if (v>1000u) return false; ++p;} out=v; return true; };
  if (!parseUInt(a) || *p!='/') return 0; ++p;
  if (!parseUInt(b) || *p!='/') return 0; ++p;
  if (!parseUInt(c) || *p!='\0') return 0;
  if (a>31u || b>7u || c>255u) return 0;
  return (uint16_t)((a & 0x1F) << 11 | (b & 0x07) << 8 | (c & 0xFF));
}

// ---- parse individual address "a.b.c" ----
static uint16_t _parsePA_impl(const char* s) {
  if (!s || !*s) return 0;
  uint32_t area=0,line=0,dev=0; const char* p=s;
  auto parseUInt=[&](uint32_t& out){ if (*p<'0'||*p>'9') return false; uint32_t v=0; while(*p>='0'&&*p<='9'){ v = v*10u + (uint32_t)(*p-'0'); if (v>1000u) return false; ++p;} out=v; return true; };
  if (!parseUInt(area) || *p!='.') return 0; ++p;
  if (!parseUInt(line) || *p!='.') return 0; ++p;
  if (!parseUInt(dev)  || *p!='\0') return 0;
  if (area>15u || line>15u || dev>255u) return 0;
  return (uint16_t)((area & 0x0F) << 12 | (line & 0x0F) << 8 | (dev & 0xFF));
}

static uint8_t _step_pct(uint8_t sc) {
  switch(sc){
    case 1: return 100; case 2: return 50; case 3: return 25; case 4: return 12; case 5: return 6; case 6: return 3; case 7: return 1; default: return 0; }
}
static int16_t _step_delta(uint8_t nibble, uint16_t maxVal) {
  if (nibble==0) return 0;
  bool inc = (nibble & 0x8)!=0;
  uint8_t pct = _step_pct(nibble & 0x7);
  uint16_t mag = (uint32_t)maxVal * pct / 100U;
  if (mag==0) mag = 1;
  return inc ? (int16_t)mag : -(int16_t)mag;
}

// RGB <-> HSV (0<=h<360, s,v 0..1)
static void _rgbToHsv(uint8_t r,uint8_t g,uint8_t b,float& h,float& s,float& v){
  float rf=r/255.f,gf=g/255.f,bf=b/255.f; float maxv=rf; if(gf>maxv)maxv=gf; if(bf>maxv)maxv=bf; float minv=rf; if(gf<minv)minv=gf; if(bf<minv)minv=bf; v=maxv; float d=maxv-minv; s = (maxv==0)?0:(d/maxv); if(d==0){ h=0; return; } float hue;
  if (maxv==rf) hue = fmodf(((gf-bf)/d),6.f); else if (maxv==gf) hue = ((bf-rf)/d)+2.f; else hue = ((rf-gf)/d)+4.f; hue*=60.f; if (hue<0) hue+=360.f; h=hue;
}
static void _hsvToRgb(float h,float s,float v,uint8_t& r,uint8_t& g,uint8_t& b){
  if (s<=0.f){ r=g=b=(uint8_t)lrintf(v*255.f); return; }
  h = fmodf(h,360.f); if (h<0) h+=360.f; float c=v*s; float x=c*(1.f - fabsf(fmodf(h/60.f,2.f)-1.f)); float m=v-c; float rf=0,gf=0,bf=0; int seg=(int)floorf(h/60.f);
  switch(seg){ case 0: rf=c; gf=x; bf=0; break; case 1: rf=x; gf=c; bf=0; break; case 2: rf=0; gf=c; bf=x; break; case 3: rf=0; gf=x; bf=c; break; case 4: rf=x; gf=0; bf=c; break; default: rf=c; gf=0; bf=x; break; }
  r=(uint8_t)lrintf((rf+m)*255.f); g=(uint8_t)lrintf((gf+m)*255.f); b=(uint8_t)lrintf((bf+m)*255.f);
}

extern "C" {
  uint16_t knx_test_parseGA(const char* s){ return _parseGA_impl(s); }
  uint16_t knx_test_parsePA(const char* s){ return _parsePA_impl(s); }
  uint8_t  knx_test_step_pct(uint8_t sc){ return _step_pct(sc); }
  int16_t  knx_test_step_delta(uint8_t nibble, uint16_t maxVal){ return _step_delta(nibble, maxVal); }
  void     knx_test_rgbToHsv(uint8_t r,uint8_t g,uint8_t b,float& h,float& s,float& v){ _rgbToHsv(r,g,b,h,s,v); }
  void     knx_test_hsvToRgb(float h,float s,float v,uint8_t& r,uint8_t& g,uint8_t& b){ _hsvToRgb(h,s,v,r,g,b); }
}

// Arduino framework expects these symbols when linking, provide empty stubs.
#ifdef ARDUINO
void setup() {}
void loop() {}
#endif

#endif // UNIT_TEST
