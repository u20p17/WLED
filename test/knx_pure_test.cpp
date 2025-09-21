#ifdef UNIT_TEST
// Standalone pure implementations of KNX test helpers (no WLED core dependencies) for native/ESP32 test envs.
#include <stdint.h>
#include <stddef.h>
#include <math.h>

// Unity expects optional user-provided setUp()/tearDown() symbols; provide empty stubs for native build.
extern "C" void setUp(void) {}
extern "C" void tearDown(void) {}

static uint16_t _parseGA_impl(const char* s) {
  if (!s || !*s) return 0; uint32_t a=0,b=0,c=0; const char* p=s;
  auto parseUInt=[&](uint32_t& out){ if(*p<'0'||*p>'9') return false; uint32_t v=0; while(*p>='0'&&*p<='9'){ v=v*10u+(uint32_t)(*p-'0'); if(v>1000u) return false; ++p;} out=v; return true; };
  if(!parseUInt(a)||*p!='/') return 0; ++p; if(!parseUInt(b)||*p!='/') return 0; ++p; if(!parseUInt(c)||*p!='\0') return 0; if(a>31u||b>7u||c>255u) return 0; return (uint16_t)((a&0x1F)<<11|(b&0x07)<<8|(c&0xFF));
}
static uint16_t _parsePA_impl(const char* s) {
  if (!s || !*s) return 0; uint32_t area=0,line=0,dev=0; const char* p=s;
  auto parseUInt=[&](uint32_t& out){ if(*p<'0'||*p>'9') return false; uint32_t v=0; while(*p>='0'&&*p<='9'){ v=v*10u+(uint32_t)(*p-'0'); if(v>1000u) return false; ++p;} out=v; return true; };
  if(!parseUInt(area)||*p!='.') return 0; ++p; if(!parseUInt(line)||*p!='.') return 0; ++p; if(!parseUInt(dev)||*p!='\0') return 0; if(area>15u||line>15u||dev>255u) return 0; return (uint16_t)((area&0x0F)<<12|(line&0x0F)<<8|(dev&0xFF));
}
static uint8_t _step_pct(uint8_t sc){ switch(sc){ case 1:return 100; case 2:return 50; case 3:return 25; case 4:return 12; case 5:return 6; case 6:return 3; case 7:return 1; default:return 0; }}
static int16_t _step_delta(uint8_t nibble, uint16_t maxVal){ if(nibble==0) return 0; bool inc=(nibble&0x8)!=0; uint8_t pct=_step_pct(nibble&0x7); uint16_t mag=(uint32_t)maxVal*pct/100U; if(mag==0) mag=1; return inc?(int16_t)mag:-(int16_t)mag; }
static void _rgbToHsv(uint8_t r,uint8_t g,uint8_t b,float& h,float& s,float& v){ float rf=r/255.f,gf=g/255.f,bf=b/255.f; float maxv=rf;if(gf>maxv)maxv=gf;if(bf>maxv)maxv=bf; float minv=rf;if(gf<minv)minv=gf;if(bf<minv)minv=bf; v=maxv; float d=maxv-minv; s=(maxv==0)?0:(d/maxv); if(d==0){h=0;return;} float hue; if(maxv==rf) hue=fmodf(((gf-bf)/d),6.f); else if(maxv==gf) hue=((bf-rf)/d)+2.f; else hue=((rf-gf)/d)+4.f; hue*=60.f; if(hue<0) hue+=360.f; h=hue; }
static void _hsvToRgb(float h,float s,float v,uint8_t& r,uint8_t& g,uint8_t& b){ if(s<=0.f){ r=g=b=(uint8_t)lrintf(v*255.f); return;} h=fmodf(h,360.f); if(h<0) h+=360.f; float c=v*s; float x=c*(1.f-fabsf(fmodf(h/60.f,2.f)-1.f)); float m=v-c; float rf=0,gf=0,bf=0; int seg=(int)floorf(h/60.f); switch(seg){case 0: rf=c;gf=x;bf=0;break;case 1: rf=x;gf=c;bf=0;break;case 2: rf=0;gf=c;bf=x;break;case 3: rf=0;gf=x;bf=c;break;case 4: rf=x;gf=0;bf=c;break;default: rf=c;gf=0;bf=x;break;} r=(uint8_t)lrintf((rf+m)*255.f); g=(uint8_t)lrintf((gf+m)*255.f); b=(uint8_t)lrintf((bf+m)*255.f); }

// White split relative logic (mirrors firmware adjustWhiteSplitRel behavior)
static void _white_split_apply(uint8_t w, uint8_t cct, int16_t delta, bool adjustWarm, uint8_t& outW, uint8_t& outCct) {
  if (delta==0 || (w==0 && delta<0)) { outW = w; outCct = cct; return; }
  uint16_t warm = (uint16_t)w * (255 - cct) / 255;
  uint16_t cold = (uint16_t)w * cct / 255;
  if (adjustWarm) {
    int nwarm = (int)warm + delta; if (nwarm<0) nwarm=0; else if (nwarm>255) nwarm=255; warm=(uint16_t)nwarm;
  } else {
    int ncold = (int)cold + delta; if (ncold<0) ncold=0; else if (ncold>255) ncold=255; cold=(uint16_t)ncold;
  }
  uint16_t sum = warm + cold; if (sum>255) sum=255; outW = (uint8_t)sum;
  if (sum==0) { outCct = cct; return; }
  outCct = (uint8_t)((cold * 255u + sum/2)/sum);
}

static inline uint8_t _clamp_u8(int v){ if(v<0) return 0; if(v>255) return 255; return (uint8_t)v; }

static void _rgb_rel(uint8_t r,uint8_t g,uint8_t b,uint8_t rCtl,uint8_t gCtl,uint8_t bCtl,uint8_t& or_,uint8_t& og_,uint8_t& ob_){
  int16_t dr=_step_delta(rCtl&0x0F,255); int16_t dg=_step_delta(gCtl&0x0F,255); int16_t db=_step_delta(bCtl&0x0F,255);
  or_=_clamp_u8((int)r+dr); og_=_clamp_u8((int)g+dg); ob_=_clamp_u8((int)b+db);
}
static void _hsv_rel(uint8_t r,uint8_t g,uint8_t b,uint8_t hCtl,uint8_t sCtl,uint8_t vCtl,uint8_t& or_,uint8_t& og_,uint8_t& ob_){
  int16_t dh=_step_delta(hCtl&0x0F,30); int16_t ds=_step_delta(sCtl&0x0F,255); int16_t dv=_step_delta(vCtl&0x0F,255);
  float h,s,v; _rgbToHsv(r,g,b,h,s,v);
  if(dh) { h += (float)dh; while(h<0) h+=360.f; while(h>=360.f) h-=360.f; }
  if(ds) { s += (float)ds/255.f; if(s<0) s=0; if(s>1) s=1; }
  if(dv) { v += (float)dv/255.f; if(v<0) v=0; if(v>1) v=1; }
  _hsvToRgb(h,s,v,or_,og_,ob_);
}
static void _rgbw_rel(uint8_t r,uint8_t g,uint8_t b,uint8_t w,uint8_t rCtl,uint8_t gCtl,uint8_t bCtl,uint8_t wCtl,uint8_t& or_,uint8_t& og_,uint8_t& ob_,uint8_t& ow_){
  int16_t dr=_step_delta(rCtl&0x0F,255); int16_t dg=_step_delta(gCtl&0x0F,255); int16_t db=_step_delta(bCtl&0x0F,255); int16_t dw=_step_delta(wCtl&0x0F,255);
  or_=_clamp_u8((int)r+dr); og_=_clamp_u8((int)g+dg); ob_=_clamp_u8((int)b+db); ow_=_clamp_u8((int)w+dw);
}

extern "C" {
  uint16_t knx_test_parseGA(const char* s){ return _parseGA_impl(s); }
  uint16_t knx_test_parsePA(const char* s){ return _parsePA_impl(s); }
  uint8_t  knx_test_step_pct(uint8_t sc){ return _step_pct(sc); }
  int16_t  knx_test_step_delta(uint8_t nibble, uint16_t maxVal){ return _step_delta(nibble,maxVal); }
  void     knx_test_rgbToHsv(uint8_t r,uint8_t g,uint8_t b,float& h,float& s,float& v){ _rgbToHsv(r,g,b,h,s,v); }
  void     knx_test_hsvToRgb(float h,float s,float v,uint8_t& r,uint8_t& g,uint8_t& b){ _hsvToRgb(h,s,v,r,g,b); }
  void     knx_test_white_split(uint8_t w,uint8_t cct,int16_t delta,int adjustWarm,uint8_t* outW,uint8_t* outCct){ _white_split_apply(w,cct,delta,adjustWarm!=0,*outW,*outCct); }
  void     knx_test_rgb_rel(uint8_t r,uint8_t g,uint8_t b,uint8_t rCtl,uint8_t gCtl,uint8_t bCtl,uint8_t* or_,uint8_t* og_,uint8_t* ob_){ _rgb_rel(r,g,b,rCtl,gCtl,bCtl,*or_,*og_,*ob_); }
  void     knx_test_hsv_rel(uint8_t r,uint8_t g,uint8_t b,uint8_t hCtl,uint8_t sCtl,uint8_t vCtl,uint8_t* or_,uint8_t* og_,uint8_t* ob_){ _hsv_rel(r,g,b,hCtl,sCtl,vCtl,*or_,*og_,*ob_); }
  void     knx_test_rgbw_rel(uint8_t r,uint8_t g,uint8_t b,uint8_t w,uint8_t rCtl,uint8_t gCtl,uint8_t bCtl,uint8_t wCtl,uint8_t* or_,uint8_t* og_,uint8_t* ob_,uint8_t* ow_){ _rgbw_rel(r,g,b,w,rCtl,gCtl,bCtl,wCtl,*or_,*og_,*ob_,*ow_); }
  uint8_t  knx_test_clamp100(uint8_t v) { return (v>100)?100:v; }
  uint8_t  knx_test_pct_to_0_255(uint8_t pct) { return (uint8_t)((pct * 255u + 50u) / 100u); }
  uint8_t  knx_test_to_pct_0_100(uint8_t v0_255) { return (uint8_t)((v0_255 * 100u + 127u) / 255u); }
  // CCT conversion helpers using default ranges (2700-6500K)
  uint8_t  knx_test_kelvin_to_cct255(uint16_t k) { 
    const uint16_t kmin = 2700, kmax = 6500;
    if (k <= kmin) return 0;
    if (k >= kmax) return 255;
    uint32_t span = (uint32_t)kmax - (uint32_t)kmin;
    uint32_t pos  = (uint32_t)k  - (uint32_t)kmin;
    return (uint8_t)((pos * 255u + (span/2)) / span);
  }
  uint16_t knx_test_cct255_to_kelvin(uint8_t cct) {
    const uint16_t kmin = 2700, kmax = 6500;
    uint32_t span = (uint32_t)kmax - (uint32_t)kmin;
    return (uint16_t)(kmin + (uint32_t)cct * span / 255u);
  }
}
#endif // UNIT_TEST
