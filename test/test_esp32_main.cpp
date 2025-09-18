#ifdef ARDUINO
#ifdef UNIT_TEST

#include <Arduino.h>
#include <unity.h>

// Forward declarations (normal C++ linkage) of tests implemented in test_knx_ip_basic.cpp
void test_parseGA_valid();
void test_parseGA_invalid();
void test_step_pct_mapping();
void test_step_delta_inc_dec();
void test_hsv_roundtrip_primary_colors();

static void runAll()
{
  RUN_TEST(test_parseGA_valid);
  RUN_TEST(test_parseGA_invalid);
  RUN_TEST(test_step_pct_mapping);
  RUN_TEST(test_step_delta_inc_dec);
  RUN_TEST(test_hsv_roundtrip_primary_colors);
}

void setup()
{
  delay(200);
  UNITY_BEGIN();
  runAll();
  UNITY_END();
}

void loop() { /* nothing */ }

#endif
#endif