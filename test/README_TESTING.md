# KNX Per-Segment Testing Guide

This directory contains comprehensive tests for the KNX IP usermod's per-segment functionality. All test files have been centralized here for better organization and maintainability.

## 📁 Test Files

### Core Test Files

**`test_segment_knx.cpp`** - Complete standalone test suite with mocks. Can be compiled and run independently for development testing.
- Mock WLED strip and KNX library
- Comprehensive address calculation tests  
- Memory management validation
- Boundary condition testing

**`knx_segment_tests.h`** - Integration test framework designed to run within the WLED environment.
- Real-time segment detection
- Live KNX registration testing
- Actual segment handler validation
- Configuration debugging output

**`knx_unit_tests.cpp`** - Lightweight unit tests for CI/CD integration.
- Minimal overhead unit tests
- Simple assertion framework
- Build-flag controlled execution

### GA Conflict Detection Tests

**`test_ga_conflicts.cpp`** - Comprehensive GA conflict detection test suite.
- Address conflict validation
- Zero offset detection
- Cross-segment conflict testing
- Boundary condition validation

**`GA_CONFLICT_TESTS.md`** - Detailed documentation for GA conflict testing.

### Legacy Test Files

**`knx_pure_test.cpp`** - Original test suite (pre-existing)
**`knx_pure_test_usermod.cpp`** - Usermod-specific version of pure tests

## 🚀 Quick Start

### Option 1: Enable Unit Tests (Recommended)

1. **Add build flag** in `platformio.ini`:
   ```ini
   build_flags = 
       -D KNX_SEGMENT_UNIT_TESTS
       -D KNX_UM_DEBUG
   ```

2. **Add declarations** to `usermod_knx_ip.h`:
   ```cpp
   #ifdef KNX_SEGMENT_UNIT_TESTS
   bool unitTestCalculateSegmentGA();
   bool unitTestMemoryManagement(); 
   bool runUnitTests();
   #endif
   ```

3. **Add implementations** from `knx_unit_tests.cpp` to `usermod_knx_ip.cpp`

4. **Call from setup()**:
   ```cpp
   void setup() {
       // ... existing setup code ...
       
       #ifdef KNX_SEGMENT_UNIT_TESTS
       runUnitTests();
       #endif
   }
   ```

### Option 2: Integration Tests

1. **Add declarations** from `knx_segment_tests.h` to `usermod_knx_ip.h`
2. **Add implementations** to `usermod_knx_ip.cpp`
3. **Call manually** via serial command or web interface:
   ```cpp
   if (command == "knx-test") {
       runSegmentTests();
   }
   ```

### Option 3: Standalone Testing

1. **Compile** `test_segment_knx.cpp` with a C++ compiler
2. **Run** the executable for comprehensive offline testing

## 🧪 Test Coverage

### Address Calculation Tests
- ✅ Formula validation: `Segment N = (central_main + L*N, central_middle + M*N, central_sub + N*N)`
- ✅ Boundary condition validation (31/7/255 limits)
- ✅ Invalid input handling (empty/malformed GAs)
- ✅ Multiple offset configurations
- ✅ Edge cases (segment 0, high segment numbers)

### Memory Management Tests
- ✅ Dynamic array allocation
- ✅ Proper deallocation and cleanup
- ✅ Zero-initialization verification
- ✅ Null pointer handling
- ✅ Memory leak prevention

### Handler Tests
- ✅ Segment power control
- ✅ Segment brightness control
- ✅ Segment effect control
- ✅ Segment RGB control
- ✅ Handler isolation (only affects target segment)

### Integration Tests
- ✅ KNX group object registration
- ✅ Real segment detection
- ✅ Live configuration validation
- ✅ Performance under load

## 📊 Expected Test Results

### Unit Test Output
```
[KNX-UNIT-TEST] Starting KNX Per-Segment Unit Tests
[KNX-UNIT-TEST] Testing calculateSegmentGA function...
[KNX-UNIT-TEST] calculateSegmentGA tests PASSED
[KNX-UNIT-TEST] Testing memory management...
[KNX-UNIT-TEST] Memory management tests PASSED
[KNX-UNIT-TEST] ✅ ALL UNIT TESTS PASSED
```

### Integration Test Output
```
[KNX-TEST] Starting Per-Segment KNX Tests
[KNX-TEST] Current Configuration:
[KNX-TEST] - Total segments: 3
[KNX-TEST] - Segment offsets: L=1, M=0, N=10
[KNX-TEST] Central GA: 1/2/100
[KNX-TEST] Segment 0: 1/2/100 (0x0964)
[KNX-TEST] Segment 1: 2/2/110 (0x096E)
[KNX-TEST] Segment 2: 3/2/120 (0x0978)
[KNX-TEST] Registered segments: 3
[KNX-TEST] Per-Segment KNX Tests Completed
```

## 🔧 Manual Testing Scenarios

### Configuration Test
1. Set segment offsets: L=1, M=0, N=10
2. Set central power GA: 1/2/10
3. Verify calculated segment GAs:
   - Segment 0: 1/2/10 (central)
   - Segment 1: 2/2/20
   - Segment 2: 3/2/30

### KNX Bus Test
1. Send boolean '1' to segment 1 power GA → should turn on segment 1 only
2. Send value '128' to segment 1 brightness GA → should set segment 1 to 50%
3. Send value '5' to segment 2 effect GA → should change segment 2 effect
4. Verify other segments remain unchanged

### Performance Test
1. Create 10+ segments
2. Verify memory usage is reasonable
3. Test KNX registration time
4. Send rapid commands to different segments

### Edge Case Test
1. Test with 0 segments
2. Test with 32+ segments (should limit to 32)
3. Test with offsets that would exceed KNX limits
4. Test segment index out of bounds

## 🐛 Troubleshooting

### Common Issues

**Tests not running:**
- Verify `KNX_SEGMENT_UNIT_TESTS` build flag is set
- Check that function declarations are added to header
- Ensure debug output is enabled with `KNX_UM_DEBUG`

**Test failures:**
- Check segment offset configuration (L≤31, M≤7, N≤255)
- Verify central GA strings are valid
- Ensure sufficient memory for segment arrays
- Check that WLED segments are properly configured

**Memory issues:**
- Monitor free heap during tests
- Verify `clearSegmentKOs()` is called before re-registration
- Check for memory leaks with repeated test runs

### Debug Output

Enable verbose debug output:
```ini
build_flags = 
    -D KNX_UM_DEBUG
    -D KNX_SEGMENT_UNIT_TESTS
```

Monitor serial output for detailed test progress and any failure messages.

## 📈 Continuous Integration

For automated testing in CI/CD pipelines:

1. Add unit test build target
2. Run tests during build process
3. Fail build on test failures
4. Generate test reports

Example CI configuration:
```yaml
- name: Run KNX Unit Tests
  run: |
    pio run -e test_knx_segments
    # Parse test output for failures
```

## 🤝 Contributing

When adding new per-segment functionality:

1. **Add unit tests** for new functions
2. **Update integration tests** for end-to-end validation
3. **Document test scenarios** in this README
4. **Verify all tests pass** before submitting PR

## 📚 References

- [WLED Segment Documentation](https://kno.wled.ge/features/segments/)
- [KNX Group Address Format](https://www.knx.org/)
- [ESP32 Memory Management Best Practices](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/memory-types.html)