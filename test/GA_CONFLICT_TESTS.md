# GA Conflict Detection Tests

## Overview

The KNX IP usermod includes comprehensive GA (Group Address) conflict detection tests to ensure that per-segment KNX functionality doesn't create duplicate or conflicting addresses. All test files are centralized in this `/test/` directory for better organization.

**Test Files in this directory:**
- `test_ga_conflicts.cpp` - Main GA conflict test implementation
- `knx_segment_tests.h` - Integration test framework
- `README_TESTING.md` - General testing documentation

This documentation explains how to run and interpret the GA conflict detection tests.

## What GA Conflicts Are

When using per-segment KNX functionality, each segment gets its own calculated GAs based on:
- Central GAs (configured in settings)
- Segment offsets (L, M, N parameters)
- Segment index

**Conflicts occur when:**
1. Segment 0 uses the same GA as central addresses (offset = 0)
2. Different segments calculate to the same GA
3. Calculated GAs exceed KNX limits (31/7/255)

## Test Functions Available

### 1. `testGAConflictDetection()`
Tests the core conflict detection algorithms:
- ✓ Valid configurations (no conflicts)
- ✓ Zero offset detection (segment 0 = central)
- ✓ Cross-segment conflicts
- ✓ Individual GA usage checking
- ✓ Boundary validation

### 2. `testValidationIntegration()`
Tests integration with registration system:
- ✓ Registration prevention with conflicts
- ✓ Successful registration without conflicts
- ✓ State preservation and cleanup

### 3. `runGAConflictTests()`
Runs all GA conflict tests in sequence with detailed output.

## How to Run the Tests

### Method 1: Call from Setup (Automatic)

Add to your usermod setup() or initialization:

```cpp
void setup() {
    // ... existing setup code ...
    
    #ifdef KNX_UM_DEBUG
    // Run GA conflict tests on startup
    knxUsermod.runGAConflictTests();
    #endif
}
```

### Method 2: Manual Test Execution

In your code, call the test functions directly:

```cpp
// Run all GA conflict detection tests
knxUsermod.runGAConflictTests();

// Or run individual tests
knxUsermod.testGAConflictDetection();
knxUsermod.testValidationIntegration();
```

### Method 3: Integrated with Existing Tests

The GA conflict tests are already integrated into the main test suite in `knx_segment_tests.h`:

```cpp
void KnxIpUsermod::runSegmentTests() {
    // ... existing segment tests ...
    
    // GA conflict tests
    testGAConflictDetection();
    testValidationIntegration();
}
```

## Understanding Test Output

### Successful Test Output

```
[KNX-TEST] ==========================================
[KNX-TEST] Starting GA Conflict Detection Tests
[KNX-TEST] ==========================================
[KNX-TEST] Testing GA conflict detection system...
[KNX-TEST] Test 1: Valid configuration
[KNX-TEST] ✓ No conflicts detected with valid offsets
[KNX-TEST] Test 2: Zero offsets (segment 0 = central)
[KNX-TEST] ✓ Conflicts detected with zero offsets
[KNX-TEST] Test 3: Cross-segment conflicts
[KNX-TEST] ✓ Cross-segment conflicts detected
[KNX-TEST] Test 4: Individual GA usage check
[KNX-TEST] ✓ GA 1/2/10 correctly detected as in use
[KNX-TEST] ✓ GA 7/7/7 correctly detected as unused
[KNX-TEST] GA conflict detection test completed
[KNX-TEST] Testing validation integration...
[KNX-TEST] Test 1: Registration with conflicts
[KNX-TEST] ✓ Registration correctly failed due to conflicts
[KNX-TEST] Test 2: Registration without conflicts
[KNX-TEST] ✓ Registration succeeded without conflicts
[KNX-TEST] Validation integration test completed
[KNX-TEST] ==========================================
[KNX-TEST] GA Conflict Detection Tests Completed
[KNX-TEST] ==========================================
```

### Failed Test Indicators

Look for ✗ symbols which indicate test failures:

```
[KNX-TEST] ✗ Unexpected conflicts with valid offsets
[KNX-TEST] ✗ Should have detected conflicts with zero offsets
[KNX-TEST] ✗ Registration should have failed
```

## Test Configuration

### Debug Output

Enable debug output by defining `KNX_UM_DEBUG` in your build flags:

```ini
# In platformio.ini
build_flags = 
    -DKNX_UM_DEBUG
```

### Test Data

The tests use predefined configurations:
- **Valid config**: Large offsets (L=10, M=0, N=50) to avoid conflicts
- **Conflict config**: Zero offsets to force segment 0 = central conflicts
- **Cross-segment**: Small offsets (L=1) to create overlapping GAs

## GA Conflict Analysis Tool

The tests include a detailed analysis tool that shows:

```
[KNX-UM] ==========================================
[KNX-UM] GA Conflict Analysis
[KNX-UM] ==========================================
[KNX-UM] Current configuration:
[KNX-UM] - Segments: 3
[KNX-UM] - Offsets: L=1, M=0, N=0
[KNX-UM] Central GAs:
[KNX-UM] - Power IN: 1/2/10, OUT: 1/2/11
[KNX-UM] - Brightness IN: 2/2/10, OUT: 2/2/11
[KNX-UM] - Effect IN: 3/2/10, OUT: 3/2/11
[KNX-UM] Calculated segment GAs:
[KNX-UM] Segment 0:
[KNX-UM]   Power IN : 1/2/10 ✓
[KNX-UM]   Power OUT: 1/2/11 ✓
[KNX-UM]   Bri IN   : 2/2/10 ⚠ CONFLICT
[KNX-UM]   Bri OUT  : 2/2/11 ⚠ CONFLICT
[KNX-UM] ==========================================
```

## Troubleshooting Test Failures

### Common Issues

1. **Tests not running**: Ensure `KNX_UM_DEBUG` is defined
2. **Compilation errors**: Check that test methods are declared in header
3. **Memory issues**: Tests save/restore configuration, ensure sufficient heap

### Debugging Steps

1. Check serial output for detailed test logs
2. Use `analyzeGAConflicts()` for detailed conflict analysis
3. Verify segment configuration with `printSegmentConfiguration()`
4. Test individual GA checking with `isGAInUse(ga)`

## Integration with CI/CD

The tests can be integrated into automated testing:

```cpp
bool runAutomatedTests() {
    KNX_UM_DEBUGF("[AUTO-TEST] Starting automated GA conflict tests\n");
    
    // Run tests and capture results
    // Return false if any test fails
    
    return true; // All tests passed
}
```

## Best Practices

1. **Run tests after configuration changes**: Always test after modifying GA settings or offsets
2. **Test with real segment counts**: Use actual segment numbers from your setup
3. **Validate before deployment**: Run full test suite before deploying to production
4. **Monitor test output**: Check for warnings and suggestions in test logs

## Test Coverage

The GA conflict detection tests cover:

- ✅ Basic conflict detection algorithms
- ✅ Edge cases (zero offsets, boundary limits)
- ✅ Integration with registration system
- ✅ Configuration save/restore
- ✅ Memory management
- ✅ Cross-segment validation
- ✅ Individual GA usage tracking

This comprehensive test suite ensures reliable GA conflict prevention in production deployments.