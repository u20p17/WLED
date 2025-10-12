/**
 * KNX Per-Segment Unit Tests
 * 
 * Simple unit tests for the calculateSegmentGA function that can be
 * easily run during development or as part of CI/CD.
 */

// Add this to the end of usermod_knx_ip.cpp for testing

#ifdef KNX_SEGMENT_UNIT_TESTS

/**
 * Simple assertion macro for testing
 */
#define KNX_ASSERT(condition, message) \
    if (!(condition)) { \
        KNX_UM_DEBUGF("[KNX-UNIT-TEST] FAILED: %s\n", message); \
        return false; \
    }

/**
 * Unit test for calculateSegmentGA function
 */
bool KnxIpUsermod::unitTestCalculateSegmentGA() {
    KNX_UM_DEBUGF("[KNX-UNIT-TEST] Testing calculateSegmentGA function...\n");
    
    // Setup test configuration
    segmentOffsetL = 1;
    segmentOffsetM = 0; 
    segmentOffsetN = 10;
    
    // Test 1: Segment 0 should equal central
    uint16_t central = parseGA("1/2/50");
    uint16_t seg0 = calculateSegmentGA("1/2/50", 0);
    KNX_ASSERT(seg0 == central, "Segment 0 should equal central GA");
    
    // Test 2: Segment 1 calculation
    uint16_t seg1 = calculateSegmentGA("1/2/50", 1);
    uint16_t expected1 = knxMakeGroupAddress(2, 2, 60); // 1+1, 2+0, 50+10
    KNX_ASSERT(seg1 == expected1, "Segment 1 calculation incorrect");
    
    // Test 3: Segment 2 calculation  
    uint16_t seg2 = calculateSegmentGA("1/2/50", 2);
    uint16_t expected2 = knxMakeGroupAddress(3, 2, 70); // 1+2, 2+0, 50+20
    KNX_ASSERT(seg2 == expected2, "Segment 2 calculation incorrect");
    
    // Test 4: Boundary validation - should return 0 for overflow
    uint16_t overflow = calculateSegmentGA("30/7/250", 2); // Would be 32/7/270 (invalid)
    KNX_ASSERT(overflow == 0, "Boundary validation failed - should return 0 for overflow");
    
    // Test 5: Empty GA string
    uint16_t empty = calculateSegmentGA("", 1);
    KNX_ASSERT(empty == 0, "Empty GA string should return 0");
    
    // Test 6: Invalid GA string
    uint16_t invalid = calculateSegmentGA("invalid", 1);
    KNX_ASSERT(invalid == 0, "Invalid GA string should return 0");
    
    // Test 7: Different offset configuration
    segmentOffsetL = 2;
    segmentOffsetM = 1;
    segmentOffsetN = 5;
    
    uint16_t seg1_new = calculateSegmentGA("5/3/100", 1);
    uint16_t expected1_new = knxMakeGroupAddress(7, 4, 105); // 5+2, 3+1, 100+5
    KNX_ASSERT(seg1_new == expected1_new, "Different offset configuration failed");
    
    // Reset to original offsets
    segmentOffsetL = 1;
    segmentOffsetM = 0;
    segmentOffsetN = 10;
    
    KNX_UM_DEBUGF("[KNX-UNIT-TEST] calculateSegmentGA tests PASSED\n");
    return true;
}

/**
 * Unit test for memory management
 */
bool KnxIpUsermod::unitTestMemoryManagement() {
    KNX_UM_DEBUGF("[KNX-UNIT-TEST] Testing memory management...\n");
    
    // Test initial state
    KNX_ASSERT(GA_SEG_IN_PWR == nullptr, "Initial GA_SEG_IN_PWR should be null");
    KNX_ASSERT(GA_SEG_IN_BRI == nullptr, "Initial GA_SEG_IN_BRI should be null");
    KNX_ASSERT(GA_SEG_IN_FX == nullptr, "Initial GA_SEG_IN_FX should be null");
    KNX_ASSERT(numSegments == 0, "Initial numSegments should be 0");
    
    // Simulate allocation (manual test since we can't mock strip easily)
    const uint8_t testSegments = 3;
    
    // Clear first to ensure clean state
    clearSegmentKOs();
    
    // Manual allocation for testing
    numSegments = testSegments;
    GA_SEG_IN_PWR = new uint16_t[testSegments]();
    GA_SEG_IN_BRI = new uint16_t[testSegments]();
    GA_SEG_IN_FX = new uint16_t[testSegments]();
    GA_SEG_OUT_PWR = new uint16_t[testSegments]();
    GA_SEG_OUT_BRI = new uint16_t[testSegments]();
    GA_SEG_OUT_FX = new uint16_t[testSegments]();
    
    // Test allocation
    KNX_ASSERT(GA_SEG_IN_PWR != nullptr, "GA_SEG_IN_PWR allocation failed");
    KNX_ASSERT(GA_SEG_IN_BRI != nullptr, "GA_SEG_IN_BRI allocation failed");
    KNX_ASSERT(GA_SEG_IN_FX != nullptr, "GA_SEG_IN_FX allocation failed");
    KNX_ASSERT(numSegments == testSegments, "numSegments not set correctly");
    
    // Test array initialization (should be zero-initialized)
    KNX_ASSERT(GA_SEG_IN_PWR[0] == 0, "Array not zero-initialized");
    KNX_ASSERT(GA_SEG_IN_BRI[1] == 0, "Array not zero-initialized");
    KNX_ASSERT(GA_SEG_IN_FX[2] == 0, "Array not zero-initialized");
    
    // Test cleanup
    clearSegmentKOs();
    
    KNX_ASSERT(GA_SEG_IN_PWR == nullptr, "GA_SEG_IN_PWR not cleaned up");
    KNX_ASSERT(GA_SEG_IN_BRI == nullptr, "GA_SEG_IN_BRI not cleaned up");
    KNX_ASSERT(GA_SEG_IN_FX == nullptr, "GA_SEG_IN_FX not cleaned up");
    KNX_ASSERT(numSegments == 0, "numSegments not reset");
    
    KNX_UM_DEBUGF("[KNX-UNIT-TEST] Memory management tests PASSED\n");
    return true;
}

/**
 * Run all unit tests
 */
bool KnxIpUsermod::runUnitTests() {
    KNX_UM_DEBUGF("[KNX-UNIT-TEST] ================================\n");
    KNX_UM_DEBUGF("[KNX-UNIT-TEST] Starting KNX Per-Segment Unit Tests\n");
    KNX_UM_DEBUGF("[KNX-UNIT-TEST] ================================\n");
    
    bool allPassed = true;
    
    if (!unitTestCalculateSegmentGA()) {
        allPassed = false;
    }
    
    if (!unitTestMemoryManagement()) {
        allPassed = false;
    }
    
    if (allPassed) {
        KNX_UM_DEBUGF("[KNX-UNIT-TEST] ================================\n");
        KNX_UM_DEBUGF("[KNX-UNIT-TEST] ✅ ALL UNIT TESTS PASSED\n");
        KNX_UM_DEBUGF("[KNX-UNIT-TEST] ================================\n");
    } else {
        KNX_UM_DEBUGF("[KNX-UNIT-TEST] ================================\n");
        KNX_UM_DEBUGF("[KNX-UNIT-TEST] ❌ SOME UNIT TESTS FAILED\n");
        KNX_UM_DEBUGF("[KNX-UNIT-TEST] ================================\n");
    }
    
    return allPassed;
}

#endif // KNX_SEGMENT_UNIT_TESTS

/*
 * To enable unit tests:
 * =====================
 * 
 * 1. Add to build_flags in platformio.ini:
 *    build_flags = 
 *        -D KNX_SEGMENT_UNIT_TESTS
 *        -D KNX_UM_DEBUG
 * 
 * 2. Add function declarations to usermod_knx_ip.h:
 *    #ifdef KNX_SEGMENT_UNIT_TESTS
 *    bool unitTestCalculateSegmentGA();
 *    bool unitTestMemoryManagement();
 *    bool runUnitTests();
 *    #endif
 * 
 * 3. Call from setup() after KNX initialization:
 *    #ifdef KNX_SEGMENT_UNIT_TESTS
 *    runUnitTests();
 *    #endif
 * 
 * 4. Or add to web interface for manual testing
 * 
 * Example Output:
 * ===============
 * [KNX-UNIT-TEST] Starting KNX Per-Segment Unit Tests
 * [KNX-UNIT-TEST] Testing calculateSegmentGA function...
 * [KNX-UNIT-TEST] calculateSegmentGA tests PASSED
 * [KNX-UNIT-TEST] Testing memory management...
 * [KNX-UNIT-TEST] Memory management tests PASSED
 * [KNX-UNIT-TEST] ✅ ALL UNIT TESTS PASSED
 */