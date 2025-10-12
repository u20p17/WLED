/**
 * KNX GA Conflict Detection Tests
 * 
 * Tests for the GA conflict detection system to ensure
 * segment GAs don't conflict with existing central GAs.
 */

// Add these test functions to knx_unit_tests.cpp

#ifdef KNX_SEGMENT_UNIT_TESTS

/**
 * Unit test for GA conflict detection
 */
bool KnxIpUsermod::unitTestGAConflictDetection() {
    KNX_UM_DEBUGF("[KNX-UNIT-TEST] Testing GA conflict detection...\n");
    
    // Save original configuration
    uint8_t origL = segmentOffsetL;
    uint8_t origM = segmentOffsetM;
    uint8_t origN = segmentOffsetN;
    char origPowerIn[16], origBriIn[16];
    strlcpy(origPowerIn, gaInPower, sizeof(origPowerIn));
    strlcpy(origBriIn, gaInBri, sizeof(origBriIn));
    
    // Test 1: No conflicts with proper offsets
    segmentOffsetL = 10; // Large offset to avoid conflicts
    segmentOffsetM = 0;
    segmentOffsetN = 100;
    strlcpy(gaInPower, "1/1/1", sizeof(gaInPower));
    strlcpy(gaInBri, "1/1/2", sizeof(gaInBri));
    
    KNX_ASSERT(!hasGAConflicts(2), "Should have no conflicts with large offsets");
    
    // Test 2: Conflict with central GA
    segmentOffsetL = 0; // Zero offset means segment 0 = central
    segmentOffsetM = 0;
    segmentOffsetN = 0;
    
    KNX_ASSERT(hasGAConflicts(2), "Should detect conflict when segment 0 = central");
    
    // Test 3: Duplicate segment GAs
    segmentOffsetL = 0; // All segments will have same GA
    segmentOffsetM = 0;
    segmentOffsetN = 0;
    strlcpy(gaInPower, "5/5/5", sizeof(gaInPower)); // Use different GA to avoid central conflict
    strlcpy(gaInBri, "5/5/6", sizeof(gaInBri));
    
    KNX_ASSERT(hasGAConflicts(3), "Should detect duplicate segment GAs");
    
    // Test 4: Boundary overflow
    segmentOffsetL = 30; // Will exceed 31 limit
    segmentOffsetM = 0;
    segmentOffsetN = 0;
    strlcpy(gaInPower, "20/5/100", sizeof(gaInPower));
    
    KNX_ASSERT(!hasGAConflicts(2), "Should handle boundary overflow gracefully (invalid GAs = 0)");
    
    // Test 5: GA collision with existing central GAs
    segmentOffsetL = 1;
    segmentOffsetM = 0;
    segmentOffsetN = 0;
    strlcpy(gaInPower, "1/1/10", sizeof(gaInPower));
    strlcpy(gaInBri, "2/1/10", sizeof(gaInBri)); // Segment 1 power will be 2/1/10, same as central brightness
    
    KNX_ASSERT(hasGAConflicts(2), "Should detect collision between segment and central GAs");
    
    // Test 6: Individual GA usage check
    uint16_t testGA = parseGA("3/3/3");
    strlcpy(gaInPower, "3/3/3", sizeof(gaInPower)); // Make this GA "in use"
    KNX_ASSERT(isGAInUse(testGA), "Should detect GA is in use");
    
    uint16_t unusedGA = parseGA("7/7/7");
    KNX_ASSERT(!isGAInUse(unusedGA), "Should detect GA is not in use");
    
    // Test 7: getAllUsedGAs completeness
    auto allGAs = getAllUsedGAs();
    bool foundPowerGA = std::find(allGAs.begin(), allGAs.end(), testGA) != allGAs.end();
    KNX_ASSERT(foundPowerGA, "getAllUsedGAs should include central power GA");
    
    // Restore original configuration
    segmentOffsetL = origL;
    segmentOffsetM = origM;
    segmentOffsetN = origN;
    strlcpy(gaInPower, origPowerIn, sizeof(gaInPower));
    strlcpy(gaInBri, origBriIn, sizeof(gaInBri));
    
    KNX_UM_DEBUGF("[KNX-UNIT-TEST] GA conflict detection tests PASSED\n");
    return true;
}

/**
 * Test conflict analysis output
 */
bool KnxIpUsermod::unitTestConflictAnalysis() {
    KNX_UM_DEBUGF("[KNX-UNIT-TEST] Testing conflict analysis output...\n");
    
    // Set up conflicting configuration
    segmentOffsetL = 1;
    segmentOffsetM = 0;
    segmentOffsetN = 0;
    strlcpy(gaInPower, "1/1/1", sizeof(gaInPower));
    strlcpy(gaInBri, "2/1/1", sizeof(gaInBri)); // Will conflict with segment 1 power
    
    // This should show conflicts in debug output
    analyzeGAConflicts();
    
    // Just verify the function runs without crashing
    KNX_UM_DEBUGF("[KNX-UNIT-TEST] Conflict analysis completed (check output above)\n");
    return true;
}

/**
 * Test validation integration
 */
bool KnxIpUsermod::unitTestValidationIntegration() {
    KNX_UM_DEBUGF("[KNX-UNIT-TEST] Testing validation integration...\n");
    
    // Save current state
    uint8_t origSegments = numSegments;
    uint16_t* origPWR = GA_SEG_IN_PWR;
    uint16_t* origBRI = GA_SEG_IN_BRI;
    uint16_t* origFX = GA_SEG_IN_FX;
    
    // Clear arrays to test clean state
    GA_SEG_IN_PWR = nullptr;
    GA_SEG_IN_BRI = nullptr;
    GA_SEG_IN_FX = nullptr;
    numSegments = 0;
    
    // Set up conflicting configuration that should prevent registration
    segmentOffsetL = 0;
    segmentOffsetM = 0; 
    segmentOffsetN = 0;
    
    // Try to register - should fail due to conflicts
    registerSegmentKOs();
    
    // Should have failed and not allocated arrays
    KNX_ASSERT(GA_SEG_IN_PWR == nullptr, "Registration should have failed due to conflicts");
    KNX_ASSERT(numSegments == 0, "Segment count should remain 0 after failed registration");
    
    // Test with valid configuration
    segmentOffsetL = 10;
    segmentOffsetM = 0;
    segmentOffsetN = 50;
    
    // This should succeed (if segments exist)
    registerSegmentKOs();
    
    // Clean up
    clearSegmentKOs();
    
    // Restore original state
    numSegments = origSegments;
    GA_SEG_IN_PWR = origPWR;
    GA_SEG_IN_BRI = origBRI;
    GA_SEG_IN_FX = origFX;
    
    KNX_UM_DEBUGF("[KNX-UNIT-TEST] Validation integration tests PASSED\n");
    return true;
}

/**
 * Update the main unit test runner to include conflict detection tests
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
    
    if (!unitTestGAConflictDetection()) {
        allPassed = false;
    }
    
    if (!unitTestConflictAnalysis()) {
        allPassed = false;
    }
    
    if (!unitTestValidationIntegration()) {
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
 * Additional Test Declarations for usermod_knx_ip.h:
 * ==================================================
 * 
 * Add these to the header file in the unit test section:
 * 
 * #ifdef KNX_SEGMENT_UNIT_TESTS
 * bool unitTestCalculateSegmentGA();
 * bool unitTestMemoryManagement();
 * bool unitTestGAConflictDetection();
 * bool unitTestConflictAnalysis();
 * bool unitTestValidationIntegration();
 * bool runUnitTests();
 * #endif
 * 
 * Manual Testing Scenarios:
 * ========================
 * 
 * Scenario 1: Conflicting Configuration
 * -------------------------------------
 * 1. Set central power GA: "1/2/10"
 * 2. Set central brightness GA: "2/2/10" 
 * 3. Set segment offsets: L=1, M=0, N=0
 * 4. Expected: Segment 1 power GA will be "2/2/10" (conflicts with central brightness)
 * 5. Result: Registration should fail with conflict warning
 * 
 * Scenario 2: Valid Configuration  
 * -------------------------------
 * 1. Set central power GA: "1/2/10"
 * 2. Set central brightness GA: "1/2/20"
 * 3. Set segment offsets: L=1, M=0, N=100
 * 4. Expected: No conflicts
 * 5. Result: Registration should succeed
 * 
 * Scenario 3: Boundary Testing
 * ----------------------------
 * 1. Set central GA: "30/7/250"
 * 2. Set segment offsets: L=2, M=1, N=10
 * 3. Expected: Segment 1+ would exceed KNX limits
 * 4. Result: Invalid GAs return 0, no conflicts reported
 * 
 * Expected Debug Output:
 * =====================
 * 
 * With conflicts:
 * [KNX-UM][CONFLICT] Segment GA 2/2/10 (0x1011) conflicts with existing GA
 * [KNX-UM][ERROR] GA conflicts detected! Skipping segment KO registration
 * 
 * Without conflicts:
 * [KNX-UM] Segment GA validation passed - no conflicts detected
 * [KNX-UM] Registering per-segment KOs for 3 segments
 */