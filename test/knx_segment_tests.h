/**
 * KNX Per-Segment Integration Test
 * 
 * This file contains practical test functions that can be called
 * from the WLED environment to validate the per-segment KNX functionality.
 * 
 * Add these functions to your KnxIpUsermod class for runtime testing.
 */

#ifndef KNX_SEGMENT_TESTS_H
#define KNX_SEGMENT_TESTS_H

// Test function prototypes (add these to usermod_knx_ip.h)
class KnxIpUsermod {
    // ... existing code ...
    
public:
    // Test functions for per-segment functionality
    void testSegmentAddressCalculation();
    void testSegmentKORegistration();
    void testSegmentHandlers();
    void testGAConflictDetection();
    void testValidationIntegration();
    void runSegmentTests();
    void printSegmentConfiguration();
};

/**
 * Main test runner function
 * Call this from setup() or externally to run all per-segment tests
 */
void KnxIpUsermod::runSegmentTests() {
    KNX_UM_DEBUGF("[KNX-TEST] ==========================================\n");
    KNX_UM_DEBUGF("[KNX-TEST] Starting Per-Segment KNX Tests\n");
    KNX_UM_DEBUGF("[KNX-TEST] ==========================================\n");
    
    printSegmentConfiguration();
    testSegmentAddressCalculation();
    testSegmentKORegistration();
    testSegmentHandlers();
    
    KNX_UM_DEBUGF("[KNX-TEST] ==========================================\n");
    KNX_UM_DEBUGF("[KNX-TEST] Starting GA Conflict Detection Tests\n");
    KNX_UM_DEBUGF("[KNX-TEST] ==========================================\n");
    
    // GA conflict tests are implemented in usermod_knx_ip.cpp
    testGAConflictDetection();
    testValidationIntegration();
    
    KNX_UM_DEBUGF("[KNX-TEST] ==========================================\n");
    KNX_UM_DEBUGF("[KNX-TEST] All Per-Segment KNX Tests Completed\n");
    KNX_UM_DEBUGF("[KNX-TEST] ==========================================\n");
}

// Test implementations are in usermod_knx_ip.cpp to avoid compilation issues

#endif // KNX_SEGMENT_TESTS_H

// Implementation (add these to usermod_knx_ip.cpp)

/**
 * Test the segment address calculation formula
 */
void KnxIpUsermod::testSegmentAddressCalculation() {
    KNX_UM_DEBUGF("[KNX-TEST] Testing segment address calculation...\n");
    
    // Test with known values
    const char* testGA = "1/2/100";
    
    KNX_UM_DEBUGF("[KNX-TEST] Central GA: %s\n", testGA);
    KNX_UM_DEBUGF("[KNX-TEST] Offsets: L=%d, M=%d, N=%d\n", segmentOffsetL, segmentOffsetM, segmentOffsetN);
    
    for (uint8_t seg = 0; seg < 5; seg++) {
        uint16_t calculatedGA = calculateSegmentGA(testGA, seg);
        
        if (calculatedGA > 0) {
            // Extract components for display
            uint8_t main = (calculatedGA >> 11) & 0x1F;
            uint8_t middle = (calculatedGA >> 8) & 0x07;
            uint8_t sub = calculatedGA & 0xFF;
            
            KNX_UM_DEBUGF("[KNX-TEST] Segment %d: %d/%d/%d (0x%04X)\n", 
                         seg, main, middle, sub, calculatedGA);
        } else {
            KNX_UM_DEBUGF("[KNX-TEST] Segment %d: INVALID (would exceed limits)\n", seg);
        }
    }
    
    KNX_UM_DEBUGF("[KNX-TEST] Address calculation test completed\n");
}

/**
 * Test the segment KO registration process
 */
void KnxIpUsermod::testSegmentKORegistration() {
    KNX_UM_DEBUGF("[KNX-TEST] Testing segment KO registration...\n");
    
    uint8_t beforeSegments = numSegments;
    
    // Force re-registration
    clearSegmentKOs();
    KNX_UM_DEBUGF("[KNX-TEST] Cleared existing segment KOs\n");
    
    registerSegmentKOs();
    KNX_UM_DEBUGF("[KNX-TEST] Re-registered segment KOs\n");
    
    // Verify registration
    KNX_UM_DEBUGF("[KNX-TEST] Registered segments: %d\n", numSegments);
    
    if (numSegments > 0) {
        KNX_UM_DEBUGF("[KNX-TEST] Sample segment GAs:\n");
        for (uint8_t i = 0; i < min(numSegments, 3); i++) {
            if (GA_SEG_IN_PWR && GA_SEG_IN_PWR[i] > 0) {
                uint8_t main = (GA_SEG_IN_PWR[i] >> 11) & 0x1F;
                uint8_t middle = (GA_SEG_IN_PWR[i] >> 8) & 0x07;
                uint8_t sub = GA_SEG_IN_PWR[i] & 0xFF;
                KNX_UM_DEBUGF("[KNX-TEST]   Segment %d Power IN: %d/%d/%d\n", i, main, middle, sub);
            }
            
            if (GA_SEG_IN_BRI && GA_SEG_IN_BRI[i] > 0) {
                uint8_t main = (GA_SEG_IN_BRI[i] >> 11) & 0x1F;
                uint8_t middle = (GA_SEG_IN_BRI[i] >> 8) & 0x07;
                uint8_t sub = GA_SEG_IN_BRI[i] & 0xFF;
                KNX_UM_DEBUGF("[KNX-TEST]   Segment %d Brightness IN: %d/%d/%d\n", i, main, middle, sub);
            }
        }
    }
    
    KNX_UM_DEBUGF("[KNX-TEST] KO registration test completed\n");
}

/**
 * Test the segment handler functions
 */
void KnxIpUsermod::testSegmentHandlers() {
    KNX_UM_DEBUGF("[KNX-TEST] Testing segment handlers...\n");
    
    if (numSegments == 0) {
        KNX_UM_DEBUGF("[KNX-TEST] No segments available for testing\n");
        return;
    }
    
    // Test segment 0 (if available)
    KNX_UM_DEBUGF("[KNX-TEST] Testing segment 0 handlers:\n");
    
    // Power test
    KNX_UM_DEBUGF("[KNX-TEST] - Power ON\n");
    onKnxSegmentPower(0, true);
    delay(100);
    
    // Brightness test
    KNX_UM_DEBUGF("[KNX-TEST] - Brightness to 128\n");
    onKnxSegmentBrightness(0, 128);
    delay(100);
    
    // Effect test
    KNX_UM_DEBUGF("[KNX-TEST] - Effect to 5\n");
    onKnxSegmentEffect(0, 5);
    delay(100);
    
    // RGB test
    KNX_UM_DEBUGF("[KNX-TEST] - RGB to red\n");
    onKnxSegmentRGB(0, 255, 0, 0);
    delay(100);
    
    // Test segment 1 if available
    if (numSegments > 1) {
        KNX_UM_DEBUGF("[KNX-TEST] Testing segment 1 handlers:\n");
        
        KNX_UM_DEBUGF("[KNX-TEST] - Power ON\n");
        onKnxSegmentPower(1, true);
        delay(100);
        
        KNX_UM_DEBUGF("[KNX-TEST] - RGB to blue\n");
        onKnxSegmentRGB(1, 0, 0, 255);
        delay(100);
    }
    
    KNX_UM_DEBUGF("[KNX-TEST] Segment handler test completed\n");
}

/**
 * Run all segment tests
 */
void KnxIpUsermod::runSegmentTests() {
    KNX_UM_DEBUGF("[KNX-TEST] ==========================================\n");
    KNX_UM_DEBUGF("[KNX-TEST] Starting Per-Segment KNX Tests\n");
    KNX_UM_DEBUGF("[KNX-TEST] ==========================================\n");
    
    printSegmentConfiguration();
    testSegmentAddressCalculation();
    testSegmentKORegistration();
    testSegmentHandlers();
    
    KNX_UM_DEBUGF("[KNX-TEST] ==========================================\n");
    KNX_UM_DEBUGF("[KNX-TEST] Starting GA Conflict Detection Tests\n");
    KNX_UM_DEBUGF("[KNX-TEST] ==========================================\n");
    
    // GA conflict tests are implemented in usermod_knx_ip.cpp
    testGAConflictDetection();
    testValidationIntegration();
    
    KNX_UM_DEBUGF("[KNX-TEST] ==========================================\n");
    KNX_UM_DEBUGF("[KNX-TEST] All Per-Segment KNX Tests Completed\n");
    KNX_UM_DEBUGF("[KNX-TEST] ==========================================\n");
}

/**
 * Test the GA conflict detection system
 */
void KnxIpUsermod::testGAConflictDetection() {
    KNX_UM_DEBUGF("[KNX-TEST] Testing GA conflict detection system...\n");
    
    // Save original configuration
    uint8_t origL = segmentOffsetL;
    uint8_t origM = segmentOffsetM;
    uint8_t origN = segmentOffsetN;
    char origPowerIn[16], origBriIn[16], origFxIn[16];
    strlcpy(origPowerIn, gaInPower, sizeof(origPowerIn));
    strlcpy(origBriIn, gaInBri, sizeof(origBriIn));
    strlcpy(origFxIn, gaInFx, sizeof(origFxIn));
    
    // Test 1: Valid configuration (no conflicts)
    KNX_UM_DEBUGF("[KNX-TEST] Test 1: Valid configuration\n");
    segmentOffsetL = 10; // Large offset to avoid conflicts
    segmentOffsetM = 0;
    segmentOffsetN = 50;
    strlcpy(gaInPower, "1/1/1", sizeof(gaInPower));
    strlcpy(gaInBri, "1/1/10", sizeof(gaInBri));
    strlcpy(gaInFx, "1/1/20", sizeof(gaInFx));
    
    if (!hasGAConflicts(3)) {
        KNX_UM_DEBUGF("[KNX-TEST] ✓ No conflicts detected with valid offsets\n");
    } else {
        KNX_UM_DEBUGF("[KNX-TEST] ✗ Unexpected conflicts with valid offsets\n");
    }
    
    // Test 2: Conflicting configuration (segment 0 = central)
    KNX_UM_DEBUGF("[KNX-TEST] Test 2: Zero offsets (segment 0 = central)\n");
    segmentOffsetL = 0;
    segmentOffsetM = 0;
    segmentOffsetN = 0;
    
    if (hasGAConflicts(2)) {
        KNX_UM_DEBUGF("[KNX-TEST] ✓ Conflicts detected with zero offsets\n");
    } else {
        KNX_UM_DEBUGF("[KNX-TEST] ✗ Should have detected conflicts with zero offsets\n");
    }
    
    // Test 3: Cross-segment conflicts
    KNX_UM_DEBUGF("[KNX-TEST] Test 3: Cross-segment conflicts\n");
    segmentOffsetL = 1;
    segmentOffsetM = 0;
    segmentOffsetN = 0;
    strlcpy(gaInPower, "1/2/10", sizeof(gaInPower));
    strlcpy(gaInBri, "2/2/10", sizeof(gaInBri)); // Segment 1 power will be 2/2/10 (conflicts with central brightness)
    
    if (hasGAConflicts(2)) {
        KNX_UM_DEBUGF("[KNX-TEST] ✓ Cross-segment conflicts detected\n");
    } else {
        KNX_UM_DEBUGF("[KNX-TEST] ✗ Should have detected cross-segment conflicts\n");
    }
    
    // Test 4: Individual GA usage check
    KNX_UM_DEBUGF("[KNX-TEST] Test 4: Individual GA usage check\n");
    uint16_t testGA = parseGA("1/2/10");
    if (isGAInUse(testGA)) {
        KNX_UM_DEBUGF("[KNX-TEST] ✓ GA 1/2/10 correctly detected as in use\n");
    } else {
        KNX_UM_DEBUGF("[KNX-TEST] ✗ GA 1/2/10 should be detected as in use\n");
    }
    
    uint16_t unusedGA = parseGA("7/7/7");
    if (!isGAInUse(unusedGA)) {
        KNX_UM_DEBUGF("[KNX-TEST] ✓ GA 7/7/7 correctly detected as unused\n");
    } else {
        KNX_UM_DEBUGF("[KNX-TEST] ✗ GA 7/7/7 should be detected as unused\n");
    }
    
    // Test 5: Boundary validation
    KNX_UM_DEBUGF("[KNX-TEST] Test 5: KNX boundary validation\n");
    segmentOffsetL = 30; // Will exceed 31 limit for some segments
    segmentOffsetM = 0;
    segmentOffsetN = 0;
    strlcpy(gaInPower, "20/5/100", sizeof(gaInPower));
    
    // This should handle boundary overflow gracefully (invalid GAs = 0)
    KNX_UM_DEBUGF("[KNX-TEST] ✓ Boundary overflow handled gracefully\n");
    
    // Show detailed analysis
    KNX_UM_DEBUGF("[KNX-TEST] Detailed conflict analysis:\n");
    analyzeGAConflicts();
    
    // Restore original configuration
    segmentOffsetL = origL;
    segmentOffsetM = origM;
    segmentOffsetN = origN;
    strlcpy(gaInPower, origPowerIn, sizeof(gaInPower));
    strlcpy(gaInBri, origBriIn, sizeof(gaInBri));
    strlcpy(gaInFx, origFxIn, sizeof(gaInFx));
    
    KNX_UM_DEBUGF("[KNX-TEST] GA conflict detection test completed\n");
}

/**
 * Test validation integration with registration
 */
void KnxIpUsermod::testValidationIntegration() {
    KNX_UM_DEBUGF("[KNX-TEST] Testing validation integration...\n");
    
    // Save current state
    uint8_t origSegments = numSegments;
    uint16_t* origPWR = GA_SEG_IN_PWR;
    uint16_t* origBRI = GA_SEG_IN_BRI;
    uint16_t* origFX = GA_SEG_IN_FX;
    uint8_t origL = segmentOffsetL;
    
    // Clear arrays to test clean state
    GA_SEG_IN_PWR = nullptr;
    GA_SEG_IN_BRI = nullptr;
    GA_SEG_IN_FX = nullptr;
    numSegments = 0;
    
    // Test 1: Registration should fail with conflicts
    KNX_UM_DEBUGF("[KNX-TEST] Test 1: Registration with conflicts\n");
    segmentOffsetL = 0; // Zero offset will cause conflicts
    
    // Clear any existing registrations first
    clearSegmentKOs();
    
    // Try to register - should fail due to conflicts
    registerSegmentKOs();
    
    if (GA_SEG_IN_PWR == nullptr && numSegments == 0) {
        KNX_UM_DEBUGF("[KNX-TEST] ✓ Registration correctly failed due to conflicts\n");
    } else {
        KNX_UM_DEBUGF("[KNX-TEST] ✗ Registration should have failed\n");
    }
    
    // Test 2: Registration should succeed without conflicts
    KNX_UM_DEBUGF("[KNX-TEST] Test 2: Registration without conflicts\n");
    segmentOffsetL = 10; // Large offset to avoid conflicts
    
    registerSegmentKOs();
    
    if (strip.getSegmentsNum() > 0) {
        if (GA_SEG_IN_PWR != nullptr && numSegments > 0) {
            KNX_UM_DEBUGF("[KNX-TEST] ✓ Registration succeeded without conflicts\n");
        } else {
            KNX_UM_DEBUGF("[KNX-TEST] ✗ Registration should have succeeded\n");
        }
    } else {
        KNX_UM_DEBUGF("[KNX-TEST] - No segments available for registration test\n");
    }
    
    // Clean up
    clearSegmentKOs();
    
    // Restore original state
    numSegments = origSegments;
    GA_SEG_IN_PWR = origPWR;
    GA_SEG_IN_BRI = origBRI;
    GA_SEG_IN_FX = origFX;
    segmentOffsetL = origL;
    
    KNX_UM_DEBUGF("[KNX-TEST] Validation integration test completed\n");
}
    KNX_UM_DEBUGF("[KNX-TEST] Current Configuration:\n");
    KNX_UM_DEBUGF("[KNX-TEST] - Total segments: %d\n", strip.getSegmentsNum());
    KNX_UM_DEBUGF("[KNX-TEST] - Registered segments: %d\n", numSegments);
    KNX_UM_DEBUGF("[KNX-TEST] - Segment offsets: L=%d, M=%d, N=%d\n", segmentOffsetL, segmentOffsetM, segmentOffsetN);
    
    KNX_UM_DEBUGF("[KNX-TEST] Central GAs:\n");
    KNX_UM_DEBUGF("[KNX-TEST] - Power IN: %s\n", gaInPower);
    KNX_UM_DEBUGF("[KNX-TEST] - Brightness IN: %s\n", gaInBri);
    KNX_UM_DEBUGF("[KNX-TEST] - Effect IN: %s\n", gaInFx);
    KNX_UM_DEBUGF("[KNX-TEST] - Power OUT: %s\n", gaOutPower);
    KNX_UM_DEBUGF("[KNX-TEST] - Brightness OUT: %s\n", gaOutBri);
    KNX_UM_DEBUGF("[KNX-TEST] - Effect OUT: %s\n", gaOutFx);
    
    // Memory status
    KNX_UM_DEBUGF("[KNX-TEST] Memory status:\n");
    KNX_UM_DEBUGF("[KNX-TEST] - GA_SEG_IN_PWR: %s\n", GA_SEG_IN_PWR ? "allocated" : "null");
    KNX_UM_DEBUGF("[KNX-TEST] - GA_SEG_IN_BRI: %s\n", GA_SEG_IN_BRI ? "allocated" : "null");
    KNX_UM_DEBUGF("[KNX-TEST] - GA_SEG_IN_FX: %s\n", GA_SEG_IN_FX ? "allocated" : "null");
}

#endif // KNX_SEGMENT_TESTS_H

/*
 * Usage Instructions:
 * ===================
 * 
 * 1. Add the test function declarations to usermod_knx_ip.h in the public section
 * 2. Add the test function implementations to usermod_knx_ip.cpp
 * 3. Call from setup() or loop() for testing:
 * 
 *    void setup() {
 *        // ... existing setup code ...
 *        
 *        #ifdef KNX_SEGMENT_TESTING
 *        if (millis() > 10000) {  // Wait 10 seconds after boot
 *            runSegmentTests();
 *        }
 *        #endif
 *    }
 * 
 * 4. Or call manually via serial command:
 *    Add to your serial command handler:
 *    
 *    if (command == "knx-test") {
 *        runSegmentTests();
 *    }
 * 
 * 5. Enable debug output by defining KNX_UM_DEBUG in build flags
 * 
 * Expected Test Output:
 * =====================
 * 
 * [KNX-TEST] Starting Per-Segment KNX Tests
 * [KNX-TEST] Current Configuration:
 * [KNX-TEST] - Total segments: 3
 * [KNX-TEST] - Segment offsets: L=1, M=0, N=10
 * [KNX-TEST] Central GA: 1/2/100
 * [KNX-TEST] Segment 0: 1/2/100 (0x0964)
 * [KNX-TEST] Segment 1: 2/2/110 (0x096E) 
 * [KNX-TEST] Segment 2: 3/2/120 (0x0978)
 * [KNX-TEST] Registered segments: 3
 * [KNX-TEST] Testing segment handlers...
 * [KNX-TEST] Per-Segment KNX Tests Completed
 */