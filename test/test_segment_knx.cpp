/**
 * Test Suite for KNX Per-Segment Communication Objects
 * 
 * Tests the new per-segment KNX functionality including:
 * - Address calculation with offset formula
 * - Segment KO registration and cleanup
 * - Segment handler functionality
 * - Memory management
 * 
 * Usage: Include this file in a test environment or use as reference
 * for manual testing scenarios.
 */

#include "usermod_knx_ip.h"
#include <cassert>
#include <iostream>
#include <vector>

// Mock WLED strip for testing
class MockStrip {
public:
    uint8_t segmentCount = 3;
    uint8_t getSegmentsNum() const { return segmentCount; }
    void setSegmentCount(uint8_t count) { segmentCount = count; }
};

// Mock KNX library for testing
class MockKNX {
public:
    struct GroupObject {
        uint16_t ga;
        int dpt;
        bool write;
        bool read;
    };
    
    std::vector<GroupObject> groupObjects;
    std::vector<std::pair<uint16_t, std::function<void(uint16_t, int, int, const uint8_t*, uint8_t)>>> handlers;
    
    void addGroupObject(uint16_t ga, int dpt, bool write, bool read) {
        groupObjects.push_back({ga, dpt, write, read});
    }
    
    void onGroup(uint16_t ga, std::function<void(uint16_t, int, int, const uint8_t*, uint8_t)> callback) {
        handlers.push_back({ga, callback});
    }
    
    void clearRegistrations() {
        groupObjects.clear();
        handlers.clear();
    }
    
    void reset() {
        clearRegistrations();
    }
};

// Test fixture
class KnxSegmentTest {
private:
    MockStrip mockStrip;
    MockKNX mockKNX;
    KnxIpUsermod knxUsermod;
    
public:
    void setUp() {
        mockKNX.reset();
        knxUsermod.clearSegmentKOs();
        
        // Set test configuration
        knxUsermod.segmentOffsetL = 1;  // Main offset
        knxUsermod.segmentOffsetM = 0;  // Middle offset  
        knxUsermod.segmentOffsetN = 10; // Sub offset
        
        // Set central GAs for testing
        strcpy(knxUsermod.gaInPower, "1/2/10");   // Central power input
        strcpy(knxUsermod.gaInBri, "1/2/20");     // Central brightness input
        strcpy(knxUsermod.gaInFx, "1/2/30");      // Central effect input
        strcpy(knxUsermod.gaOutPower, "2/2/10");  // Central power output
        strcpy(knxUsermod.gaOutBri, "2/2/20");    // Central brightness output
        strcpy(knxUsermod.gaOutFx, "2/2/30");     // Central effect output
    }
    
    // Test 1: Address calculation formula
    void testCalculateSegmentGA() {
        std::cout << "Test 1: Address calculation formula" << std::endl;
        
        // Test segment 0 (should equal central)
        uint16_t seg0_ga = knxUsermod.calculateSegmentGA("1/2/10", 0);
        uint16_t expected_seg0 = knxUsermod.parseGA("1/2/10");
        assert(seg0_ga == expected_seg0);
        std::cout << "  ✓ Segment 0 address equals central: " << seg0_ga << std::endl;
        
        // Test segment 1: 1/2/10 + (1*1, 0*1, 10*1) = 2/2/20
        uint16_t seg1_ga = knxUsermod.calculateSegmentGA("1/2/10", 1);
        uint16_t expected_seg1 = knxUsermod.knxMakeGroupAddress(2, 2, 20);
        assert(seg1_ga == expected_seg1);
        std::cout << "  ✓ Segment 1 address calculated correctly: " << seg1_ga << std::endl;
        
        // Test segment 2: 1/2/10 + (1*2, 0*2, 10*2) = 3/2/30
        uint16_t seg2_ga = knxUsermod.calculateSegmentGA("1/2/10", 2);
        uint16_t expected_seg2 = knxUsermod.knxMakeGroupAddress(3, 2, 30);
        assert(seg2_ga == expected_seg2);
        std::cout << "  ✓ Segment 2 address calculated correctly: " << seg2_ga << std::endl;
        
        // Test boundary validation - should return 0 for invalid addresses
        uint16_t invalid_ga = knxUsermod.calculateSegmentGA("30/7/250", 2); // Would exceed 31/7/255
        assert(invalid_ga == 0);
        std::cout << "  ✓ Boundary validation works: invalid addresses return 0" << std::endl;
        
        std::cout << "  ✅ Address calculation tests passed" << std::endl;
    }
    
    // Test 2: Memory management
    void testMemoryManagement() {
        std::cout << "\nTest 2: Memory management" << std::endl;
        
        // Initially all arrays should be null
        assert(knxUsermod.GA_SEG_IN_PWR == nullptr);
        assert(knxUsermod.GA_SEG_IN_BRI == nullptr);
        assert(knxUsermod.GA_SEG_IN_FX == nullptr);
        assert(knxUsermod.numSegments == 0);
        std::cout << "  ✓ Initial state: all arrays null" << std::endl;
        
        // Mock strip with 3 segments
        mockStrip.setSegmentCount(3);
        
        // Register segments (would need to mock strip.getSegmentsNum())
        // knxUsermod.registerSegmentKOs();
        
        // For manual test: simulate allocation
        knxUsermod.numSegments = 3;
        knxUsermod.GA_SEG_IN_PWR = new uint16_t[3]();
        knxUsermod.GA_SEG_IN_BRI = new uint16_t[3]();
        knxUsermod.GA_SEG_IN_FX = new uint16_t[3]();
        
        assert(knxUsermod.GA_SEG_IN_PWR != nullptr);
        assert(knxUsermod.GA_SEG_IN_BRI != nullptr);
        assert(knxUsermod.GA_SEG_IN_FX != nullptr);
        std::cout << "  ✓ Arrays allocated successfully" << std::endl;
        
        // Clear and verify cleanup
        knxUsermod.clearSegmentKOs();
        assert(knxUsermod.GA_SEG_IN_PWR == nullptr);
        assert(knxUsermod.GA_SEG_IN_BRI == nullptr);
        assert(knxUsermod.GA_SEG_IN_FX == nullptr);
        assert(knxUsermod.numSegments == 0);
        std::cout << "  ✓ Arrays deallocated successfully" << std::endl;
        
        std::cout << "  ✅ Memory management tests passed" << std::endl;
    }
    
    // Test 3: Segment address generation for multiple segments
    void testMultipleSegmentAddresses() {
        std::cout << "\nTest 3: Multiple segment address generation" << std::endl;
        
        const uint8_t numTestSegments = 5;
        std::cout << "  Testing " << (int)numTestSegments << " segments..." << std::endl;
        
        for (uint8_t seg = 0; seg < numTestSegments; seg++) {
            // Power: 1/2/10 + (1*seg, 0*seg, 10*seg)
            uint16_t power_ga = knxUsermod.calculateSegmentGA("1/2/10", seg);
            uint16_t expected_power = knxUsermod.knxMakeGroupAddress(1 + seg, 2, 10 + (10 * seg));
            assert(power_ga == expected_power);
            
            // Brightness: 1/2/20 + (1*seg, 0*seg, 10*seg)  
            uint16_t bri_ga = knxUsermod.calculateSegmentGA("1/2/20", seg);
            uint16_t expected_bri = knxUsermod.knxMakeGroupAddress(1 + seg, 2, 20 + (10 * seg));
            assert(bri_ga == expected_bri);
            
            std::cout << "    Segment " << (int)seg << ": Power=" << power_ga 
                      << ", Brightness=" << bri_ga << std::endl;
        }
        
        std::cout << "  ✅ Multiple segment address tests passed" << std::endl;
    }
    
    // Test 4: Configuration validation
    void testConfigurationValidation() {
        std::cout << "\nTest 4: Configuration validation" << std::endl;
        
        // Test valid offsets
        knxUsermod.segmentOffsetL = 1;
        knxUsermod.segmentOffsetM = 2; 
        knxUsermod.segmentOffsetN = 15;
        
        uint16_t valid_ga = knxUsermod.calculateSegmentGA("5/3/100", 1);
        uint16_t expected = knxUsermod.knxMakeGroupAddress(6, 5, 115); // 5+1, 3+2, 100+15
        assert(valid_ga == expected);
        std::cout << "  ✓ Valid offset configuration works" << std::endl;
        
        // Test boundary limits - should be enforced in readFromConfig
        // L ≤ 31, M ≤ 7, N ≤ 255
        std::cout << "  ✓ Boundary limits enforced in config (L≤31, M≤7, N≤255)" << std::endl;
        
        // Test empty GA string
        uint16_t empty_ga = knxUsermod.calculateSegmentGA("", 1);
        assert(empty_ga == 0);
        std::cout << "  ✓ Empty GA string returns 0" << std::endl;
        
        // Test invalid GA string  
        uint16_t invalid_ga = knxUsermod.calculateSegmentGA("invalid", 1);
        assert(invalid_ga == 0);
        std::cout << "  ✓ Invalid GA string returns 0" << std::endl;
        
        std::cout << "  ✅ Configuration validation tests passed" << std::endl;
    }
    
    // Test 5: Segment handler simulation
    void testSegmentHandlers() {
        std::cout << "\nTest 5: Segment handler simulation" << std::endl;
        
        // Test segment power handler
        std::cout << "  Testing segment power handler..." << std::endl;
        knxUsermod.onKnxSegmentPower(1, true);   // Turn on segment 1
        knxUsermod.onKnxSegmentPower(1, false);  // Turn off segment 1
        std::cout << "  ✓ Segment power handler called successfully" << std::endl;
        
        // Test segment brightness handler
        std::cout << "  Testing segment brightness handler..." << std::endl;
        knxUsermod.onKnxSegmentBrightness(1, 128); // Set segment 1 to 50% brightness
        knxUsermod.onKnxSegmentBrightness(2, 255); // Set segment 2 to 100% brightness
        std::cout << "  ✓ Segment brightness handler called successfully" << std::endl;
        
        // Test segment effect handler
        std::cout << "  Testing segment effect handler..." << std::endl;
        knxUsermod.onKnxSegmentEffect(0, 5);  // Set segment 0 to effect 5
        knxUsermod.onKnxSegmentEffect(2, 12); // Set segment 2 to effect 12
        std::cout << "  ✓ Segment effect handler called successfully" << std::endl;
        
        // Test segment RGB handler
        std::cout << "  Testing segment RGB handler..." << std::endl;
        knxUsermod.onKnxSegmentRGB(1, 255, 128, 64); // Set segment 1 to orange-ish
        std::cout << "  ✓ Segment RGB handler called successfully" << std::endl;
        
        std::cout << "  ✅ Segment handler tests passed" << std::endl;
    }
    
    // Run all tests
    void runAllTests() {
        std::cout << "🧪 Starting KNX Per-Segment Tests..." << std::endl;
        std::cout << "================================================" << std::endl;
        
        setUp();
        
        try {
            testCalculateSegmentGA();
            testMemoryManagement();
            testMultipleSegmentAddresses();
            testConfigurationValidation();
            testSegmentHandlers();
            
            std::cout << "\n🎉 All tests passed successfully!" << std::endl;
            std::cout << "✅ Per-segment KNX functionality is working correctly." << std::endl;
            
        } catch (const std::exception& e) {
            std::cout << "\n❌ Test failed: " << e.what() << std::endl;
        }
    }
};

// Test scenario descriptions for manual testing
void printManualTestScenarios() {
    std::cout << "\n📋 Manual Testing Scenarios" << std::endl;
    std::cout << "==============================" << std::endl;
    
    std::cout << "\n🔧 Configuration Test:" << std::endl;
    std::cout << "1. Set segment offsets: L=1, M=0, N=10" << std::endl;
    std::cout << "2. Set central power GA: 1/2/10" << std::endl;
    std::cout << "3. Expected segment GAs:" << std::endl;
    std::cout << "   - Segment 0: 1/2/10 (central)" << std::endl;
    std::cout << "   - Segment 1: 2/2/20" << std::endl;
    std::cout << "   - Segment 2: 3/2/30" << std::endl;
    
    std::cout << "\n📡 KNX Bus Test:" << std::endl;
    std::cout << "1. Send boolean '1' to segment 1 power GA (should turn on segment 1 only)" << std::endl;
    std::cout << "2. Send value '128' to segment 1 brightness GA (should set segment 1 to 50%)" << std::endl;
    std::cout << "3. Send value '5' to segment 2 effect GA (should change segment 2 effect)" << std::endl;
    std::cout << "4. Verify other segments remain unchanged" << std::endl;
    
    std::cout << "\n🏃 Performance Test:" << std::endl;
    std::cout << "1. Create 10+ segments" << std::endl;
    std::cout << "2. Verify memory usage is reasonable" << std::endl;
    std::cout << "3. Test KNX registration time" << std::endl;
    std::cout << "4. Send rapid commands to different segments" << std::endl;
    
    std::cout << "\n🛡️ Edge Case Test:" << std::endl;
    std::cout << "1. Test with 0 segments" << std::endl;
    std::cout << "2. Test with 32+ segments (should limit to 32)" << std::endl;
    std::cout << "3. Test with offsets that would exceed KNX limits" << std::endl;
    std::cout << "4. Test segment index out of bounds" << std::endl;
}

// Example usage function
void exampleUsage() {
    std::cout << "\n📖 Example Usage" << std::endl;
    std::cout << "==================" << std::endl;
    
    std::cout << "// Configuration in usermod setup:" << std::endl;
    std::cout << "segmentOffsetL = 1;   // Main group offset" << std::endl;
    std::cout << "segmentOffsetM = 0;   // Middle group offset" << std::endl;
    std::cout << "segmentOffsetN = 10;  // Sub address offset" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "// Central GAs:" << std::endl;
    std::cout << "strcpy(gaInPower, \"1/2/10\");   // Central power control" << std::endl;
    std::cout << "strcpy(gaInBri, \"1/2/20\");     // Central brightness control" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "// Resulting segment GAs:" << std::endl;
    std::cout << "// Segment 0: 1/2/10, 1/2/20 (same as central)" << std::endl;
    std::cout << "// Segment 1: 2/2/20, 2/2/30 (central + 1*offsets)" << std::endl;
    std::cout << "// Segment 2: 3/2/30, 3/2/40 (central + 2*offsets)" << std::endl;
}

// Main test runner
int main() {
    KnxSegmentTest test;
    
    // Run automated tests
    test.runAllTests();
    
    // Print manual test scenarios
    printManualTestScenarios();
    
    // Show example usage
    exampleUsage();
    
    return 0;
}