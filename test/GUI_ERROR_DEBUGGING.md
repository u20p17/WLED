# GUI Error Message Debugging Guide

This document explains the complete solution for displaying detailed KNX GA conflict messages in the WLED GUI.

## Problem Description

Users reported that when KNX Group Address conflicts were detected, the WLED GUI only showed "Error33:" instead of the detailed conflict information including specific GA addresses and recommendations.

## Root Cause Analysis

The issue was traced to the web interface JavaScript code not handling:
1. Error code 33 (ERR_KNX_GA_CONFLICT) 
2. The `error_msg` field containing detailed error information

## Complete Solution

### 1. Backend Error Details (Already Implemented)
- **File**: `/wled00/wled.h`
  - Added global `errorDetails[256]` buffer for detailed error messages
- **File**: `/wled00/const.h` 
  - Added `ERR_KNX_GA_CONFLICT = 33` error code
- **File**: `/wled00/json.cpp`
  - Enhanced JSON API to include `error_msg` field when `errorDetails` contains data
  - Added debug output to track error message transmission
- **File**: `/usermods/KNX_IP/usermod_knx_ip.cpp`
  - Enhanced `hasGAConflicts()` to populate `errorDetails` with specific conflict information
  - Fixed `sizeof()` issue with extern buffer declaration
  - Added debug output to verify error details content

### 2. Frontend JavaScript Fix (New Implementation)
- **File**: `/wled00/data/index.js`
  - Added case 33 handling in error switch statement
  - Added support for `error_msg` field from JSON API response
  - Enhanced error display to use detailed messages when available

## Code Changes Made

### JavaScript Error Handling Enhancement
```javascript
// Added case 33 for KNX GA conflicts
case 33:
    errstr = "KNX Group Address conflict detected.";
    break;

// Use detailed error message if available
if (s.error_msg && s.error_msg.length > 0) {
    errstr = s.error_msg;
}
```

### C++ Debug Output for Troubleshooting
```cpp
// In hasGAConflicts() - fixed buffer size calculation
const size_t errorDetailsSize = 256;
strncpy(errorDetails, message.c_str(), errorDetailsSize - 1);
errorDetails[errorDetailsSize - 1] = '\0';

// Added debug output to verify content
KNX_UM_DEBUGF("Error details set: '%s'", errorDetails);
```

### JSON API Debug Output
```cpp
// In json.cpp - added debug tracking
if (errorDetails[0] != '\0') {
    root[F("error_msg")] = errorDetails;
    #ifdef WLED_DEBUG
        Serial.printf("[JSON] Adding error details: '%s'\n", errorDetails);
    #endif
    errorDetails[0] = '\0'; // Clear after sending
}
```

## Testing Process

### 1. Build Process
```bash
# Build web interface (required after JavaScript changes)
npm run build

# Build firmware 
pio run -e esp32dev

# Upload firmware (optional, for live testing)
pio run -e esp32dev -t upload
```

### 2. Expected Debug Output
When GA conflicts occur, you should see:
```
[KNX-UM] Error details set: 'KNX GA conflicts: 1/1/1, 1/1/2, 2/1/1, 2/1/2. Check segment offsets.'
[JSON] Adding error details: 'KNX GA conflicts: 1/1/1, 1/1/2, 2/1/1, 2/1/2. Check segment offsets.'
```

### 3. Expected GUI Behavior
Instead of showing:
```
Error33:
```

The GUI should now display:
```
Error 33: KNX GA conflicts: 1/1/1, 1/1/2, 2/1/1, 2/1/2. Check segment offsets.
```

## Build Requirements

### Web Interface Build Required
When JavaScript files are modified, the web interface must be rebuilt:
1. Run `npm run build` to process JavaScript changes
2. This converts JS/HTML/CSS files to C++ header files
3. Rebuild firmware to include updated web interface

### Debug Output Requirements
- Enable `WLED_DEBUG` for JSON debug output
- Enable `KNX_UM_DEBUG` for KNX usermod debug output

## File Dependencies

### Modified Files
- `/wled00/data/index.js` - Frontend error handling
- `/wled00/json.cpp` - JSON API with error_msg field
- `/usermods/KNX_IP/usermod_knx_ip.cpp` - Backend error details

### Generated Files (Auto-updated by build)
- `/wled00/html_ui.h` - Contains compiled JavaScript from index.js

## Troubleshooting

### 1. Still seeing "Error33:" only
- Verify web interface was rebuilt with `npm run build`
- Check that firmware was rebuilt after web interface build
- Enable debug output to verify error_msg transmission

### 2. No error details in debug output
- Verify `errorDetails` buffer is being populated in KNX usermod
- Check that `errorFlag = ERR_KNX_GA_CONFLICT` is being set
- Ensure JSON API is including error_msg field

### 3. Build Issues
- Run `npm install` if web build fails
- Check that all modified files compile without errors
- Verify PlatformIO environment is correctly configured

## Future Enhancements

### Potential Improvements
1. **Localization**: Add multi-language support for error messages
2. **UI Enhancement**: Create dedicated KNX configuration error dialog
3. **Auto-resolution**: Suggest optimal GA offset values automatically
4. **Export/Import**: Save/load KNX configurations with conflict checking

### Error Code Extension
The enhanced error handling system can be extended for other usermods:
- Use global `errorDetails` buffer for detailed messages
- Add specific error codes to the JavaScript switch statement
- Include `error_msg` field in JSON responses for rich error information
