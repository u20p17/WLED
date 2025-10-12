# KNX GA Conflict GUI Error Notifications

## Overview

The KNX IP usermod now provides **automatic GUI error notifications** when Group Address (GA) conflicts are detected. These errors appear directly in the WLED web interface, making it easy for users to identify and resolve GA conflicts without needing to check serial logs.

## How It Works

### Automatic Detection
The system automatically checks for GA conflicts in several scenarios:
1. **During setup** - After KNX initialization 
2. **After configuration changes** - When settings are saved via GUI
3. **Manual trigger** - Via `checkGAConflictsAndNotifyGUI()` method

### GUI Error Display
When conflicts are detected:
- **Error flag is set** in WLED's error system (`errorFlag = 33`)
- **Error message appears** in the WLED web interface
- **Red notification** is displayed to the user
- **Error persists** until conflicts are resolved

## Error Code

**Error Code: 33** - `ERR_KNX_GA_CONFLICT`
- **Description**: KNX Group Address conflict detected (duplicate GAs)
- **Location**: Added to `/wled00/const.h`
- **Usage**: Automatically set by KNX usermod when conflicts are found

## When Errors Are Triggered

### 1. **Zero Offset Conflicts**
```
Segment 0 GAs = Central GAs (offsets L=0, M=0, N=0)
→ GUI Error: "KNX GA conflict detected"
```

### 2. **Cross-Segment Conflicts**
```
Segment 1 Power GA = Segment 2 Brightness GA
→ GUI Error: "KNX GA conflict detected"
```

### 3. **Central vs Segment Conflicts**
```
Segment N Power GA = Central Brightness GA
→ GUI Error: "KNX GA conflict detected"
```

### 4. **Boundary Overflow**
```
Calculated GA exceeds KNX limits (31/7/255)
→ GUI Error: "KNX GA conflict detected"
```

## User Experience

### Error Notification Flow
1. **User changes KNX settings** (GAs, segment offsets)
2. **Settings are saved** via WLED GUI
3. **Conflict detection runs** automatically
4. **If conflicts found:**
   - ❌ Error notification appears in GUI
   - ⚠️ Segment registration is blocked
   - 📋 Detailed conflict info in serial logs
5. **If no conflicts:**
   - ✅ Settings saved successfully
   - ✅ Segment KOs registered normally

### Resolving Errors
To clear the GUI error notification:
1. **Adjust segment offsets** (increase L, M, or N values)
2. **Change central GAs** to different ranges  
3. **Reduce segment count** if using too many segments
4. **Save settings** - error will clear automatically if no conflicts

## Implementation Details

### Key Methods

**`checkGAConflictsAndNotifyGUI()`**
- Checks for conflicts using existing `hasGAConflicts()` method
- Sets `errorFlag = 33` if conflicts found
- Clears `errorFlag = 0` if conflicts resolved
- Logs detailed conflict info for debugging

**`validateSegmentGAs()`** - Enhanced
- Original validation logic preserved
- Now sets GUI error flag when conflicts detected
- Called during segment KO registration

**`registerSegmentKOs()`** - Enhanced  
- Calls validation before registration
- Sets GUI error flag if validation fails
- Blocks registration when conflicts exist

### Error Flag Integration
```cpp
// Set error flag for GUI notification
extern byte errorFlag;
errorFlag = 33; // ERR_KNX_GA_CONFLICT

// Clear error flag when resolved
extern byte errorFlag;
if (errorFlag == 33) {
    errorFlag = 0; // ERR_NONE
}
```

### Automatic Triggers
```cpp
void setup() {
    // ... KNX initialization ...
    
    // Check for conflicts after setup
    checkGAConflictsAndNotifyGUI();
}

bool readFromConfig(JsonObject& root) {
    // ... configuration reading ...
    
    // Check for conflicts after config changes
    checkGAConflictsAndNotifyGUI();
    
    return true;
}
```

## Benefits

### 🎯 **User-Friendly**
- No need to check serial console
- Immediate visual feedback in GUI
- Clear indication of configuration problems

### 🚀 **Proactive**
- Catches conflicts before they cause issues
- Prevents problematic KNX registrations
- Validates configuration automatically

### 🔧 **Developer-Friendly**
- Detailed logging still available for debugging
- Error state properly managed
- Integration with WLED's existing error system

### 📱 **Accessible**
- Works on any device with web browser
- No special tools or knowledge required
- Consistent with WLED's user experience

## Testing the Feature

### 1. **Create a Conflict**
- Set segment offsets to: L=0, M=0, N=0
- Save settings
- **Expected**: Red error notification in GUI

### 2. **Resolve the Conflict**  
- Change offsets to: L=5, M=0, N=10
- Save settings
- **Expected**: Error notification disappears

### 3. **Verify Logs**
```
[KNX-UM] GA conflicts detected - check WLED GUI for notification
[KNX-UM] GA Conflict Analysis
[KNX-UM] Segment 0: Power IN: 1/2/10 ⚠ CONFLICT
```

### 4. **Test Different Scenarios**
- Cross-segment conflicts (overlapping ranges)
- Boundary conditions (GAs > 31/7/255)
- Multiple simultaneous conflicts

## Technical Notes

### Error Persistence
- Error flag persists until conflicts are resolved
- GUI shows error on every page load while flag is set
- Error automatically clears when conflicts are fixed

### Performance Impact
- Minimal overhead during normal operation
- Conflict checking only triggered when needed
- No impact on KNX bus communication performance

### Backward Compatibility
- Existing installations continue to work unchanged
- New error notifications are additive enhancement
- Original validation warnings still appear in logs

## Integration Examples

### For Usermod Developers
```cpp
// Check and notify GUI of conflicts
void checkConfiguration() {
    knxUsermod.checkGAConflictsAndNotifyGUI();
}

// Manual conflict validation
bool isConfigValid() {
    return knxUsermod.validateSegmentGAs();
}
```

### For End Users
1. **Monitor GUI** for red error notifications
2. **Check serial logs** for detailed conflict analysis  
3. **Adjust settings** based on error messages
4. **Verify resolution** by confirming error disappears

This feature significantly improves the user experience by making GA conflict detection visible and actionable directly in the WLED web interface! 🎯