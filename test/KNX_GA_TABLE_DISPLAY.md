# KNX Group Address Table Display Feature

This document describes the implementation of the KNX GA table display in the WLED Info panel.

## Feature Overview

The KNX GA table provides a comprehensive overview of all Group Address mappings for both the main group (segment 0) and individual segments. The table shows:

- **Rows**: Different GA types (Power, Brightness, Effect) for both Input and Output
- **Columns**: Main segment and individual segments (up to 8 segments displayed)
- **Conflict Detection**: GAs that conflict with existing addresses are highlighted in red
- **Offset Information**: Shows current L/M/N offset values used for segment calculations

## Implementation Details

### 1. Backend Implementation

#### KNX Usermod Header (`usermod_knx_ip.h`)
```cpp
// GA table for Info panel display
String getGATableHTML() const;

// Added to Usermod API section
void addToJsonInfo(JsonObject& root);
```

#### KNX Usermod Implementation (`usermod_knx_ip.cpp`)

**GA Table Generation (`getGATableHTML()`)**:
- Generates HTML table with segments as columns and GA types as rows
- Uses `calculateSegmentGA()` to compute segment-specific addresses
- Formats GAs in standard KNX notation (main/middle/sub)
- Highlights conflicting GAs with red background
- Includes offset information below the table
- Limits display to 8 segments for readability

**JSON Info Integration (`addToJsonInfo()`)**:
- Called by WLED core when building `/json/info` response
- Adds "KNX GA Table" field containing the HTML table
- Only includes data when KNX usermod is enabled

### 2. Frontend Implementation

#### JavaScript Enhancement (`index.js`)

**Usermod Data Processing**:
```javascript
var knxTable = "";
if (i.u) {
    for (const [k, val] of Object.entries(i.u)) {
        if (k === "KNX GA Table" && typeof val === "string") {
            // Special handling for KNX GA table - display as raw HTML
            knxTable = val;
        } else if (val[1]) {
            urows += inforow(k,val[0],val[1]);
        } else {
            urows += inforow(k,val);
        }
    }
}
```

**Info Panel Display**:
```javascript
${knxTable ? '<tr><td colspan=2 style="padding:5px;"><div style="font-weight:bold;margin-bottom:5px;">KNX Group Address Mapping</div>' + knxTable + '</td></tr>' : ''}
```

## Table Structure

### Column Layout
- **Column 1**: GA Type (Power, Brightness, Effect)
- **Column 2**: Main Segment (calculated for segment 0)
- **Columns 3+**: Individual segments (Seg1, Seg2, etc.)

### Row Sections
1. **Input GAs (KNX → WLED)**: Commands from KNX bus to WLED
2. **Output GAs (WLED → KNX)**: Status feedback from WLED to KNX bus

### Visual Features
- **Header Styling**: Uses CSS variables for consistent theming
- **Conflict Highlighting**: Red background for conflicting GAs
- **Compact Design**: Small font size and minimal padding for info panel
- **Responsive Layout**: Adapts to available segments (max 8 displayed)

## Example Table Output

```
┌─────────────┬──────────┬──────────┬──────────┬──────────┐
│ GA Type     │   Main   │   Seg1   │   Seg2   │   Seg3   │
├─────────────┼──────────┼──────────┼──────────┼──────────┤
│           Input GAs (KNX → WLED)                        │
├─────────────┼──────────┼──────────┼──────────┼──────────┤
│ Power       │  1/0/1   │  1/1/1   │  1/2/1   │  1/3/1   │
│ Brightness  │  1/0/2   │  1/1/2   │  1/2/2   │  1/3/2   │
│ Effect      │  1/1/11  │  1/2/11  │  1/3/11  │  1/4/11  │
├─────────────┼──────────┼──────────┼──────────┼──────────┤
│          Output GAs (WLED → KNX)                        │
├─────────────┼──────────┼──────────┼──────────┼──────────┤
│ Power       │  2/0/1   │  2/1/1   │  2/2/1   │  2/3/1   │
│ Brightness  │  2/0/2   │  2/1/2   │  2/2/2   │  2/3/2   │
│ Effect      │  2/1/11  │  2/2/11  │  2/3/11  │  2/4/11  │
└─────────────┴──────────┴──────────┴──────────┴──────────┘
Offsets: L=0, M=1, N=0
```

## CSS Styling

The table uses WLED's CSS variables for consistent theming:
- `--c-2`: Header background
- `--c-3`: Border color  
- `--c-4`: Section divider background
- `--c-r`: Conflict highlighting (red)

## Integration Points

### 1. Info Panel Display
- Table appears in the Info panel when KNX usermod is enabled
- Positioned after other usermod information but before system info
- Separated by horizontal rules for visual clarity

### 2. JSON API Response
- Included in `/json/info` endpoint under usermod section
- Field name: "KNX GA Table"
- Content: HTML string with complete table markup

### 3. Real-time Updates
- Table refreshes when Info panel is updated
- Reflects current segment configuration and offset values
- Shows conflicts based on current GA assignments

## Build Requirements

### Web Interface Build
After modifying JavaScript files:
```bash
npm run build
pio run -e esp32dev
```

### Dependencies
- KNX usermod must be enabled
- Requires segments to be configured in WLED
- Uses existing GA conflict detection system

## Future Enhancements

### Potential Improvements
1. **Interactive Features**: Click to edit GAs directly from table
2. **Export Function**: Download table as CSV or PDF
3. **Conflict Resolution**: Suggest optimal offset values
4. **Color Coding**: Different colors for different GA types
5. **Pagination**: Support for more than 8 segments
6. **Tooltips**: Show GA descriptions and DPT types

### Mobile Optimization
- Consider horizontal scrolling for many segments
- Responsive column width based on screen size
- Collapsible sections for mobile view

## Troubleshooting

### Table Not Appearing
1. Verify KNX usermod is enabled
2. Check that segments are configured
3. Ensure web interface was rebuilt after changes
4. Verify JSON API includes "KNX GA Table" field

### Styling Issues
1. Check CSS variables are properly defined
2. Verify table HTML structure is correct
3. Test in different browsers for compatibility

### Performance Considerations
- Table generation is O(n) where n = number of segments
- Limited to 8 segments for display performance
- HTML string is cached until configuration changes