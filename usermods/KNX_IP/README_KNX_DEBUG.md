# KNX IP Usermod Debug Logging

The KNX IP usermod includes an optional verbose debug logging facility to help diagnose state publishing, change detection, GA configuration, and network behavior.

## Overview
Verbose logs are compiled in only when the preprocessor symbol `KNX_UM_DEBUG` is defined at build time. When disabled (default), almost all diagnostic `Serial` output from the KNX usermod is removed, minimizing flash size and runtime overhead.

Logging is routed through macros defined in `usermods/KNX_IP/usermod_knx_ip.h`:

```
KNX_UM_DEBUGF(fmt, ...)    // verbose printf-style log (enabled only with KNX_UM_DEBUG)
KNX_UM_DEBUGLN(msg)        // verbose single-line message
KNX_UM_WARNF(fmt, ...)     // warning (always on unless KNX_UM_SUPPRESS_WARN defined)
KNX_UM_WARNLN(msg)         // warning single-line message
```

If `KNX_UM_DEBUG` is not defined the DEBUG macros expand to no-ops and generate no code/data.
Warnings are emitted unconditionally by default so that configuration or validation problems are visible even in production builds.

To silence warnings (not generally recommended) add:
```
-DKNX_UM_SUPPRESS_WARN
```
to your `build_flags`. This converts `KNX_UM_WARN*` macros into no-ops as well.

## Enabling Debug Logs (PlatformIO)
Add a build flag for the environment you are using (example for a custom ESP32 dev environment) in `platformio_override.ini`:

```
[env:custom_esp32dev_knx_ip]
extends = env:esp32dev
build_flags = 
  ${common.build_flags}
  -DKNX_UM_DEBUG
```

If you already have a `build_flags` section, just append `-DKNX_UM_DEBUG` on its own line (one flag per line is fine) or space separated on the same line.

Rebuild and flash:
```
pio run -e custom_esp32dev_knx_ip -t upload
```

## What You Will See
Typical debug lines (examples):
```
[KNX-UM] scheduleStatePublish() skipped - KNX not running (enabled=0, running=0)
[KNX-UM] Power changed: 0→1
[KNX-UM] Brightness changed: 12→34
[KNX-UM] Color/CCT changed: R:0→128 G:0→64 B:0→32 W:0→10 CCT:128→140
[KNX-UM] Scheduled publish in 120ms (pwr=1, bri=1, fx=0, cct=1, rgbw=1, preset=0)
[KNX-UM] publishState(seq=5 at 123456ms) pendingFlags: PWR=1 BRI=1 FX=0 COLOR=1 PRE=0
[KNX-UM] publishState(seq=5) done. Snapshot R=128 G=64 B=32 W=10 CCT=140 bri=86 on=1
[KNX-UM] Scheduled publish triggered
```

These help answer:
- Which fields were detected as changed
- Whether multiple rapid changes were coalesced into a single publish
- Sequence/order of publishes
- Whether KNX stack was ready when a publish was attempted
- Network IP change handling and GA registration rebuild events

## Performance Impact
With debug disabled (default) only warning lines (e.g. invalid GA / PA strings, sanity checks) still print using the WARN macros. Enabling debug:
- Increases flash usage slightly (format strings + code)
- Adds additional Serial I/O (can slow loop if baud is low)
- Should not materially affect KNX timing, but for maximum performance keep it disabled in production

## Disabling Again
Simply remove or comment out the `-DKNX_UM_DEBUG` line and rebuild.

## Adding New Debug Lines
Prefer using the existing macros so they automatically follow the global enable/disable flag:
```
KNX_UM_DEBUGF("[KNX-UM] Something changed: val=%u\n", someValue);
KNX_UM_DEBUGLN("[KNX-UM] Simple message");
```
Avoid raw `Serial.print*` calls for verbose info unless the message must always appear.

## Testing Integration

Debug output is particularly useful when running the test suites located in `/test/`:
- Enable debug output with `-DKNX_UM_DEBUG`
- Run tests from `/test/GA_CONFLICT_TESTS.md` to validate GA conflict detection
- Use integration tests from `/test/README_TESTING.md` for end-to-end validation

Example test output with debug enabled:
```
[KNX-TEST] Starting GA Conflict Detection Tests
[KNX-UM] GA conflict detected: 1/2/10 used by both central and segment 0
[KNX-TEST] ✓ Conflicts detected with zero offsets
```

## Related Settings
`color_out_mode` (0=per-channel only, 1=composites only, 2=both) is often tuned while observing debug output to confirm the correct telegram set is sent.

---
