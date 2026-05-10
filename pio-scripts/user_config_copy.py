Import('env')
import os
import shutil
from pathlib import Path


def _generate_sdkconfig_header(build_env):
    if build_env.get("PIOPLATFORM") != "espressif32":
        return

    framework_dir = build_env.PioPlatform().get_package_dir("framework-arduinoespressif32")
    if not framework_dir:
        return

    mcu = build_env.BoardConfig().get("build.mcu", "esp32")
    sdkconfig_path = Path(framework_dir) / "tools" / "sdk" / mcu / "sdkconfig"
    if not sdkconfig_path.is_file():
        return

    header_dir = Path(build_env.subst("$BUILD_DIR")) / "generated-compat"
    header_dir.mkdir(parents=True, exist_ok=True)
    header_path = header_dir / "sdkconfig.h"

    header_lines = [
        "#pragma once",
        "/* Auto-generated from the framework sdkconfig file for compatibility. */",
        "",
    ]

    for raw_line in sdkconfig_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if not line.startswith("CONFIG_") or "=" not in line:
            continue

        name, value = line.split("=", 1)
        if value == "y":
            value = "1"
        elif value == "n":
            continue

        header_lines.append(f"#define {name} {value}")

    header_content = "\n".join(header_lines) + "\n"
    previous_content = header_path.read_text(encoding="utf-8") if header_path.is_file() else None
    if previous_content != header_content:
        header_path.write_text(header_content, encoding="utf-8")
        print(f"*** generated sdkconfig.h compatibility header for {mcu} ***")

    build_env.PrependUnique(CPPPATH=[str(header_dir)])

# copy WLED00/my_config_sample.h to WLED00/my_config.h
if os.path.isfile("wled00/my_config.h"):
    print ("*** use existing my_config.h ***")
else: 
    shutil.copy("wled00/my_config_sample.h", "wled00/my_config.h")

_generate_sdkconfig_header(env)
