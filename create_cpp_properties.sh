#!/bin/bash

# Default path for Arduino installation on Windows (modify as needed)
ArduinoPath="C:/Program Files (x86)/Arduino"
AvrPath="$ArduinoPath/hardware/tools/avr"
ArduinoAvrPath="$ArduinoPath/hardware/arduino/avr"

# Check if the Arduino path exists
if [ ! -d "$ArduinoPath" ]; then
    echo "Arduino directory not found at the expected path."
    exit 1
fi

# Find the AVR GCC binary path
AvrGccBinary=$(find "$AvrPath/bin" -name "avr-gcc*.exe" | head -n 1)
AvrGccBinaryPath="$AvrPath/bin/$(basename "$AvrGccBinary")"

# Find the AVR GCC include path
AvrGccVersion=$(find "$AvrPath/lib/gcc/avr" -maxdepth 1 -type d | head -n 1)
AvrGccIncludePath="$AvrPath/lib/gcc/avr/$AvrGccVersion/include"

# Replace backslashes with forward slashes
AvrPath="${AvrPath//\\//}"
ArduinoAvrPath="${ArduinoAvrPath//\\//}"
AvrGccIncludePath="${AvrGccIncludePath//\\//}"
AvrGccBinaryPath="${AvrGccBinaryPath//\\//}"

# Construct the JSON configuration
jsonConfig=$(cat <<EOF
{
    "version": 4,
    "configurations": [
        {
            "name": "Win32",
            "compilerPath": "$AvrGccBinaryPath",
            "compilerArgs": [],
            "intelliSenseMode": "\${default}",
            "includePath": [
                "\${workspaceFolder}/**",
                "$AvrPath/avr/include",
                "$ArduinoAvrPath/variants/leonardo",
                "$ArduinoAvrPath/cores/arduino",
                "$AvrGccIncludePath"
            ],
            "forcedInclude": [],
            "cStandard": "\${default}",
            "cppStandard": "\${default}",
            "defines": [
                "_DEBUG",
                "UNICODE",
                "_UNICODE"
            ]
        }
    ]
}
EOF
)

# Write the JSON configuration to c_cpp_properties.json
echo "$jsonConfig" > .vscode/c_cpp_properties.json

echo "c_cpp_properties.json has been updated with AVR GCC compiler path."
