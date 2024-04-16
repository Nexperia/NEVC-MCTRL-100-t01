# PowerShell script to create c_cpp_properties.json for Arduino environment
# To run this you must have first installed Arduino IDE v1 

# Default path for Arduino installation on Windows (modify as needed)
$ArduinoPath = "C:\Program Files (x86)\Arduino"
$AvrPath = Join-Path $ArduinoPath "hardware\tools\avr"
$ArduinoAvrPath = Join-Path $ArduinoPath "hardware\arduino\avr"

# Check if the Arduino path exists
if (-Not (Test-Path $ArduinoPath)) {
    Write-Host "Arduino directory not found at the expected path."
    exit
}

# Find the AVR GCC binary path
$AvrGccBinary = Get-ChildItem (Join-Path $AvrPath "bin") -Filter "avr-gcc*.exe" | Select-Object -First 1
$AvrGccBinaryPath = Join-Path $AvrPath "bin\$($AvrGccBinary.Name)"

# Find the AVR GCC include path
$AvrGccVersion = Get-ChildItem (Join-Path $AvrPath "lib\gcc\avr") | Select-Object -First 1
$AvrGccIncludePath = Join-Path $AvrPath "lib\gcc\avr\$($AvrGccVersion.Name)\include"

# Replace backslashes with forward slashes
$AvrPath = $AvrPath -replace '\\', '/'
$ArduinoAvrPath = $ArduinoAvrPath -replace '\\', '/'
$AvrGccIncludePath = $AvrGccIncludePath -replace '\\', '/'
$AvrGccBinaryPath = $AvrGccBinaryPath -replace '\\', '/'

# Construct the JSON configuration
$jsonConfig = @"
{
    "version": 4,
    "configurations": [
        {
            "name": "Win32",
            "compilerPath": "$AvrGccBinaryPath",
            "compilerArgs": [],
            "intelliSenseMode": "`${default}",
            "includePath": [
                "`${workspaceFolder}/**",
                "$AvrPath/avr/include",
                "$ArduinoAvrPath/variants/leonardo",
                "$ArduinoAvrPath/cores/arduino",
                "$AvrGccIncludePath"
            ],
            "forcedInclude": [],
            "cStandard": "`${default}",
            "cppStandard": "`${default}",
            "defines": [
                "_DEBUG",
                "UNICODE",
                "_UNICODE"
            ]
        }
    ]
}
"@

# Write the JSON configuration to c_cpp_properties.json
$jsonConfig | Out-File -FilePath ".vscode/c_cpp_properties.json" -Encoding UTF8

Write-Host "c_cpp_properties.json has been updated with AVR GCC compiler path."
