<#
.SYNOPSIS
  Builds, tests and packages hullcad for Windows x64 and/or ARM64.

.DESCRIPTION
  Runs CMake configure -> build -> test -> install (Qt DLLs deployed with windeployqt) -> zip.
  Run it from a normal PowerShell window; Visual Studio 2022 is located automatically.

  Qt kits (Qt online installer, Qt 6.8 or newer):
    x64 PC  : "MSVC 2022 64-bit"                      -> e.g. C:\Qt\6.8.3\msvc2022_64
    ARM64   : "MSVC 2022 ARM64 (cross-compiled)" on an x64 PC -> C:\Qt\6.8.3\msvc2022_arm64
              or "MSVC 2022 ARM64" (Qt 6.11+) on an ARM64 PC   -> C:\Qt\6.11.3\msvc2022_arm64

.EXAMPLE
  .\scripts\build-windows.ps1 -Arch x64 -QtX64 C:\Qt\6.8.3\msvc2022_64
.EXAMPLE
  .\scripts\build-windows.ps1 -Arch all -QtX64 C:\Qt\6.8.3\msvc2022_64 -QtArm64 C:\Qt\6.8.3\msvc2022_arm64
#>
param(
    [ValidateSet('x64', 'arm64', 'all')] [string] $Arch = 'x64',
    [string] $QtX64 = $env:QT_DIR_X64,
    [string] $QtArm64 = $env:QT_DIR_ARM64,
    [switch] $SkipTests
)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

function Invoke-Step([string] $what, [scriptblock] $cmd) {
    Write-Host "==> $what" -ForegroundColor Cyan
    & $cmd
    if ($LASTEXITCODE -ne 0) { throw "$what failed (exit code $LASTEXITCODE)" }
}

$hostArm = $env:PROCESSOR_ARCHITECTURE -eq 'ARM64'
$targets = if ($Arch -eq 'all') { @('x64', 'arm64') } else { @($Arch) }

foreach ($t in $targets) {
    if ($t -eq 'x64') {
        if (-not $QtX64) { throw 'Qt x64 kit not given: use -QtX64 or set QT_DIR_X64' }
        $env:QT_DIR_X64 = $QtX64
        $preset = 'windows-x64'
        $runTests = $true           # also runs on ARM64 Windows 11 through x64 emulation
    } else {
        if (-not $QtArm64) { throw 'Qt ARM64 kit not given: use -QtArm64 or set QT_DIR_ARM64' }
        $env:QT_DIR_ARM64 = $QtArm64
        if ($hostArm) {
            $preset = 'windows-arm64'
            $runTests = $true
        } else {
            if (-not $QtX64) { throw 'Cross-building ARM64 on an x64 PC needs the x64 Qt kit as host tools: -QtX64' }
            $env:QT_DIR_X64 = $QtX64
            $preset = 'windows-arm64-cross'
            $runTests = $false        # ARM64 binaries cannot run on an x64 PC
        }
    }

    Invoke-Step "configure ($preset)" { cmake --preset $preset }
    Invoke-Step "build ($preset)" { cmake --build --preset $preset --parallel }
    if ($runTests -and -not $SkipTests) {
        Invoke-Step "test ($preset)" { ctest --preset ($preset -replace '-cross$', '') }
    } else {
        Write-Host "    (tests skipped for $preset)" -ForegroundColor Yellow
    }

    $dist = Join-Path $root "dist\$preset"
    if (Test-Path $dist) { Remove-Item -Recurse -Force $dist }
    Invoke-Step "install + deploy Qt ($preset)" { cmake --install "build\$preset" --config Release }

    # Cross builds: if the deploy step could not copy the Qt DLLs, run windeployqt from the host kit.
    $exe = Join-Path $dist 'bin\hullcad.exe'
    if ($preset -eq 'windows-arm64-cross' -and -not (Test-Path (Join-Path $dist 'bin\Qt6Core.dll'))) {
        $wdq = Join-Path $QtX64 'bin\windeployqt.exe'
        $qtpaths = Join-Path $QtArm64 'bin\qtpaths.bat'
        Invoke-Step 'windeployqt (ARM64 target, x64 host tools)' { & $wdq --release --qtpaths $qtpaths $exe }
    }

    $zip = Join-Path $root "dist\hullcad-windows-$t.zip"
    if (Test-Path $zip) { Remove-Item $zip }
    Compress-Archive -Path (Join-Path $dist '*') -DestinationPath $zip
    Write-Host "    package: $zip" -ForegroundColor Green
}
