# Host compile-check for the real firmware lift sources.
#
# WHY: `lift_control_task.c` is the only place that drives the whole lift chain,
# and nothing on the host used to compile it -- Keil was the first thing to see
# it. This script compiles the REAL firmware sources (no copies) against the
# stubs in `fwstubs/` with -Wall -Wextra -Wpedantic -Werror.
#
# It does NOT link and it does NOT run anything. It only proves the files
# compile, and it catches exactly the class of mistake Keil used to catch first
# (typo in a struct field, missing include, wrong enum name, ...).
#
# ASCII-only on purpose: Windows PowerShell 5.1 reads BOM-less .ps1 as ANSI.
#
# Usage:  powershell -ExecutionPolicy Bypass -File compile_check.ps1

$ErrorActionPreference = 'Stop'
$dir = $PSScriptRoot

$fwRoot  = (Resolve-Path (Join-Path $dir '..\..')).Path
$liftInc = Join-Path $fwRoot 'Application\Lift\Inc'
$liftSrc = Join-Path $fwRoot 'Application\Lift\Src'
$jogInc  = Join-Path $fwRoot 'Diagnostics\MotorTest\Inc'
$pidInc  = Join-Path $fwRoot 'Algorithms\Pid\Inc'
$stubs   = Join-Path $dir 'fwstubs'

# `lift_control_task.c` is the one that matters most: it is the integration
# point and it is not covered by any of the unit suites (they cannot link it
# without a full RTOS/HAL). Everything else is compiled by the unit suites
# already, but compiling it here too keeps this check self-contained.
$targets = @(
    @{ name = 'lift_control_task.c'; src = (Join-Path $liftSrc 'lift_control_task.c') },
    @{ name = 'lift_top.c';          src = (Join-Path $liftSrc 'lift_top.c') },
    @{ name = 'lift_seq.c';          src = (Join-Path $liftSrc 'lift_seq.c') },
    @{ name = 'lift_safety.c';       src = (Join-Path $liftSrc 'lift_safety.c') },
    @{ name = 'lift_kinematics.c';   src = (Join-Path $liftSrc 'lift_kinematics.c') }
)

$failed = 0

foreach ($t in $targets) {
    Write-Host ("=== compile-check " + $t.name + " ===")
    $args = @('-std=c11', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-c')
    $args += $t.src
    $args += ('-I' + $stubs)
    $args += ('-I' + $liftInc)
    $args += ('-I' + $pidInc)
    $args += ('-I' + $jogInc)
    # The stub include dir must come FIRST so the fake HAL/RTOS headers win.
    $args += '-o'
    $args += (Join-Path $env:TEMP ($t.name + '.o'))

    & gcc @args
    if ($LASTEXITCODE -ne 0) {
        Write-Host ("  COMPILE FAILED (" + $t.name + ")")
        $failed++
    } else {
        Write-Host ("  OK (" + $t.name + ")")
    }
    Write-Host ''
}

if ($failed -ne 0) {
    Write-Host ("RESULT: " + $failed + " file(s) failed to compile")
    exit 1
}
Write-Host 'RESULT: all firmware lift sources compile clean'
exit 0
