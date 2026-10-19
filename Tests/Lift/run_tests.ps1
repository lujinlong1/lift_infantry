# Host unit tests for the lift modules.
# ASCII-only on purpose: Windows PowerShell 5.1 reads BOM-less .ps1 as ANSI,
# so any non-ASCII character here would corrupt the script itself.
#
# Usage:  powershell -ExecutionPolicy Bypass -File run_tests.ps1
#
# IMPORTANT: the suite compiles the *real firmware sources* under
#   lift_infantry/Application/Lift/{Inc,Src}
# not local copies. Keeping stale copies here once let the suite pass while
# the shipped lift_homing.c had already changed -- that is exactly the
# failure mode this layout removes.
#
# The PID used by lift_control is also the real one from the firmware project.

$ErrorActionPreference = 'Stop'
$dir = $PSScriptRoot
$out = Join-Path $env:TEMP 'lift_host_tests'
New-Item -ItemType Directory -Path $out -Force | Out-Null

$fwRoot = (Resolve-Path (Join-Path $dir '..\..')).Path
$pidInc = Join-Path $fwRoot 'Algorithms\Pid\Inc'
$pidSrc = Join-Path $fwRoot 'Algorithms\Pid\Src\pid.c'
$liftInc = Join-Path $fwRoot 'Application\Lift\Inc'
$liftSrc = Join-Path $fwRoot 'Application\Lift\Src'
$chassisInc = Join-Path $fwRoot 'Application\Chassis\Inc'
# Stub headers for the bench module's HAL/RTOS/motor dependencies.
$jogStubs = Join-Path $dir 'jogstubs'
$jogInc = Join-Path $fwRoot 'Diagnostics\MotorTest\Inc'
$jogSrc = Join-Path $fwRoot 'Diagnostics\MotorTest\Src'

function LiftSrc([string] $name) { return (Join-Path $liftSrc $name) }

# Shared source sets: keeping these in one place stops a suite from silently
# compiling a different subset than the firmware does.
$kinSrc = (LiftSrc 'lift_kinematics.c')
$safeSrc = (LiftSrc 'lift_safety.c')
$coreSrc = @((LiftSrc 'lift_control.c'), (LiftSrc 'lift_sync.c'), $pidSrc)
$fullSrc = @((LiftSrc 'lift_control.c'), (LiftSrc 'lift_sync.c'),
             (LiftSrc 'lift_guard.c'), (LiftSrc 'lift_homing.c'),
             $kinSrc, $safeSrc, $pidSrc)

$motorInc = Join-Path $fwRoot 'Devices\RobotMotors\Inc'
$djiInc = Join-Path $fwRoot 'Devices\DjiMotor\Inc'
$canInc = Join-Path $fwRoot 'Services\Can\Inc'
$configInc = Join-Path $fwRoot 'Config\Inc'
$suites = @(
    @{ name = 'lift_owner_concurrency'; src = @($safeSrc); inc = @($liftInc); threaded = $true },
    @{ name = 'lift_diag_concurrency'; src = @((LiftSrc 'lift_diagnostics.c'), $safeSrc); inc = @($liftInc); threaded = $true },
    @{ name = 'lift_telemetry'; src = @((LiftSrc 'lift_diagnostics.c'), $safeSrc); inc = @((Join-Path $dir 'telemetrystubs'), (Join-Path $dir 'fwstubs'), (Join-Path $fwRoot 'Diagnostics\Telemetry\Inc'), $liftInc, $jogInc, $pidInc) },
    @{ name = 'lift_diagnostics'; src = @((LiftSrc 'lift_diagnostics.c'), $safeSrc); inc = @($liftInc) },
    @{ name = 'lift_transport'; src = @((LiftSrc 'lift_diagnostics.c'), $safeSrc, (Join-Path $fwRoot 'Devices\DjiMotor\Src\dji_motor.c')); inc = @($motorInc, $djiInc, $canInc, $configInc, (Join-Path $dir 'fwstubs'), $liftInc) },
    @{ name = 'chassis_stair'; src = @($pidSrc); inc = @((Join-Path $dir 'fwstubs'), $chassisInc, $pidInc) },
    @{ name = 'lift_axes'; src = ($fullSrc + @((LiftSrc 'lift_top.c'), (LiftSrc 'lift_seq.c'), (LiftSrc 'lift_diagnostics.c'))); inc = @((Join-Path $dir 'fwstubs'), $liftInc, $pidInc, $jogInc) },
    @{ name = 'lift_homing';  src = @((LiftSrc 'lift_homing.c'));  inc = @($liftInc) },
    @{ name = 'lift_guard';   src = @((LiftSrc 'lift_guard.c'));   inc = @($liftInc) },
    @{ name = 'lift_sync';    src = @((LiftSrc 'lift_sync.c'));    inc = @($liftInc) },
    @{ name = 'lift_kinematics'; src = @($kinSrc);                 inc = @($liftInc) },
    @{ name = 'lift_safety';  src = @($safeSrc);                   inc = @($liftInc) },
    @{ name = 'lift_control'; src = $coreSrc;                      inc = @($liftInc, $pidInc) },
    @{ name = 'lift_system';  src = $fullSrc;                      inc = @($liftInc, $pidInc) },
    @{ name = 'lift_top';     src = ($fullSrc + (LiftSrc 'lift_top.c')); inc = @($liftInc, $pidInc) },
    # lift_seq pulls in lift_safety (stop policy); its own source is in the list.
    @{ name = 'lift_seq';     src = ($coreSrc + $safeSrc + (LiftSrc 'lift_seq.c')); inc = @($liftInc, $pidInc) },
    @{ name = 'lift_geometry'; src = @((LiftSrc 'lift_geometry.c')); inc = @($liftInc) },
    @{ name = 'lift_properties'; src = ($fullSrc + @((LiftSrc 'lift_top.c'), (LiftSrc 'lift_seq.c'), (LiftSrc 'lift_geometry.c'))); inc = @($liftInc, $pidInc) },
    # Bench module: the test #includes the real lift_jog_task.c, so pid.c and
    # the ownership token (lift_safety.c) are compiled separately.
    # It needs the stub include dir for HAL/RTOS/motors.
    @{ name = 'lift_jog'; src = @($pidSrc, $safeSrc); inc = @($jogStubs, $jogInc, $jogSrc, $liftInc, $pidInc) }
)

$failed = 0
Push-Location $dir
try {
    foreach ($s in $suites) {
        $exe = Join-Path $out ($s.name + '.exe')
        Write-Host ("=== " + $s.name + " ===")

        $gccArgs = @('-std=c11', '-Wall', '-Wextra', '-Wpedantic', '-Werror')
        if ($s.threaded) { $gccArgs += '-pthread' }
        $gccArgs += ('test_' + $s.name + '.c')
        $gccArgs += $s.src
        foreach ($i in $s.inc) { $gccArgs += ('-I' + $i) }
        $gccArgs += ('-I' + $chassisInc)
        $gccArgs += '-o'
        $gccArgs += $exe

        & gcc @gccArgs
        if ($LASTEXITCODE -ne 0) {
            Write-Host ("  COMPILE FAILED (" + $s.name + ")")
            $failed++
            continue
        }
        & $exe
        if ($LASTEXITCODE -ne 0) {
            Write-Host ("  TESTS FAILED (" + $s.name + ")")
            $failed++
        } else {
            Write-Host ("  OK (" + $s.name + ")")
        }
        Write-Host ''
    }
}
finally {
    Pop-Location
}

if ($failed -ne 0) {
    Write-Host ("RESULT: " + $failed + " suite(s) failed")
    exit 1
}

# Firmware-only compile check. The unit suites above cannot link
# lift_control_task.c (it needs a full RTOS/HAL), so nothing used to compile it
# on the host -- Keil was the first thing to see it. This step compiles the real
# sources against the stubs in fwstubs\ with -Werror. It does not run anything.
Write-Host ''
Write-Host '=== firmware compile check ==='
& powershell -ExecutionPolicy Bypass -File (Join-Path $dir 'compile_check.ps1')
if ($LASTEXITCODE -ne 0) {
    Write-Host 'RESULT: all suites passed but the firmware compile check failed'
    exit 1
}

Write-Host 'RESULT: all suites passed'
exit 0
