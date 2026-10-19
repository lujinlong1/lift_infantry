# Run after a Keil build: host tests do not exercise the MCU task stack.
param([string] $FirmwareRoot = (Join-Path $PSScriptRoot '..\..'))
$ErrorActionPreference = 'Stop'
$FirmwareRoot = (Resolve-Path -LiteralPath $FirmwareRoot).Path
$source = Get-Content -LiteralPath (Join-Path $FirmwareRoot 'Application\Lift\Src\lift_control_task.c') -Raw
$report = Get-Content -LiteralPath (Join-Path $FirmwareRoot 'MDK-ARM\lift_infantry\lift_infantry.htm') -Raw
$map = Get-Content -LiteralPath (Join-Path $FirmwareRoot 'MDK-ARM\lift_infantry\lift_infantry.map') -Raw
$words = [regex]::Match($source, '#define\s+LIFT_CONTROL_TASK_STACK_WORDS\s+(\d+)U')
$depth = [regex]::Match($report, '(?s)lift_control_task</STRONG>.*?Max Depth =\s*(\d+)')
$allocation = [regex]::Match($map, '(?m)^\s*lift_task_stack\s+0x[0-9a-fA-F]+\s+Data\s+(\d+)\s+lift_control_task\.o')
if (!$words.Success -or !$depth.Success -or !$allocation.Success) {
    throw 'Cannot read lift task stack budget; build with Keil call graph and map enabled.'
}
$budget = [int]$words.Groups[1].Value * 4
$required = [int]$depth.Groups[1].Value
$linked = [int]$allocation.Groups[1].Value
if ($linked -ne $budget) { throw 'Stack allocation differs from source; rebuild the firmware.' }
# Reserve space for saved RTOS/FPU context and unreported calls. This is
# a static lower-bound check, not proof of the runtime maximum.
$reserve = 1024
Write-Output "lift_ctrl: linked stack $linked bytes; reported depth $required bytes; reserve $reserve bytes"
if ($required + $reserve -gt $budget) {
    throw "Lift task stack budget exceeded: $required + $reserve > $budget bytes."
}
Write-Output 'PASS: lift task static stack budget'
