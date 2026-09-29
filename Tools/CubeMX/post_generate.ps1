[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$iocPath = Join-Path $projectRoot 'lift_infantry.ioc'
$uvprojxPath = Join-Path $projectRoot 'MDK-ARM\lift_infantry.uvprojx'
$targetPort = Join-Path $projectRoot 'Middlewares\Third_Party\FreeRTOS\Source\portable\GCC\ARM_CM4F'
$requiredCompiler = '6210000::V6.21::ARMCLANG'
$requiredPack = 'Keil.STM32F4xx_DFP.3.1.1'

function Write-Step([string]$Message) {
    Write-Host "[CubeMX post-generation] $Message"
}

function Copy-FileIfDifferent([string]$Source, [string]$Destination) {
    $copyRequired = -not (Test-Path -LiteralPath $Destination)
    if (-not $copyRequired) {
        $sourceHash = (Get-FileHash -LiteralPath $Source -Algorithm SHA256).Hash
        $destinationHash = (Get-FileHash -LiteralPath $Destination -Algorithm SHA256).Hash
        $copyRequired = $sourceHash -ne $destinationHash
    }

    if ($copyRequired) {
        Copy-Item -LiteralPath $Source -Destination $Destination -Force
        Write-Step "Restored $(Split-Path -Leaf $Destination)"
    }
}

if (-not (Test-Path -LiteralPath $iocPath)) {
    throw "CubeMX project file not found: $iocPath"
}

$iocText = Get-Content -LiteralPath $iocPath -Raw
$firmwareMatch = [regex]::Match(
    $iocText,
    '(?m)^ProjectManager\.FirmwarePackage=STM32Cube FW_F4 V(?<version>\d+\.\d+\.\d+)\s*$'
)
if (-not $firmwareMatch.Success) {
    throw 'Cannot determine the STM32CubeF4 firmware version from lift_infantry.ioc.'
}

$firmwareVersion = $firmwareMatch.Groups['version'].Value
$repositoryRoot = if ($env:STM32CUBE_REPOSITORY) {
    $env:STM32CUBE_REPOSITORY
} else {
    Join-Path $env:USERPROFILE 'STM32Cube\Repository'
}
$sourcePort = Join-Path $repositoryRoot "STM32Cube_FW_F4_V$firmwareVersion\Middlewares\Third_Party\FreeRTOS\Source\portable\GCC\ARM_CM4F"

foreach ($fileName in @('port.c', 'portmacro.h')) {
    $sourceFile = Join-Path $sourcePort $fileName
    if (-not (Test-Path -LiteralPath $sourceFile)) {
        throw "Required FreeRTOS AC6 port file not found: $sourceFile"
    }
}

New-Item -ItemType Directory -Path $targetPort -Force | Out-Null
Copy-FileIfDifferent (Join-Path $sourcePort 'port.c') (Join-Path $targetPort 'port.c')
Copy-FileIfDifferent (Join-Path $sourcePort 'portmacro.h') (Join-Path $targetPort 'portmacro.h')

if (-not (Test-Path -LiteralPath $uvprojxPath)) {
    throw "Keil project file not found: $uvprojxPath"
}

$projectText = Get-Content -LiteralPath $uvprojxPath -Raw
$updatedText = $projectText
$updatedText = $updatedText.Replace('portable/RVDS/ARM_CM4F', 'portable/GCC/ARM_CM4F')
$updatedText = $updatedText.Replace('portable\RVDS\ARM_CM4F', 'portable\GCC\ARM_CM4F')
$updatedText = [regex]::Replace(
    $updatedText,
    '<pCCUsed>[^<]*</pCCUsed>',
    "<pCCUsed>$requiredCompiler</pCCUsed>"
)
$updatedText = [regex]::Replace($updatedText, '<uAC6>[^<]*</uAC6>', '<uAC6>1</uAC6>')
$updatedText = [regex]::Replace(
    $updatedText,
    '<PackID>Keil\.STM32F4xx_DFP\.[^<]*</PackID>',
    "<PackID>$requiredPack</PackID>"
)

if ($updatedText -notmatch "<pCCUsed>$([regex]::Escape($requiredCompiler))</pCCUsed>" -or
    $updatedText -notmatch '<uAC6>1</uAC6>' -or
    $updatedText -notmatch "<PackID>$([regex]::Escape($requiredPack))</PackID>") {
    throw 'Unable to restore the required Keil compiler or Device Pack configuration.'
}
if ($updatedText -notmatch 'portable/GCC/ARM_CM4F') {
    throw 'Keil project does not contain the FreeRTOS GCC/ARM_CM4F include or source reference.'
}

if ($updatedText -cne $projectText) {
    $utf8NoBom = New-Object System.Text.UTF8Encoding($false)
    [System.IO.File]::WriteAllText($uvprojxPath, $updatedText, $utf8NoBom)
    Write-Step 'Restored Keil AC6, Device Pack and FreeRTOS port settings'
}

Write-Step "OK: STM32CubeF4 V$firmwareVersion, Arm Compiler 6 FreeRTOS port is ready"
