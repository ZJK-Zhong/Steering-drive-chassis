param(
    [string]$KeilRoot = 'C:\Keil_v5',
    [ValidateSet('FVP', 'Unicorn')][string]$Simulator = 'FVP',
    [string]$PythonExecutable = 'python'
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$mdkRoot = Join-Path $projectRoot 'MDK-ARM'
$outputRoot = Join-Path $mdkRoot 'standard_tpye_c\remote_mode_tests'
New-Item -ItemType Directory -Path $outputRoot -Force | Out-Null
# Compile the actual INS snapshot functions without unrelated SPI/RTOS hardware code.
$insSource = [IO.File]::ReadAllText((Join-Path $projectRoot 'application\INS_task.c'), [Text.Encoding]::GetEncoding(936))
$snapshotStart = $insSource.IndexOf('static ins_motion_t ins_motion;')
$snapshotEnd = $insSource.IndexOf('void INS_task(', $snapshotStart)
if ($snapshotStart -lt 0 -or $snapshotEnd -lt 0) { throw 'INS snapshot source block not found' }
[IO.File]::WriteAllText((Join-Path $outputRoot 'ins_motion_snapshot.inc'),
    $insSource.Substring($snapshotStart, $snapshotEnd - $snapshotStart), [Text.UTF8Encoding]::new($false))
[xml]$project = Get-Content (Join-Path $mdkRoot 'standard_robot.uvprojx')
$controls = $project.Project.Targets.Target.TargetOption.TargetArmAds.Cads.VariousControls
$includes = @($controls.IncludePath.Split(';') | ForEach-Object {
    '-I' + [IO.Path]::GetFullPath((Join-Path $mdkRoot $_.Trim()))
})
$includes += '-I' + $outputRoot
$defines = @($controls.Define.Split(',') | Select-Object -Unique | Where-Object { $_.Trim() -notmatch '^__FPU_USED=' } | ForEach-Object { '-D' + $_.Trim() })
$compiler = Join-Path $KeilRoot 'ARM\ARMCC\bin\armcc.exe'
$assembler = Join-Path $KeilRoot 'ARM\ARMCC\bin\armasm.exe'
$linker = Join-Path $KeilRoot 'ARM\ARMCC\bin\armlink.exe'
$sources = @('tests\remote_modes_test.c', 'components\controller\pid.c', 'components\algorithm\user_lib.c',
    'application\robot.c', 'application\cmd\robot_cmd.c', 'modules\message_center\message_center.c',
    'modules\swerve\swerve_kinematics.c', 'modules\swerve\swerve_module.c')
$objects = @()
foreach ($source in $sources) {
    $object = Join-Path $outputRoot (([IO.Path]::GetFileNameWithoutExtension($source)) + '.o')
    & $compiler --cpu Cortex-M4 --fpu SoftVFP --c99 --split_sections -O1 @includes @defines -c (Join-Path $projectRoot $source) -o $object
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $source" }
    $objects += $object
}
$startup = Join-Path $outputRoot 'startup.o'
& $assembler --cpu Cortex-M4 (Join-Path $PSScriptRoot 'remote_modes_startup.s') -o $startup
if ($LASTEXITCODE -ne 0) { throw 'Startup assembly failed' }
$image = Join-Path $outputRoot 'remote_modes.axf'
& $linker --cpu Cortex-M4 --entry Reset_Handler --ro_base 0x0 --rw_base 0x20000000 --first 'startup.o(RESET)' $startup @objects -o $image
if ($LASTEXITCODE -ne 0) { throw 'Link failed' }
if ($Simulator -eq 'Unicorn') {
    $result = & $PythonExecutable (Join-Path $PSScriptRoot 'emulate_remote_modes.py') $image 2>&1
} else {
    $simulator = Join-Path $KeilRoot 'ARM\FVP\MPS2_Cortex-M\FVP_MPS2_Cortex-M4_MDK.exe'
    $result = & $simulator -a $image -T 20 -C fvp_mps2.mps2_visualisation.disable-visualisation=1 -C fvp_mps2.telnetterminal0.start_telnet=0 -C fvp_mps2.telnetterminal1.start_telnet=0 -C fvp_mps2.telnetterminal2.start_telnet=0 2>&1
}
$result | ForEach-Object { Write-Output $_ }
if ($LASTEXITCODE -ne 0 -or ($result -join "`n") -notmatch 'PASS: switch modes') { throw 'Simulator tests did not pass' }
