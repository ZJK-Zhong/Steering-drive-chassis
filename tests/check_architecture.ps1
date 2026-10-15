$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$modulePaths = @('modules\swerve\swerve_module.c', 'modules\swerve\swerve_module.h',
    'modules\swerve\swerve_kinematics.c', 'modules\swerve\swerve_kinematics.h',
    'modules\message_center\message_center.c', 'modules\message_center\message_center.h')
foreach ($modulePath in $modulePaths) {
    $source = [IO.File]::ReadAllText((Join-Path $projectRoot $modulePath))
    if ($source -match '#include\s+"(?:main|chassis|CAN_receive|INS_task|remote_control)\.h"|\bHAL_\w+|\b__get_PRIMASK\b|\b__disable_irq\b|\b__set_PRIMASK\b') {
        throw "Module exposes application or STM32 runtime dependencies: $modulePath"
    }
}
$robotSource = [IO.File]::ReadAllText((Join-Path $projectRoot 'application\robot.c'))
if ($robotSource -notmatch '(?s)void RobotInit\(void\)\s*\{[^}]*ChassisInit\(\)') {
    throw 'Chassis instances must initialize in RobotInit before the scheduler'
}
$taskSource = [IO.File]::ReadAllText((Join-Path $projectRoot 'application\robot_task.c'))
if ($taskSource -match '\b(?:ChassisInit|RobotControlInit)\s*\(' -or
    $taskSource -notmatch '(?s)vTaskDelay\(pdMS_TO_TICKS\(CHASSIS_TASK_INIT_TIME\)\);\s*RobotControlEnable\(\);') {
    throw 'The startup task must only enable initialized control after settling'
}
[xml]$project = Get-Content -LiteralPath (Join-Path $projectRoot 'MDK-ARM\standard_robot.uvprojx')
$files = @($project.Project.Targets.Target.Groups.Group.Files.File)
foreach ($requiredFile in @('bsp_runtime.c', 'swerve_module.c', 'swerve_kinematics.c', 'chassis.c')) {
    $entries = @($files | Where-Object { $_.FileName -eq $requiredFile })
    if ($entries.Count -ne 1 -or
        -not (Test-Path -LiteralPath (Join-Path (Join-Path $projectRoot 'MDK-ARM') $entries[0].FilePath))) {
        throw "Keil must include one valid production source: $requiredFile"
    }
}
Write-Output 'PASS: module boundaries, pre-scheduler initialization, Keil source registration'
