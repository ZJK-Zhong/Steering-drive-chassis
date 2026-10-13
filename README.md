# 轮舵哨兵底盘 Keil 工程

这是一个基于 STM32F407IGHx 的 RoboMaster 风格轮舵底盘控制工程，使用 STM32CubeMX 生成外设初始化代码，使用 Keil MDK-ARM 编译，运行 CMSIS-RTOS v1 / FreeRTOS。

## 工程信息

- MCU: STM32F407IGHx
- Keil 工程: `MDK-ARM/standard_robot.uvprojx`
- CubeMX 配置: `standard_robot.ioc`
- 主入口: `Src/main.c`
- RTOS 任务入口: `Src/freertos.c`
- Keil Target: `standard_robot`
- 编译输出目录: `MDK-ARM/standard_tpye_c/`

## 目录结构

```text
application/   机器人业务逻辑：底盘、云台、遥控器、INS、裁判系统、电压检测等
bsp/boards/    板级外设封装：CAN、USART、RC、SPI、I2C、LED、蜂鸣器等
components/    通用组件：PID、AHRS、BMI088、IST8310、OLED、CRC、FIFO 等
Drivers/       STM32 HAL 和 CMSIS
Inc/           CubeMX 生成的头文件
Src/           CubeMX 生成的源文件、main 和 freertos 初始化
Middlewares/   FreeRTOS 和 USB Device 中间件
MDK-ARM/       Keil 工程文件、启动文件和本地编译输出
```

## 当前主要运行链路

`main.c` 完成 HAL、时钟和外设初始化后，初始化 CAN 过滤器、延时、校准参数、遥控器和 USART DMA，然后进入 FreeRTOS 调度。

当前 `Src/freertos.c` 中启用的主要任务包括：

- `calibrate_task`
- `chassis_task`
- `INS_task`
- `led_RGB_flow_task`
- `oled_task`
- `battery_voltage_task`

部分模块代码已保留但任务创建暂时注释，例如 `detect_task`、`gimbal_task`、`referee_usart_task`、`usb_task`、`servo_task`。

底盘控制核心位于 `application/chassis_task.c`，控制周期为 2 ms，流程大致为：

```text
反馈更新 -> 遥控/行为控制量设定 -> 轮舵运动解算 -> PID 控制 -> CAN 发送
```

CAN 电机收发位于 `application/CAN_receive.c`：

- CAN1 接收 3508 底盘驱动电机反馈，常用 ID 为 `0x201` 到 `0x204`
- CAN2 接收 6020 舵向电机反馈，常用 ID 为 `0x205` 到 `0x208`
- `CAN_cmd_chassis()` 发送驱动电机电流
- `CAN_cmd_steering_chassis()` 发送舵向电机电流

## 遥控器档位与底盘调试

接收机接底盘板，底盘只使用右拨杆选择模式，左拨杆上/中/下均不影响底盘模式。
按用户反馈修正实际左右对应：右拨杆为 `rc.s[0]`，左拨杆为 `rc.s[1]`；
上/中/下的数值分别为 1/3/2。旧解析器的左右注释不适用于这只遥控器。
对应宏为 `CHASSIS_RIGHT_SWITCH = 0`、`CHASSIS_LEFT_SWITCH = 1`。

| 右拨杆 | 左拨杆 | 请求模式 | 底盘行为 |
| --- | --- | --- | --- |
| 下 | 任意档 | ZERO_FORCE | 四驱动、四舵向全部零指令，不主动保持角度 |
| 中 | 任意档 | RC_DEBUG | 独立遥控底盘，不需要云台或 INS 在线 |
| 上 | 任意档 | SPIN | 底盘 INS 小陀螺，目标 120 度/秒，不需要云台在线 |

右拨杆无效或遥控帧校验/超时失败时零力。左拨杆不参与模式选择，但无效拨杆
编码仍作为损坏遥控帧处理，不放宽接收安全校验。
本次没有修改云台板固件，也没有启用底盘保留的云台/射击任务。
云台跟随、视觉和发射模式不再通过底盘拨杆选择，保留的代码不代表当前可进入。

### 只烧录底盘时

先架空底盘，右拨杆置下档，再切换到中档；左拨杆任意档。
`ch[1]` 控制前后、`ch[0]` 控制横移、`ch[2]` 控制旋转。
前后、横移沿用原通道方向和灵敏度，前后速度限幅 2 m/s、横移限幅 1.5 m/s；
独立遥控档旋转输入取反：`wz_set = -ch[2] * 0.2f`，用于修正实车左打时顺时针的问题。
此修改只影响独立遥控档，不改变四轮运动解算、云台跟随或小陀螺指令；
实车左打逆时针、右打顺时针仍需低速确认。
旋转灵敏度为 0.2 度/秒/通道单位。摇杆回中时驱动速度设为零、舵向保持当前目标；
右拨杆切到下档才是完全零力。遥控数据无效或超过 100 ms 未更新，八个电机指令归零。
独立调试档不因云台掉线而停机。

### INS 小陀螺

右拨杆上档进入小陀螺，左拨杆任意档。先确认 `chassis_move.ins_online = 1`，
摇杆回中时原地旋转；`ch[1]` 前后、`ch[0]` 横移，`ch[2]` 不控制旋转。
进入模式时记录底盘 yaw，之后平移指令始终以该时刻的朝向为参考，
不会随车身旋转而改变操作方向。退出后再次进入会重新记录参考朝向。
这是进入时朝向参考，不是云台参考；长时间运行会受 INS yaw 漂移影响。

`CHASSIS_SPIN_SPEED_DPS = 120` 为目标旋转速度，可改为负值反向。
`CHASSIS_SPIN_ACCEL_DPS2 = 60` 将目标从零升至 120 度/秒，约需 2 秒。
使用校准、安装变换后的 INS Z 轴角速度，转为度/秒；在目标速度前馈上叠加
旋转速度 PID 修正。初始 Kp=0.3、Ki=Kd=0，修正限幅 30 度/秒，
最终运动解算角速度限幅 180 度/秒。这些是保守的初始参数，仍需实车调试。
四轮解算、驱动/舵向 PID 增益和零偏保持不变。

`CHASSIS_INS_YAW_SIGN = 1` 要求 INS yaw 和角速度正方向与运动解算 `wz_set` 正方向一致。
该参数不等同于遥控 `ch[2]` 的取反。若实测 `ins_wz_dps` 与旋转指令方向相反，
立即将右拨杆置下档停机，确认安装方向后把该参数改为 -1；不能通过增加 PID 增益补偿符号错误。

INS 快照只在实际读到新陀螺仪数据并更新姿态后发布；未初始化、超过 20 ms
未发布新数据、yaw/角速度无效时，小陀螺切回八电机零力。
遥控超时同样零力。数据恢复后若拨杆仍在小陀螺档，会重新记录朝向并从零升速；
排查时保持右拨杆下档。云台 CAN 掉线不影响本档。

Keil Watch 可添加：

- `chassis_move.rc_snapshot.rc.s`、`chassis_move.rc_snapshot.rc.ch`
- `chassis_move.robot_mode`：右拨杆请求模式
- `chassis_move.mode`：实际执行档位，0=零力、1=独立遥控、3=小陀螺；保留值 2 当前不由拨杆选择
- `chassis_move.rc_online`、`chassis_move.gimbal_online`
- `chassis_move.ins_online`：INS 数据是否有效且及时更新
- `chassis_move.ins_yaw`、`chassis_move.spin_reference_yaw`、`chassis_move.spin_relative_yaw`：弧度
- `chassis_move.spin_rate_set_dps`、`chassis_move.ins_wz_dps`：目标/实际旋转速度，度/秒
- `chassis_move.spin_rate_pid.out`：速度反馈修正量，度/秒
- `chassis_move.vx_set`、`chassis_move.vy_set`、`chassis_move.wz_set`（旋转为度/秒）
- `chassis_move.motor_chassis`：四路驱动 `speed`、`speed_set`、`give_current`
- `chassis_move.steering_motor`：四路舵向 `steering_ecd_angle`、`steering_ecd_angle_set`、`steering_given_current`

### 云台联调接口

底盘 CAN1 接收标准数据帧 `0x250`，DLC=8，四个有符号 16 位大端字段：
`vx`、`vy`、`relative_angle`（度）、旧版云台 `mode`。
底盘只使用 `relative_angle`，本地遥控器决定模式和移动量，不受云台帧中的模式覆盖。
角度有效范围为 -360 到 360 度，归一化至 [-PI, PI]。
当前三档底盘模式均不依赖此帧；保留的云台跟随代码当前没有拨杆入口。
仓库 `sentry_gimbal_steering_code` 从它的 CAN2 发送该帧，应连到本工程底盘 CAN1，
CAN_H/CAN_L 对应并共地，检查波特率及终端电阻。

本工程保留 USART1 转发遥控原始帧：`0xA6 + 18字节DBUS + 1字节累加校验`。
它不是直接可接普通 DBUS 解析器的 18 字节帧；云台板需实现对应接收解包和
云台端自己的模式映射。底盘当前只实现右拨杆三档，不执行摩擦轮、发射及视觉模式。

驱动 PID 输出上限修正为 16000，舵向输出上限修正为 30000，避免原超大限幅
转为 `int16_t` 时溢出。保留原 PID 增益、舵向零偏与当前运动解算。

### 软件回归测试

`powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_remote_modes.ps1`
使用 Keil ARMCC 编译实际控制源码，并用 Keil FVP Cortex-M4 模拟器测试右拨杆三档及左拨杆独立性、
DBUS 解析、独立遥控、云台模式不覆盖本地选择、INS 小陀螺升速/反馈、yaw 回绕、INS 无效数据/超时、
CAN 帧校验、超时回绕、输出限幅和零力发送。
FVP 需要有效许可证，本机报 `No License found`。也可用隔离安装的 Unicorn
运行同一 ARM 测试镜像，不需要 FVP 许可证：

```powershell
python -m pip install --target MDK-ARM/standard_tpye_c/remote_mode_tests/runtime -r tests/requirements.txt
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_remote_modes.ps1 -Simulator Unicorn
```

Python 不在 PATH 时可加 `-PythonExecutable` 指定路径。测试从当前 INS 源码提取
快照发布/读取函数，直接编译底盘、遥控和 CAN 控制源码，只替代 HAL/RTOS 硬件接口。
本次固件构建通过（0 错误、5 个旧代码未使用函数警告），Unicorn 软件回归测试通过。
未测试实体 SPI 采集、电机和真实机械动态，未自动烧录；首次仍需架空低速验证。

## 打开和编译

1. 使用 Keil uVision 打开 `MDK-ARM/standard_robot.uvprojx`。
2. 确认本机已安装 STM32F4xx 相关 Device Pack。
3. 选择 Target `standard_robot`。
4. 直接 Build/Rebuild，编译产物会生成到 `MDK-ARM/standard_tpye_c/`。

## Git 管理说明

仓库保留源码、CubeMX 配置、Keil 工程文件和必要启动/RTE 文件；忽略 Keil 编译输出、下载调试日志、用户界面状态和临时文件。

如果需要保存某次可烧录固件，建议单独建立发布目录或使用 Git tag/Release 管理，不建议把每次构建生成的 `.hex`、`.axf`、`.map` 直接纳入日常版本控制。
