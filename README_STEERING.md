# Steering-drive-chassis

## 工程来源和边界

本工程以 `E:/RM/rm-sentry` 为基础建立，保留其 FreeRTOS 任务、
RobotInit/RobotTask、CMD、message_center、BSP、DJIMotor、PowerControl、
BMI088/QuaternionEKF/DWT、裁判系统、超级电容和构建工具。
以该目录当前磁盘内容为准，包括其未提交修改；原全向轮和旧舵轮工程均不修改。不携带原工程的 Git 历史及旧编译产物。
原 USB 视觉/遥测初始化保留；独立底盘工程默认由底盘USART3接收DBUS，也可编译切换为云台C板CAN命令输入。

从 `E:/RM/舵轮哨兵代码/Steering-drive-chassis` 仅复用了：

- `modules/swerve/swerve_kinematics.c/.h` 的四舵轮速度矢量、最短转向及驱动反向算法。
- 已调试的电机型号、CAN 分配、轮序、零位、轮周长、减速比和 P 参数。

没有移植旧舵轮工程的 CAN_receive、remote_control、INS_task、PID、RTOS 或 BSP。
新工程的电流由原全向轮框架 MotorControlTask 发送，不在 ChassisTask 内另发一套 CAN 帧。

当前只包含底盘应用和底盘 CMD；已删除 `application/gimbal`、`application/shoot`、
原整车 CMD 的停用分支，以及云台/发射专用的控制参数和消息结构。
裁判系统 UI 和 USB 公共协议中的状态枚举与字段仍保留，不会注册云台或发射电机。
底盘侧跟随/导航坐标转换仍保留，但本地遥控只提供车体系遥控和入陀螺参考系两种输入，
双板CAN接收与底盘反馈复用了 `rm-sentry` 的CANComm；本工程仍是底盘板固件，不包含云台C板应用。

## 源码保留范围

全向轮框架以 `E:/RM/rm-sentry` 的修改版为准，包括该目录当时的未提交修改。
保留其底层 BSP、HAL、FreeRTOS、消息中心、INS、PID、电机、功率控制、通信、
裁判系统和构建工具；只复用旧舵轮的运动学和已确认硬件参数。
Makefile 保留修改版的 JFlash 安装路径和 OpenOCD 下载命令，VS Code 调试目标指向本工程 ELF。
基础目录里的 `.assets`、`.Doc` 保留作资料，不参与编译。
本版运行入口只有 `RobotCMDTask -> ChassisTask`，不再保留被条件编译隐藏的整车控制分支。

源码注释按原框架的开发指南整理：对外接口在头文件中使用Doxygen说明用途、参数、
单位、返回值与调用条件；私有函数在源文件中说明；复杂解算段解释公式、数据来源和设计原因。
应用与模块的总体说明、接口、类型、私有数据、使用范例分别见
[底盘说明](application/chassis/chassis.md)、[CMD说明](application/cmd/robot_cmd.md)和
[舵轮解算说明](modules/swerve/swerve_kinematics.md)。
裁判 UI 内的历史演示状态不代表真实云台/发射反馈；当前调底盘以 `swerve_debug` 为准。

## 调用链

```text
独立底盘默认：底盘 USART3 / RemoteControl -> RobotCMDTask -> 原 message_center 的 chassis_cmd
双板配置：云台C板接收机 -> 云台C板解析 -> CAN1 0x312 -> 底盘CAN2 CANComm
  -> RobotCMDTask -> 同一个 chassis_cmd 话题
  -> ChassisTask：模式、坐标转换、SwerveSolve、舵向误差补偿
  -> PowerControlInit 注册的四个驱动电机 + DJIMotorInit 注册的四个舵向电机
  -> 原 MotorControlTask：DJIMotorControl + PowerControl -> 原 BSP CAN
底盘 chassis_feed -> RobotCMDTask CANComm -> 底盘 CAN2 0x311 -> 云台C板 CAN1
原 INS_Task：BMI088 + QuaternionEKF + DWT 实测 dt -> 底盘航向/转速反馈
```

保留原 RobotTask 5ms、MotorTask 1ms、DaemonTask 10ms 的任务结构。
本地 CMD 与底盘使用原消息中心；双板CAN仅在选中对应控制源时收发，不引入额外中断入口。

## 电机配置

轮序统一为：0 左前、1 右前、2 右后、3 左后。

| 轮位 | 驱动 M3508 | 舵向 GM6020 | 舵向零位 |
|---|---|---|---|
| 左前 | CAN1 ID1 / 反馈 0x201 | CAN2 ID1 / 反馈 0x205 | 4976 |
| 右前 | CAN1 ID2 / 反馈 0x202 | CAN2 ID2 / 反馈 0x206 | 701 |
| 右后 | CAN1 ID3 / 反馈 0x203 | CAN2 ID3 / 反馈 0x207 | 7532 |
| 左后 | CAN1 ID4 / 反馈 0x204 | CAN2 ID4 / 反馈 0x208 | 688 |

驱动命令 CAN1 0x200，舵向命令 CAN2 0x1FF；舵向反馈角取编码器负方向，
输出取原角度 P 环的负值，保留已调试方向。驱动 P=5，以 rpm 为反馈；舵向 P=360，以度为反馈。
驱动输出上限 16000、舵向上限 30000。舵向使用原 DJIMotor 的 OTHER_FEED 角度环，
驱动使用原 PowerControl 的 rpm 速度环及功率模型，避免把度/秒误当成 rpm。
双板时另占用底盘CAN2标准ID 0x312接收命令、0x311发送底盘反馈；该总线还连接舵向电机和超级电容。

几何与限幅集中在 `application/chassis/swerve_config.h`。
轮心距 0.216m、轮周长 0.38m、输出轴/转子转速比 0.0526；保留原正方形布局假设。
四轮整体限速，不分别裁剪轮速；舵向误差较大时通过 cos(误差) 降低驱动轮速。

## 遥控器

独立底盘工程默认接收机插在底盘USART3，使用原全向轮工程 DBUS 解析与串口 BSP。
与云台C板组成双板时，接收机插云台C板，由云台C板解析后经CAN下发底盘命令。

| 实际右拨杆 | 模式 |
|---|---|
| 下 | 八电机零输出，无力 |
| 中 | 车体系遥控：前后、左右平移和旋转 |
| 上 | INS 小陀螺：默认 120度/秒，60度/秒平方爬升 |

左拨杆任意合法档位不影响模式，也不重置小陀螺参考方向。
上电或遥控器失联恢复后，必须先拨右下档，再拨中或上档才能运动。
右摇杆前后 ch1 控制 X，右摇杆左右 ch0 控制 Y，左摇杆水平 ch2 控制手动旋转。
四个通道及拨轮、左右开关均进行有效性校验，摇杆死区为 ±10。

`application/chassis/swerve_config.h` 默认选择 `SWERVE_COMMAND_SOURCE_LOCAL_RC`。
与云台C板组成双板时，将 `SWERVE_COMMAND_SOURCE` 改为 `SWERVE_COMMAND_SOURCE_GIMBAL_CAN` 并重编底盘板；
云台C板解析遥控器后，通过CANComm发送控制命令。
云台CAN1发送ID 0x312的 `Chassis_Ctrl_Cmd_s`，底盘CAN2回传ID 0x311的 `Chassis_Upload_Data_s`；
两板必须匹配 `robot_def.h` 的pack(1)结构布局。CAN命令约50ms未更新、CRC错误或字段超限时无力；
底盘反馈按20ms一次发送。详细字段和云台侧CANComm配置见 [CMD双板接口说明](application/cmd/robot_cmd.md)。

## 单位和方向

CMD/底盘反馈及双板命令保留全向轮应用约定：X 前进、Y 向右，速度 mm/s，角速度 rad/s，
俯视逆时针角速度为正。INS 原框架输出 Yaw 为度、Gyro 为 rad/s。
进入舵轮解算时才转换成 m/s、Y 向左、deg/s；轮速参考输出 rpm，舵向参考输出度。

手动旋转保留此前已经确认的遥控器方向：`wz = -ch2 * 0.2 * DEGREE_2_RAD`。
小陀螺 X 正方向是进入该模式时的底盘前方，不是云台朝向。
只改变左拨杆档位不会重新捕获此方向；退出后重新进入才捕获。

本版 INS 使用全向轮框架的 EKF 和实测 DWT 积分间隔，未复制旧 INS。
但仅靠底盘 IMU 不能保证长期绝对航向无漂移；舵向误差补偿也不是位置闭环。
此前实车越转越偏的问题仍需要对比实际转角、Yaw 和舵向跟踪验证，不能凭仿真宣称已消除。

## 安全与外围

- 本地遥控器100ms超时、短帧、非法通道或开关：CMD无力，恢复需要右下重新解锁。
- 双板CAN模式要求收到一条有效完整命令后才允许运动；通信超时、CRC失败或数值越界会清除目标。
- 任一电机从未收到反馈或反馈超过 20ms：底盘应用停止全部八电机。
- 电机任务还会独立检查反馈和目标更新时间；超过 20ms 不再持续输出旧指令。
- 小陀螺要求 INS 快照有效，20ms 未更新、陀螺仪芯片 ID 读取失败或非有限数值时停止。
- 零力是 CAN 输出零，不是轮速闭环的零速刹车。P 参数及方向仍需架空实测。
- 保留裁判系统 USART6、UI、超级电容 CAN2 0x061/0x052；无裁判功率上限时使用 45W。
- 保留原 PowerControl 功率模型和超电策略，尚未针对舵轮重新标定。
  此模型只覆盖四个驱动电机，不包含舵向功耗，不能视为已经完成比赛整车功率合规验证。
- 上电时保持底盘静止，原 BMI088 初始化包含零偏校准；不要在校准期间转动底盘。

## 构建

在本目录运行，工具链为 ARM GNU GCC：

```powershell
mingw32-make -j8
```

生成 `build/Steering-drive-chassis.elf`、`.hex`、`.bin`。
保留原 CMake 构建，避免与 Make 共用缓存目录：

```powershell
cmake -S . -B build-cmake -G "MinGW Makefiles"
cmake --build build-cmake -j8
```

CMake 默认保留 `-Wall`，`-DSTRICT_WARNINGS=ON` 可启用警告视为错误。
原框架有未使用变量/函数等警告，严格模式下需要先处理这些警告。
本机已验证 GCC 13.3.0 的 Make 完整编译；本机未找到 CMake，未执行 CMake 配置和构建。
链接器会提示 newlib-nosys 的 `_read/_write/_close/_lseek` 不实现，这是原裸机库的占位警告。
无 Keil 工程，不需要从旧舵轮工程复制驱动或启动文件。

## 软件测试

测试使用实际新工程的 CMD、DBUS 解析、消息中心、底盘、电机 PID 和 CAN 分组代码；
模拟物理 UART、CAN、INS 和裁判系统，不替代实车测试。

```powershell
python -m pip install -r tests/requirements.txt
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run.ps1
```

也可指定 `-PythonExecutable` 和已安装依赖的 `-RuntimePath`，不需要 Keil。
本地和云台CAN两种控制源分别编译并运行；覆盖最短转向与反转、跨圈角度、混合速度矢量、
零速保持、右三档、左档独立、单位转换、CAN ID/反馈节流/CAN通信超时、
上电解锁、小陀螺爬升与跨 ±180度、INS 无效、遥控短帧/越界/失联、
电机失联以及目标过期时八电机 CAN 零输出。

## 首次实车检查

先架空、留好断电手段，右拨杆下档，确认八电机均无力；上电校准期间不移动。
观察 `swerve_debug`，等八电机在线后，低速逐项验证前进、横移和旋转方向。
确认实际舵向与目标一致，再进行低速小陀螺和落地测试。

`command_online` 表示本地 CMD 消息是否到达，不等同于遥控器在线；遥控器离线时 CMD 仍发布无力指令。
建议 Watch：

```c
swerve_debug.mode
swerve_debug.command_online
swerve_debug.motors_online
swerve_debug.ins_online
swerve_debug.yaw_deg
swerve_debug.relative_yaw_deg
swerve_debug.gyro_z_radps
swerve_debug.spin_rate_ref_radps
swerve_debug.wheel_target_rpm
swerve_debug.wheel_actual_rpm
swerve_debug.steering_target_deg
swerve_debug.steering_actual_deg
```

本版只完成软件构建与仿真验证，未连接硬件、未烧录。
