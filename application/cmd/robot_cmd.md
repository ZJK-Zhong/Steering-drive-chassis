# robot_cmd

## 总体说明

本模块把一种控制来源转换成 `Chassis_Ctrl_Cmd_s`，再通过 `chassis_cmd` 话题交给底盘任务。
控制源由 `application/chassis/swerve_config.h` 的 `SWERVE_COMMAND_SOURCE` 在编译时唯一选择：

- `SWERVE_COMMAND_SOURCE_LOCAL_RC`：独立底盘工程默认值，底盘USART3直接接收DBUS。
- `SWERVE_COMMAND_SOURCE_GIMBAL_CAN`：与云台C板组成双板时使用，底盘CAN2接收命令并回传底盘状态。

两种路径使用同一个话题和消息结构，`ChassisTask` 不需要区分控制来源。不要同时轮询两路输入，
避免本地遥控和云台指令争用底盘目标。

## 代码结构与外部接口

- `robot_cmd.c`：控制源初始化、输入校验、控制目标发布和双板反馈发送。
- `robot_cmd.h`：初始化及周期任务的调用约定。
- `RobotCMDInit(void)`：在BSP初始化后、创建RTOS任务前调用一次。
- `RobotCMDTask(void)`：由 `RobotTask` 周期调用，且每轮先于 `ChassisTask`。

## 双板CAN接口

线序和ID沿用 `rm-sentry` 的协议方向：底盘连接本板CAN2，云台C板连接本板CAN1；两个CAN收发器
连接同一条总线并使用相同波特率。CAN ID均为标准帧。

| 方向 | CAN接口 | CAN ID | 数据 |
| --- | --- | --- | --- |
| 云台C板 -> 底盘 | 云台CAN1 -> 底盘CAN2 | `0x312` | `Chassis_Ctrl_Cmd_s` |
| 底盘 -> 云台C板 | 底盘CAN2 -> 云台CAN1 | `0x311` | `Chassis_Upload_Data_s` |

云台C板的 `CANComm_Init_Config_s` 应与底盘配置相反：`can_handle=&hcan1`、`tx_id=0x312`、
`rx_id=0x311`、`send_data_len=sizeof(Chassis_Ctrl_Cmd_s)`、
`recv_data_len=sizeof(Chassis_Upload_Data_s)`。两边必须使用相同的 `robot_def.h` 通信结构布局；
这些类型位于 `#pragma pack(1)` 区域。CANComm会拆分长结构并增加帧头、长度和CRC8，不能把裸结构
直接当作单个CAN数据帧发送。

云台C板的通信初始化和发送形式如下，控制量单位按下方字段约定填写：

```c
CANComm_Init_Config_s comm_config = {
    .can_config = {.can_handle = &hcan1, .tx_id = 0x312, .rx_id = 0x311},
    .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    .recv_data_len = sizeof(Chassis_Upload_Data_s),
    .daemon_count = 10,
};
CANCommInstance *chassis_comm = CANCommInit(&comm_config);
Chassis_Ctrl_Cmd_s chassis_cmd = {0};
/* 按遥控器/上位机状态更新chassis_cmd，再周期发送完整结构。 */
CANCommSend(chassis_comm, (uint8_t *)&chassis_cmd);
```

云台侧可按约5ms周期通过 `CANCommSend()` 发送最新完整目标。底盘CANComm以100Hz守护任务计时，
连续约50ms没有完整且CRC正确的新命令后，CMD立即发布无力目标；无效数值或越界速度也会清除旧目标。
底盘每20ms回传一次 `chassis_feed`，避免对与八个电机和超级电容共用的CAN2造成不必要负载。

命令字段约定见 `application/robot_def.h`：`vx/vy` 为mm/s，X向前、Y向右；`wz`和
`chassis_spin_speed` 为rad/s，俯视逆时针为正；小陀螺模式还可使用 `mode=3` 表示入口航向参考系。
具体执行模式由 `chassis_mode` 选择。云台代码发送新格式前需同步两块板上的结构定义和枚举值。

## 本地遥控路径

默认控制源为接收机直连底盘USART3。上电或遥控失联后需要右拨杆先置下档完成解锁；
右下为无力、右中为车体系遥控、右上为小陀螺，左拨杆不选择模式。

| 通道 | 转换 | 结果 |
| --- | --- | --- |
| ch1 | 死区后乘6并限幅 | `vx`，mm/s |
| ch0 | 死区后乘2并限幅 | `vy`，mm/s，向右为正 |
| ch2 | 死区后乘-0.2，再转弧度 | `wz`，rad/s，仅右中档 |

与云台C板组成双板时，将 `SWERVE_COMMAND_SOURCE` 改为 `SWERVE_COMMAND_SOURCE_GIMBAL_CAN` 并重编底盘板；
此时底盘不读取本地USART3接收机，云台C板解析遥控器后发送CAN命令。两种来源始终互斥，
切换控制源需重新编译和烧录。

## 私有状态

| 名称 | 作用 |
| --- | --- |
| `local_chassis_pub` | 发布统一的 `chassis_cmd` 话题 |
| `chassis_can_comm` | CAN控制源下的收发实例，底盘CAN2使用0x311/0x312 |
| `gimbal_command` | 最近一次通过完整性、数值和范围校验的双板控制目标 |
| `gimbal_command_valid` | 首帧前、收到无效命令或通信超时后清零，防止启动/断链沿用旧目标 |
| `local_feedback` | 从 `chassis_feed` 取出的回传数据 |
| `last_feedback_tick` | 限制CAN反馈发送为50Hz |

## 使用注意

- 双板 CAN ID需避开现有底盘电机和超级电容使用的ID；当前配置为CAN2发送0x311、接收0x312。
- 断线后的无力延迟由100Hz守护任务轮询决定，约50ms并带一个轮询周期的调度偏差。
- `Chassis_Upload_Data_s` 当前反馈四舵轮反算出的速度；保留的热量、弹速和颜色字段不是底盘板实时更新的反馈。
- 本接口只负责接收云台控制目标并回传底盘速度，不包含云台C板固件，也不负责遥控器在云台板上的解析。
- 本地默认配置和CAN配置均需分别编译验证；烧录前确认所选控制源与实际接线一致。
