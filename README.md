# Steering-drive-chassis

基于 `E:/RM/rm-sentry` 修改版全向轮哨兵框架的独立舵轮底盘工程。

本工程控制四个 M3508 驱动电机和四个 GM6020 舵向电机，保留原框架的 FreeRTOS、
CMD、消息中心、BSP、电机控制、INS、裁判系统、超级电容和 USB 遥测。
云台、发射应用和原整车 CMD 已移除；公共通信报文和裁判 UI 所需的状态定义保留。

接收机接底盘 USART3：右拨杆下无力、中遥控、上小陀螺，左拨杆不选择模式。
上电或遥控失联恢复后，必须先拨右下解锁。

配置、运行调用链、软件测试和烧录前检查见 [README_STEERING.md](README_STEERING.md)。
