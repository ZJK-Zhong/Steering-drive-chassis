/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @details 本底盘工程保留原消息类型和模式编码，硬件专用参数集中在swerve_config.h。
 *          云台/发射的状态枚举仅供原裁判UI使用，不代表此板注册了对应电机。
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"

/* 本工程只编译底盘应用，接收机接底盘 USART3。 */
#define CHASSIS_BOARD

#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 305.47013f       // 216mm回转半径、正方形布局
#define TRACK_WIDTH 305.47013f
#define RADIUS_WHEEL (380.0f / (2.0f * 3.14159265358979323846f))
#define REDUCTION_RATIO_WHEEL (1.0f / 0.0526f)
#define CHASSIS_CENTER 216.0f

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

/**
 * @brief 底盘执行模式，原模式编码保留，本地遥控新增CHASSIS_REMOTE
 * @note 本地右拨杆只发布ZERO_FORCE、REMOTE和ROTATE。
 *       NO_FOLLOW和FOLLOW_GIMBAL_YAW保留底盘侧处理，当前没有遥控入口。
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
    CHASSIS_REMOTE,           // 本地遥控，车体系平移和旋转
} chassis_mode_e;

// 以下模式枚举供原裁判系统 UI 显示状态使用，不注册或控制云台、发射电机。
// 云台显示模式
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // UI 显示无力状态。
    GIMBAL_FREE_MODE,      // UI 显示自由模式。
    GIMBAL_GYRO_MODE,      // UI 显示惯性稳定模式。
} gimbal_mode_e;

// 发射显示模式
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

/**
 * @brief 原裁判UI使用的底盘功率显示数据，单位W
 * @note 该结构只供显示，不是PowerControl的闭环输入。
 */
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;

/* CMD 通过原消息中心发布底盘目标，底盘任务订阅。 */
/**
 * @brief CMD发布到chassis_cmd话题的底盘目标
 * @note vx/vy/wz分别为mm/s、mm/s和rad/s；X向前、Y向右，俯视逆时针为正。
 *       mode只选择平移坐标转换方式，chassis_mode选择底盘实际执行方式，两者含义不同。
 *       本地手动使用mode=0，小陀螺使用mode=3；结构布局沿用原框架并保持pack(1)。
 */
typedef struct
{
    // 控制部分
    float vx;           // mm/s, 前进为正
    float vy;           // mm/s, 向右为正
    float wz;           // rad/s, 俯视逆时针为正
    float offset_angle; // 底盘坐标转换/跟随用角度，单位度；本地遥控保持为 0
    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
    // UI部分
    //  ...

    uint8_t mode;       // 0:车体系, 1:导航, 2:云台系, 3:进入小陀螺时的参考系
    float chassis_spin_speed; // 小陀螺目标角速度，rad/s
} Chassis_Ctrl_Cmd_s;

/**
 * @brief 底盘发布到chassis_feed话题的实际速度和原框架共享反馈字段
 * @note real_vx/real_vy由四轮实际轮速及舵角反算，real_wz为运动学估计，不直接等于陀螺仪读数。
 *       当前底盘只更新速度字段，热量、弹速和颜色字段保留原布局，不能据其默认值判断整车状态。
 */
typedef struct
{
    float real_vx; // 车体系 X 速度，mm/s
    float real_vy; // 车体系 Y 向右速度，mm/s
    float real_wz; // 俯视逆时针角速度，rad/s

    uint8_t rest_heat;           // 剩余枪口热量
    Bullet_Speed_e bullet_speed; // 弹速限制
    Enemy_Color_e enemy_color;   // 0 for blue, 1 for red

} Chassis_Upload_Data_s;


#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H
