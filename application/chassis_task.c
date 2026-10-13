/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "stdio.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "bsp_usart.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
#include "bsp_delay.h"
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }



static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static void chassis_motor_moving_solution(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set,chassis_move_t *chassis_move_solution);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
static void chassis_CAN_Send(void);
static fp32 normalize_rad(fp32 rad);
	










#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif



//底盘运动数据
chassis_move_t chassis_move;

/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    chassis_init(&chassis_move);

    while (1)
    {
		chassis_feedback_update(&chassis_move);
        chassis_set_contorl(&chassis_move);
        if (chassis_move.mode != CHASSIS_MODE_ZERO_FORCE)
        {
            chassis_motor_moving_solution(chassis_move.vx_set,chassis_move.vy_set,chassis_move.wz_set,&chassis_move);
        }
        chassis_control_loop(&chassis_move);
		chassis_CAN_Send();

        //系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}




/**
  *  初始化
 **/
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
	
	 uint8_t i;
	
	//所用PID参数定义
    const static fp32 driver_motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    const static fp32 steering_motor_angle_pid[3]={STEERING_ENCODE_RELATIVE_PID_KP, STEERING_ENCODE_RELATIVE_PID_KI, STEERING_ENCODE_RELATIVE_PID_KD};
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    const static fp32 spin_rate_pid[3] = {CHASSIS_SPIN_RATE_PID_KP, CHASSIS_SPIN_RATE_PID_KI, CHASSIS_SPIN_RATE_PID_KD};



    //获得数据指针
    chassis_move_init->chassis_RC = get_remote_control_point();//获取遥控器指针 
	for(i=0;i<4;i++)
	{
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);  //驱动电机数据指针
		chassis_move_init->steering_motor[i].steering_motor_measure=get_steering_motor_measure_point(i); //舵向电机数据指针
	}

	
	
	
	
    //初始化所用PID
    for (i = 0;i<4;i++)
    {
		//初始化舵向电机角度环PID
		PID_init(&chassis_move_init->steering_motor[i].steering_motor_relative_angle_pid,PID_POSITION,steering_motor_angle_pid,STEERING_ENCODE_RELATIVE_PID_MAX_OUT,STEERING_ENCODE_RELATIVE_PID_MAX_IOUT);
		//初始化驱动电机速度环PID
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, driver_motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
        //初始化底盘角度计算PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    PID_init(&chassis_move_init->spin_rate_pid, PID_POSITION, spin_rate_pid,
             CHASSIS_SPIN_RATE_PID_MAX_OUT, CHASSIS_SPIN_RATE_PID_MAX_IOUT);
    
	
	
	
	

	



	

	//舵向电机编码器零偏值
	chassis_move_init->steering_motor[0].steering_zero_ecd=4976;
	chassis_move_init->steering_motor[1].steering_zero_ecd=701;
	chassis_move_init->steering_motor[2].steering_zero_ecd=7532;
	chassis_move_init->steering_motor[3].steering_zero_ecd=688;
	
	for(i=0;i<4;i++)
	{
		chassis_move_init->steering_motor[i].steering_motor_circle=0;
	}
	
    //更新一下数据
    chassis_feedback_update(chassis_move_init);
    chassis_move_init->mode = CHASSIS_MODE_ZERO_FORCE;
    chassis_move_init->robot_mode = ROBOT_ZERO_FORCE;
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->steering_motor[i].steering_ecd_angle_set =
            chassis_move_init->steering_motor[i].steering_ecd_angle;
    }
}




/**
  *  状态更新-获得反馈值
 **/
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

	
    uint8_t i = 0;
	static fp32 last_err_ecd[4]={0};
	fp32 err_ecd=0;
	
	
	//舵向电机更新数据
	for(i=0;i<4;i++)
	{
		chassis_move_update->steering_motor[i].steering_motor_measure=get_steering_motor_measure_point(i);
		err_ecd=chassis_move_update->steering_motor[i].steering_motor_measure->ecd-chassis_move_update->steering_motor[i].steering_zero_ecd;
		err_ecd=-err_ecd / 8192 * 360;
		
		if(err_ecd-last_err_ecd[i]>180)
		{
			chassis_move_update->steering_motor[i].steering_motor_circle--;
		}
		if(err_ecd-last_err_ecd[i]<-180)
		{
			chassis_move_update->steering_motor[i].steering_motor_circle++;
			
		}
		
		last_err_ecd[i]=err_ecd;
		chassis_move_update->steering_motor[i].steering_ecd_angle=err_ecd+chassis_move_update->steering_motor[i].steering_motor_circle*360;
	}
	
	
	
	
	//驱动电机更新数据
    for (i=0;i<4;i++)
    {
		
		chassis_move_update->motor_chassis[i].chassis_motor_measure=get_chassis_motor_measure_point(i);
        //更新电机速度，加速度           
        chassis_move_update->motor_chassis[i].speed = chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
    }

}








/**
  *  获得解算值
 **/
robot_mode_e robot_mode_choose(const RC_ctrl_t *rc)
{
    if (rc == NULL) return ROBOT_SAFE;
    switch (rc->rc.s[CHASSIS_RIGHT_SWITCH])
    {
        case RC_SW_DOWN: return ROBOT_ZERO_FORCE;
        case RC_SW_MID: return ROBOT_RC_DEBUG;
        case RC_SW_UP: return ROBOT_SPIN;
        default: return ROBOT_SAFE;
    }
}

static void chassis_set_contorl(chassis_move_t *control)
{
    gimbal_chassis_command_t gimbal;
    ins_motion_t ins;
    chassis_mode_e last_mode;
    int16_t vx_channel, vy_channel, wz_channel;
    fp32 vx, vy, angle;
    if (control == NULL) return;
    last_mode = control->mode;
    control->mode = CHASSIS_MODE_ZERO_FORCE;
    control->vx_set = control->vy_set = control->wz_set = 0.0f;
    control->rc_online = remote_control_snapshot(&control->rc_snapshot);
    control->gimbal_online = gimbal_chassis_snapshot(&gimbal);
    control->ins_online = INS_motion_snapshot(&ins);
    control->ins_yaw = control->ins_online ? CHASSIS_INS_YAW_SIGN * ins.yaw : 0.0f;
    control->ins_wz_dps = control->ins_online ? CHASSIS_INS_YAW_SIGN * ins.gyro_z * 180.0f / PI : 0.0f;
    control->robot_mode = control->rc_online ? robot_mode_choose(&control->rc_snapshot) : ROBOT_SAFE;
    control->chassis_relative_angle = control->gimbal_online ?
        normalize_rad(gimbal.relative_angle * PI / 180.0f) : 0.0f;

    switch (control->robot_mode)
    {
        case ROBOT_RC_DEBUG:
            control->mode = CHASSIS_RC_DEBUG;
            break;
        case ROBOT_MOVE:
            if (control->gimbal_online) control->mode = CHASSIS_FOLLOW_GIMBAL;
            break;
        case ROBOT_SPIN:
            if (control->ins_online) control->mode = CHASSIS_SPIN;
            break;
        default:
            /* Vision and shooting are not implemented on this chassis board. */
            break;
    }
    if (control->mode != last_mode || control->mode == CHASSIS_MODE_ZERO_FORCE)
        PID_clear(&control->chassis_angle_pid);
    if (control->mode != last_mode || control->mode != CHASSIS_SPIN)
    {
        PID_clear(&control->spin_rate_pid);
        control->spin_rate_set_dps = 0.0f;
        control->spin_relative_yaw = 0.0f;
        control->spin_reference_yaw = control->ins_yaw;
    }
    if (control->mode == CHASSIS_MODE_ZERO_FORCE) return;

    rc_deadband_limit(control->rc_snapshot.rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(control->rc_snapshot.rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(control->rc_snapshot.rc.ch[CHASSIS_WZ_CHANNEL], wz_channel, CHASSIS_RC_DEADLINE);
    vx = fp32_constrain(vx_channel * CHASSIS_VX_RC_SEN, -NORMAL_MAX_CHASSIS_SPEED_X, NORMAL_MAX_CHASSIS_SPEED_X);
    vy = fp32_constrain(-vy_channel * CHASSIS_VY_RC_SEN, -NORMAL_MAX_CHASSIS_SPEED_Y, NORMAL_MAX_CHASSIS_SPEED_Y);
    if (control->mode == CHASSIS_RC_DEBUG)
    {
        control->vx_set = vx;
        control->vy_set = vy;
        control->wz_set = -wz_channel * CHASSIS_WZ_RC_SEN;
        return;
    }
    if (control->mode == CHASSIS_SPIN)
    {
        control->spin_relative_yaw = normalize_rad(control->ins_yaw - control->spin_reference_yaw);
        angle = control->spin_relative_yaw;
        control->spin_rate_set_dps += fp32_constrain(CHASSIS_SPIN_SPEED_DPS - control->spin_rate_set_dps,
            -CHASSIS_SPIN_ACCEL_DPS2 * CHASSIS_CONTROL_TIME, CHASSIS_SPIN_ACCEL_DPS2 * CHASSIS_CONTROL_TIME);
        /* Feed forward the desired rate; the outer loop corrects measured yaw rate. */
        control->wz_set = fp32_constrain(control->spin_rate_set_dps +
            PID_calc(&control->spin_rate_pid, control->ins_wz_dps, control->spin_rate_set_dps),
            -CHASSIS_SPIN_MAX_COMMAND_DPS, CHASSIS_SPIN_MAX_COMMAND_DPS);
    }
    else
    {
        angle = control->chassis_relative_angle;
        control->wz_set = -PID_calc(&control->chassis_angle_pid, angle, 0.0f) * 180.0f / PI;
    }
    control->vx_set = arm_cos_f32(angle) * vx + arm_sin_f32(angle) * vy;
    control->vy_set = -arm_sin_f32(angle) * vx + arm_cos_f32(angle) * vy;
}



static fp32 normalize_rad(fp32 rad)
{
    // 把任意角度归一化到 [-PI, PI]
    while (rad > PI)   rad -= 2.0f * PI;
    while (rad < -PI)  rad+= 2.0f * PI;
    return rad;	
}







/**
  *  运动解算-获得目标值
 **/


static void chassis_motor_moving_solution(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set,chassis_move_t *chassis_move_solution)
{
	//由角速度计算分解到每个舵轮的线速度
	uint8_t i=0;
	fp32 theta = atan(1.0/1.0);
	fp32 wheel_angle[4]={0};
	fp32 wheel_speed[4]={0};
	fp32 vw_set =wz_set*3.14f/180.0f * CHASSIS_CENTER_TO_WHEEL;  //转化弧度线速度
	fp32 rpm_ratio=60.0f/(CHASSIS_MOTOT_PER*0.0526f);

    if (vx_set == 0.0f && vy_set == 0.0f && wz_set == 0.0f)
    {
        for (i = 0; i < 4; i++) chassis_move_solution->motor_chassis[i].speed_set = 0.0f;
        return; /* Hold steering direction when sticks are centered. */
    }

	
	
	
//	//赋值四个舵向轮角度
////	wheel_angle[0]=atan2((vy_set+vw_set*arm_cos_f32(theta)),(vx_set-vw_set*arm_sin_f32(theta)));
////	wheel_angle[1]=atan2((vy_set+vw_set*arm_cos_f32(theta)),(vx_set+vw_set*arm_sin_f32(theta)));
////	wheel_angle[2]=atan2((vy_set-vw_set*arm_cos_f32(theta)),(vx_set+vw_set*arm_sin_f32(theta)));
////	wheel_angle[3]=atan2((vy_set-vw_set*arm_cos_f32(theta)),(vx_set-vw_set*arm_sin_f32(theta)));	
//	 
//	wheel_angle[0]=atan2((vy_set-vw_set*arm_cos_f32(theta)),(vx_set+vw_set*arm_sin_f32(theta)));
//	wheel_angle[1]=atan2((vy_set-vw_set*arm_cos_f32(theta)),(vx_set-vw_set*arm_sin_f32(theta)));
//	wheel_angle[2]=atan2((vy_set+vw_set*arm_cos_f32(theta)),(vx_set-vw_set*arm_sin_f32(theta)));
//	wheel_angle[3]=atan2((vy_set+vw_set*arm_cos_f32(theta)),(vx_set+vw_set*arm_sin_f32(theta)));		
	
	//赋值四个舵向轮角度
	wheel_angle[0]=atan2((vy_set+vw_set*arm_cos_f32(theta)),(vx_set-vw_set*arm_sin_f32(theta)));
	wheel_angle[1]=atan2((vy_set+vw_set*arm_cos_f32(theta)),(vx_set+vw_set*arm_sin_f32(theta)));
	wheel_angle[2]=atan2((vy_set-vw_set*arm_cos_f32(theta)),(vx_set+vw_set*arm_sin_f32(theta)));
	wheel_angle[3]=atan2((vy_set-vw_set*arm_cos_f32(theta)),(vx_set-vw_set*arm_sin_f32(theta)));	
	
	for(i=0;i<4;i++)
	{
		fp32 current_angle_deg=chassis_move_solution->steering_motor[i].steering_ecd_angle;
		fp32 current_angle=current_angle_deg*PI/180.0f;   //弧度
		fp32 current_angle_set=wheel_angle[i];
		
		fp32 diff=normalize_rad(current_angle_set-current_angle);    //求差量（单位：弧度）,归一化

		
		chassis_move_solution->steering_motor[i].steering_turn_flag=0;
		
		if(diff>(PI/2.0f))
		{
			diff=diff-PI;
			chassis_move_solution->steering_motor[i].steering_turn_flag=1;
		}
		else if(diff<(-PI/2.0f))
		{
			diff=diff+PI;
			chassis_move_solution->steering_motor[i].steering_turn_flag=1;
		}

		fp32 final_target_angle = current_angle + diff;
		chassis_move_solution->steering_motor[i].steering_ecd_angle_set=final_target_angle*180.0f/PI;
	}
	
	
//	
//	//赋值四个轮向速度大小并有m/s转化为rpm
////	wheel_speed[0]=sqrt(pow((vy_set+vw_set*arm_cos_f32(theta)),2)+pow((vx_set-vw_set*arm_sin_f32(theta)),2))*rpm_ratio;  //左前
////	wheel_speed[1]=sqrt(pow((vy_set+vw_set*arm_cos_f32(theta)),2)+pow((vx_set+vw_set*arm_sin_f32(theta)),2))*rpm_ratio;  //右前
////	wheel_speed[2]=sqrt(pow((vy_set-vw_set*arm_cos_f32(theta)),2)+pow((vx_set+vw_set*arm_sin_f32(theta)),2))*rpm_ratio;  //右后
////	wheel_speed[3]=sqrt(pow((vy_set-vw_set*arm_cos_f32(theta)),2)+pow((vx_set-vw_set*arm_sin_f32(theta)),2))*rpm_ratio;  //左后
//	
//	wheel_speed[0]=sqrt(pow((vy_set-vw_set*arm_cos_f32(theta)),2)+pow((vx_set+vw_set*arm_sin_f32(theta)),2))*rpm_ratio;  //左前
//	wheel_speed[1]=sqrt(pow((vy_set-vw_set*arm_cos_f32(theta)),2)+pow((vx_set-vw_set*arm_sin_f32(theta)),2))*rpm_ratio;  //右前
//	wheel_speed[2]=sqrt(pow((vy_set+vw_set*arm_cos_f32(theta)),2)+pow((vx_set-vw_set*arm_sin_f32(theta)),2))*rpm_ratio;  //右后
//	wheel_speed[3]=sqrt(pow((vy_set+vw_set*arm_cos_f32(theta)),2)+pow((vx_set+vw_set*arm_sin_f32(theta)),2))*rpm_ratio;  //左后	
	
	//赋值四个轮向速度大小并有m/s转化为rpm
	wheel_speed[0]=sqrt(pow((vy_set+vw_set*arm_cos_f32(theta)),2)+pow((vx_set-vw_set*arm_sin_f32(theta)),2))*rpm_ratio;  //左前
	wheel_speed[1]=sqrt(pow((vy_set+vw_set*arm_cos_f32(theta)),2)+pow((vx_set+vw_set*arm_sin_f32(theta)),2))*rpm_ratio;  //右前
	wheel_speed[2]=sqrt(pow((vy_set-vw_set*arm_cos_f32(theta)),2)+pow((vx_set+vw_set*arm_sin_f32(theta)),2))*rpm_ratio;  //右后
	wheel_speed[3]=sqrt(pow((vy_set-vw_set*arm_cos_f32(theta)),2)+pow((vx_set-vw_set*arm_sin_f32(theta)),2))*rpm_ratio;  //左后
	
	for(i=0;i<4;i++)
	{
		if(chassis_move_solution->steering_motor[i].steering_turn_flag==1)
		{
			wheel_speed[i]=wheel_speed[i]*(-1);
			chassis_move_solution->motor_chassis[i].speed_set=wheel_speed[i];
		}
		else if(chassis_move_solution->steering_motor[i].steering_turn_flag==0)
		{
			chassis_move_solution->motor_chassis[i].speed_set=wheel_speed[i];
		}
	}
	
}





/**
  *  PID
 **/

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{

    uint8_t i = 0;

    if (chassis_move_control_loop->mode == CHASSIS_MODE_ZERO_FORCE)
    {
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set = 0.0f;
            chassis_move_control_loop->motor_chassis[i].give_current_set = 0.0f;
            chassis_move_control_loop->motor_chassis[i].give_current = 0;
            chassis_move_control_loop->steering_motor[i].steering_current_set = 0.0f;
            chassis_move_control_loop->steering_motor[i].steering_given_current = 0;
            chassis_move_control_loop->steering_motor[i].steering_ecd_angle_set =
                chassis_move_control_loop->steering_motor[i].steering_ecd_angle;
            chassis_move_control_loop->steering_motor[i].steering_turn_flag = 0;
            PID_clear(&chassis_move_control_loop->motor_speed_pid[i]);
            PID_clear(&chassis_move_control_loop->steering_motor[i].steering_motor_relative_angle_pid);
        }
        return;
    }

	
	//驱动电机
	for (i = 0; i < 4; i++)
	{
		chassis_move_control_loop->motor_chassis[i].give_current_set=PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
		chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)chassis_move_control_loop->motor_chassis[i].give_current_set;
		
	}
	
	
	
	//舵向电机
	for(i=0;i<4;i++)
	{
		chassis_move_control_loop->steering_motor[i].steering_current_set=-PID_calc(&chassis_move_control_loop->steering_motor[i].steering_motor_relative_angle_pid,
		chassis_move_control_loop->steering_motor[i].steering_ecd_angle,chassis_move_control_loop->steering_motor[i].steering_ecd_angle_set);
		
		chassis_move_control_loop->steering_motor[i].steering_given_current=(int16_t)chassis_move_control_loop->steering_motor[i].steering_current_set;
	}
	
		
	
}


/**
  *  CAN报文发送
 **/
static void chassis_CAN_Send(void)
{
	CAN_cmd_steering_chassis(chassis_move.steering_motor[0].steering_given_current,chassis_move.steering_motor[1].steering_given_current,chassis_move.steering_motor[2].steering_given_current,chassis_move.steering_motor[3].steering_given_current);	
	CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current,chassis_move.motor_chassis[1].give_current,chassis_move.motor_chassis[2].give_current,chassis_move.motor_chassis[3].give_current);
			
}

