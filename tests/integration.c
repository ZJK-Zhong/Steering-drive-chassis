/* Execute production CMD, remote parser, chassis, motor controllers and pub/sub.
 * Only physical UART/CAN/INS/referee/DWT interfaces are simulated. */
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../application/chassis/chassis.c"
#include "../application/cmd/robot_cmd.c"
#include "../modules/remote/remote_control.c"
#if SWERVE_COMMAND_SOURCE == SWERVE_COMMAND_SOURCE_GIMBAL_CAN
#include "../modules/can_comm/can_comm.h"
#endif

CAN_HandleTypeDef hcan1, hcan2;
UART_HandleTypeDef huart1, huart3, huart6;
static uint32_t test_tick = 100;
static CANInstance *test_can[16];
static unsigned test_can_count;
static USARTInstance test_uart;
static referee_info_t test_referee;
static attitude_t test_imu;
static uint8_t test_ins_online = 1;
static uint8_t drive_frame[8], steer_frame[8];
#if SWERVE_COMMAND_SOURCE == SWERVE_COMMAND_SOURCE_GIMBAL_CAN
static CANCommInstance test_chassis_comm;
static uint8_t test_chassis_comm_online = 1;
static uint8_t test_feedback_send_count;
static Chassis_Upload_Data_s test_feedback_sent;
#endif
static char test_heap[32768] __attribute__((aligned(8)));
static ptrdiff_t heap_used;

void *_sbrk(ptrdiff_t increment)
{
    if (increment < 0 || heap_used + increment > sizeof(test_heap)) return (void *)-1;
    void *result = test_heap + heap_used;
    heap_used += increment;
    return result;
}

static void host_call(uint32_t operation, const void *argument)
{
    register uint32_t op __asm__("r0") = operation;
    register const void *arg __asm__("r1") = argument;
    __asm__ volatile("bkpt 0xab" : "+r"(op) : "r"(arg) : "memory");
}

static void finish(uint32_t code)
{
    uint32_t args[2] = {0x20026, code};
    host_call(0x20, args);
    for (;;) {}
}

#define STR_(n) #n
#define STR(n) STR_(n)
#define CHECK(condition) do { if (!(condition)) { \
    host_call(4, "FAIL line " STR(__LINE__) ": " #condition "\n"); finish(__LINE__); } } while (0)
#define NEAR(a, b, eps) CHECK(fabsf((a) - (b)) <= (eps))

uint32_t HAL_GetTick(void) { return test_tick; }
float DWT_GetDeltaT(uint32_t *counter) { *counter = test_tick * 168000u; return 0.001f; }
CANInstance *CANRegister(CAN_Init_Config_s *cfg)
{
    CHECK(test_can_count < 16);
    CANInstance *result = calloc(1, sizeof(*result));
    CHECK(result != NULL);
    result->can_handle = cfg->can_handle;
    result->rx_id = cfg->rx_id;
    result->tx_id = cfg->tx_id;
    result->id = cfg->id;
    result->can_module_callback = cfg->can_module_callback;
    test_can[test_can_count++] = result;
    return result;
}
uint8_t CANTransmit(CANInstance *instance, float timeout)
{
    if (instance->can_handle == &hcan1 && instance->txconf.StdId == 0x200)
        memcpy(drive_frame, instance->tx_buff, 8);
    if (instance->can_handle == &hcan2 && instance->txconf.StdId == 0x1ff)
        memcpy(steer_frame, instance->tx_buff, 8);
    return 1;
}
USARTInstance *USARTRegister(USART_Init_Config_s *cfg)
{
    test_uart.usart_handle = cfg->usart_handle;
    test_uart.recv_buff_size = cfg->recv_buff_size;
    test_uart.module_callback = cfg->module_callback;
    return &test_uart;
}
void USARTServiceInit(USARTInstance *instance) {}
attitude_t *INS_Init(void) { return &test_imu; }
uint8_t INS_GetSnapshot(attitude_t *out) { *out = test_imu; return test_ins_online; }
referee_info_t *UITaskInit(UART_HandleTypeDef *uart, Referee_Interactive_info_t *ui)
{ return &test_referee; }
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *cfg) { return NULL; }
SuperCapInstance *GetSuperCapInstance(void) { return NULL; }
void SuperCapSend(SuperCapInstance *instance, SuperCap_Control_t *control) {}
SuperCap_Feedback_t SuperCapGet(SuperCapInstance *instance)
{ SuperCap_Feedback_t out = {0}; return out; }
#if SWERVE_COMMAND_SOURCE == SWERVE_COMMAND_SOURCE_GIMBAL_CAN
CANCommInstance *CANCommInit(CANComm_Init_Config_s *config)
{
    CHECK(config->can_config.can_handle == &hcan2);
    CHECK(config->can_config.tx_id == SWERVE_CHASSIS_CAN_TX_ID);
    CHECK(config->can_config.rx_id == SWERVE_CHASSIS_CAN_RX_ID);
    CHECK(config->send_data_len == sizeof(Chassis_Upload_Data_s));
    CHECK(config->recv_data_len == sizeof(Chassis_Ctrl_Cmd_s));
    CHECK(sizeof(Chassis_Upload_Data_s) <= CAN_COMM_MAX_BUFFSIZE);
    CHECK(sizeof(Chassis_Ctrl_Cmd_s) <= CAN_COMM_MAX_BUFFSIZE);
    CHECK(config->daemon_count == SWERVE_CHASSIS_CAN_DAEMON_TICKS);
    memset(&test_chassis_comm, 0, sizeof(test_chassis_comm));
    test_chassis_comm.can_ins = CANRegister(&config->can_config);
    return &test_chassis_comm;
}
void *CANCommGet(CANCommInstance *instance)
{
    instance->update_flag = 0;
    return instance->unpacked_recv_data;
}
uint8_t CANCommIsOnline(CANCommInstance *instance)
{ return test_chassis_comm_online; }
void CANCommSend(CANCommInstance *instance, uint8_t *data)
{
    CHECK(instance == &test_chassis_comm);
    memcpy(&test_feedback_sent, data, sizeof(test_feedback_sent));
    test_feedback_send_count++;
}
#endif
void VisionSetChassisVel(float vx, float vy, float wz) {}
Vision_Recv_s *VisionInit(UART_HandleTypeDef *uart)
{ CHECK(uart == &huart1); return NULL; }

static void frame(int16_t horizontal, int16_t vertical, int16_t rotation,
                  uint8_t right, uint8_t left)
{
    uint16_t ch[4] = {horizontal + 1024, vertical + 1024, rotation + 1024, 1024};
    uint8_t *p = test_uart.recv_buff;
    memset(p, 0, 18);
    p[0] = ch[0]; p[1] = ch[0] >> 8 | ch[1] << 3;
    p[2] = ch[1] >> 5 | ch[2] << 6; p[3] = ch[2] >> 2;
    p[4] = ch[2] >> 10 | ch[3] << 1;
    p[5] = ch[3] >> 7 | right << 4 | left << 6;
    p[16] = 0; p[17] = 4;
    test_uart.recv_len = 18;
    test_uart.module_callback();
}

static void motor_frame(CANInstance *can, uint16_t ecd, int16_t rpm)
{
    memset(can->rx_buff, 0, 8);
    can->rx_buff[0] = ecd >> 8; can->rx_buff[1] = ecd;
    can->rx_buff[2] = rpm >> 8; can->rx_buff[3] = rpm;
    can->rx_len = 8;
    can->can_module_callback(can);
}

static void all_feedback(void)
{
    for (unsigned i = 0; i < 4; ++i)
    {
        motor_frame(drive[i]->motor_can_instance, 100, 0);
        motor_frame(steer[i]->motor_can_instance, zero_ecd[i], 0);
    }
}

static void motor_task(void) { DJIMotorControl(); PowerControl(); }
static void step(int16_t x_ch, int16_t y_ch, int16_t wz_ch, uint8_t right, uint8_t left)
{
    test_tick += 5;
    all_feedback();
#if SWERVE_COMMAND_SOURCE == SWERVE_COMMAND_SOURCE_LOCAL_RC
    frame(y_ch, x_ch, wz_ch, right, left);
#else
    (void)x_ch; (void)y_ch; (void)wz_ch; (void)right; (void)left;
#endif
    RobotCMDTask();
    ChassisTask();
    motor_task();
}

static void check_stopped(void)
{
    CHECK(swerve_debug.mode == CHASSIS_ZERO_FORCE);
    for (unsigned i = 0; i < 8; ++i) CHECK(drive_frame[i] == 0 && steer_frame[i] == 0);
    for (unsigned i = 0; i < 4; ++i)
        CHECK(drive[i]->stop_flag == MOTOR_STOP && steer[i]->stop_flag == MOTOR_STOP);
}

static void geometry_tests(void)
{
    float actual[4] = {0};
    SwerveTarget out[4] = {0};
    const float ratio = 60.0f / (0.38f * 0.0526f);
    SwerveSolve(1, 0, 0, actual, out);
    for (unsigned i = 0; i < 4; ++i) { NEAR(out[i].speed_rpm, ratio, 0.01f); NEAR(out[i].angle_deg, 0, 0.001f); }
    SwerveSolve(-1, 0, 0, actual, out);
    for (unsigned i = 0; i < 4; ++i) { NEAR(out[i].speed_rpm, -ratio, 0.01f); NEAR(out[i].angle_deg, 0, 0.001f); }
    /* Reconstruct vectors for translation + spin at varying multi-turn steering angles. */
    const float vx = 0.7f, vy = -0.3f, dps = 120;
    const float a = 0.216f * 0.7071067811865475f;
    const float px[4] = {a, a, -a, -a}, py[4] = {a, -a, -a, a};
    for (unsigned k = 0; k < 20; ++k)
    {
        for (unsigned i = 0; i < 4; ++i) actual[i] = (int)(k * 43 + i * 97) - 720;
        SwerveSolve(vx, vy, dps, actual, out);
        for (unsigned i = 0; i < 4; ++i)
        {
            CHECK(fabsf(out[i].angle_deg - actual[i]) <= 90.001f);
            NEAR(out[i].speed_rpm / ratio * cosf(out[i].angle_deg * DEGREE_2_RAD), vx - dps * 3.14f / 180 * py[i], 0.0001f);
            NEAR(out[i].speed_rpm / ratio * sinf(out[i].angle_deg * DEGREE_2_RAD), vy + dps * 3.14f / 180 * px[i], 0.0001f);
        }
    }
    float held = out[0].angle_deg;
    SwerveSolve(0, 0, 0, actual, out);
    NEAR(out[0].angle_deg, held, 0.001f);
    for (unsigned i = 0; i < 4; ++i) CHECK(out[i].speed_rpm == 0);
    host_call(4, "PASS: swerve vectors, reversal, multi-turn and zero hold\n");
}

void test_main(void)
{
    geometry_tests();
    RobotCMDInit();
    ChassisInit();
#if SWERVE_COMMAND_SOURCE == SWERVE_COMMAND_SOURCE_LOCAL_RC
    CHECK(test_can_count == 8);
    for (unsigned i = 0; i < 4; ++i)
    {
        CHECK(drive[i]->motor_can_instance->can_handle == &hcan1);
        CHECK(drive[i]->motor_can_instance->rx_id == 0x201 + i);
        CHECK(steer[i]->motor_can_instance->can_handle == &hcan2);
        CHECK(steer[i]->motor_can_instance->rx_id == 0x205 + i);
    }
    step(100, 0, 0, RC_SW_MID, RC_SW_UP); check_stopped();
    step(100, 0, 0, RC_SW_DOWN, RC_SW_UP); check_stopped();
    step(100, 0, 100, RC_SW_MID, RC_SW_UP);
    CHECK(swerve_debug.mode == CHASSIS_REMOTE);
    NEAR(swerve_debug.vx_body_mmps, 600, 0.001f);
    NEAR(swerve_debug.wz_radps, -20 * DEGREE_2_RAD, 0.0001f);
    for (unsigned i = 0; i < 4; ++i) CHECK(isfinite(swerve_debug.wheel_target_rpm[i]));
    for (uint8_t left = 1; left <= 3; ++left)
    {
        step(100, 0, 0, RC_SW_MID, left);
        CHECK(swerve_debug.mode == CHASSIS_REMOTE);
        NEAR(swerve_debug.wheel_target_rpm[0], 600 * 0.001f * 60 / (0.38f * 0.0526f), 0.1f);
    }
    step(0, 100, 0, RC_SW_MID, RC_SW_MID);
    NEAR(swerve_debug.vy_body_mmps, 200, 0.001f);
    for (unsigned i = 0; i < 4; ++i) CHECK(fabsf(swerve_debug.wheel_target_rpm[i]) < 0.1f);
    step(5, -5, 5, RC_SW_MID, RC_SW_MID);
    CHECK(swerve_debug.vx_body_mmps == 0 && swerve_debug.vy_body_mmps == 0 && swerve_debug.wz_radps == 0);
    test_imu.Yaw = 170;
    step(100, 0, 0, RC_SW_UP, RC_SW_DOWN);
    CHECK(swerve_debug.mode == CHASSIS_ROTATE);
    NEAR(swerve_debug.spin_rate_ref_radps, 0.3f * DEGREE_2_RAD, 0.0001f);
    test_imu.Yaw = -100; // +90 degrees across the yaw wrap.
    step(100, 0, 0, RC_SW_UP, RC_SW_MID);
    NEAR(swerve_debug.relative_yaw_deg, 90, 0.001f);
    NEAR(swerve_debug.vx_body_mmps, 0, 0.01f);
    NEAR(swerve_debug.vy_body_mmps, 600, 0.01f);
    CHECK(swerve_debug.spin_rate_ref_radps > 0.5f * DEGREE_2_RAD);
    for (unsigned i = 0; i < 450; ++i) step(100, 0, 0, RC_SW_UP, (i % 3) + 1);
    NEAR(swerve_debug.spin_rate_ref_radps, 120 * DEGREE_2_RAD, 0.0001f);
    test_ins_online = 0;
    step(100, 0, 0, RC_SW_UP, RC_SW_MID); check_stopped();
    step(100, 0, 0, RC_SW_MID, RC_SW_MID); CHECK(swerve_debug.mode == CHASSIS_REMOTE);
    test_ins_online = 1;
    test_imu.Yaw = NAN;
    step(100, 0, 0, RC_SW_UP, RC_SW_MID); check_stopped();
    test_imu.Yaw = 0;
    step(100, 0, 0, RC_SW_MID, RC_SW_MID);
    test_tick += 101; all_feedback(); RobotCMDTask(); ChassisTask(); motor_task(); check_stopped();
    step(100, 0, 0, RC_SW_MID, RC_SW_MID); check_stopped(); // Must re-arm down.
    step(0, 0, 0, RC_SW_DOWN, RC_SW_MID);
    step(100, 0, 0, RC_SW_MID, RC_SW_MID);
    CHECK(swerve_debug.mode == CHASSIS_REMOTE);
    test_tick += 21; all_feedback(); motor_task();
    for (unsigned i = 0; i < 8; ++i) CHECK(drive_frame[i] == 0 && steer_frame[i] == 0);
    step(100, 0, 0, RC_SW_MID, RC_SW_MID);
    drive[0]->last_rx_ms = test_tick - 21;
    test_tick += 5; frame(0, 100, 0, RC_SW_MID, RC_SW_MID);
    RobotCMDTask(); ChassisTask(); motor_task(); check_stopped();
    step(100, 0, 0, RC_SW_MID, RC_SW_MID);
    test_uart.recv_len = 17; test_uart.module_callback();
    RobotCMDTask(); ChassisTask(); motor_task(); check_stopped();
    step(0, 0, 0, RC_SW_DOWN, RC_SW_MID);
    step(100, 0, 0, RC_SW_MID, 0); check_stopped();
    step(0, 0, 0, RC_SW_DOWN, RC_SW_MID);
    step(100, 0, 0, RC_SW_MID, RC_SW_MID);
    test_uart.recv_buff[0] = 0; test_uart.recv_buff[1] &= 0xf8;
    test_uart.module_callback(); RobotCMDTask(); ChassisTask(); motor_task(); check_stopped();
    host_call(4, "PASS: original CMD/pubsub, switches, units, spin ramp/wrap, INS/motor/RC fail-safe and CAN zero force\n");
#else
    CHECK(test_can_count == 9);
    CHECK(test_can[0]->can_handle == &hcan2);
    CHECK(test_can[0]->tx_id == SWERVE_CHASSIS_CAN_TX_ID);
    CHECK(test_can[0]->rx_id == SWERVE_CHASSIS_CAN_RX_ID);
    for (unsigned i = 0; i < 4; ++i)
    {
        CHECK(drive[i]->motor_can_instance->can_handle == &hcan1);
        CHECK(drive[i]->motor_can_instance->rx_id == 0x201 + i);
        CHECK(steer[i]->motor_can_instance->can_handle == &hcan2);
        CHECK(steer[i]->motor_can_instance->rx_id == 0x205 + i);
    }

    step(0, 0, 0, 0, 0); check_stopped(); // Must receive one complete command before enabling movement.
    Chassis_Ctrl_Cmd_s command = {0};
    command.vx = 600;
    command.wz = -20 * DEGREE_2_RAD;
    command.chassis_mode = CHASSIS_REMOTE;
    memcpy(test_chassis_comm.unpacked_recv_data, &command, sizeof(command));
    test_chassis_comm.update_flag = 1;
    step(0, 0, 0, 0, 0);
    CHECK(swerve_debug.mode == CHASSIS_REMOTE);
    NEAR(swerve_debug.vx_body_mmps, 600, 0.001f);
    NEAR(swerve_debug.wz_radps, -20 * DEGREE_2_RAD, 0.0001f);

    for (unsigned i = 0; i < 3; ++i) step(0, 0, 0, 0, 0);
    CHECK(test_feedback_send_count == 1);
    NEAR(test_feedback_sent.real_vx, chassis_feedback_data.real_vx, 0.001f);
    for (unsigned i = 0; i < 2; ++i) step(0, 0, 0, 0, 0);
    CHECK(test_feedback_send_count == 1); // Feedback is rate limited to 50Hz.

    command.vx = NAN;
    memcpy(test_chassis_comm.unpacked_recv_data, &command, sizeof(command));
    test_chassis_comm.update_flag = 1;
    step(0, 0, 0, 0, 0); check_stopped(); // Invalid values clear the held target.

    test_chassis_comm_online = 0;
    step(0, 0, 0, 0, 0); check_stopped();
    test_chassis_comm_online = 1;
    step(0, 0, 0, 0, 0); check_stopped(); // Reconnection requires a new valid command.

    command.vx = 0;
    command.wz = 0;
    command.chassis_mode = CHASSIS_ROTATE;
    command.chassis_spin_speed = 120 * DEGREE_2_RAD;
    command.mode = 3;
    memcpy(test_chassis_comm.unpacked_recv_data, &command, sizeof(command));
    test_chassis_comm.update_flag = 1;
    step(0, 0, 0, 0, 0);
    CHECK(swerve_debug.mode == CHASSIS_ROTATE);
    host_call(4, "PASS: gimbal CAN IDs, command validation, online timeout, feedback and swerve control\n");
#endif
    finish(0);
}
