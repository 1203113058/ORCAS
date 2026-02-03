/*
 * fire_ctrl_utils.h
 *
 *  Created on: Jan 21, 2025
 *      Author: yuchung886
 */

#ifndef INC_FIRE_CTRL_UTILS_H_
#define INC_FIRE_CTRL_UTILS_H_

/*
 * Fire Control（开火/供弹/指示灯等）控制模块对外接口
 * - init(): 模块初始化
 * - cmds_dispatch(): UART 命令分发入口（解析参数并产出 ACK）
 * - perodic_routines(): 周期任务（按最小周期调度）
 * - isr(): GPIO 外部中断入口（限位/按键等事件）
 */

/* 模块初始化；返回 0 表示成功，非 0 表示失败/错误码 */
uint8_t fire_ctrl_init();

/*
 * Fire Control 命令分发
 * @cmd_buf:  输入命令缓冲区
 * @ack_buf:  输出 ACK 缓冲区
 * 返回 0 表示成功，非 0 表示失败/错误码
 */
uint8_t fire_ctrl_cmds_dispatch(uint8_t* cmd_buf, uint8_t* ack_buf);

/* 周期任务最小调度周期（微秒）；外部调度器需保证不小于该周期调用 */
#define FIRE_CTRL_ROUTINES_MIN_PERIOD_US	1000

/* Fire Control 周期任务入口；返回 0 表示成功，非 0 表示失败/错误码 */
uint8_t fire_ctrl_perodic_routines();

/*
 * GPIO 中断回调入口
 * @GPIO_Pin: 触发中断的引脚编号（HAL 传入）
 * 返回 0 表示已处理/成功，非 0 表示未处理/错误
 */
uint8_t fire_ctrl_isr(uint16_t GPIO_Pin);

/* fire_ctrl_status 位定义（bit mask） */
#define STATUS__SAFETY_EN					0x01
#define STATUS__AEG_MOTOR_EN				0x02
#define STATUS__PULLING_PISTON_TIMEOUT		0x04
#define STATUS__RED_DOT_EN					0x08
#define STATUS__FEEDER_MOTOR_DIR			0x10
#define STATUS__FEEDER_MOTOR_STEP			0x20
#define STATUS__TARGETING_LED_PWM_UPDATED	0x40
#define STATUS__TARGETING_LED_EN			0x80

/* Fire Control 当前状态（由模块内部维护，外部只读使用） */
extern uint8_t fire_ctrl_status;

#endif /* INC_FIRE_CTRL_UTILS_H_ */
