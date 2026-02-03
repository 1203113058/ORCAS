/*
 * orientation_ctrl_utils.h
 *
 *  Created on: Feb 3, 2025
 *      Author: yuchung886
 */

#ifndef INC_ORIENTATION_CTRL_UTILS_H_
#define INC_ORIENTATION_CTRL_UTILS_H_

/*
 * Orientation Control（方位/俯仰等）控制模块对外接口
 * - init(): 模块初始化
 * - cmds_dispatch(): UART 命令分发入口
 * - perodic_routines(): 周期任务入口
 * - isr(): GPIO 外部中断入口
 */

/* 模块初始化；返回 0 表示成功，非 0 表示失败/错误码 */
uint8_t orientation_ctrl_init();

/*
 * Orientation Control 命令分发
 * @cmd_buf: 输入命令缓冲区
 * @ack_buf: 输出 ACK 缓冲区
 * 返回 0 表示成功，非 0 表示失败/错误码
 */
uint8_t orientation_ctrl_cmds_dispatch(uint8_t* cmd_buf, uint8_t* ack_buf);

/* 周期任务最小调度周期（微秒）；外部调度器需保证不小于该周期调用 */
#define ORIENTATION_CTRL_ROUTINES_MIN_PERIOD_US	250

/* Orientation Control 周期任务入口；返回 0 表示成功，非 0 表示失败/错误码 */
uint8_t orientation_ctrl_perodic_routines();

/*
 * GPIO 中断回调入口
 * @GPIO_Pin: 触发中断的引脚编号（HAL 传入）
 */
uint8_t orientation_ctrl_isr(uint16_t GPIO_Pin);

/* 当前 Pan 角度（以步进电机步数表示；单位：step） */
extern uint16_t curr_pan_angle_in_steps;

/* 单步对应的角度（单位：度/step 或 rad/step，由实现约定） */
extern float pan_angle_per_step;

#endif /* INC_ORIENTATION_CTRL_UTILS_H_ */
