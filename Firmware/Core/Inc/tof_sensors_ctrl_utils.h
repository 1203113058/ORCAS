/*
 * tof_sensors_utils.h
 *
 *  Created on: Mar 7, 2025
 *      Author: yuchung886
 */

#ifndef INC_TOF_SENSORS_CTRL_UTILS_H_
#define INC_TOF_SENSORS_CTRL_UTILS_H_

/*
 * TOF Sensors 控制模块对外接口
 * - init(): 模块初始化（初始化各测距传感器）
 * - cmds_dispatch(): UART 命令分发入口
 * - perodic_routines(): 周期任务入口
 * - isr(): GPIO 外部中断入口
 */

/* 模块初始化；返回 0 表示成功，非 0 表示失败/错误码 */
uint8_t tof_sensors_init();

/*
 * TOF Sensors 命令分发
 * @cmd_buf: 输入命令缓冲区
 * @ack_buf: 输出 ACK 缓冲区
 * 返回 0 表示成功，非 0 表示失败/错误码
 */
uint8_t tof_sensors_cmds_dispatch(uint8_t* cmd_buf, uint8_t* ack_buf);

/* 周期任务最小调度周期（微秒）；外部调度器需保证不小于该周期调用 */
#define TOF_SENSORS_ROUTINES_MIN_PERIOD_US	500

/* TOF Sensors 周期任务入口；返回 0 表示成功，非 0 表示失败/错误码 */
uint8_t tof_sensors_perodic_routines();

/* GPIO 中断回调入口（如有使用）；@GPIO_Pin: 触发中断的引脚编号 */
uint8_t tof_sensors_isr(uint16_t GPIO_Pin);


#endif /* INC_TOF_SENSORS_CTRL_UTILS_H_ */
