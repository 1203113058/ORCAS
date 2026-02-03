/*
 * error_code.h
 *
 *  Created on: Feb 3, 2025
 *      Author: yuchung886
 */

#ifndef INC_ERROR_CODE_H_
#define INC_ERROR_CODE_H_

/*
 * 错误码定义
 * 约定：
 * - 0x01~0x0F: 通讯/协议类错误
 * - 0x10~0x1F: 运动/电机/姿态相关错误
 * - 0x20~0x2F: 传感器/雷达相关错误
 * - 0x30~0x3F: 执行机构/超时类错误
 */

/* UART 指令解析/协议错误 */
#define ERR__INVALID_UART_CMD_CODE		0x01  /* UART 指令码非法/不支持 */
#define ERR__INVALID_UART_CMD_CHECKSUM	0x02  /* UART 指令校验和错误 */

/* 姿态传感器/电机控制相关错误 */
#define ERR__MMA845X_COMM_FAIL			0x10  /* MMA845x 通讯失败（I2C/SPI 读写异常） */
#define ERR__TILT_MOTOR_STEP_INIT_FAIL	0x11  /* Tilt（俯仰）步进电机初始化失败 */
#define ERR__TILT_ENDSTOP_REACHED		0x12  /* Tilt 方向触发限位开关/到达行程终点 */
#define ERR__PAN_MOTOR_STEP_INIT_FAIL	0x13  /* Pan（水平）步进电机初始化失败 */

/* 雷达/测距类传感器相关错误 */
#define ERR__RADAR_MOTOR_STEP_INIT_FAIL	0x20  /* Radar 机构步进电机初始化失败 */
#define ERR__GY_US42_COMM_FAIL			0x21  /* GY-US42 通讯失败 */
#define ERR__GY_TOF10M_INIT_FAIL		0x22  /* GY-TOF10M 初始化失败 */
#define ERR__GY_TOF10M_COMM_FAIL		0x23  /* GY-TOF10M 通讯失败 */

/* 执行机构/超时类错误 */
#define ERR__AEG_PISTON_PULLING_TIMEOUT	0x30  /* AEG 活塞回拉超时（在限定时间内未完成） */

#endif /* INC_ERROR_CODE_H_ */
