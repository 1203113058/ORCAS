/*
 * uart_cmds_handle.h
 *
 *  Created on: Jan 13, 2025
 *      Author: yuchung886
 */

#ifndef INC_UART_CMDS_HANDLE_H_
#define INC_UART_CMDS_HANDLE_H_

/*
 * UART 命令处理/协议定义
 * - 定义帧格式字段索引、命令码、各命令参数长度、ACK 格式等
 * - uart_cmds_handle_perodic_routines() 通常由主循环/调度器按周期调用
 */

/* UART 命令处理周期任务最小调度周期（微秒） */
#define UART_CMDS_HANDLE_ROUTINES_MIN_PERIOD_US		10000

/* 命令帧：Header 字段位置与固定值 */
#define CMD_HEADER_INDEX				0
#define CMD_HEADER_CHAR				0x5A

/* 命令帧：命令码字段位置与命令码定义 */
#define CMD_CODE_INDEX							1
#define CMD_CODE__GET_FW_VER					0x01
#define CMD_CODE__GET_FIRE_CTRL_STATUS			0x02
#define CMD_CODE__SET_FIRE_COUNT				0x03
#define CMD_CODE__SET_FIRE_CTRL_CONFIG			0x04
#define CMD_CODE__SET_FIRE_SAFETY				0x05
#define CMD_CODE__SET_AMMO_FEEDER_CONFIG		0x06
#define CMD_CODE__SET_AMMO_FEEDING				0x07
#define CMD_CODE__GET_PAN_TILT_STEP_ANGLE		0x08
#define CMD_CODE__GET_PAN_TILT_CURR_ANGLE		0x09
#define CMD_CODE__SET_PAN_TILT_ROTATE_ANGLE		0x0A
#define CMD_CODE__GET_TOF_SENSROS_CTRL_STATUS	0x0B
#define CMD_CODE__SET_RADAR_EN					0x0C
#define CMD_CODE__SET_RADAR_CONFIG				0x0D
#define CMD_CODE__GET_RADAR_RANGING				0x0E
#define CMD_CODE__GET_AIMING_DISTANCE			0x0F
#define CMD_CODE__SET_SEARCHLIGHT_PWM			0x10

/* 命令帧：参数起始位置（紧跟在 cmd code 之后） */
#define CMD_PARAMS_INDEX								2

/* 各命令参数长度（字节）；用于校验/解析 */
#define CMD_PARAMS_LENGTH__GET_FIRE_CTRL_STATUS			1
#define CMD_PARAMS_LENGTH__GET_FW_VER					15
#define CMD_PARAMS_LENGTH__SET_FIRE_COUNT				1
#define CMD_PARAMS_LENGTH__SET_FIRE_CTRL_CONFIG			3
#define CMD_PARAMS_LENGTH__SET_FIRE_SAFETY				1
#define CMD_PARAMS_LENGTH__SET_AMMO_FEEDER_CONFIG		1
#define CMD_PARAMS_LENGTH__SET_AMMO_FEEDING				3
#define CMD_PARAMS_LENGTH__GET_PAN_TILT_STEP_ANGLE		4
#define CMD_PARAMS_LENGTH__GET_PAN_TILT_CURR_ANGLE		4
#define CMD_PARAMS_LENGTH__SET_PAN_TILT_ROTATE_ANGLE	4
#define CMD_PARAMS_LENGTH__GET_TOF_SENSROS_CTRL_STATUS	1
#define CMD_PARAMS_LENGTH__SET_RADAR_EN					1
#define CMD_PARAMS_LENGTH__SET_RADAR_CONFIG				1
#define CMD_PARAMS_LENGTH__GET_RADAR_RANGING			36
#define CMD_PARAMS_LENGTH__GET_AIMING_DISTANCE			1
#define CMD_PARAMS_LENGTH__SET_SEARCHLIGHT_PWM			1

/* UART 接收缓冲区（由 UART RX DMA/ISR 等逻辑填充，供命令处理解析使用） */
extern uint8_t uart_rx_buf[];

/* ACK 帧：Header 字段位置与固定值 */
#define ACK_HEADER_INDEX	0
#define ACK_HEADER_CHAR		0x5A

/* ACK 帧：状态码位置与定义 */
#define ACK_CODE_INDEX		1
#define ACK_CODE__SUCCESS	0x00

/* ACK 帧：参数起始位置 */
#define ACK_PARAMS_INDEX	2

/* UART 命令处理周期任务入口；返回 0 表示成功，非 0 表示失败/错误码 */
uint8_t uart_cmds_handle_perodic_routines();

#endif /* INC_UART_CMDS_HANDLE_H_ */
