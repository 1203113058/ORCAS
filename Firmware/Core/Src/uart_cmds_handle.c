/*
 * uart_cmds_handle.c
 *
 *  Created on: Jan 12, 2025
 *      Author: yuchung886
 */

#include <string.h>
#include "main.h"
#include "error_code.h"
#include "fire_ctrl_utils.h"
#include "orientation_ctrl_utils.h"
#include "tof_sensors_ctrl_utils.h"
#include "uart_cmds_handle.h"

/* UART 接收指针：指向 uart_rx_buf 当前写入位置（按字节中断接收） */
uint8_t uart_rx_index = 0;
#define UART_RX_BUF_SIZE	32
/* UART 接收缓冲区：按协议顺序依次填充 header/code/params/checksum */
uint8_t uart_rx_buf[UART_RX_BUF_SIZE] = {0};

/* 命令接收状态机：按帧格式依次接收并校验 */
#define CMD_RECIVING_STATE__HEADER		0
#define CMD_RECIVING_STATE__CODE		1
#define CMD_RECIVING_STATE__PARAMETERS	2
#define CMD_RECIVING_STATE__CHECKSUM	3
uint8_t cmd_receiving_state = CMD_RECIVING_STATE__HEADER;

/*
 * 命令参数长度查表
 * - 索引即命令码（CMD_CODE__*），值为该命令对应参数字节数
 * - 用于决定何时结束参数接收并进入 checksum 阶段
 */
uint8_t cmd_params_length[] = {0,
							   CMD_PARAMS_LENGTH__GET_FW_VER,
							   CMD_PARAMS_LENGTH__GET_FIRE_CTRL_STATUS,
							   CMD_PARAMS_LENGTH__SET_FIRE_COUNT,
							   CMD_PARAMS_LENGTH__SET_FIRE_CTRL_CONFIG,
							   CMD_PARAMS_LENGTH__SET_FIRE_SAFETY,
							   CMD_PARAMS_LENGTH__SET_AMMO_FEEDER_CONFIG,
							   CMD_PARAMS_LENGTH__SET_AMMO_FEEDING,
							   CMD_PARAMS_LENGTH__GET_PAN_TILT_STEP_ANGLE,
							   CMD_PARAMS_LENGTH__GET_PAN_TILT_CURR_ANGLE,
							   CMD_PARAMS_LENGTH__SET_PAN_TILT_ROTATE_ANGLE,
							   CMD_PARAMS_LENGTH__GET_TOF_SENSROS_CTRL_STATUS,
							   CMD_PARAMS_LENGTH__SET_RADAR_EN,
							   CMD_PARAMS_LENGTH__SET_RADAR_CONFIG,
							   CMD_PARAMS_LENGTH__GET_RADAR_RANGING,
							   CMD_PARAMS_LENGTH__GET_AIMING_DISTANCE,
							   CMD_PARAMS_LENGTH__SET_SEARCHLIGHT_PWM};

#define UART_TX_BUF_SIZE	64
/* UART 发送缓冲区：用于组装 ACK 帧 */
uint8_t uart_tx_buf[UART_TX_BUF_SIZE] = {0};

/* UART 命令接收看门狗：长时间未完成一帧则复位接收状态机 */
#define UART_CMDS_WATCHDOG_TIMEOUT_MS	1000
uint16_t uart_cmds_watchdog_timeout = 0;

extern UART_HandleTypeDef huart1;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	uint8_t i = 0, j = 0;
	uint8_t checksum = 0;

	if(huart == &huart1){
		/*
		 * 逐字节接收状态机：
		 * - HEADER: 校验帧头
		 * - CODE:   校验/分类命令码
		 * - PARAMS: 接收固定长度参数
		 * - CHKSUM: 校验通过后执行命令并发送 ACK
		 */
		switch(cmd_receiving_state){
		case CMD_RECIVING_STATE__HEADER:
			/* 等待同步字（header），不匹配则丢弃并继续等待 */
			if(uart_rx_buf[CMD_HEADER_INDEX] == CMD_HEADER_CHAR){
				cmd_receiving_state = CMD_RECIVING_STATE__CODE;
				uart_rx_index++;
				uart_cmds_watchdog_timeout = 0;
			}else{
				uart_rx_index = 0;
			}
			break;
		case CMD_RECIVING_STATE__CODE:
			/* 根据命令码决定是否需要接收参数，或直接进入 checksum */
			switch(uart_rx_buf[CMD_CODE_INDEX]){
			case CMD_CODE__GET_FW_VER:
#ifdef FIRE_CTRL
			case CMD_CODE__GET_FIRE_CTRL_STATUS:
#endif
#if defined(PAN_CTRL) || defined(TILT_CTRL)
			case CMD_CODE__GET_PAN_TILT_CURR_ANGLE:
			case CMD_CODE__GET_PAN_TILT_STEP_ANGLE:
#endif
#ifdef TOF_SENSORS_CTRL
			case CMD_CODE__GET_TOF_SENSROS_CTRL_STATUS:
			case CMD_CODE__GET_RADAR_RANGING:
			case CMD_CODE__GET_AIMING_DISTANCE:
#endif
				uart_rx_index++;
				cmd_receiving_state = CMD_RECIVING_STATE__CHECKSUM;
				break;
#ifdef FIRE_CTRL
			case CMD_CODE__SET_FIRE_COUNT:
			case CMD_CODE__SET_FIRE_CTRL_CONFIG:
			case CMD_CODE__SET_FIRE_SAFETY:
			case CMD_CODE__SET_AMMO_FEEDER_CONFIG:
			case CMD_CODE__SET_AMMO_FEEDING:
			case CMD_CODE__SET_SEARCHLIGHT_PWM:
#endif
#if defined(PAN_CTRL) || defined(TILT_CTRL)
			case CMD_CODE__SET_PAN_TILT_ROTATE_ANGLE:
#endif
#ifdef TOF_SENSORS_CTRL
			case CMD_CODE__SET_RADAR_EN:
			case CMD_CODE__SET_RADAR_CONFIG:
#endif
				uart_rx_index++;
				cmd_receiving_state = CMD_RECIVING_STATE__PARAMETERS;
				break;
			default:
				/* 命令码非法：立刻回复错误 ACK（使用错误码作为 ACK code） */
				memset(uart_tx_buf, 0, UART_TX_BUF_SIZE);
				checksum = 0;
				i = 0;
				uart_tx_buf[i++] = ACK_HEADER_CHAR;
				uart_tx_buf[i++] = ERR__INVALID_UART_CMD_CODE;
				for(j = 0; j < i; j++){checksum += uart_tx_buf[j];}
				uart_tx_buf[i++] = checksum;
				HAL_UART_Transmit_IT(&huart1, uart_tx_buf, i);
				uart_rx_index = 0;
				cmd_receiving_state = CMD_RECIVING_STATE__HEADER;
				break;
			}
			break;
		case CMD_RECIVING_STATE__PARAMETERS:
			/* 按查表长度接收参数，收满后进入 checksum */
			uart_rx_index++;
			if(uart_rx_index == CMD_PARAMS_INDEX + cmd_params_length[uart_rx_buf[CMD_CODE_INDEX]]){
				cmd_receiving_state = CMD_RECIVING_STATE__CHECKSUM;
			}
			break;
		case CMD_RECIVING_STATE__CHECKSUM:
			/* checksum 校验：对 header/code/params 求和，与末尾 checksum 字节比对 */
			for(i = 0; i < uart_rx_index; i++){
				checksum += uart_rx_buf[i];
			}
			if(checksum == uart_rx_buf[uart_rx_index]){
				/* 校验通过：执行命令，组装 ACK 并发送 */
				switch(uart_rx_buf[CMD_CODE_INDEX]){
				case CMD_CODE__GET_FW_VER:
#ifdef FIRE_CTRL
				case CMD_CODE__GET_FIRE_CTRL_STATUS:
#endif
#if defined(PAN_CTRL) || defined(TILT_CTRL)
				case CMD_CODE__GET_PAN_TILT_STEP_ANGLE:
				case CMD_CODE__GET_PAN_TILT_CURR_ANGLE:
#endif
#ifdef TOF_SENSORS_CTRL
				case CMD_CODE__GET_TOF_SENSROS_CTRL_STATUS:
				case CMD_CODE__GET_RADAR_RANGING:
				case CMD_CODE__GET_AIMING_DISTANCE:
#endif
					memset(uart_tx_buf, 0, UART_TX_BUF_SIZE);
					checksum = 0;
					i = 0;
					uart_tx_buf[i++] = ACK_HEADER_CHAR;
					switch(uart_rx_buf[CMD_CODE_INDEX]){
					case CMD_CODE__GET_FW_VER:
						/* 查询固件版本：ACK 中携带 FW_VER 字符串 */
						uart_tx_buf[i] = ACK_CODE__SUCCESS;
						memcpy(&uart_tx_buf[i + 1], FW_VER, cmd_params_length[CMD_CODE__GET_FW_VER]);
						break;
					case CMD_CODE__GET_FIRE_CTRL_STATUS:
						/* 查询 Fire Control 状态（由 fire_ctrl 模块填充 ACK 参数） */
						fire_ctrl_cmds_dispatch(uart_rx_buf, uart_tx_buf);
						break;
					case CMD_CODE__GET_PAN_TILT_STEP_ANGLE:
					case CMD_CODE__GET_PAN_TILT_CURR_ANGLE:
						/* 查询姿态/角度信息（由 orientation_ctrl 模块填充 ACK 参数） */
						orientation_ctrl_cmds_dispatch(uart_rx_buf, uart_tx_buf);
						break;
					case CMD_CODE__GET_TOF_SENSROS_CTRL_STATUS:
					case CMD_CODE__GET_RADAR_RANGING:
					case CMD_CODE__GET_AIMING_DISTANCE:
						/* 查询测距/雷达信息（由 tof_sensors 模块填充 ACK 参数） */
						tof_sensors_cmds_dispatch(uart_rx_buf, uart_tx_buf);
						break;
					default:
						break;
					}
					i = i + 1 + cmd_params_length[uart_rx_buf[CMD_CODE_INDEX]];
					for(j = 0; j < i; j++){checksum += uart_tx_buf[j];}
					uart_tx_buf[i++] = checksum;
					HAL_UART_Transmit_IT(&huart1, uart_tx_buf, i);
					uart_rx_index = 0;
					cmd_receiving_state = CMD_RECIVING_STATE__HEADER;
					break;
#ifdef FIRE_CTRL
				case CMD_CODE__SET_FIRE_COUNT:
				case CMD_CODE__SET_FIRE_CTRL_CONFIG:
				case CMD_CODE__SET_FIRE_SAFETY:
				case CMD_CODE__SET_AMMO_FEEDER_CONFIG:
				case CMD_CODE__SET_AMMO_FEEDING:
				case CMD_CODE__SET_SEARCHLIGHT_PWM:
#endif
#if defined(PAN_CTRL) || defined(TILT_CTRL)
				case CMD_CODE__SET_PAN_TILT_ROTATE_ANGLE:
#endif
#ifdef TOF_SENSORS_CTRL
				case CMD_CODE__SET_RADAR_EN:
				case CMD_CODE__SET_RADAR_CONFIG:
#endif
					memset(uart_tx_buf, 0, UART_TX_BUF_SIZE);
					checksum = 0;
					i = 0;
					uart_tx_buf[i++] = ACK_HEADER_CHAR;

					switch(uart_rx_buf[CMD_CODE_INDEX]){
					case CMD_CODE__SET_FIRE_COUNT:
					case CMD_CODE__SET_FIRE_CTRL_CONFIG:
					case CMD_CODE__SET_FIRE_SAFETY:
					case CMD_CODE__SET_AMMO_FEEDER_CONFIG:
					case CMD_CODE__SET_AMMO_FEEDING:
					case CMD_CODE__SET_SEARCHLIGHT_PWM:
						/* Fire Control 设置类命令：由 fire_ctrl 模块执行并写入 ACK code */
						fire_ctrl_cmds_dispatch(uart_rx_buf, uart_tx_buf);
						break;
					case CMD_CODE__SET_PAN_TILT_ROTATE_ANGLE:
						/* 姿态控制设置类命令：由 orientation_ctrl 模块执行并写入 ACK code */
						orientation_ctrl_cmds_dispatch(uart_rx_buf, uart_tx_buf);
						break;
					case CMD_CODE__SET_RADAR_EN:
					case CMD_CODE__SET_RADAR_CONFIG:
						/* 测距/雷达设置类命令：由 tof_sensors 模块执行并写入 ACK code */
						tof_sensors_cmds_dispatch(uart_rx_buf, uart_tx_buf);
						break;
					default:
						break;
					}
					i++; /* ACK_CODE_INDEX：由各模块 dispatch 写入 */

					for(j = 0; j < i; j++){checksum += uart_tx_buf[j];}
					uart_tx_buf[i++] = checksum;
					HAL_UART_Transmit_IT(&huart1, uart_tx_buf, i);
					uart_rx_index = 0;
					cmd_receiving_state = CMD_RECIVING_STATE__HEADER;
					break;
				default:
					break;
				}
			}else{
				/* checksum 不匹配：回复校验和错误 */
				memset(uart_tx_buf, 0, UART_TX_BUF_SIZE);
				checksum = 0;
				i = 0;
				uart_tx_buf[i++] = ACK_HEADER_CHAR;
				uart_tx_buf[i++] = ERR__INVALID_UART_CMD_CHECKSUM;
				for(j = 0; j < i; j++){checksum += uart_tx_buf[j];}
				uart_tx_buf[i++] = checksum;
				HAL_UART_Transmit_IT(&huart1, uart_tx_buf, i);
				uart_rx_index = 0;
				cmd_receiving_state = CMD_RECIVING_STATE__HEADER;
				break;
			}
			break;
		default:
			break;
		}
		/* 继续接收下一个字节（按状态机推进 uart_rx_index） */
		HAL_UART_Receive_IT(&huart1, &uart_rx_buf[uart_rx_index], 1);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	if(huart == &huart1){
		/* ORE（Overrun）溢出错误：读取 SR/DR 清除错误标志，避免卡死 */
		if((huart1.ErrorCode & HAL_UART_ERROR_ORE)){
			READ_REG(huart1.Instance->SR);
			READ_REG(huart1.Instance->DR);
		}
	}
}

uint8_t uart_cmds_handle_perodic_routines(){

	/* 看门狗：若长时间未完成一帧接收，强制复位接收状态机 */
	uart_cmds_watchdog_timeout += (UART_CMDS_HANDLE_ROUTINES_MIN_PERIOD_US / 1000);
	if(uart_cmds_watchdog_timeout >= UART_CMDS_WATCHDOG_TIMEOUT_MS){
		uart_rx_index = 0;
		HAL_UART_Receive_IT(&huart1, &uart_rx_buf[uart_rx_index], 1);
		cmd_receiving_state = CMD_RECIVING_STATE__HEADER;
		uart_cmds_watchdog_timeout = 0;
	}

	return 0;
}

