 /*
 * tof_sensors_utils.c
 *
 *  Created on: Mar 7, 2025
 *      Author: yuchung886
 */

#include <string.h>
#include "main.h"
#include "error_code.h"
#include "tof_sensors_ctrl_utils.h"
#include "orientation_ctrl_utils.h"
#include "uart_cmds_handle.h"
#include "gy_us42v2.h"
#include "gy_tof10m.h"

extern I2C_HandleTypeDef hi2c1;

/*
 * TOF/Radar 传感器控制实现
 * - 雷达步进电机完成 360° 扫描（按扇区保存测距结果）
 * - 同时管理 TOF10M 测距模块（用于瞄准距离）
 * - tof_sensors_cmds_dispatch(): 提供 UART 命令查询/配置入口
 */

/* 周期任务分组：电机控制高频，测距采样低频 */
#define TOF_SENSORS__ROUTINES_GRP1_PERIOD_US TOF_SENSORS_ROUTINES_MIN_PERIOD_US
#define TOF_SENSORS__ROUTINES_GRP2_PERIOD_US 100000

uint16_t tof_sensors_perodic_routines_timer = 0;

/* tof_sensors_ctrl_status 位定义（bit mask） */
#define STATUS__RADAR_EN					0x01
#define STATUS__RADAR_DISABLE_DISPATCHED	0x02
#define STATUS__RADAR_MOTOR_STEP_HIGH		0x04
#define STATUS__GY_US42_COMM_FAIL			0x08
#define STATUS__GY_TOF10M_INIT_FAIL			0x10
#define STATUS__GY_TOF10M_COMM_FAIL		0x20
// uint8_t tof_sensors_ctrl_status = STATUS__RADAR_EN;
uint8_t tof_sensors_ctrl_status = 0;

uint16_t radar_steps_per_round = 0;
float radar_angle_per_step = 0;
uint16_t curr_radar_steps = 0;

#define RANGING_SECTORS_NUMBER	36
#define RANGING_SECTOR_ANGLE	10
/* 扇区偏移：用于校准雷达扇区的起始角（单位：度） */
int8_t ranging_offset_angle = 0;

uint8_t ranging_of_sectors[RANGING_SECTORS_NUMBER] = {0};
static uint8_t radar_motor_ctrl();
static uint8_t radar_sensing_ctrl();

uint8_t aiming_distance = 0;
static uint8_t get_aiming_distance();

uint8_t tof_sensors_init(){
	/* 雷达步进电机回零并计算 steps_per_round / angle_per_step */
	uint8_t ret = 0;
	uint16_t i = 0, j = 0;
	#define RADAR_INIT_STATE__LEAVE_ORIGIN	0
	#define RADAR_INIT_STATE__ORIGIN_SEARCH_1	1
	#define RADAR_INIT_STATE__ORIGIN_SEARCH_2	2
	#define RADAR_INIT_STATE__COMPLETE		3
	uint8_t radar_init_state = RADAR_INIT_STATE__LEAVE_ORIGIN;

	HAL_Delay(1000);
	HAL_GPIO_WritePin(RADAR_MOTOR_DIR_GPIO_Port, RADAR_MOTOR_DIR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RADAR_MOTOR_SLEEP_GPIO_Port, RADAR_MOTOR_SLEEP_Pin, GPIO_PIN_SET);
	HAL_Delay(5);
	while(!ret && (radar_init_state < RADAR_INIT_STATE__COMPLETE)){
		switch(radar_init_state){
		case RADAR_INIT_STATE__LEAVE_ORIGIN:
			if(HAL_GPIO_ReadPin(RADAR_MOTOR_ORIGIN_GPIO_Port, RADAR_MOTOR_ORIGIN_Pin) == GPIO_PIN_SET){
				i = 0;
				radar_init_state = RADAR_INIT_STATE__ORIGIN_SEARCH_1;
			}else{
				for(i = 0; i < 500; i++){
					for(j = 0; j < 300; j++){};
					HAL_GPIO_WritePin(RADAR_MOTOR_STEP_GPIO_Port, RADAR_MOTOR_STEP_Pin, GPIO_PIN_SET);
					for(j = 0; j < 300; j++){};
					HAL_GPIO_WritePin(RADAR_MOTOR_STEP_GPIO_Port, RADAR_MOTOR_STEP_Pin, GPIO_PIN_RESET);
				}
				if(HAL_GPIO_ReadPin(RADAR_MOTOR_ORIGIN_GPIO_Port, RADAR_MOTOR_ORIGIN_Pin) == GPIO_PIN_RESET){
					ret = ERR__RADAR_MOTOR_STEP_INIT_FAIL;
				}
			}
			break;
		case RADAR_INIT_STATE__ORIGIN_SEARCH_1:
			if(HAL_GPIO_ReadPin(RADAR_MOTOR_ORIGIN_GPIO_Port, RADAR_MOTOR_ORIGIN_Pin) == GPIO_PIN_RESET){
				for(i = 0; i < 500; i++){
					for(j = 0; j < 300; j++){};
					HAL_GPIO_WritePin(RADAR_MOTOR_STEP_GPIO_Port, RADAR_MOTOR_STEP_Pin, GPIO_PIN_SET);
					for(j = 0; j < 300; j++){};
					HAL_GPIO_WritePin(RADAR_MOTOR_STEP_GPIO_Port, RADAR_MOTOR_STEP_Pin, GPIO_PIN_RESET);
				}
				radar_init_state = RADAR_INIT_STATE__ORIGIN_SEARCH_2;
			}else{
				if(i < 10000){
					for(j = 0; j < 300; j++){};
					HAL_GPIO_WritePin(RADAR_MOTOR_STEP_GPIO_Port, RADAR_MOTOR_STEP_Pin, GPIO_PIN_SET);
					for(j = 0; j < 300; j++){};
					HAL_GPIO_WritePin(RADAR_MOTOR_STEP_GPIO_Port, RADAR_MOTOR_STEP_Pin, GPIO_PIN_RESET);
					i++;
				}else{
					ret = ERR__RADAR_MOTOR_STEP_INIT_FAIL;
				}
			}
			break;
		case RADAR_INIT_STATE__ORIGIN_SEARCH_2:
			if(HAL_GPIO_ReadPin(RADAR_MOTOR_ORIGIN_GPIO_Port, RADAR_MOTOR_ORIGIN_Pin) == GPIO_PIN_RESET){
				radar_steps_per_round = i;
				radar_angle_per_step = (float) 360 / radar_steps_per_round;
				curr_radar_steps = 0;
				radar_init_state = RADAR_INIT_STATE__COMPLETE;
			}else{
				if(i < 10000){
					for(j = 0; j < 300; j++){};
					HAL_GPIO_WritePin(RADAR_MOTOR_STEP_GPIO_Port, RADAR_MOTOR_STEP_Pin, GPIO_PIN_SET);
					for(j = 0; j < 300; j++){};
					HAL_GPIO_WritePin(RADAR_MOTOR_STEP_GPIO_Port, RADAR_MOTOR_STEP_Pin, GPIO_PIN_RESET);
					i++;
				}else{
					ret = ERR__RADAR_MOTOR_STEP_INIT_FAIL;
				}
			}
			break;
		default:
			break;
		}
	}
	HAL_GPIO_WritePin(RADAR_MOTOR_SLEEP_GPIO_Port, RADAR_MOTOR_SLEEP_Pin, GPIO_PIN_RESET);
	if(ret){return ret;}

	/* 初始化 TOF10M 测距模块（用于 aiming distance） */
	ret = gy_tof10m__init(&hi2c1);
	if(ret){
		tof_sensors_ctrl_status |= STATUS__GY_TOF10M_INIT_FAIL;
		return ERR__GY_TOF10M_INIT_FAIL;
	}

	return 0;
}

uint8_t tof_sensors_cmds_dispatch(uint8_t* cmd_buf, uint8_t* ack_buf){
	/* UART 命令分发：查询状态、开启/关闭雷达、配置扇区偏移、读取扫描结果/瞄准距离 */
	uint8_t i, j;

	switch(cmd_buf[CMD_CODE_INDEX]){
	case CMD_CODE__GET_TOF_SENSROS_CTRL_STATUS:
		ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
		ack_buf[ACK_PARAMS_INDEX] = tof_sensors_ctrl_status;
		break;
	case CMD_CODE__SET_RADAR_EN:
		/* 雷达使能：1=开启扫描；0=请求关闭（延迟关闭） */
		if(!radar_angle_per_step){
			ack_buf[ACK_CODE_INDEX] = ERR__RADAR_MOTOR_STEP_INIT_FAIL;
		}else{
			if(cmd_buf[CMD_PARAMS_INDEX] & 0x01){
				tof_sensors_ctrl_status |= STATUS__RADAR_EN;
				/* 清空扇区测距结果 */
				for(i = 0; i < RANGING_SECTORS_NUMBER; i++){
					ranging_of_sectors[i] = 0;
				}
				HAL_GPIO_WritePin(RADAR_MOTOR_SLEEP_GPIO_Port, RADAR_MOTOR_SLEEP_Pin, GPIO_PIN_SET);
			}else{
				if(tof_sensors_ctrl_status & STATUS__RADAR_EN){
					/* 延迟关闭：交由周期任务在安全位置关闭 */
					tof_sensors_ctrl_status |= STATUS__RADAR_DISABLE_DISPATCHED;
				}
			}
			ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
		}
		break;
	case CMD_CODE__SET_RADAR_CONFIG:
		/* 配置扇区角度偏移：参数为扇区偏移（-N/2 ~ +N/2） */
		i = (int8_t)cmd_buf[CMD_PARAMS_INDEX];
		if((i <= (RANGING_SECTORS_NUMBER / 2)) && (i > -(RANGING_SECTORS_NUMBER / 2))){
			ranging_offset_angle = RANGING_SECTOR_ANGLE * i;
		}
		ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
		break;
	case CMD_CODE__GET_RADAR_RANGING:
		/* 返回 36 个扇区的测距结果；若某扇区为 0，则用相邻扇区均值补全 */
		if(!radar_angle_per_step){
			ack_buf[ACK_CODE_INDEX] = ERR__RADAR_MOTOR_STEP_INIT_FAIL;
		}else if(tof_sensors_ctrl_status & STATUS__GY_US42_COMM_FAIL){
			tof_sensors_ctrl_status &= ~STATUS__GY_US42_COMM_FAIL;
			ack_buf[ACK_CODE_INDEX] = ERR__GY_US42_COMM_FAIL;
		}else{
			ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
		}

		for(j = 0; j < RANGING_SECTORS_NUMBER; j++){
			if(ranging_of_sectors[j]){
				ack_buf[ACK_PARAMS_INDEX + j] = ranging_of_sectors[j];
			}else{
				ack_buf[ACK_PARAMS_INDEX + j] = (ranging_of_sectors[(j + 1) % RANGING_SECTORS_NUMBER] +
												 ranging_of_sectors[(j + RANGING_SECTORS_NUMBER - 1) % RANGING_SECTORS_NUMBER]) / 2;
			}

		}
		break;
	case CMD_CODE__GET_AIMING_DISTANCE:
		/* 返回瞄准距离：来自 TOF10M；若之前通信失败，这里回报一次错误并清标志 */
		if(tof_sensors_ctrl_status & STATUS__GY_TOF10M_INIT_FAIL){
			ack_buf[ACK_CODE_INDEX] = ERR__GY_TOF10M_INIT_FAIL;
		}else if(tof_sensors_ctrl_status & STATUS__GY_TOF10M_COMM_FAIL){
			tof_sensors_ctrl_status &= ~STATUS__GY_TOF10M_COMM_FAIL;
			ack_buf[ACK_CODE_INDEX] = ERR__GY_TOF10M_COMM_FAIL;
		}else{
			ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
		}
		ack_buf[ACK_PARAMS_INDEX] = aiming_distance;
		break;
	default:
		break;
	}

	return 0;
}

uint8_t tof_sensors_perodic_routines(){
	/* 周期调度：Grp1 控制雷达电机；Grp2 执行测距采样与计算瞄准距离 */
	uint8_t ret;

	tof_sensors_perodic_routines_timer++;
	if(tof_sensors_perodic_routines_timer == (TOF_SENSORS__ROUTINES_GRP2_PERIOD_US / TOF_SENSORS_ROUTINES_MIN_PERIOD_US)){
		tof_sensors_perodic_routines_timer = 0;
	}

	if(tof_sensors_perodic_routines_timer % (TOF_SENSORS__ROUTINES_GRP1_PERIOD_US / TOF_SENSORS_ROUTINES_MIN_PERIOD_US) == 0){
		radar_motor_ctrl();
	}
	if(tof_sensors_perodic_routines_timer % (TOF_SENSORS__ROUTINES_GRP2_PERIOD_US / TOF_SENSORS_ROUTINES_MIN_PERIOD_US) == 0){
		ret = radar_sensing_ctrl();
		if(ret & STATUS__GY_US42_COMM_FAIL){
			/* I2C 异常恢复：强制清除 BUSY，防止后续通信全部失败 */
			MX_I2C_ForceClearBusyFlag(&hi2c1, I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, I2C1_SCL_GPIO_Port, I2C1_SCL_Pin);
		}
		ret = get_aiming_distance();
		if(ret & STATUS__GY_TOF10M_COMM_FAIL){
			MX_I2C_ForceClearBusyFlag(&hi2c1, I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, I2C1_SCL_GPIO_Port, I2C1_SCL_Pin);
		}
	}

	return 0;
}

uint8_t tof_sensors_isr(uint16_t GPIO_Pin){
	if(GPIO_Pin == RADAR_MOTOR_ORIGIN_Pin){
		/* 雷达原点触发：将当前步数清零；若收到关闭请求则在原点处关闭雷达 */
		curr_radar_steps = 0;
		if(tof_sensors_ctrl_status & STATUS__RADAR_DISABLE_DISPATCHED){
			tof_sensors_ctrl_status &= ~STATUS__RADAR_DISABLE_DISPATCHED;
			tof_sensors_ctrl_status &= ~STATUS__RADAR_EN;
			HAL_GPIO_WritePin(RADAR_MOTOR_SLEEP_GPIO_Port, RADAR_MOTOR_SLEEP_Pin, GPIO_PIN_RESET);
		}
	}

	return 0;
}

static uint8_t radar_motor_ctrl(){
	/* 雷达电机步进控制：在使能状态下翻转 STEP 引脚，并按一圈取模维护 curr_radar_steps */
	if(tof_sensors_ctrl_status & STATUS__RADAR_EN){
		if(tof_sensors_ctrl_status & STATUS__RADAR_MOTOR_STEP_HIGH){
			HAL_GPIO_WritePin(RADAR_MOTOR_STEP_GPIO_Port, RADAR_MOTOR_STEP_Pin, GPIO_PIN_RESET);
			tof_sensors_ctrl_status &= ~STATUS__RADAR_MOTOR_STEP_HIGH;
			if(curr_radar_steps){
				curr_radar_steps--;
			}else{
				curr_radar_steps = radar_steps_per_round;
			}
		}else{
			HAL_GPIO_WritePin(RADAR_MOTOR_STEP_GPIO_Port, RADAR_MOTOR_STEP_Pin, GPIO_PIN_SET);
			tof_sensors_ctrl_status |= STATUS__RADAR_MOTOR_STEP_HIGH;
		}
	}
	return 0;
}

static uint8_t radar_sensing_ctrl(){
	/*
	 * 雷达测距采样
	 * - 读取 US42 当前距离
	 * - 根据 radar 步数 + pan 步数合成全局角度，并映射到 36 个扇区
	 * - 触发下一次 US42 测距
	 */
	uint16_t range;
	float angle;

	if(tof_sensors_ctrl_status & STATUS__RADAR_EN){
		if(gy_us42v2__get_range(&hi2c1, &range)){
			tof_sensors_ctrl_status |= STATUS__GY_US42_COMM_FAIL;
			return STATUS__GY_US42_COMM_FAIL;
		}
		angle = (curr_radar_steps * radar_angle_per_step) + (curr_pan_angle_in_steps * pan_angle_per_step);
		angle = angle + ranging_offset_angle;
		if(angle > 360){angle = angle - 360;}
		/* 写入扇区：这里将 range 缩放后保存（与协议/显示一致的单位） */
		ranging_of_sectors[(uint8_t)(angle / RANGING_SECTOR_ANGLE)] = range / 4;

		if(gy_us42v2__start_ranging(&hi2c1)){
			tof_sensors_ctrl_status |= STATUS__GY_US42_COMM_FAIL;
			return STATUS__GY_US42_COMM_FAIL;
		}
	}

	return 0;
}

static uint8_t get_aiming_distance(){
	/*
	 * 瞄准距离计算
	 * - 读取 TOF10M 距离
	 * - 进行单位缩放后写入 aiming_distance（与 UART 协议定义一致）
	 */
	uint16_t range;

	if(gy_tof10m__get_range(&hi2c1, &range)){
		tof_sensors_ctrl_status |= STATUS__GY_TOF10M_COMM_FAIL;
		return ERR__GY_TOF10M_COMM_FAIL;
	}

	aiming_distance = (range / 10) / 4;
	return 0;
}






