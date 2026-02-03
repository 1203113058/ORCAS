/*
 * gy_us42v2.c
 *
 *  Created on: Mar 7, 2025
 *      Author: yuchung886
 */

#include "main.h"
#include "error_code.h"

/* GY-US42V2 超声波测距模块驱动（I2C） */

#define I2C_TIMEOUT			10

/* 注意：该地址为 8-bit address（与 HAL I2C API 匹配） */
#define GYUS42V2_SLAVE_ADDR	0xE0

#define REG_ADDR__CMD		0x00
/* 测距命令：inch/cm/ms（本工程使用 cm） */
#define CMD__RANGING_INCH		0x50
#define CMD__RANGING_CM			0x51
#define CMD__RANGING_MS			0x52
#define REG_ADDR__RANGE_H	0x02
#define REG_ADDR__RANGE_L	0x03

uint8_t gy_us42v2_ctrl_buf[8] = {0};

uint8_t gy_us42v2__start_ranging(I2C_HandleTypeDef *hi2c){
	/* 触发一次测距转换（发送测距命令） */
	uint8_t ret = 0;

	gy_us42v2_ctrl_buf[0] = CMD__RANGING_CM;
	if(!ret){ret = HAL_I2C_Master_Transmit(hi2c, GYUS42V2_SLAVE_ADDR, gy_us42v2_ctrl_buf, 1, I2C_TIMEOUT);}
	if(ret){
		return ERR__GY_US42_COMM_FAIL;
	}else{
		return 0;
	}
}

uint8_t gy_us42v2__get_range(I2C_HandleTypeDef *hi2c, uint16_t* range){
	/* 读取测距结果：先写寄存器地址，再读 2 字节距离 */
	uint8_t ret = 0;
	gy_us42v2_ctrl_buf[0] = REG_ADDR__RANGE_H;
	if(!ret){ret = HAL_I2C_Master_Transmit(hi2c, GYUS42V2_SLAVE_ADDR, gy_us42v2_ctrl_buf, 1, I2C_TIMEOUT);}
	if(!ret){ret = HAL_I2C_Master_Receive(hi2c, GYUS42V2_SLAVE_ADDR, gy_us42v2_ctrl_buf, 2, I2C_TIMEOUT);}

	if(ret){
		return ERR__GY_US42_COMM_FAIL;
	}else{
		*range = (gy_us42v2_ctrl_buf[0] << 8) | gy_us42v2_ctrl_buf[1];
		return 0;
	}
}
