/*
 * mma845x.c
 *
 *  Created on: Feb 3, 2025
 *      Author: yuchung886
 */

#include "main.h"
#include "error_code.h"

/* MMA845x 三轴加速度计驱动（I2C） */

#define I2C_TIMEOUT			10

/* 注意：该地址为 8-bit address（已包含 R/W 位位置），与 HAL I2C API 匹配 */
#define MMA845X_I2C_ADDR	0x38

/* 寄存器地址定义 */
#define REG_ADDR__STATUS			0x00
#define REG_ADDR__OUT_X_MSB			0x01
#define REG_ADDR__OUT_X_LSB			0x02
#define REG_ADDR__OUT_Y_MSB			0x03
#define REG_ADDR__OUT_Y_LSB			0x04
#define REG_ADDR__OUT_Z_MSB			0x05
#define REG_ADDR__OUT_Z_LSB			0x06

#define REG_ADDR__XYZ_DATA_CONFIG	0x0E
#define XYZ_DATA_CONFIG__FS_2G		0x00
#define XYZ_DATA_CONFIG__FS_4G		0x01
#define XYZ_DATA_CONFIG__FS_8G		0x02

#define REG_ADDR__HP_FILTER_CUTOFF	0x0F
#define HP_FILTER_CUTOFF__SEL0		0x00

#define REG_ADDR__CTRL_REG1			0x2A
#define CTRL_REG1__DR_50HZ			0x20
#define CTRL_REG1__DR_6P25HZ		0x30
#define CTRL_REG1__ACTIVE			0x01

#define REG_ADDR__CTRL_REG2			0x2B
#define CTRL_REG2__MODS_NORMAL		0x00

uint8_t mma845x_ctrl_buf[8] = {0};

uint8_t mma845x_init(I2C_HandleTypeDef *hi2c){
	/* 初始化 MMA845x：进入 standby 配置量程/模式，再进入 active */
	uint8_t ret = 0;

	mma845x_ctrl_buf[0] = REG_ADDR__CTRL_REG1;
	mma845x_ctrl_buf[1] = 0; // 进入 Standby 模式以允许写配置
	if(!ret) ret = HAL_I2C_Master_Transmit(hi2c, MMA845X_I2C_ADDR, mma845x_ctrl_buf, 2, I2C_TIMEOUT);

	mma845x_ctrl_buf[0] = REG_ADDR__XYZ_DATA_CONFIG;
	mma845x_ctrl_buf[1] = XYZ_DATA_CONFIG__FS_2G;
	/* 配置量程：±2g */
	if(!ret) ret = HAL_I2C_Master_Transmit(hi2c, MMA845X_I2C_ADDR, mma845x_ctrl_buf, 2, I2C_TIMEOUT);

	mma845x_ctrl_buf[0] = REG_ADDR__CTRL_REG2;
	mma845x_ctrl_buf[1] = CTRL_REG2__MODS_NORMAL;
	/* 配置工作模式：Normal */
	if(!ret) ret = HAL_I2C_Master_Transmit(hi2c, MMA845X_I2C_ADDR, mma845x_ctrl_buf, 2, I2C_TIMEOUT);

	/*
	mma845x_ctrl_buf[0] = REG_ADDR__HP_FILTER_CUTOFF;
	mma845x_ctrl_buf[1] = HP_FILTER_CUTOFF__SEL0;
	if(!ret) ret = HAL_I2C_Master_Transmit(hi2c, MMA845X_I2C_ADDR, mma845x_ctrl_buf, 2, I2C_TIMEOUT);
	*/

	mma845x_ctrl_buf[0] = REG_ADDR__CTRL_REG1;
	mma845x_ctrl_buf[1] = CTRL_REG1__DR_6P25HZ | CTRL_REG1__ACTIVE;
	/* 设置输出速率并进入 Active 模式 */
	if(!ret) ret = HAL_I2C_Master_Transmit(hi2c, MMA845X_I2C_ADDR, mma845x_ctrl_buf, 2, I2C_TIMEOUT);

	if(ret){
		return ERR__MMA845X_COMM_FAIL;
	}else{
		return 0;
	}
}

/* 记录最近一次 I2C 返回码，便于调试定位 */
uint8_t mma845x_ret = 0;
uint8_t mma845x_get_xyz(I2C_HandleTypeDef *hi2c, int16_t* x, int16_t* y, int16_t* z){
	/*
	 * 读取 XYZ 三轴原始值
	 * - 连续读取 STATUS ~ OUT_Z_LSB
	 * - 数据为 14-bit 左对齐，除以 4 转换为带符号 14-bit 值
	 */
	uint8_t ret = 0;

	if(!ret) ret = HAL_I2C_Master_Receive(hi2c, MMA845X_I2C_ADDR, mma845x_ctrl_buf, REG_ADDR__OUT_Z_LSB - REG_ADDR__STATUS + 1, I2C_TIMEOUT);
	if(!ret){
		*x = (int16_t)((mma845x_ctrl_buf[1] << 8) | mma845x_ctrl_buf[2]) / 4;
		*y = (int16_t)((mma845x_ctrl_buf[3] << 8) | mma845x_ctrl_buf[4]) / 4;
		*z = (int16_t)((mma845x_ctrl_buf[5] << 8) | mma845x_ctrl_buf[6]) / 4;
		return 0;
	}else{
		mma845x_ret = ret;
		return ERR__MMA845X_COMM_FAIL;
	}
}
