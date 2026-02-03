/*
 * srf02.h
 *
 *  Created on: Mar 7, 2025
 *      Author: yuchung886
 */

#ifndef INC_GY_US42V2_H_
#define INC_GY_US42V2_H_

/*
 * GY-US42V2（超声波测距模块）驱动接口
 * - 通过 I2C 触发测距并读取距离
 */

/* 启动一次测距（触发测距转换）；返回 0 表示成功，非 0 表示失败/错误码 */
uint8_t gy_us42v2__start_ranging(I2C_HandleTypeDef *hi2c);

/*
 * 读取测距值
 * @range: 输出距离（单位由模块实现约定，通常为 mm）
 * 返回 0 表示成功，非 0 表示失败/错误码
 */
uint8_t gy_us42v2__get_range(I2C_HandleTypeDef *hi2c, uint16_t* range);

#endif /* INC_GY_US42V2_H_ */
