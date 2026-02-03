/*
 * gy_tof10m.h
 *
 *  Created on: Mar 12, 2025
 *      Author: yuchung886
 */

#ifndef INC_GY_TOF10M_H_
#define INC_GY_TOF10M_H_

/*
 * GY-TOF10M（TOF 测距模块）驱动接口
 * - 通过 I2C 进行初始化与距离读取
 */

/* 初始化传感器；返回 0 表示成功，非 0 表示失败/错误码 */
uint8_t gy_tof10m__init(I2C_HandleTypeDef *hi2c);

/*
 * 读取测距值
 * @range: 输出距离（单位由模块实现约定，通常为 mm）
 * 返回 0 表示成功，非 0 表示失败/错误码
 */
uint8_t gy_tof10m__get_range(I2C_HandleTypeDef *hi2c, uint16_t* range);

#endif /* INC_GY_TOF10M_H_ */
