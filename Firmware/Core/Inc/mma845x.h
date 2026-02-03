/*
 * mma845x.h
 *
 *  Created on: Feb 3, 2025
 *      Author: yuchung886
 */

#ifndef INC_MMA845X_H_
#define INC_MMA845X_H_

/*
 * MMA845x 三轴加速度计驱动接口
 * - 通过 I2C 初始化与读取 XYZ 三轴原始数据
 */

/* 初始化传感器；返回 0 表示成功，非 0 表示失败/错误码 */
uint8_t mma845x_init(I2C_HandleTypeDef *hi2c);

/*
 * 读取三轴加速度原始值
 * @x/@y/@z: 输出三轴原始数据（量程/比例由实现配置决定）
 * 返回 0 表示成功，非 0 表示失败/错误码
 */
uint8_t mma845x_get_xyz(I2C_HandleTypeDef *hi2c, int16_t* x, int16_t* y, int16_t* z);

/* 最近一次驱动调用返回值/状态（由驱动内部更新） */
extern uint8_t mma845x_ret;

#endif /* INC_MMA845X_H_ */
