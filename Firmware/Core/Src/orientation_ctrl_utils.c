/*
 * orientation_ctrl_utils.c
 *
 *  Created on: Feb 3, 2025
 *      Author: yuchung886
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "main.h"
#include "orientation_ctrl_utils.h"
#include "fire_ctrl_utils.h"
#include "mma845x.h"
#include "uart_cmds_handle.h"
#include "error_code.h"

extern I2C_HandleTypeDef hi2c2;

/*
 * Orientation Control（方位/俯仰）控制实现
 * - PAN：步进电机回零，计算一圈步数/步距角，维护当前角度（step）
 * - TILT：通过 MMA845x 读取重力方向计算俯仰角，并用电机步进做角度控制
 * - orientation_ctrl_cmds_dispatch(): 提供 UART 查询/设置入口
 * - orientation_ctrl_perodic_routines(): 周期更新 tilt 角度并推进步进输出
 */

/* 周期任务分组：Grp1 低频（传感器读取），Grp2 高频（步进输出） */
#define ORIENTATION_CTRL_ROUTINES_GRP1_PERIOD_US 200000
#define ORIENTATION_CTRL_ROUTINES_GRP2_PERIOD_US ORIENTATION_CTRL_ROUTINES_MIN_PERIOD_US
uint16_t orientation_ctrl_perodic_routines_timer = 0;

/* 模块内部状态位（bit mask） */
#define ORIENTATION_CTRL_STATUS__MMA845X_COMM_FAIL		0x01
uint8_t orientation_ctrl_status = 0;

/* 与姿态相关的角度/限位配置（单位：度） */
#define ACCELEROMETER_ROTATE_ANGLE 	120
#define TILT_UP_ENDSTOP_ANGLE		40
#define TILT_DOWN_ENDSTOP_ANGLE		(-15)
uint16_t curr_pan_angle_in_steps = 0;
float curr_tilt_angle = 0;

typedef struct stepper_motor_handle {

	/* status 位定义：当前 STEP 电平、当前方向、下发方向等 */

	#define STATUS__STEP_HIGH				0x01
	#define STATUS__CURR_DIR_CCW 			0x02
	#define STATUS__CURR_TILT_UP			STATUS__CURR_DIR_CCW
	#define STATUS__CURR_PAN_LEFT			STATUS__CURR_DIR_CCW
	#define STATUS__DISPATCHED_DIR_CCW		0x04
	#define STATUS__DISPATCHED_TILT_UP		STATUS__DISPATCHED_DIR_CCW
	#define STATUS__DISPATCHED_PAN_LEFT		STATUS__DISPATCHED_DIR_CCW
	uint8_t status;

	/* 步进输出状态机：停止/加速/减速/匀速/换向 */
	#define STATE__STOP			0
	#define STATE__ACC			1
	#define STATE__DEC			2
	#define STATE__CONST		3
	#define STATE__TURN_AROUND	4
	uint8_t state;

	/* 加减速参数：acc_dec_steps 为加减速步数；lookup 为半周期表（单位：最小周期 tick） */
	uint8_t acc_dec_steps;
	uint8_t* acc_dec_half_period_lookup;

	/* 步进时序计数：half_period_index 指向当前半周期表项；half_period_timer 用于计数 */
	uint8_t half_period_index;
	uint8_t half_period_timer;
	/*
	 * dispatched_steps: 本次命令下发步数
	 * remain_steps:     剩余未完成步数
	 */
	uint16_t dispatched_steps;
	uint16_t remain_steps;
	/* 对应 DIR/STEP 引脚 */
	GPIO_TypeDef* dir_gpio_port;
	uint16_t dir_gpio_pin;
	GPIO_TypeDef* step_gpio_port;
	uint16_t step_gpio_pin;
} stepper_motor_handle_t;

#if defined(PAN_CTRL) && defined(TILT_CTRL)
uint16_t pan_steps_per_round = 0;
float pan_angle_per_step = 0;
float tilt_angle_per_step = 0;
#define TILT_ACC_DEC_STEPS 64
#define TILT_ACC_DEC_HALF_PERIOD_MIN	5
#define PAN_ACC_DEC_HALF_PERIOD_MIN		2
uint8_t pan_acc_dec_half_period_lookup[256];
uint8_t tilt_acc_dec_half_period_lookup[256];
#else // defined(PAN_CTRL) && defined(TILT_CTRL)
uint16_t pan_steps_per_round = 8009;
float pan_angle_per_step = 0.045;
float tilt_angle_per_step = 0.104;

#define PAN_ACC_DEC_STEPS	148
uint8_t pan_acc_dec_half_period_lookup[PAN_ACC_DEC_STEPS] = {
	2,
	3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
	4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
	5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
	6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
	7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
	8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
	9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10
};
#define TILT_ACC_DEC_STEPS 64
uint8_t tilt_acc_dec_half_period_lookup[TILT_ACC_DEC_STEPS] = {
	5,
	6, 6, 6, 6, 6, 6,
	7, 7, 7, 7, 7,
	8, 8, 8,
	9, 9, 9, 9,
	10, 10,
	11, 11, 11,
	12, 12, 12,
	13, 13,
	14, 14,
	15, 15, 15,
	16, 16,
	17, 17,
	18, 18, 18,
	19, 19, 19,
	20, 20,
	21, 21, 21, 21,
	22, 22, 22,
	23, 23, 23, 23, 23,
	24, 24, 24, 24, 24, 24,
};
#endif // defined(PAN_CTRL) && defined(TILT_CTRL)

stepper_motor_handle_t pan_motor_handle = {
	0,								// status
	STATE__STOP,					// state
	0,								// acc_dec_steps
	pan_acc_dec_half_period_lookup,	// acc_dec_half_period_lookup
	0, 								// half_period_index
	0, 								// half_period_timer
	0,								// dispatched_steps
	0, 								// remain_stapes
	PAN_MOTOR_DIR_GPIO_Port,		// dir_gpio_port
	PAN_MOTOR_DIR_Pin,				// dir_gpio_pin
	PAN_MOTOR_STEP_GPIO_Port,		// step_gpio_port
	PAN_MOTOR_STEP_Pin,				// step_gpio_pin
};

stepper_motor_handle_t tilt_motor_handle = {
	0,								// status
	STATE__STOP,					// state
	0, 								// acc_dec_steps
	tilt_acc_dec_half_period_lookup,// acc_dec_half_period_lookup
	0, 								// half_period_index
	0, 								// half_period_timer
	0,								// dispatched_steps
	0, 								// remain_stapes
	TILT_MOTOR_DIR_GPIO_Port,		// dir_gpio_port
	TILT_MOTOR_DIR_Pin,				// dir_gpio_pin
	TILT_MOTOR_STEP_GPIO_Port,		// step_gpio_port
	TILT_MOTOR_STEP_Pin,			// step_gpio_pin
};


static uint8_t motor_handles_init();
static uint8_t get_tilt_angle(float* angle);
static uint8_t pan_tilt_ctrl(stepper_motor_handle_t* handle);

uint8_t orientation_ctrl_init(){
	/*
	 * 初始化流程：
	 * - PAN：回零并计算 pan_steps_per_round / pan_angle_per_step
	 * - TILT：初始化加速度计并估算 tilt_angle_per_step（标定 100 步对应角度变化）
	 */
	uint8_t ret = 0;
	uint16_t i = 0, j = 0;
	float angle_1, angle_2;

#ifdef PAN_CTRL
	#define PAN_INIT_STATE__LEAVE_ORIGIN	0
	#define PAN_INIT_STATE__ORIGIN_SEARCH_1	1
	#define PAN_INIT_STATE__ORIGIN_SEARCH_2	2
	#define PAN_INIT_STATE__COMPLETE		3
	uint8_t pan_init_state = PAN_INIT_STATE__LEAVE_ORIGIN;

	HAL_GPIO_WritePin(PAN_MOTOR_DIR_GPIO_Port, PAN_MOTOR_DIR_Pin, GPIO_PIN_RESET);
	while(!ret && (pan_init_state < PAN_INIT_STATE__COMPLETE)){
		/* PAN 回零：离开原点 -> 搜索原点边沿 -> 计算一圈步数 */
		switch(pan_init_state){
		case PAN_INIT_STATE__LEAVE_ORIGIN:
			if(HAL_GPIO_ReadPin(PAN_MOTOR_ORIGIN_GPIO_Port, PAN_MOTOR_ORIGIN_Pin) == GPIO_PIN_SET){
				i = 0;
				pan_init_state = PAN_INIT_STATE__ORIGIN_SEARCH_1;
			}else{
				for(i = 0; i < 100; i++){
					for(j = 0; j < 400; j++){};
					HAL_GPIO_WritePin(PAN_MOTOR_STEP_GPIO_Port, PAN_MOTOR_STEP_Pin, GPIO_PIN_SET);
					for(j = 0; j < 400; j++){};
					HAL_GPIO_WritePin(PAN_MOTOR_STEP_GPIO_Port, PAN_MOTOR_STEP_Pin, GPIO_PIN_RESET);
				}
				if(HAL_GPIO_ReadPin(PAN_MOTOR_ORIGIN_GPIO_Port, PAN_MOTOR_ORIGIN_Pin) == GPIO_PIN_RESET){
					ret = ERR__PAN_MOTOR_STEP_INIT_FAIL;
				}
			}
			break;
		case PAN_INIT_STATE__ORIGIN_SEARCH_1:
			if(HAL_GPIO_ReadPin(PAN_MOTOR_ORIGIN_GPIO_Port, PAN_MOTOR_ORIGIN_Pin) == GPIO_PIN_RESET){
				for(i = 0; i < 100; i++){
					for(j = 0; j < 400; j++){};
					HAL_GPIO_WritePin(PAN_MOTOR_STEP_GPIO_Port, PAN_MOTOR_STEP_Pin, GPIO_PIN_SET);
					for(j = 0; j < 400; j++){};
					HAL_GPIO_WritePin(PAN_MOTOR_STEP_GPIO_Port, PAN_MOTOR_STEP_Pin, GPIO_PIN_RESET);
				}
				pan_init_state = PAN_INIT_STATE__ORIGIN_SEARCH_2;
			}else{
				if(i < 10000){
					for(j = 0; j < 400; j++){};
					HAL_GPIO_WritePin(PAN_MOTOR_STEP_GPIO_Port, PAN_MOTOR_STEP_Pin, GPIO_PIN_SET);
					for(j = 0; j < 400; j++){};
					HAL_GPIO_WritePin(PAN_MOTOR_STEP_GPIO_Port, PAN_MOTOR_STEP_Pin, GPIO_PIN_RESET);
					i++;
				}else{
					ret = ERR__PAN_MOTOR_STEP_INIT_FAIL;
				}
			}
			break;
		case PAN_INIT_STATE__ORIGIN_SEARCH_2:
			if(HAL_GPIO_ReadPin(PAN_MOTOR_ORIGIN_GPIO_Port, PAN_MOTOR_ORIGIN_Pin) == GPIO_PIN_RESET){
				/* 在第二次触发边沿处认为完成一圈，得到 steps_per_round 并换算步距角 */
				pan_steps_per_round = i;
				pan_angle_per_step = (float)360 / pan_steps_per_round;
				curr_pan_angle_in_steps = 0;
				pan_init_state = PAN_INIT_STATE__COMPLETE;
			}else{
				if(i < 10000){
					for(j = 0; j < 400; j++){};
					HAL_GPIO_WritePin(PAN_MOTOR_STEP_GPIO_Port, PAN_MOTOR_STEP_Pin, GPIO_PIN_SET);
					for(j = 0; j < 400; j++){};
					HAL_GPIO_WritePin(PAN_MOTOR_STEP_GPIO_Port, PAN_MOTOR_STEP_Pin, GPIO_PIN_RESET);
					i++;
				}else{
					ret = ERR__PAN_MOTOR_STEP_INIT_FAIL;
				}
			}
			break;
		default:
			break;
		}
	}
	if(ret){
		return ret;
	}
#endif // PAN_CTRL


#ifdef TILT_CTRL
	/* TILT 标定：通过加速度计读角度，手动走 100 步，估算 tilt_angle_per_step */
	HAL_Delay(1000);
	ret = mma845x_init(&hi2c2);
	if(ret){
		orientation_ctrl_status |= ORIENTATION_CTRL_STATUS__MMA845X_COMM_FAIL;
		return ret;
	}

	HAL_Delay(500);
	get_tilt_angle(&angle_1);
	if(ret){
		orientation_ctrl_status |= ORIENTATION_CTRL_STATUS__MMA845X_COMM_FAIL;
		return ret;
	}
	HAL_GPIO_WritePin(TILT_MOTOR_DIR_GPIO_Port, TILT_MOTOR_DIR_Pin, GPIO_PIN_SET);		// Tilt up
	for(i = 0; i < 100; i++){
		HAL_Delay(2);
		HAL_GPIO_WritePin(TILT_MOTOR_STEP_GPIO_Port, TILT_MOTOR_STEP_Pin, GPIO_PIN_SET);
		HAL_Delay(2);
		HAL_GPIO_WritePin(TILT_MOTOR_STEP_GPIO_Port, TILT_MOTOR_STEP_Pin, GPIO_PIN_RESET);
	}

	HAL_Delay(500);
	get_tilt_angle(&angle_2);
	if(ret){
		orientation_ctrl_status |= ORIENTATION_CTRL_STATUS__MMA845X_COMM_FAIL;
		return ret;
	}
	tilt_angle_per_step = fabs(angle_2 - angle_1) / 100;

	if(angle_2 > 0){
		tilt_motor_handle.status &= ~STATUS__DISPATCHED_TILT_UP;
	}else{
		tilt_motor_handle.status |= STATUS__DISPATCHED_TILT_UP;
	}
	tilt_motor_handle.dispatched_steps = (uint16_t)(fabs(angle_2) / tilt_angle_per_step);
#endif // TILT_CTRL

	motor_handles_init();

	return 0;
}

uint8_t orientation_ctrl_cmds_dispatch(uint8_t* cmd_buf, uint8_t* ack_buf){
	/* UART 命令分发：查询步距角/当前角度，或下发旋转角度（单位：0.1°） */
	float angle;

	switch(cmd_buf[CMD_CODE_INDEX]){
	case CMD_CODE__GET_PAN_TILT_STEP_ANGLE:
		/* 将步距角放大 1000 倍后以 int16_t 形式返回（高字节在前） */
		ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
		ack_buf[ACK_PARAMS_INDEX] = (uint8_t)(((int16_t)(pan_angle_per_step * 1000)) >> 8);
		ack_buf[ACK_PARAMS_INDEX + 1] = (uint8_t)((int16_t)(pan_angle_per_step * 1000)) & 0x00FF;
		ack_buf[ACK_PARAMS_INDEX + 2] = (uint8_t)(((int16_t)(tilt_angle_per_step * 1000)) >> 8);
		ack_buf[ACK_PARAMS_INDEX + 3] = (uint8_t)((int16_t)(tilt_angle_per_step * 1000)) & 0x00FF;
		break;
	case CMD_CODE__GET_PAN_TILT_CURR_ANGLE:
		/* 返回当前 Pan/Tilt 角度（放大 10 倍，以 0.1° 单位） */
		ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
#ifdef PAN_CTRL
		if(!pan_angle_per_step){
			ack_buf[ACK_CODE_INDEX] = ERR__PAN_MOTOR_STEP_INIT_FAIL;
		}
#endif
#ifdef TILT_CTRL
		if(orientation_ctrl_status & ORIENTATION_CTRL_STATUS__MMA845X_COMM_FAIL){
			ack_buf[ACK_CODE_INDEX] = ERR__MMA845X_COMM_FAIL;
			orientation_ctrl_status &= ~ORIENTATION_CTRL_STATUS__MMA845X_COMM_FAIL;
		}
#endif
		angle = (curr_pan_angle_in_steps * pan_angle_per_step);
		if(angle > 180){angle = angle - 360;}
		ack_buf[ACK_PARAMS_INDEX] = (uint8_t)(((int16_t)(angle * 10)) >> 8);;
		ack_buf[ACK_PARAMS_INDEX + 1] = (uint8_t)((int16_t)(angle * 10)) & 0x00FF;;
		ack_buf[ACK_PARAMS_INDEX + 2] = (uint8_t)(((int16_t)(curr_tilt_angle * 10)) >> 8);
		ack_buf[ACK_PARAMS_INDEX + 3] = (uint8_t)((int16_t)(curr_tilt_angle * 10)) & 0x00FF;

		break;
	case CMD_CODE__SET_PAN_TILT_ROTATE_ANGLE:
		/* 下发旋转角度：Pan 参数在前 2 字节，Tilt 参数在后 2 字节（均为 0.1°） */
		ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
#ifdef PAN_CTRL
		if(!pan_angle_per_step){
			ack_buf[ACK_CODE_INDEX] = ERR__PAN_MOTOR_STEP_INIT_FAIL;
		}else{
			angle = (float)((int16_t)((cmd_buf[CMD_PARAMS_INDEX] << 8) | cmd_buf[CMD_PARAMS_INDEX + 1])) / 10;

			if(angle > 0){
				pan_motor_handle.status &= ~STATUS__DISPATCHED_PAN_LEFT;
			}else{
				pan_motor_handle.status |= STATUS__DISPATCHED_PAN_LEFT;
			}
			pan_motor_handle.dispatched_steps = (uint16_t)fabs(angle / pan_angle_per_step);
		}
#endif
#ifdef TILT_CTRL
		if(!tilt_angle_per_step){
			ack_buf[ACK_CODE_INDEX] = ERR__TILT_MOTOR_STEP_INIT_FAIL;
		}else if(orientation_ctrl_status & ORIENTATION_CTRL_STATUS__MMA845X_COMM_FAIL){
			ack_buf[ACK_CODE_INDEX] = ERR__MMA845X_COMM_FAIL;
			orientation_ctrl_status &= ~ORIENTATION_CTRL_STATUS__MMA845X_COMM_FAIL;
		}else{
			angle = (float)((int16_t)((cmd_buf[CMD_PARAMS_INDEX + 2] << 8) | cmd_buf[CMD_PARAMS_INDEX + 3])) / 10;
			if(angle > 0){
				/* 向上抬：检查上限位，必要时裁剪到限位角 */
				if(curr_tilt_angle >= TILT_UP_ENDSTOP_ANGLE){
					ack_buf[ACK_CODE_INDEX] = ERR__TILT_ENDSTOP_REACHED;
					break;
				}else{
					tilt_motor_handle.status |= STATUS__DISPATCHED_TILT_UP;
					if(curr_tilt_angle + angle > TILT_UP_ENDSTOP_ANGLE){
						angle = TILT_UP_ENDSTOP_ANGLE - curr_tilt_angle;
					}
				}
			}else{
				/* 向下俯：检查下限位，必要时裁剪到限位角 */
				if(curr_tilt_angle <= TILT_DOWN_ENDSTOP_ANGLE){
					ack_buf[ACK_CODE_INDEX] = ERR__TILT_ENDSTOP_REACHED;
					break;
				}else{
					tilt_motor_handle.status &= ~STATUS__DISPATCHED_TILT_UP;
					if(curr_tilt_angle + angle < TILT_DOWN_ENDSTOP_ANGLE){
						angle = TILT_DOWN_ENDSTOP_ANGLE - curr_tilt_angle;
					}
				}
			}
			tilt_motor_handle.dispatched_steps = (uint16_t)fabs(angle / tilt_angle_per_step);
		}
#endif
		break;
	default:
		break;
	}

	return 0;
}

uint8_t orientation_ctrl_perodic_routines(){
	/* 周期调度：Grp1 更新倾角（低频 I2C 读取），Grp2 推进步进脉冲输出 */
	uint8_t ret = 0;

	orientation_ctrl_perodic_routines_timer++;
	if(orientation_ctrl_perodic_routines_timer == (ORIENTATION_CTRL_ROUTINES_GRP1_PERIOD_US / ORIENTATION_CTRL_ROUTINES_MIN_PERIOD_US)){
		orientation_ctrl_perodic_routines_timer = 0;
	}

#ifdef TILT_CTRL
	if(orientation_ctrl_perodic_routines_timer % (ORIENTATION_CTRL_ROUTINES_GRP1_PERIOD_US / ORIENTATION_CTRL_ROUTINES_MIN_PERIOD_US) == 0){
		ret = get_tilt_angle(&curr_tilt_angle);
		if(ret){
			orientation_ctrl_status |= ORIENTATION_CTRL_STATUS__MMA845X_COMM_FAIL;
			MX_I2C_ForceClearBusyFlag(&hi2c2, I2C2_SDA_GPIO_Port, I2C2_SDA_Pin, I2C2_SCL_GPIO_Port, I2C2_SCL_Pin);
		}
	}
#endif

	if(orientation_ctrl_perodic_routines_timer % (ORIENTATION_CTRL_ROUTINES_GRP2_PERIOD_US / ORIENTATION_CTRL_ROUTINES_MIN_PERIOD_US) == 0){
		/* 高频：对每个启用的电机句柄调用一次控制器，产生 STEP 脉冲 */
#ifdef PAN_CTRL
		pan_tilt_ctrl(&pan_motor_handle);
#endif
#ifdef TILT_CTRL
		pan_tilt_ctrl(&tilt_motor_handle);
#endif
	}

	return 0;
}

uint8_t orientation_ctrl_isr(uint16_t GPIO_Pin){
#ifdef PAN_CTRL
	if(GPIO_Pin == PAN_MOTOR_ORIGIN_Pin){
		/* PAN 原点中断：仅在向右（非左）经过原点时将角度清零 */
		if(!(pan_motor_handle.status & STATUS__CURR_PAN_LEFT)){
			curr_pan_angle_in_steps = 0;
		}
	}
#endif
	return 0;
}

static uint8_t motor_handles_init(){
#if defined(PAN_CTRL) && defined(TILT_CTRL)
	/*
	 * PAN+TILT 同时使能时：根据两轴步距角比例，生成平滑加减速半周期表
	 * 目标：两轴运动速度感知一致（角速度相近）
	 */
	uint8_t tilt_acc_dec_steps = TILT_ACC_DEC_STEPS;
	uint8_t tilt_acc_dec_half_period_min = TILT_ACC_DEC_HALF_PERIOD_MIN;
	uint8_t tilt_acc_dec_half_period_max;
	uint8_t pan_acc_dec_steps;
	uint8_t pan_acc_dec_half_period_min;
	uint8_t pan_acc_dec_half_period_max;

	#define STATE__CONFIG_START_END_HALF_PERIOD	0
	#define STATE__CONFIG_ACC_DEC_STEPS_COUNT	1
	#define STATE__GEN_SMOOTH_STEP				2
	#define STATE__ERR							3
	uint8_t state = STATE__CONFIG_START_END_HALF_PERIOD;

	uint8_t i = 0;
	float s, t = 0;

	while(state < STATE__ERR){
		switch(state){
		case STATE__CONFIG_START_END_HALF_PERIOD:
			/* 根据步距角比例推导 PAN 轴最小半周期，若过小则放慢 TILT 最小半周期 */
			pan_acc_dec_half_period_min = round(tilt_acc_dec_half_period_min / (tilt_angle_per_step / pan_angle_per_step));
			if(pan_acc_dec_half_period_min < PAN_ACC_DEC_HALF_PERIOD_MIN){
				tilt_acc_dec_half_period_min++;
			}else{
				tilt_acc_dec_half_period_max = tilt_acc_dec_half_period_min * 5;
				pan_acc_dec_half_period_max = pan_acc_dec_half_period_min * 5;
				state = STATE__CONFIG_ACC_DEC_STEPS_COUNT;
			}
			break;
		case STATE__CONFIG_ACC_DEC_STEPS_COUNT:
			/* 限制 lookup 表长度不超过 256 */
			if(round(tilt_acc_dec_steps * (tilt_angle_per_step / pan_angle_per_step)) > 256){
				tilt_acc_dec_steps--;
			}else{
				pan_acc_dec_steps = round(tilt_acc_dec_steps * (tilt_angle_per_step / pan_angle_per_step));
				state = STATE__GEN_SMOOTH_STEP;
			}
			break;
		case STATE__GEN_SMOOTH_STEP:
			/* 采用 smoothstep 曲线生成加减速半周期表，使速度变化更平滑 */
			pan_motor_handle.acc_dec_steps = pan_acc_dec_steps;
			pan_motor_handle.half_period_index = pan_acc_dec_steps - 1;
			for(i = 0; i < pan_acc_dec_steps; i++){
				s = (float)(pan_acc_dec_half_period_max - (pan_acc_dec_half_period_min + 1));
				t = i * (s / pan_acc_dec_steps) / s;
			    if(t > 1.0){t = 1.0;}
			    if(t < 0.0){t = 0.0;}
			    t = t * t * (3.0 - 2.0 * t);
			    pan_acc_dec_half_period_lookup[i] = round((pan_acc_dec_half_period_min + 1) + s * t);
			}
			pan_acc_dec_half_period_lookup[0] = pan_acc_dec_half_period_min;

			tilt_motor_handle.acc_dec_steps = tilt_acc_dec_steps;
			tilt_motor_handle.half_period_index = tilt_acc_dec_steps - 1;
			for(i = 0; i < tilt_acc_dec_steps; i++){
				s = (float)(tilt_acc_dec_half_period_max - (tilt_acc_dec_half_period_min + 1));
				t = i * (s / tilt_acc_dec_steps) / s;
			    if(t > 1.0){t = 1.0;}
			    if(t < 0.0){t = 0.0;}
			    t = t * t * (3.0 - 2.0 * t);
			    tilt_acc_dec_half_period_lookup[i] = round((tilt_acc_dec_half_period_min + 1) + s * t);
			}
			tilt_acc_dec_half_period_lookup[0] = tilt_acc_dec_half_period_min;

			return 0;
			break;
		default:
			break;
		}
	}

	return 1;
#else // defined(PAN_CTRL) && defined(TILT_CTRL)
	pan_motor_handle.acc_dec_steps = PAN_ACC_DEC_STEPS;
	pan_motor_handle.half_period_index = PAN_ACC_DEC_STEPS - 1;
	tilt_motor_handle.acc_dec_steps = TILT_ACC_DEC_STEPS;
	tilt_motor_handle.half_period_index = TILT_ACC_DEC_STEPS - 1;
	return 0;
#endif // defined(PAN_CTRL) && defined(TILT_CTRL)
}

static uint8_t get_tilt_angle(float* angle){
	/*
	 * 读取加速度计并换算俯仰角
	 * - 使用 atan2(x, y) 得到重力投影角度（单位：度）
	 * - 通过 ACCELEROMETER_ROTATE_ANGLE 修正安装角偏置
	 */
	uint8_t ret = 0;
	int16_t x, y, z;
	float a;

	ret = mma845x_get_xyz(&hi2c2, &x, &y, &z);
	if(!ret){
		a = atan2(x, y) * 180 / M_PI;
		if(a < 0){a += 360;}
		*angle = a - ACCELEROMETER_ROTATE_ANGLE;
	}

	return ret;
}

static uint8_t pan_tilt_ctrl(stepper_motor_handle_t* handle){
	/*
	 * 步进电机控制器
	 * - 由最小周期调用，通过 half_period_lookup 控制 STEP 脉冲频率
	 * - 支持加速/匀速/减速，并在中途接收到反向命令时执行换向流程
	 */
	switch(handle->state){
	case STATE__STOP:
		/* STOP：等待新命令下发（dispatched_steps > 0） */
		if(handle->dispatched_steps){
			handle->remain_steps = handle->dispatched_steps;
			handle->dispatched_steps = 0;
			handle->half_period_index = handle->acc_dec_steps - 1;
			/* 设置方向并更新当前方向标志 */
			if(handle->status & STATUS__DISPATCHED_DIR_CCW){
				HAL_GPIO_WritePin(handle->dir_gpio_port, handle->dir_gpio_pin, GPIO_PIN_SET);
				handle->status |= STATUS__CURR_DIR_CCW;
			}else{
				HAL_GPIO_WritePin(handle->dir_gpio_port, handle->dir_gpio_pin, GPIO_PIN_RESET);
				handle->status &= ~STATUS__CURR_DIR_CCW;
			}
			handle->state = STATE__ACC;
		}
		break;

	case STATE__ACC:
	case STATE__CONST:
	case STATE__DEC:
	case STATE__TURN_AROUND:
		/*
		 * 运行态：按半周期表计数，到期则翻转 STEP 电平
		 * - STEP 下降沿时认为完成一步：更新角度/剩余步数
		 */
		if(++handle->half_period_timer == handle->acc_dec_half_period_lookup[handle->half_period_index]){
			handle->half_period_timer = 0;
			if(handle->status & STATUS__STEP_HIGH){
				HAL_GPIO_WritePin(handle->step_gpio_port, handle->step_gpio_pin, GPIO_PIN_RESET);
				handle->status &= ~STATUS__STEP_HIGH;

				if(handle == &pan_motor_handle){
					/* PAN：每走一步更新 curr_pan_angle_in_steps，并按一圈取模 */
					if((handle->status) & STATUS__CURR_DIR_CCW){
						if(curr_pan_angle_in_steps){
							curr_pan_angle_in_steps--;
						}else{
							curr_pan_angle_in_steps = pan_steps_per_round;
						}
					}else{
						if(curr_pan_angle_in_steps < pan_steps_per_round){
							curr_pan_angle_in_steps++;
						}else{
							curr_pan_angle_in_steps = 0;
						}
					}
				}

				if(handle->remain_steps > 0){
					handle->remain_steps--;
				}

				if(handle->dispatched_steps){
					/* 新命令在运动过程中到达：同向则替换 remain_steps，反向则进入换向 */
					if(((handle->status & STATUS__DISPATCHED_DIR_CCW) && (handle->status & STATUS__CURR_DIR_CCW)) ||
						(!(handle->status & STATUS__DISPATCHED_DIR_CCW) && !(handle->status & STATUS__CURR_DIR_CCW))){
						handle->remain_steps = handle->dispatched_steps;
						handle->dispatched_steps = 0;
					}else{
						handle->state = STATE__TURN_AROUND;
					}
				}

				switch(handle->state){
				case STATE__ACC:
					/* 加速：逐步减小半周期（提高频率）；接近目标时切换到减速 */
					if(handle->half_period_index > 0){
						handle->half_period_index--;
						if(handle->remain_steps > (handle->acc_dec_steps - handle->half_period_index)){
							handle->state = STATE__ACC;
						}else{
							handle->state = STATE__DEC;
						}
					}else{
						handle->state = STATE__CONST;
					}
					break;
				case STATE__CONST:
					/* 匀速：剩余步数足够则保持，否则进入减速 */
					if(handle->remain_steps > handle->acc_dec_steps){
						handle->state = STATE__CONST;
					}else{
						handle->state = STATE__DEC;
					}
					break;
				case STATE__DEC:
					/* 减速：逐步增大半周期（降低频率），直到停止 */
					if(handle->half_period_index < (handle->acc_dec_steps - 1)){
						handle->half_period_index++;
						if(handle->remain_steps < (handle->acc_dec_steps - handle->half_period_index)){
							handle->state = STATE__DEC;
						}else{
							handle->state = STATE__ACC;
						}
					}else{
						handle->state = STATE__STOP;
					}
					break;
				case STATE__TURN_AROUND:
					/* 换向：先减速到停（通过调整 half_period_index），并累计回补步数 */
					if(handle->half_period_index < (handle->acc_dec_steps - 1)){
						handle->half_period_index--;
						handle->dispatched_steps++;
					}else{
						handle->state = STATE__STOP;
					}
					break;
				default:
					break;
				}
			}else{
				HAL_GPIO_WritePin(handle->step_gpio_port, handle->step_gpio_pin, GPIO_PIN_SET);
				handle->status |= STATUS__STEP_HIGH;
			}
		}
		break;
	default:
		break;
	}

	return 0;
}
