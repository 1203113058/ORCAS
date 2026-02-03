/*
 * fire_ctrl_utils.c
 *
 *  Created on: Jan 21, 2025
 *      Author: yuchung886
 */
#include "main.h"
#include "fire_ctrl_utils.h"
#include "uart_cmds_handle.h"
#include "mma845x.h"
#include "error_code.h"

/*
 * Fire Control（开火/AEG 活塞/供弹/红点/照明等）控制实现
 * - fire_ctrl_cmds_dispatch(): 处理 UART 设置/查询命令
 * - fire_ctrl_perodic_routines(): 周期调度各子任务
 * - fire_ctrl_isr(): 外部中断事件（例如活塞限位触发）
 */

/* 周期任务分组：将耗时/低频逻辑与高频逻辑分开调度 */
#define FIRE_CTRL_ROUTINES_GRP1_PERIOD_US 10000
#define FIRE_CTRL_ROUTINES_GRP2_PERIOD_US FIRE_CTRL_ROUTINES_MIN_PERIOD_US
uint8_t fire_ctrl_perodic_routines_timer = 0;

/* Fire Control 状态位（bit mask），具体位定义见 fire_ctrl_utils.h */
uint8_t fire_ctrl_status = STATUS__SAFETY_EN;

/* AEG（自动电枪）电机状态机：READY -> PULLING -> RELEASING */
#define AEG_MOTOR_STATE__READY				0
#define AEG_MOTOR_STATE__PULLING_PISTON		1
#define AEG_MOTOR_STATE__RELEASING_PISTON	2
uint8_t aeg_motor_state = AEG_MOTOR_STATE__READY;

/* 关键时序参数：拉活塞超时、启动去抖等 */
#define	AEG_PISTON_PULLING_TIMEOUT_MS		500
#define AEG_MOTOR_ON_DEBOUNCE_TIMEOUT_MS	20
uint16_t aeg_piston_released_timeout_ms = 100;
uint16_t aeg_motor_timer_ms = 0;

uint8_t aeg_motor_duty__auto = 60;
uint8_t aeg_motor_duty__semi = 60;
uint8_t fire_count = 0;

uint8_t aeg_tracer_duty = 0;
#define AEG_TRACER_DIMMING_DELAY_MS	100
uint16_t aeg_tracer_timer_ms = 0;

#define RED_DOT_FLASH_HALF_PERIOD_MS	50
uint16_t red_dot_timer_ms = 0;

/* 供弹电机方向定义（与 STATUS__FEEDER_MOTOR_DIR 对应） */
#define AMMO_FEERER_MOTOR_DIR_CW	0	// Load ammo
#define AMMO_FEERER_MOTOR_DIR_CCW	1	// Un-load ammo
#define AMMO_FEERER_MOTOR_STEPS_PER_SHOT_DEFAULT	220
uint16_t ammo_feeder_motor_steps_per_shot = AMMO_FEERER_MOTOR_STEPS_PER_SHOT_DEFAULT;
uint16_t ammo_feeder_motor_steps_remained = 0;

uint8_t targeting_led_duty = 0;

extern TIM_HandleTypeDef htim3;
#define AEG_MOTOR_PWM TIM_CHANNEL_1
#define AEG_TRACER_PWM TIM_CHANNEL_2
#define SEARCHLIGHT_PWM TIM_CHANNEL_4
TIM_OC_InitTypeDef sConfigOC = {TIM_OCMODE_PWM1, 0, TIM_OCPOLARITY_HIGH, 0, TIM_OCFAST_DISABLE, 0, 0};

static uint8_t aeg_motor_ctrl();
static uint8_t aeg_tracer_ctrl();
static uint8_t aeg_safety_ctrl();
static uint8_t ammo_feeder_ctrl();

uint8_t fire_ctrl_init(){
	/* 初始化 PWM 输出通道：AEG 电机、Tracer、Searchlight */
	sConfigOC.Pulse = 0;
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, AEG_MOTOR_PWM);
	HAL_TIM_PWM_Start(&htim3, AEG_MOTOR_PWM);
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, AEG_TRACER_PWM);
	HAL_TIM_PWM_Start(&htim3, AEG_TRACER_PWM);
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, SEARCHLIGHT_PWM);
	HAL_TIM_PWM_Start(&htim3, SEARCHLIGHT_PWM);
	return 0;
}

uint8_t fire_ctrl_cmds_dispatch(uint8_t* cmd_buf, uint8_t* ack_buf){
	/* UART 命令分发：按 cmd code 处理设置/查询，并填充 ACK */
	switch(cmd_buf[CMD_CODE_INDEX]){
		case CMD_CODE__GET_FIRE_CTRL_STATUS:
			ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
			ack_buf[ACK_PARAMS_INDEX] = fire_ctrl_status;
			break;
		case CMD_CODE__SET_FIRE_COUNT:
			/* 设置开火次数；若之前发生拉活塞超时，这里会返回一次错误并清标志 */
			ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
			if(fire_ctrl_status & STATUS__PULLING_PISTON_TIMEOUT){
				fire_ctrl_status &= ~STATUS__PULLING_PISTON_TIMEOUT;
				ack_buf[ACK_CODE_INDEX] = ERR__AEG_PISTON_PULLING_TIMEOUT;
			}
			fire_count = cmd_buf[CMD_PARAMS_INDEX];

			break;
		case CMD_CODE__SET_FIRE_CTRL_CONFIG:
			/* 配置：半自动/全自动占空比 + 释放活塞等待时间 */
			if((cmd_buf[CMD_PARAMS_INDEX] <= 100) &&
			   (cmd_buf[CMD_PARAMS_INDEX] > 0)){
				aeg_motor_duty__semi = cmd_buf[CMD_PARAMS_INDEX];
			}
			if((cmd_buf[CMD_PARAMS_INDEX + 1] <= 100) &&
			   (cmd_buf[CMD_PARAMS_INDEX + 1] > 0)){
				aeg_motor_duty__auto = cmd_buf[CMD_PARAMS_INDEX + 1];
			}
			aeg_piston_released_timeout_ms = cmd_buf[CMD_PARAMS_INDEX + 2] * 10;
			ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
			break;
		case CMD_CODE__SET_FIRE_SAFETY:
			/* 安全开关：打开安全时清空 fire_count，防止误触发 */
			if(cmd_buf[CMD_PARAMS_INDEX] == 1){
				fire_count = 0;
				fire_ctrl_status |= STATUS__SAFETY_EN;
			}else{
				fire_ctrl_status &= ~STATUS__SAFETY_EN;
			}
			ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
			break;
		case CMD_CODE__SET_AMMO_FEEDER_CONFIG:
			ammo_feeder_motor_steps_per_shot = cmd_buf[CMD_PARAMS_INDEX] * 10;
			break;
		case CMD_CODE__SET_AMMO_FEEDING:
			/* 供弹：设置方向并写入需要执行的步数（高字节在前） */
			if(cmd_buf[CMD_PARAMS_INDEX] == AMMO_FEERER_MOTOR_DIR_CCW){
				// Un-Load ammo
				HAL_GPIO_WritePin(FEEDER_MOTOR_DIR_GPIO_Port, FEEDER_MOTOR_DIR_Pin, GPIO_PIN_SET);
				fire_ctrl_status |= STATUS__FEEDER_MOTOR_DIR;
			}else{
				// Load ammo
				HAL_GPIO_WritePin(FEEDER_MOTOR_DIR_GPIO_Port, FEEDER_MOTOR_DIR_Pin, GPIO_PIN_RESET);
				fire_ctrl_status &= ~STATUS__FEEDER_MOTOR_DIR;
			}
			ammo_feeder_motor_steps_remained = (cmd_buf[CMD_PARAMS_INDEX + 1] << 8) | cmd_buf[CMD_PARAMS_INDEX + 2];
			ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
			break;
		case CMD_CODE__SET_SEARCHLIGHT_PWM:
			/* 照明/瞄准灯 PWM：0 表示关闭；非 0 表示开启并更新 PWM 占空比 */
			if((cmd_buf[CMD_PARAMS_INDEX] <= 100) &&
			   (cmd_buf[CMD_PARAMS_INDEX] >= 0)){
				targeting_led_duty = cmd_buf[CMD_PARAMS_INDEX];
				if(targeting_led_duty == 0){
					fire_ctrl_status &= ~STATUS__TARGETING_LED_EN;
				}else{
					fire_ctrl_status |= STATUS__TARGETING_LED_EN;
				}
				sConfigOC.Pulse = targeting_led_duty;
				HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, SEARCHLIGHT_PWM);
			}
			ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
			break;
		default:
			break;
	}
	return 0;
}

uint8_t fire_ctrl_perodic_routines(){
	/* 周期调度：按最小周期递增计数器，根据分组周期调用不同子任务 */
	fire_ctrl_perodic_routines_timer++;
	if(fire_ctrl_perodic_routines_timer == (FIRE_CTRL_ROUTINES_GRP1_PERIOD_US / FIRE_CTRL_ROUTINES_MIN_PERIOD_US)){
		fire_ctrl_perodic_routines_timer = 0;
	}

	if(fire_ctrl_perodic_routines_timer % (FIRE_CTRL_ROUTINES_GRP1_PERIOD_US / FIRE_CTRL_ROUTINES_MIN_PERIOD_US) == 0){
		/* 低频任务：AEG 电机状态机、Tracer、Safety */
		aeg_motor_ctrl();
		aeg_tracer_ctrl();
		aeg_safety_ctrl();
	}
	if(fire_ctrl_perodic_routines_timer % (FIRE_CTRL_ROUTINES_GRP2_PERIOD_US / FIRE_CTRL_ROUTINES_MIN_PERIOD_US) == 0){
		/* 高频任务：供弹步进控制 */
		ammo_feeder_ctrl();
	}

	return 0;
}

uint8_t fire_ctrl_isr(uint16_t GPIO_Pin){
	/* 外部中断：活塞限位触发（认为拉活塞到位） */
	if(GPIO_Pin == AEG_PISTON_ENDSTOP_Pin){
		sConfigOC.Pulse = 0;
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, AEG_MOTOR_PWM);
		fire_ctrl_status &= ~(STATUS__AEG_MOTOR_EN);

		if((aeg_motor_state == AEG_MOTOR_STATE__PULLING_PISTON) &&
		   (aeg_motor_timer_ms >= AEG_MOTOR_ON_DEBOUNCE_TIMEOUT_MS)){
			aeg_motor_timer_ms = 0;
			aeg_motor_state = AEG_MOTOR_STATE__RELEASING_PISTON;

			aeg_tracer_timer_ms = 0;
			aeg_tracer_duty = 100;
			sConfigOC.Pulse = aeg_tracer_duty;
			HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, AEG_TRACER_PWM);

			// Load ammo
			HAL_GPIO_WritePin(FEEDER_MOTOR_DIR_GPIO_Port, FEEDER_MOTOR_DIR_Pin, GPIO_PIN_RESET);
			fire_ctrl_status &= ~STATUS__FEEDER_MOTOR_DIR;
			ammo_feeder_motor_steps_remained += ammo_feeder_motor_steps_per_shot;
		}
	}

	return 0;
}

static uint8_t aeg_motor_ctrl(){
	/*
	 * AEG 电机控制状态机
	 * - READY:       等待 fire_count 下发
	 * - PULLING:     拉活塞（由限位中断或超时结束）
	 * - RELEASING:   释放活塞后等待下一次射击或回到 READY
	 */
	if(fire_count){
		if(!(fire_ctrl_status & STATUS__SAFETY_EN)){
			switch(aeg_motor_state){
			case AEG_MOTOR_STATE__READY:
				/* 根据剩余发数选择半自动/全自动占空比 */
				if(fire_count == 1){
					sConfigOC.Pulse = aeg_motor_duty__semi;
				}else{
					sConfigOC.Pulse = aeg_motor_duty__auto;
				}
				HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, AEG_MOTOR_PWM);
				fire_ctrl_status |= STATUS__AEG_MOTOR_EN;
				aeg_motor_timer_ms = 0;
				aeg_motor_state = AEG_MOTOR_STATE__PULLING_PISTON;
				break;
			case AEG_MOTOR_STATE__PULLING_PISTON:
				/* 拉活塞超时保护：避免卡死持续上电 */
				aeg_motor_timer_ms += (FIRE_CTRL_ROUTINES_GRP1_PERIOD_US / 1000);
				if(aeg_motor_timer_ms >= AEG_PISTON_PULLING_TIMEOUT_MS){
					sConfigOC.Pulse = 0;
					HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, AEG_MOTOR_PWM);
					fire_ctrl_status &= ~(STATUS__AEG_MOTOR_EN);
					fire_count = 0;
					aeg_motor_timer_ms = 0;
					aeg_motor_state = AEG_MOTOR_STATE__READY;
					fire_ctrl_status |= STATUS__PULLING_PISTON_TIMEOUT;
				}
				break;
			case AEG_MOTOR_STATE__RELEASING_PISTON:
				/* 释放活塞等待：到期后递减 fire_count，决定继续下一发或停止 */
				aeg_motor_timer_ms += (FIRE_CTRL_ROUTINES_GRP1_PERIOD_US / 1000);
				if(aeg_motor_timer_ms >= aeg_piston_released_timeout_ms){
					if(--fire_count){
						if(fire_count == 1){
							sConfigOC.Pulse = aeg_motor_duty__semi;
						}else{
							sConfigOC.Pulse = aeg_motor_duty__auto;
						}
						HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, AEG_MOTOR_PWM);
						fire_ctrl_status |= STATUS__AEG_MOTOR_EN;
						aeg_motor_timer_ms = 0;
						aeg_motor_state = AEG_MOTOR_STATE__PULLING_PISTON;
					}else{
						sConfigOC.Pulse = 0;
						HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, AEG_MOTOR_PWM);
						fire_ctrl_status &= ~(STATUS__AEG_MOTOR_EN);
						aeg_motor_timer_ms = 0;
						aeg_motor_state = AEG_MOTOR_STATE__READY;
					}
				}
				break;
			default:
				break;
			}
		}
	}
	return 0;
}

static uint8_t aeg_tracer_ctrl(){
	/*
	 * Tracer（曳光）控制
	 * - 在活塞限位触发后点亮 tracer
	 * - 延时后逐步降低亮度；若期间有继续开火则直接熄灭
	 */

	if(aeg_tracer_duty){
		if(aeg_tracer_timer_ms < AEG_TRACER_DIMMING_DELAY_MS){
			aeg_tracer_timer_ms += (FIRE_CTRL_ROUTINES_GRP1_PERIOD_US / 1000);
		}else{
			if(fire_count){
				aeg_tracer_duty = 0;
				sConfigOC.Pulse = aeg_tracer_duty;
				HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, AEG_TRACER_PWM);
			}else{
				aeg_tracer_duty = aeg_tracer_duty - 2;
				sConfigOC.Pulse = aeg_tracer_duty;
				HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, AEG_TRACER_PWM);
			}
		}
	}

	return 0;
}

static uint8_t aeg_safety_ctrl(){
	/*
	 * 安全模式指示
	 * - 安全开启：若 fire_count 被写入，则通过红点闪烁提示并消耗 fire_count
	 * - 安全关闭：红点常亮
	 */
	if(fire_ctrl_status & STATUS__SAFETY_EN){
		if(fire_count){
			red_dot_timer_ms += (FIRE_CTRL_ROUTINES_GRP1_PERIOD_US / 1000);
			if(red_dot_timer_ms % RED_DOT_FLASH_HALF_PERIOD_MS == 0){
				if(fire_ctrl_status & STATUS__RED_DOT_EN){
					fire_ctrl_status &= ~STATUS__RED_DOT_EN;
					HAL_GPIO_WritePin(RED_DOT_EN_GPIO_Port, RED_DOT_EN_Pin, GPIO_PIN_RESET);
				}else{
					fire_ctrl_status |= STATUS__RED_DOT_EN;
					HAL_GPIO_WritePin(RED_DOT_EN_GPIO_Port, RED_DOT_EN_Pin, GPIO_PIN_SET);
				}
				if(red_dot_timer_ms >= RED_DOT_FLASH_HALF_PERIOD_MS * 6){
					red_dot_timer_ms = 0;
					fire_count--;
				}
			}
		}else{
			if(fire_ctrl_status & STATUS__RED_DOT_EN){
				fire_ctrl_status &= ~STATUS__RED_DOT_EN;
				HAL_GPIO_WritePin(RED_DOT_EN_GPIO_Port, RED_DOT_EN_Pin, GPIO_PIN_RESET);
			}
		}
	}else{
		if(!(fire_ctrl_status & STATUS__RED_DOT_EN)){
			fire_ctrl_status |= STATUS__RED_DOT_EN;
			HAL_GPIO_WritePin(RED_DOT_EN_GPIO_Port, RED_DOT_EN_Pin, GPIO_PIN_SET);
		}
	}
	return 0;
}

#define STATE_SLEEP		0
#define STATE_ACTIVE	1
uint8_t ammo_feeder_state = STATE_SLEEP;
#define AMMO_FEEDER_WAKEUP_DELAY_US	1000
uint8_t ammo_feeder_timer = 0;
static uint8_t ammo_feeder_ctrl(){
	/*
	 * 供弹步进控制
	 * - STATE_SLEEP: 关闭驱动（SLEEP=0），降低功耗
	 * - STATE_ACTIVE: 翻转 STEP 引脚产生步进脉冲，直到步数执行完毕
	 */
	switch(ammo_feeder_state){
	case STATE_SLEEP:
		/* 有待执行步数时唤醒驱动，等待稳定后进入 ACTIVE */
		if(ammo_feeder_motor_steps_remained){
			HAL_GPIO_WritePin(FEEDER_MOTOR_SLEEP_GPIO_Port, FEEDER_MOTOR_SLEEP_Pin, GPIO_PIN_SET);
			ammo_feeder_timer++;
			if(ammo_feeder_timer > (AMMO_FEEDER_WAKEUP_DELAY_US / FIRE_CTRL_ROUTINES_GRP2_PERIOD_US)){
				ammo_feeder_timer = 0;
				ammo_feeder_state = STATE_ACTIVE;
			}
		}
		break;
	case STATE_ACTIVE:
		if(ammo_feeder_motor_steps_remained){
			/* 通过翻转 STEP 电平实现步进；在 STEP 下降沿计数步数 */
			if(fire_ctrl_status & STATUS__FEEDER_MOTOR_STEP){
				HAL_GPIO_WritePin(FEEDER_MOTOR_STEP_GPIO_Port, FEEDER_MOTOR_STEP_Pin, GPIO_PIN_RESET);
				fire_ctrl_status &= ~STATUS__FEEDER_MOTOR_STEP;
				ammo_feeder_motor_steps_remained--;
				if(!ammo_feeder_motor_steps_remained){
					HAL_GPIO_WritePin(FEEDER_MOTOR_SLEEP_GPIO_Port, FEEDER_MOTOR_SLEEP_Pin, GPIO_PIN_RESET);
					ammo_feeder_state = STATE_SLEEP;
				}
			}else{
				HAL_GPIO_WritePin(FEEDER_MOTOR_STEP_GPIO_Port, FEEDER_MOTOR_STEP_Pin, GPIO_PIN_SET);
				fire_ctrl_status |= STATUS__FEEDER_MOTOR_STEP;
			}
		}
		break;
	default:
		break;
	}
	return 0;
}


