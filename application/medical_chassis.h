#ifndef __MEDICAL_CHASSIS_H
#define __MEDICAL_CHASSIS_H

#include "main.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "pid.h"
#include "INS_task.h"
#include "can.h"
#include "chassis_task.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "OLED.h"
#include "AHRS_middleware.h"
#include <stdbool.h>
#include <stdlib.h>

#define InitOver 0x10
#define GoOver	 0x11
#define StopOver 0x12
#define Point1	 0x13
#define Point3	 0x15

#define spd_change 200	//每次速度数据变化量
#define spd_cr_num 50  //变化后速度环数量 每个为2ms

#define max_spd	   2000 //最大速度


#define encode_max 8192     //C620转子编码器0~8191，对应机械角度0~360°
#define reduction_ratio 19  //M3508减速比19:1



typedef struct odo{
	float pos_x;
	float pos_y;
	float zangle;
	float xangle;
	float yangle;
	float w_z;
}odomaster;

extern odomaster odo_data;

void uart_printf(UART_HandleTypeDef* uart,uint8_t time_out,const char* str,...);

void medical_chassis_task(void const *pvParameters);
/**
 * @brief 向电机发送速度
 * 
 * @param wheel_speed 四个轮子速度
 */
void wheel_move(const int16_t *wheel_speed);
/**
 * @brief 底盘运动函数
 * 
 * @param vx x方向速度
 * @param vy y方向速度
 * @param vw 自转速度
 */
void medical_chassis_move(int16_t vx, int16_t vy, int16_t vw);
/**
 * @brief 底盘速度平滑函数,函数作用为将速度从当前速度均匀变化至目标速度
 * 该函数是实现预期速度的平滑过渡
 * 
 * @param current_speed 当前速度[0]为vx,[1]为vy,[2]为vw
 * @param target_speed 目标速度[0]为vx,[1]为vy,[2]为vw
 * @param acceleration 加速度 -128~127
 */
void chassis_smooth_speed(const int16_t *current_speed, const int16_t *target_speed, int8_t acceleration);
/**
 * @brief 底盘定点移动函数
 * 
 * @param speed 速度函数[0]为vx,[1]为vy,[2]为vw
 * @param time 移动时间0~65535
 * @param acceleration 加速度大小0~255
 */
void chassis_move_point(int16_t vx,int16_t vy,int16_t vw, uint16_t time, int8_t acceleration);
/**
 * @brief 姿态速度双闭环
 * 
 * @param angle 目标姿态 
 * @param vx 
 * @param vy 
 */
void attitude_spd(float angle,int16_t vx,int16_t vy);

void chassis_move_direction(int16_t vx, int16_t vy, int16_t vw,int8_t angle_init);
/**
 * @brief 匿名上位机通信协议
 * 
 * @param a_x 
 * @param a_y 
 * @param a_z 
 * @param a_w 
 */
void sendSenser(int16_t a_x, int16_t a_y, int16_t a_z, int16_t a_w);
/**
  * @brief          获取角速度,0:x轴, 1:y轴, 2:roll轴 单位 °/s
  * @param[in]      none
  * @retval         gyro的指针
  */
const float* get_gyro_float(void);
/**
  * @brief          获取欧拉角, 0:yaw, 1:pitch, 2:roll 单位 °
  * @param[in]      none
  * @retval         angle的指针
  */
const float* get_angle_float(void);

void move_point(float angle,int16_t posx,int16_t posy);
void Tspeed_move(float angle,int16_t target_point,uint8_t dir,uint8_t acc,uint8_t time);
void USART1_IRQHandler(void);
void Data_Analyse(uint8_t rec);
void test_move(float angle,int16_t vx,int16_t vy,uint16_t time);
void receive_data_handle(void);
uint8_t sum_str(uint8_t *str ,uint8_t size);
void clear_str(uint8_t *str,uint8_t size);

void mood2();
void mood1();
void BED1_1();
void BED3_1();
void BED1_2();
void BED3_2();
void CSH();
#endif
