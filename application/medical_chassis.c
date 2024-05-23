/**
 * @file medical_chassis.c
 * @author ��־�� 
 * @brief SITҽ�ƻ����˵����˶�
 * @version 0.2
 * @date 2022-11-18
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "medical_chassis.h"
#include "main.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "tim.h"

volatile int angle = 0;				//ŷ����
pid_type_def pid_spd[4],pid_angle,pid_point[2];	//PID�ṹ�����
float angle_pid_value[3] = {30,0,0};
float point_pid[3]={2,3,0};//λ�û�PID����
const fp32 pid_parameter[3] = {1.5f, 0.1f, 0.1f};

float kp=1;

uint8_t receive[50];
uint8_t receive_flag = 0;
odomaster odo_data;
uint8_t od_data[50];


/**
 * @brief ҽ�Ƶ���������������
 * 
 * @param pvParameters 
 */

void medical_chassis_task(void const *pvParameters)
{
	  HAL_GPIO_WritePin(BC1_GPIO_Port, BC1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(BC2_GPIO_Port, BC2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(JDQ_GPIO_Port, JDQ_Pin, GPIO_PIN_RESET);
	
	  HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	  
	CSH();
	for (int i = 0; i < 4; i++)
	{
		PID_init(&pid_spd[i], PID_POSITION, pid_parameter, 16384, 2000);
	}
	PID_init(&pid_angle,PID_POSITION,angle_pid_value,2000,100);
	PID_init(&pid_point[0],PID_POSITION,point_pid,1500,100);
	PID_init(&pid_point[1],PID_POSITION,point_pid,1500,100);
	
	while(receive_flag != 1);	
	
	
	
	float angle_init = odo_data.zangle;	
	
	
  while(HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET);

	if(HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin) == GPIO_PIN_RESET)
                {
                  mood2(); 																		
                }
					else if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
	              {
                   mood1();  																		
                }

	while (1)
	{
		    
		
		attitude_spd(angle_init,0,0);
		/* �����ٶȴ�ӡ*/
		// int16_t wheel_speed[4] = {1000,1000,1000,1000};
		// wheel_move(wheel_speed);		
		// receive_data_handle();

		/*���ٶȴ�ӡ*/
		// uint8_t accl[50];
		// const float *acc;
		// acc = get_accel_data_point();
		// sprintf((char*)accl,"x:%f,y:%f,roll:%f\r\n",acc[0],acc[1],acc[2]);
		// HAL_UART_Transmit(&huart6,accl,strlen(accl),20);
		
		/*�������������*/
		// uint8_t encode_uart[50];
		// HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
		// sprintf((char*)encode_uart,"encode:%d,last:%d\r\n",motor_chassis[0].ecd,motor_chassis[0].last_ecd);
		// HAL_UART_Transmit(&huart6,encode_uart,strlen(encode_uart),20);
		// uart_printf(&huart6,20,"x:%f,y:%f,roll:%f\r\n",acc[0],acc[1],acc[2]);

	}
 }

/**
 * @brief �㵽��ֱ��T�μӼ����˶�(��֧��x��y��������)
 * 
 * @param angle ��̬�Ƕ�
 * @param target_point Ŀ���
 * @param dir ����1Ϊx�ᣬ2Ϊy��
 * @param acc ÿ�μ���
 * @param time ���ټ��ʱ��
 */


void move_point(float angle,int16_t posx,int16_t posy)
{
	int16_t error_x = posx - odo_data.pos_y;
	int16_t error_y = posy - odo_data.pos_x;
	
	int16_t spd_x;
	int16_t spd_y;	
	
	while(((error_x > 50)||(error_x < (-50))) || ((error_y > 50)||(error_y < (-50))))
	{
		PID_calc(&pid_point[0],odo_data.pos_y,posx);
		PID_calc(&pid_point[1],odo_data.pos_x,posy);
//			spd_x = -(kp*error_x);
//			spd_y = (kp*error_y);
		
			attitude_spd(angle,-pid_point[0].out,pid_point[1].out);
		
			error_x = posx-odo_data.pos_y;
			error_y = posy-odo_data.pos_x;
	}
	attitude_spd(angle,0,0);
}
/**
 * @brief �㵽��ֱ��T�μӼ����˶�(��֧��x��y��������)
 * 
 * @param angle ��̬�Ƕ�
 * @param target_point Ŀ���
 * @param dir ����1Ϊx�ᣬ2Ϊy��
 * @param acc ÿ�μ���
 * @param time ���ټ��ʱ��
 */
void Tspeed_move(float angle,int16_t target_point,uint8_t dir,uint8_t acc,uint8_t time)
{
	int16_t error_max=0,error_now=0,speed=0,init_pos = 0;
	uint16_t total_move=0,acc_displacement=0;
	uint8_t acc_flag = 0;//û�мӵ�����ٶȱ�־λ
	int8_t sgn_flag = 0;
	if(dir == 1)
	{
		init_pos = odo_data.pos_y;
		error_max = target_point - odo_data.pos_y;
		total_move = abs(error_max);
		error_now = target_point-odo_data.pos_y;
		sgn_flag = (error_max>0)?-1:1;
		while (speed <= max_spd)//���ٹ���
		{
			if(abs(error_now)>(uint16_t)(total_move/2))
			{
				speed += acc;
				attitude_spd(angle,sgn_flag*speed,0);
				osDelay(time);				
			}
			else
			{
				acc_flag = 1;
				break;
			}
			error_now = target_point-odo_data.pos_y;
		}
		if (acc_flag == 1)//�����ٹ��̣����Ӽ���
		{
			while (speed>0)
			{
				speed -= acc;
				attitude_spd(angle,sgn_flag*speed,0);
				osDelay(time);
			}
			attitude_spd(angle,0,0);
			return;
		}
		else//T�μӼ���
		{
			acc_displacement = abs((int)odo_data.pos_y-init_pos);
			while (abs(error_now) > acc_displacement)//���ٽ׶�
			{
				attitude_spd(angle,sgn_flag*speed,0);
				osDelay(1);
				error_now = target_point-odo_data.pos_y;
			}
			while(speed>0)//���ٽ׶�
			{
				speed -= acc;
				attitude_spd(angle,sgn_flag*speed,0);
				osDelay(time);
			}
			attitude_spd(angle,0,0);
			return;
		}
	}
	else if (dir == 2)
	{
		init_pos = odo_data.pos_x;
		error_max = target_point - odo_data.pos_x;
		total_move = abs(error_max);
		error_now = target_point-odo_data.pos_x;
		sgn_flag = (error_max>0)?-1:1;
		while (speed <= max_spd)//���ٹ���
		{
			if(abs(error_now)>(uint16_t)(total_move/2))
			{
				speed += acc;
				attitude_spd(angle,0,-sgn_flag*speed);
				osDelay(time);				
			}
			else
			{
				acc_flag = 1;
				break;
			}
			error_now = target_point-odo_data.pos_x;
		}
		if (acc_flag == 1)//�����ٹ��̣����Ӽ���
		{
			while (speed>0)
			{
				speed -= acc;
				attitude_spd(angle,0,-sgn_flag*speed);
				osDelay(time);
			}
			attitude_spd(angle,0,0);
			return;
		}
		else//T�μӼ���
		{
			acc_displacement = abs((int)odo_data.pos_x-init_pos);
			while (abs(error_now) > acc_displacement)//���ٽ׶�
			{
				attitude_spd(angle,0,-sgn_flag*speed);
				osDelay(1);
				error_now = target_point-odo_data.pos_x;
			}
			while(speed>0)//���ٽ׶�
			{
				speed -= acc;
				attitude_spd(angle,0,-sgn_flag*speed);
				osDelay(time);
			}
			attitude_spd(angle,0,0);
			return;
		}
	}
	
}

void test_move(float angle,int16_t vx,int16_t vy,uint16_t time)
{
		for(uint16_t i = 0;i<time;i++)
		{
			attitude_spd(angle,vx,vy);
			osDelay(1);			
		}	
}

/**
 * @brief ���������ٶ�
 * 
 * @param wheel_speed �ĸ������ٶ�
 */
void wheel_move(const int16_t *wheel_speed)
{
	HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
	for (uint8_t i = 0; i < 4; i++)
	{
		PID_calc(&pid_spd[i], motor_chassis[i].speed_rpm, wheel_speed[i]);
	}
	CAN_cmd_chassis(pid_spd[0].out, pid_spd[1].out, pid_spd[2].out, pid_spd[3].out);

	osDelay(2); //����ʱ������ڣ��������ͻ��޹�����ת
}
/**
 * @brief �����˶�����
 * 
 * @param vx x�����ٶ�
 * @param vy y�����ٶ�
 * @param vw ��ת�ٶ�
 */
void medical_chassis_move(int16_t vx, int16_t vy, int16_t vw)
{
	int16_t wheel_speed[4];

	wheel_speed[0] = -vx - vy - vw;
	wheel_speed[1] = vx - vy - vw;
	wheel_speed[2] = vx + vy - vw;
	wheel_speed[3] = -vx + vy - vw;

	wheel_move(wheel_speed);
}
/**
 * @brief �����ٶ�ƽ������,��������Ϊ���ٶȴӵ�ǰ�ٶȾ��ȱ仯��Ŀ���ٶ�
 * �ú�����ʵ��Ԥ���ٶȵ�ƽ������
 * 
 * @param current_speed ��ǰ�ٶ�[0]Ϊvx,[1]Ϊvy,[2]Ϊvw
 * @param target_speed Ŀ���ٶ�[0]Ϊvx,[1]Ϊvy,[2]Ϊvw
 * @param acceleration ���ٶ� -128~127
 */
void chassis_smooth_speed(const int16_t *current_speed, const int16_t *target_speed, int8_t acceleration)
{
	int16_t transmit_speed[3];
	uint8_t speed_print[100];
	memcpy(transmit_speed, current_speed, 6);
	while (memcmp(transmit_speed, target_speed, 6))
	{
		for (uint8_t i = 0; i < 3; i++)
		{
			transmit_speed[i] = ((target_speed[i] - transmit_speed[i]) * acceleration <= 0) ? target_speed[i] : (transmit_speed[i] + acceleration);
		}
		medical_chassis_move(transmit_speed[0], transmit_speed[1], transmit_speed[2]);
		osDelay(5);
	}
}
/**
 * @brief ���̶����ƶ�����
 * 
 * @param vx 
 * @param vy 
 * @param vw 
 * @param time 
 * @param acceleration 
 */
void chassis_move_point(int16_t vx, int16_t vy, int16_t vw, uint16_t time, int8_t acceleration)
{
	int16_t current_speed[3] = {0, 0, 0}, speed[3] = {vx, vy, vw};
	acceleration = (speed[0] < 0 | speed[1] < 0 | speed[2] < 0) ? (-1) * acceleration : acceleration;
	chassis_smooth_speed(current_speed, speed, acceleration);
	while (time--)
	{
		medical_chassis_move(speed[0], speed[1], speed[2]);
		osDelay(2);
	}
	chassis_smooth_speed(speed, current_speed, (-1) * acceleration);
}

/**
 * @brief ��̬�ٶ�˫�ջ�
 * 
 * @param angle Ŀ����̬ 
 * @param vx 
 * @param vy 
 */
void attitude_spd(float angle,int16_t vx,int16_t vy)
{
//	PID_calc(&pid_angle,get_angle_float()[0],angle);
		PID_calc(&pid_angle,odo_data.zangle,angle);
	medical_chassis_move(vx,vy,pid_angle.out);
}
/**
  * @brief          ��ȡŷ����, 0:yaw, 1:pitch, 2:roll ��λ ��
  * @param[in]      none
  * @retval         angle��ָ��
  */
const float* get_angle_float()
{
	float angle[3];
	for (uint8_t i = 0; i < 3; i++)
	{
		angle[i] = (float)get_INS_angle_point()[i]*180/PI;
	}
	return angle;
}
/**
  * @brief          ��ȡ���ٶ�,0:x��, 1:y��, 2:roll�� ��λ ��/s
  * @param[in]      none
  * @retval         gyro��ָ��
  */
const float* get_gyro_float()
{
	float gyro[3];
	for (uint8_t i = 0; i < 3; i++)
	{
		gyro[i] = (float)get_gyro_data_point()[i]*180/PI;
	}
	return gyro;
}


/**
 * @brief ���ݽ�������  �����MCUƽ̨���������⣬ֻ�轫���ڽ��յ���ֵ����ú������ɽ���
 * @param  rec ���ڽ��յ����ֽ�����
 */
void Data_Analyse(uint8_t rec)
{
	static uint8_t ch;
	static union
	{
		uint8_t date[24];
		float ActVal[6];
	}posture;
	static uint8_t count=0;
	static uint8_t i=0;

	ch=rec;
	switch(count)
	{
		case 0:
			if(ch==0x0d)
				count++;
			else
				count=0;
			break;
		case 1:
			if(ch==0x0a)
			{
				i=0;
				count++;
			}
			else if(ch==0x0d);
			else
				count=0;
			break;
		case 2:
			posture.date[i]=ch;
			i++;
			if(i>=24)
			{
				i=0;
				count++;
			}
			break;
		case 3:
			if(ch==0x0a)
				count++;
			else
				count=0;
			break;
		case 4:
			if(ch==0x0d)
			{
				odo_data.zangle=posture.ActVal[0];
				odo_data.xangle=posture.ActVal[1];
				odo_data.yangle=posture.ActVal[2];
				odo_data.pos_x=posture.ActVal[3];
				odo_data.pos_y=posture.ActVal[4];
				odo_data.w_z=posture.ActVal[5];
			}
			count=0;
			break;
		default:
			count=0;
		break;
	}
}

	
void USART1_IRQHandler(void)
{
	
	uint8_t data;
	if(huart1.Instance->SR & UART_FLAG_RXNE)
	{
		data = huart1.Instance->DR;
		uart_printf(&huart6,20,"hello\r\n");
		Data_Analyse(data);
	}
}

void USART6_IRQHandler(void)
{
    uint8_t data;
    if (huart6.Instance->SR & UART_FLAG_RXNE)
    {
        data = huart6.Instance->DR;
				receive_flag = 1;
				Data_Analyse(data);
    }
}



void uart_printf(UART_HandleTypeDef* uart,uint8_t time_out,const char* str,...)
{
	uint16_t str_len = strlen(str);
	char p[str_len];

	va_list pArgs; //��������������
	va_start(pArgs, str);
	vsprintf(p,str,pArgs); //��ʽ�����ַ��������� ... �������һ��Ҫ��vsprintf
	HAL_UART_Transmit(uart,(uint8_t*)p,strlen(p),time_out);	
	va_end(pArgs);
}

/**
 * @brief ������ͺ���
 * 
 * @param str ����
 * @param size ���鳤��
 * @return uint8_t �����
 */

uint8_t sum_str(uint8_t *str, uint8_t size)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < size; i++)
    {
        sum += str[i];
    }
    return sum;
}

/**
 * @brief ������麯��
 * 
 * @param str Ŀ������
 * @param size �����С
 */
void clear_str(uint8_t *str, uint8_t size)
{
    for (uint8_t i = 0; i < size; i++)
    {
        str[i] = 0;
    }
}

void mood2()
{
	float angle_init = odo_data.zangle;	
	
  Tspeed_move(angle_init,-1600,2,20,10);
  osDelay(500);
	Tspeed_move(angle_init,-4680,1,20,10);
	
  BED3_2();
	CSH();
  osDelay(1000);
	
	Tspeed_move(angle_init,-3300,1,20,10);
  osDelay(500);
	Tspeed_move(angle_init,1790,2,20,10);
  osDelay(500);
	Tspeed_move(angle_init,-4580,1,20,10);
	
	BED1_2();
	CSH();
  osDelay(1000);
	
	
	Tspeed_move(angle_init,-50,1,20,10);
  osDelay(500);
	Tspeed_move(angle_init,210,2,20,10);
	
}

void mood1()
{
	float angle_init = odo_data.zangle;	
	
  Tspeed_move(angle_init,1600,2,20,10);
  osDelay(500);
	Tspeed_move(angle_init,-4550,1,20,10);
	
  BED1_1();
	CSH();
  osDelay(1000);
	
	Tspeed_move(angle_init,-3200,1,20,10);
  osDelay(500);
	Tspeed_move(angle_init,-1440,2,20,10);
  osDelay(500);
	Tspeed_move(angle_init,-4690,1,20,10);
	
	BED3_1();
	CSH();
  osDelay(1000);
	
	
	Tspeed_move(angle_init,-85,1,20,10);
  osDelay(500);
	Tspeed_move(angle_init,-95,2,20,10);
}


void BED1_1()
{    
	  
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1400);
			
	  HAL_GPIO_WritePin(BC1_GPIO_Port, BC1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(BC2_GPIO_Port, BC2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(JDQ_GPIO_Port, JDQ_Pin, GPIO_PIN_RESET);
	
	  

    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 920);
		 osDelay(500);
	
	  
		 
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2300);
	
	osDelay(2000);
	
	  HAL_GPIO_WritePin(JDQ_GPIO_Port, JDQ_Pin, GPIO_PIN_SET);

	
    osDelay(1000);
		
		
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1300);
		osDelay(1000);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 2500);
		
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 2200);
		 HAL_GPIO_WritePin(JDQ_GPIO_Port, JDQ_Pin, GPIO_PIN_RESET);
		
		osDelay(1000);
		
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1550);
		 
		 osDelay(1000);
		 
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1300);
		  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1000);
			
	  HAL_GPIO_WritePin(BC1_GPIO_Port, BC1_Pin, GPIO_PIN_RESET);
		osDelay(100);
		HAL_GPIO_WritePin(BC1_GPIO_Port, BC1_Pin, GPIO_PIN_SET);
		osDelay(500);
    
		
}
void BED3_1()
{
     __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1400);
			
	  HAL_GPIO_WritePin(BC1_GPIO_Port, BC1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(BC2_GPIO_Port, BC2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(JDQ_GPIO_Port, JDQ_Pin, GPIO_PIN_RESET);
	
	  

    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1110);
		 osDelay(500);
	
	  
		 
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2300);
	
	osDelay(2000);
	
	  HAL_GPIO_WritePin(JDQ_GPIO_Port, JDQ_Pin, GPIO_PIN_SET);

	
    osDelay(1000);
		
		
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1300);
		osDelay(1000);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 2500);
		
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 2200);
		 HAL_GPIO_WritePin(JDQ_GPIO_Port, JDQ_Pin, GPIO_PIN_RESET);
		
		osDelay(1000);
		
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1550);
		 
		 osDelay(1000);
		 
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1300);
		  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1000);
			
	  HAL_GPIO_WritePin(BC2_GPIO_Port, BC2_Pin, GPIO_PIN_RESET);
		osDelay(200);
		HAL_GPIO_WritePin(BC2_GPIO_Port, BC2_Pin, GPIO_PIN_SET);
		osDelay(500);
		
		 
		

}	
void BED1_2()
{    
	  
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1400);
			
	  HAL_GPIO_WritePin(BC1_GPIO_Port, BC1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(BC2_GPIO_Port, BC2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(JDQ_GPIO_Port, JDQ_Pin, GPIO_PIN_RESET);
	
	  

    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 920);
		 osDelay(500);
	
	  
		 
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2300);
	
	osDelay(2000);
	
	  HAL_GPIO_WritePin(JDQ_GPIO_Port, JDQ_Pin, GPIO_PIN_SET);

	
    osDelay(1000);
		
		
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1300);
		osDelay(1000);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 2500);
		
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 2200);
		 HAL_GPIO_WritePin(JDQ_GPIO_Port, JDQ_Pin, GPIO_PIN_RESET);
		
		osDelay(1000);
		
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1550);
		 
		 osDelay(1000);
		 
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1300);
		  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1000);
			
	  HAL_GPIO_WritePin(BC1_GPIO_Port, BC1_Pin, GPIO_PIN_RESET);
		osDelay(100);
		HAL_GPIO_WritePin(BC1_GPIO_Port, BC1_Pin, GPIO_PIN_SET);
		osDelay(500);
    
		
}
void BED3_2()
{
     __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1400);
			
	  HAL_GPIO_WritePin(BC1_GPIO_Port, BC1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(BC2_GPIO_Port, BC2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(JDQ_GPIO_Port, JDQ_Pin, GPIO_PIN_RESET);
	
	  

    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1110);
		 osDelay(500);
	
	  
		 
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2300);
	
	osDelay(2000);
	
	  HAL_GPIO_WritePin(JDQ_GPIO_Port, JDQ_Pin, GPIO_PIN_SET);

	
    osDelay(1000);
		
		
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1300);
		osDelay(1000);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 2500);
		
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 2220);
		 HAL_GPIO_WritePin(JDQ_GPIO_Port, JDQ_Pin, GPIO_PIN_RESET);
		
		osDelay(1000);
		
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1550);
		 
		 osDelay(1000);
		 
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1300);
		  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1000);
			
	  HAL_GPIO_WritePin(BC2_GPIO_Port, BC2_Pin, GPIO_PIN_RESET);
		osDelay(200);
		HAL_GPIO_WritePin(BC2_GPIO_Port, BC2_Pin, GPIO_PIN_SET);
		osDelay(500);
		
		 
		

}	
void CSH()//��е�۳�ʼ��
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1300);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1000);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 2500);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 2500);

}