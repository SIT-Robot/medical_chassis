/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       oled_task.c/h
  * @brief      OLED show error value.oled∆¡ƒªœ‘ æ¥ÌŒÛ¬Î
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "oled_task.h"
#include "main.h"
#include "oled.h"

#include "cmsis_os.h"
#include "detect_task.h"
#include "voltage_task.h"
#include "stdio.h"
#include "INS_task.h"
#include "medical_chassis.h"

#define OLED_CONTROL_TIME 10
#define REFRESH_RATE    10

const error_t *error_list_local;

uint8_t other_toe_name[4][4] = {"GYR\0","ACC\0","MAG\0","REF\0"};

uint8_t last_oled_error = 0;
uint8_t now_oled_errror = 0;
static uint8_t refresh_tick = 0;

/**
  * @brief          oled task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          oled»ŒŒÒ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void oled_task(void const * argument)
{
    uint8_t i;
    uint8_t show_col, show_row;
    float angle[3];
    // error_list_local = get_error_list_point();
    // osDelay(1000);
    OLED_init();
    while(1)
    {
				OLED_printf(1,1,"x:%.2f",odo_data.pos_x);
        OLED_printf(2,10,"y:%.2f ",odo_data.pos_y);
        OLED_printf(3,20,"w:%f ",odo_data.zangle);
				OLED_refresh_gram();
        osDelay(100);
    }
}

