//******************************************************************************
//                                www.ghostyu.com
//
//                 Copyright (c) 2017-2018, WUXI Ghostyu Co.,Ltd.
//                                All rights reserved.
//
//  FileName : led_blink.h
//  Date :     2018-1-09 10:30
//  Version :  V0001
//  History :  初始创建版本
//******************************************************************************
#ifndef _LED_BLINK_H
#define _LED_BLINK_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "gpio.h"
  
typedef enum
{
  LED_INDEX_0,
  LED_INDEX_1,
  LED_INDEX_2
}led_index_e;
//*********************************************
// fn : HAL_LedCtl_Init
//
// brief : 实始化Led控制结构
//
// param : none
//
// return : none
void HAL_LedCtl_Init(void);
//*********************************************
// fn : HAL_Led_poll
//
// brief : 轮询LED 状态
//
// param : none
//
// return : none
void HAL_Led_poll(void);
//*********************************************
// fn : HAL_LED_Blink
//
// brief : 设置LED 闪烁信息
//
// param : time -> 周期
//         onpct -> 占空比
//         times -> 闪烁次数。0表示无限次
//         index -> 设置led控制结构的下标
//
// return : none
void HAL_LED_Blink(uint32_t time,uint8_t onpct,uint8_t times,led_index_e index);  

//*********************************************
// fn : HAL_LED_SET
//
// brief : 控制LED 闪烁
//
// param : index -> 对应优先级的LED 控制结构
//         times -> 闪烁次数
//
// return : none
void HAL_LED_SET(led_index_e index, uint8_t times);

 
#ifdef __cplusplus
}
#endif
#endif   //_LED_BLINK_H