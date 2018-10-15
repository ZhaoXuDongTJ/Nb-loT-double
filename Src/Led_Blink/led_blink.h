//******************************************************************************
//                                www.ghostyu.com
//
//                 Copyright (c) 2017-2018, WUXI Ghostyu Co.,Ltd.
//                                All rights reserved.
//
//  FileName : led_blink.h
//  Date :     2018-1-09 10:30
//  Version :  V0001
//  History :  ��ʼ�����汾
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
// brief : ʵʼ��Led���ƽṹ
//
// param : none
//
// return : none
void HAL_LedCtl_Init(void);
//*********************************************
// fn : HAL_Led_poll
//
// brief : ��ѯLED ״̬
//
// param : none
//
// return : none
void HAL_Led_poll(void);
//*********************************************
// fn : HAL_LED_Blink
//
// brief : ����LED ��˸��Ϣ
//
// param : time -> ����
//         onpct -> ռ�ձ�
//         times -> ��˸������0��ʾ���޴�
//         index -> ����led���ƽṹ���±�
//
// return : none
void HAL_LED_Blink(uint32_t time,uint8_t onpct,uint8_t times,led_index_e index);  

//*********************************************
// fn : HAL_LED_SET
//
// brief : ����LED ��˸
//
// param : index -> ��Ӧ���ȼ���LED ���ƽṹ
//         times -> ��˸����
//
// return : none
void HAL_LED_SET(led_index_e index, uint8_t times);

 
#ifdef __cplusplus
}
#endif
#endif   //_LED_BLINK_H