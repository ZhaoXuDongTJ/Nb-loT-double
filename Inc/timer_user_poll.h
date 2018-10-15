//******************************************************************************
//                                www.ghostyu.com
//
//                 Copyright (c) 2017-2018, WUXI Ghostyu Co.,Ltd.
//                                All rights reserved.
//
//  FileName : timer_user_poll.h
//  Date     : 2017-12-04 16:11
//  Version  : V0001
// ��ʷ��¼  : ��һ�δ���
//******************************************************************************

#ifndef _TIMER_USER_POLL_H
#define _TIMER_USER_POLL_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "stm32l4xx_hal.h"
#include "nb_bc95.h"
  
void MX_TIM_Set(bc95_timeout_cb cb);

void MX_TIM_Start(uint32_t ms);

void MX_TIM_Stop(void);

void MX_TimerPoll(void);

#ifdef __cplusplus
}
#endif
#endif   //_TIMER_USER_POLL_H