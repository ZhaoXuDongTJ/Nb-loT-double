//******************************************************************************
//                                www.ghostyu.com
//
//                 Copyright (c) 2017-2018, WUXI Ghostyu Co.,Ltd.
//                                All rights reserved.
//
//  FileName : NB_Board_Cfg.c
//  Date     : 2017-12-21 5:45
//  Version  : V0001
// ��ʷ��¼  : 1.��һ�δ���
//
// ˵����
// NB_IOT��ƽ̨��ص����ñ�������
//******************************************************************************
#include "usart.h"

#include "timer_user_poll.h"

#include "NB_Board.h"
#include "NB_BC95.h"

extern int NB_MsgreportCb(msg_types_t,int ,char*);

com_fxnTable  com_fxn = 
{
  .openFxn = HAL_UARTDMA_Init,
  .sendFxn = HAL_UART_Write,
  .closeFxn = HAL_UART_Close
};

time_fxnTable time_fxn = 
{
  .initTimerFxn = MX_TIM_Set,
  .startTimerFxn = MX_TIM_Start,
  .stopTimerFxn = MX_TIM_Stop
};

hw_object_t  HWAtrrs_object = 
{
  .baudrate = 9600,
  .uart_fxnTable = &com_fxn,
  .timer_fxnTable = &time_fxn
};

NB_Config  nb_config = 
{
  .fxnTablePtr = NULL,
  .object = (void*)&HWAtrrs_object,
  .AppReceCB = NB_MsgreportCb,
  .log = NULL
};
