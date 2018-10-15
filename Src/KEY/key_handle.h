//******************************************************************************
//                                www.ghostyu.com
//
//                 Copyright (c) 2017-2018, WUXI Ghostyu Co.,Ltd.
//                                All rights reserved.
//
//  FileName : key_handle.h
//  Date :     2018-1-12 9:51
//  Version :  V0001
//  History :  ��ʼ�����汾
//******************************************************************************
#ifndef _KEY_HANDLE_H
#define _KEY_HANDLE_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "gpio.h"
  
  //���尴����ʶ
#define KEY_UP     0x01
#define KEY_LEFT   0x02
#define KEY_DOWN   0x04
#define KEY_RIGHT  0x08

//���尴���ص�����ָ��
typedef void (*key_cb)(uint8_t pin);
  
//**************************************
// fn : KEY_RegisterCb
//
// brief : ע�ᰴť�¼��ص�
//
// param : cb -> ����ť�¼�����ָ��
//
// return : none
void KEY_RegisterCb(key_cb cb);

//**************************************
// fn : KEY_Poll
//
// brief : ��ѯ��ť�¼�
//
// param : none
//
// return : none
void KEY_Poll(void);

#ifdef __cplusplus
}
#endif
#endif   //_KEY_HANDLE_H