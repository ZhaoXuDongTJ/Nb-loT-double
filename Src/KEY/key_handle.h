//******************************************************************************
//                                www.ghostyu.com
//
//                 Copyright (c) 2017-2018, WUXI Ghostyu Co.,Ltd.
//                                All rights reserved.
//
//  FileName : key_handle.h
//  Date :     2018-1-12 9:51
//  Version :  V0001
//  History :  初始创建版本
//******************************************************************************
#ifndef _KEY_HANDLE_H
#define _KEY_HANDLE_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "gpio.h"
  
  //定义按键标识
#define KEY_UP     0x01
#define KEY_LEFT   0x02
#define KEY_DOWN   0x04
#define KEY_RIGHT  0x08

//定义按键回调函数指针
typedef void (*key_cb)(uint8_t pin);
  
//**************************************
// fn : KEY_RegisterCb
//
// brief : 注册按钮事件回调
//
// param : cb -> 处理按钮事件函数指针
//
// return : none
void KEY_RegisterCb(key_cb cb);

//**************************************
// fn : KEY_Poll
//
// brief : 轮询按钮事件
//
// param : none
//
// return : none
void KEY_Poll(void);

#ifdef __cplusplus
}
#endif
#endif   //_KEY_HANDLE_H