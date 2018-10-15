//******************************************************************************
//                                www.ghostyu.com
//
//                 Copyright (c) 2017-2018, WUXI Ghostyu Co.,Ltd.
//                                All rights reserved.
//
//  FileName : json_format.c
//  Date :     2018-03-01 22:03
//  Version :  V0001
//  History :  初始创建版本
//******************************************************************************
#include <stdio.h>
#include "json_format.h"

//******************************************************************************
// fn : JSON_Ad_value
//
// brief : 将具本的光强值转换成相应的json格式
//
// param : buf ->存放json格式数据地址
//         tempValue -> 具体的光强值
//
// return : 转换之后json数据长度
//******************************************************************************

uint16_t JSON_Ad_value(char* buf,float adValue)
{
	uint16_t msgLen = 0;
	if(buf == NULL)
	{
		return 0;
	}
	msgLen = sprintf(buf,"{\"dataType\":\"Ad\",\"data\":\"%0.1f\"}",adValue);
	return msgLen;
}