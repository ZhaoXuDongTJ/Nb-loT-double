//******************************************************************************
//                                www.ghostyu.com
//
//                 Copyright (c) 2017-2018, WUXI Ghostyu Co.,Ltd.
//                                All rights reserved.
//
//  FileName : json_format.c
//  Date :     2018-03-01 22:03
//  Version :  V0001
//  History :  ��ʼ�����汾
//******************************************************************************
#include <stdio.h>
#include "json_format.h"

//******************************************************************************
// fn : JSON_Ad_value
//
// brief : ���߱��Ĺ�ǿֵת������Ӧ��json��ʽ
//
// param : buf ->���json��ʽ���ݵ�ַ
//         tempValue -> ����Ĺ�ǿֵ
//
// return : ת��֮��json���ݳ���
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