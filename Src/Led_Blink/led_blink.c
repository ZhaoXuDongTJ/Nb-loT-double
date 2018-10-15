//******************************************************************************
//                                www.ghostyu.com
//
//                 Copyright (c) 2017-2018, WUXI Ghostyu Co.,Ltd.
//                                All rights reserved.
//
//  FileName : led_blink.c
//  Date :     2018-1-09 10:40
//  Version :  V0001
//  History :  初始创建版本
//******************************************************************************

#include "led_blink.h"

//******************************************************************************
// typedef and const area
//******************************************************************************
#define HAL_LED_MODE_OFF   0x00
#define HAL_LED_MODE_ON    0x01
#define HAL_LED_BLINK      0x80

typedef struct
{
  uint8_t  led_mode;    //工作模式
  uint8_t  led_todo;    //剩余闪烁次数
  uint8_t  led_duty;    //占空比
  uint32_t led_next;    // 一次更改状态
  uint32_t led_period;  //周期
}HalLedCtl_t;

#define LED_WAIT_ATTACH_TIME    1000
#define LED_WAIT_ATTACH_DUTY    50

#define LED_NET_ATTACH_TIME     2000
#define LED_NET_ATTACH_DUTY     2

#define LED_DATA_TIME           200
#define LED_DATA_DUTY           10

#define LED_WAIT_NET            LED_INDEX_0
#define LED_ATTACH_NET          LED_INDEX_1
#define LED_NET_DATA            LED_INDEX_2
//******************************************************************************
//控制LED优先级，下标数值大的优先级高

static HalLedCtl_t  NBEK_Led[3];
static uint8_t      highPriority;

//*********************************************
// fn : HAL_LedCtl_Init
//
// brief : 初始化Led控制结构
//
// param : none
//
// return : none
void HAL_LedCtl_Init(void)
{
  //等待附着网络优先使能
  NBEK_Led[0].led_mode = HAL_LED_BLINK|HAL_LED_MODE_OFF;
  NBEK_Led[0].led_duty = LED_WAIT_ATTACH_DUTY;
  NBEK_Led[0].led_next = 0;
  NBEK_Led[0].led_todo = 0;
  NBEK_Led[0].led_period = LED_WAIT_ATTACH_TIME;
  
  NBEK_Led[1].led_mode = HAL_LED_MODE_OFF;
  NBEK_Led[1].led_duty = LED_NET_ATTACH_DUTY;
  NBEK_Led[1].led_next = 0;
  NBEK_Led[1].led_todo = 0;
  NBEK_Led[1].led_period = LED_NET_ATTACH_TIME; 
  
  NBEK_Led[2].led_mode = HAL_LED_MODE_OFF;
  NBEK_Led[2].led_duty = LED_DATA_DUTY;
  NBEK_Led[2].led_next = 0;
  NBEK_Led[2].led_todo = 0;
  NBEK_Led[2].led_period = LED_DATA_TIME;
}
//*********************************************
// fn : HAL_Led_Set
//
// brief : 设定Led状态
//
// param : state -> set or reset
//
// return : none
static void HAL_Led_Set(GPIO_PinState state)
{
  HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,state);
}
//*******************************************************************************
// fn : HalLedBlinkClear
//
// brief :对led 的状态进行设定
//
// param : [in] -> ledStatus :表示设定的led闪烁状态
//
// return :none
//******************************************************************************
static void HAL_LedBlinkClear(uint8_t ledStatus)
{
  uint8_t high;
  
  if(ledStatus > LED_INDEX_2 )
  {
      //表示状态指示值超出设定的范围
      return ;
  }
  
  NBEK_Led[ledStatus].led_mode = 0;
  high = LED_INDEX_0;
  
  if(highPriority == ledStatus)
  {
    for(uint8_t i = 0 ; i <= LED_INDEX_2 ; i++)
    {
      if((NBEK_Led[i].led_mode & HAL_LED_BLINK) > 0)
      {
        if(i > high)
        {
          high = i;
          break;
        }
      }
    }
    highPriority = high;
  }
}
//*********************************************
// fn : HalLedUpdate
//
// brief : 更新
//
// param : none
//
// return : none
static uint32_t HalLedUpdate(void)
{
  uint32_t time = 0;
  uint32_t wait = 0;
  uint8_t pct = 0;
  HalLedCtl_t *ledcs = &NBEK_Led[highPriority];
  if(ledcs->led_mode & HAL_LED_BLINK)
  {
    time = HAL_GetTick();
    if(time >= ledcs->led_next)
    {
      if(ledcs->led_mode & HAL_LED_MODE_ON)
      {
        pct = 100 - ledcs->led_duty;
        ledcs->led_mode &=  ~HAL_LED_MODE_ON; //更改为MODE_OFF

        HAL_Led_Set(GPIO_PIN_RESET);
        if(ledcs->led_todo > 0)
        {
          if(--ledcs->led_todo == 0)
          {
            //do nothing
            //ledcs->led_mode = 0;
            HAL_LedBlinkClear(highPriority);
          }
        }
      }
      else
      {
        pct = ledcs->led_duty;                 //Percentage of cycle for on 
        ledcs->led_mode |= HAL_LED_MODE_ON;    // Say it's on
        HAL_Led_Set(GPIO_PIN_SET); 
      }
      
      if(ledcs->led_mode & HAL_LED_BLINK)
      {
        ledcs->led_next = time + ((pct * ledcs->led_period ) / 100);
        wait = ledcs->led_next - time;
      }  
    }
    else
    {
      wait = ledcs->led_next - time;
    }
  }
  
  if(wait > 0)
  {
    //do somthing
  }
  return wait;
}

//*********************************************
// fn : HAL_Led_poll
//
// brief : 轮询LED 状态
//
// param : none
//
// return : none
void HAL_Led_poll(void)
{
  HalLedUpdate();
}

//*********************************************
// fn : HAL_LED_Blink
//
// brief : 设置LED 闪烁信息
//
// param : time -> 周期
//         onpct -> 占空比
//         times -> 闪烁次数。0表示无限次
//
// return : none
void HAL_LED_Blink(uint32_t time,uint8_t onpct,uint8_t times,led_index_e index)
{
  if(time < 5)
  {
    return;
  }
  
  if(index > LED_INDEX_2)
  {
    return;
  }
  NBEK_Led[index].led_period = time;
  NBEK_Led[index].led_duty = onpct;
  NBEK_Led[index].led_todo = times;
  NBEK_Led[index].led_mode = HAL_LED_BLINK|HAL_LED_MODE_OFF;
}
//*********************************************
// fn : HAL_LED_SET
//
// brief : 控制LED 闪烁
//
// param : index -> 对应优先级的LED 控制结构
//         times -> 闪烁次数
//
// return : none
void HAL_LED_SET(led_index_e index, uint8_t times)
{
  if(index > LED_INDEX_2)
  {
    return;
  }
  
  if(index == LED_NET_DATA )
  {
    if(times == 0)
    {
      times = 1;
    }
  } 
  
  NBEK_Led[index].led_todo = times;
  NBEK_Led[index].led_mode = HAL_LED_BLINK|HAL_LED_MODE_OFF;
  
  if(highPriority < index)
  {
    highPriority = index;
  }
}