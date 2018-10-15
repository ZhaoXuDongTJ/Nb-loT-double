/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "lptim.h"
#include "usart.h"
#include "spi.h"
#include "gpio.h"
#include "json_format.h"

/* USER CODE BEGIN Includes */
#include "Display.h"
#include "DisplayTFT128.h"
#include "TFT128.h"

#include "led_blink.h"
#include "key_handle.h"

#include "NB_Board.h"
#include "NB_BC95.h"
#include "NB_Board_Cfg.h"
#include "timer_user_poll.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define LCD_FIRST_LINE  2

#define LCD_LAST_LINE   7

#define JSON_BUF_LEN    100
    
typedef enum
{
  NB_NONE,
  NB_INIT,          // 初始化操作
  NB_MODULE,        // 获取NB模块厂商及固件，频段等信息
  NB_SIGN,
  NB_UDP_CR,        // 创建UDP
  NB_UDP_CL,        // 关闭UDP
  NB_UDP_REG,       // 此状态用于注册谷雨云平台信息（如果不与谷雨云平台通信，可忽略）
  NB_UDP_ST,        // 利用已经创建的UDP发送数据
  NB_UDP_RE,        // UDP接收信息
  NB_CoAP_SEVER,    // CoAP远程地址设置与获取
  NB_CoAP_ST,       // 利用CoAP发送消息
  NB_CoAP_RE,       // CoAP返回信息
  NB_RESET,         // 复位NB
  NB_END
}NB_STATE_e;

volatile NB_STATE_e  APP_STATE= NB_NONE;
int NB_MsgreportCb(msg_types_t,int ,char*);

uint8_t* NB_Module_IMEI = NULL;

//记录NB模块初始化流程顺序
#define  NB_SEQ_INIT        0x01
#define  NB_SEQ_UDP_CR      0X02
#define  NB_SET_UDP_REG     0x04

uint8_t  seq_record = 0; //标志登记变量

//管理json数据  光强数据
typedef struct
{
	char json_buf[JSON_BUF_LEN];  //用于存放json格式数据
	uint16_t json_len;            //有效数据长度
}json_info_t;

json_info_t json_info;

// 是否开启呼吸灯
uint8_t  is_rgb_pwm = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

//******************************************************************************
// fn : isGetTempTime
//
// brief : 可以读取温度值吗
//
// param : none
//
// return : true -> yes; false -> wait
uint8_t isGetTempTime(void);
void App_setReport(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//******************************************************************************
// fn : AppKey_cb
//
// brief : 处理按键事件
//
// param : key -> 按钮按下标识
//
// return : none
void AppKey_cb(uint8_t key);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* LCD Handle*/
Display_Handle  dispHandle;
/* LCD object */
LCD_Object lcdObject;

/* LCD hardware attribute structure */
const LCD_HWAttrs lcdHWAttrs = {
    .LCD_initCmd = &LCD_initCmd,
    .lcdModePin  = {LCD_CD_Pin,GPIOA},      /* LCD mode pin  */
    .lcdCsnPin   = {LCD_CS_Pin,GPIOA},       /* LCD CSn pin   */
    .spiIndex    = SPI1
};

/* LCD configuration structure */
const LCD_Config LCD_config = {
    .object  = &lcdObject,
    .hwAttrs = &lcdHWAttrs
};


/* Structures for tft1.44 */
DisplayTFT128_Object  displayTft128Object;

const DisplayTFT128_HWAttrs displayTft128HWattrs = {
    .lcdHandle = (LCD_Handle)& LCD_config,
    .powerPin  = {LCD_BL_Pin,GPIOB}
};

/* Array of displays */
const Display_Config Display_config[] = 
{
  {
    .fxnTablePtr = &DisplayTFT128_fxnTable,
    .object      = &displayTft128Object,
    .hwAttrs     = &displayTft128HWattrs  
  },

    { NULL, NULL, NULL } // Terminator
};
//******************************************************************************
// fn : LCD_Print 
//
// brief : 向屏打印信息
//
// param : str -> 字符串格式
//         a   -> 参数
// 
// return : none
void LCD_Print(char *str,char *a)
{ 
  static uint8_t line = LCD_FIRST_LINE;
  
  if(line > LCD_LAST_LINE)
  {
    Display_clearLines(dispHandle,LCD_FIRST_LINE,LCD_LAST_LINE);
    line = LCD_FIRST_LINE;
  }
  if(line < LCD_FIRST_LINE)
  {
    line = LCD_FIRST_LINE;
  }
  if(a == NULL)
  {
      Display_print0(dispHandle,line++,0,str);
  }
  else
  {
    Display_print1(dispHandle,line++,0,str,a);
  }
 
}


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  Color fontColor = GREEN;
  seq_record = 0;
  volatile uint16_t pwm_value = 0;
  volatile int16_t step = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_LPTIM1_Init();

  /* USER CODE BEGIN 2 */
  
  Bsp_init();
  HAL_LedCtl_Init();
  dispHandle = Display_open(0,NULL);
  Display_control(dispHandle,CMD_FRONT_COLOR,&fontColor);
  //显示测试信息
  Display_print0(dispHandle,0,2,"Ghostyu NB");
  Display_print0(dispHandle,1,0,"Net:OFF");
  Display_print0(dispHandle,1,8,"---dbm");
  
  LCD_Print("S1 to Init NB",NULL);
  //打开串口接收
  KEY_RegisterCb(AppKey_cb);
  NBModule_open(&nb_config);
  APP_STATE = NB_NONE;
  // 设置呼吸灯
  HAL_LPTIM_PWM_Start(&hlptim1, 99,0);
  user_pwm_setvalue(99); 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    HAL_UART_Poll();
    KEY_Poll();
    HAL_Led_poll();
    NBModule_Main(&nb_config);
    MX_TimerPoll();
    switch(APP_STATE)
    {
    case NB_NONE:
      {
        //wait for key
        
      }
      break;
    case NB_INIT:
      {
        printf("\r\n<----BC95 Init---->\r\n");
        LCD_Print("Init start...",NULL);
        seq_record = 0;
        NBModule_Init(&nb_config);
        APP_STATE = NB_END;
      }
      break;
    case NB_SIGN:
      {
         printf("\r\n<----BC95 get signal---->\r\n");
         NBModule_Sign(&nb_config);
         APP_STATE = NB_END;
      }
      break;
    case NB_MODULE:
      {
        printf("\r\n<----Module info ---->\r\n");
        LCD_Print("Module info...",NULL);
        NBModule_Info(&nb_config);
        APP_STATE = NB_END;
      }
      break;
    case NB_UDP_CR:
      {
        printf("\r\n<----Create udp ---->\r\n");
        LCD_Print("UDP Create...",NULL);
        NBModule_CreateUDP(&nb_config);
        APP_STATE = NB_END;
      }
      break;
    case NB_UDP_CL:
      {
        printf("\r\n<----Close udp ---->\r\n");
        NBModule_CloseUDP(&nb_config);
        APP_STATE = NB_END;
      }
      break;
    case NB_UDP_REG:
      {
        //发送注册包
        //如果不与谷雨云平台通信，则不需要发送注册包
        printf("\r\n<----NB registes to Cloud ---->\r\n");
        LCD_Print("Udp send...",NULL);
        
        char regPacket[30];
        uint8_t msgLen = 0;
        
        //利用库函数进行合格化数据
        msgLen = sprintf(regPacket,"ep=%s&pw=123456",NB_Module_IMEI);
        regPacket[msgLen] = 0;
        
        //char* regPacket = "ep=863703036005069&pw=123456";
        NBModule_SendData(&nb_config,msgLen,regPacket);
        seq_record |= NB_SET_UDP_REG;
        LCD_Print("S3 send UDP",NULL);
        APP_STATE = NB_END;
      }
      break;
    case NB_UDP_ST:
      {
        printf("\r\n<---- Send udp ---->\r\n");
        LCD_Print("Udp send...",NULL);
        
        if(json_info.json_len)
		{
            NBModule_SendData(&nb_config,json_info.json_len,json_info.json_buf);
		}
        
        APP_STATE = NB_END;
      }
      break;
      
    case NB_UDP_RE:
      {
        
      }
      break;
    default:
      {
      }
      break;
    }
    // 呼吸灯程序；
   if(is_rgb_pwm){
       HAL_Delay(10);
       if(pwm_value == 0) step = 1;
       if(pwm_value == 99) step = -1;
       pwm_value += step;
       user_pwm_setvalue(pwm_value); 
    }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure LSE Drive Capability 
    */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_LPUART1
                              |RCC_PERIPHCLK_LPTIM1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_LSE;
  PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_PCLK;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

//******************************************************************************
// fn : isGetTempTime
//
// brief : 可以读取温度值吗
//
// param : none
//
// return : true -> yes; false -> wait
uint8_t isGetTempTime(void)
{
  static uint32_t start_tick = 0;
  uint32_t cur_tick = 0;
  uint8_t  readFlag = 0;
  
	//定时3秒
  cur_tick = HAL_GetTick();
  if(cur_tick >= start_tick )
  {
    if(cur_tick - start_tick >= 3000)
    {
      readFlag = 1;
    }
    else
    {
      readFlag = 0;
    }
  }
  else
  {
    if((UINT32_MAX - start_tick + cur_tick) >= 3000)
    {
      readFlag = 1;
    }
    else
    {
      readFlag = 0;
    }
  }
  
  if(readFlag)
  {
    start_tick = cur_tick;
  } 
  return readFlag;
}
//******************************************************************************
// fn : App_setReport
//
// brief : 设置发送状态
//
// param : none
//
// return : none
void App_setReport(void){
    float temp = 0.0;
    if(isGetTempTime())
	{
        temp = HAL_ADC_Read();
		json_info.json_len = JSON_Ad_value(json_info.json_buf,temp);
		Display_print1(dispHandle,2,0,"Temp = %0.1fC   ",temp);
        APP_STATE = NB_UDP_ST;   //置udp发送
    }
}

//******************************************************************************
// fn : AppKey_cb
//
// brief : 处理按键事件
//
// param : key -> 按钮按下标识
//
// return : none
void AppKey_cb(uint8_t key)
{
  //闪烁LED,增加交互性
 HAL_LED_SET(LED_INDEX_2,1);
  if(key & KEY_UP)
  {
    printf("key_up press\r\n");
    APP_STATE = NB_INIT;
  }
  if(key & KEY_LEFT)
  {
    printf("key_left press\r\n");
    if(seq_record & NB_SEQ_INIT)
    {
      APP_STATE = NB_MODULE;
    }
  }
  if(key & KEY_DOWN)
  {
    printf("key_down press\r\n");
    if(seq_record & NB_SEQ_INIT)  //只有初始化成功，才能进行UDP创建工作
    {
      if(seq_record & NB_SEQ_UDP_CR)
      {
        if(seq_record & NB_SET_UDP_REG )
        {
          App_setReport();
          //APP_STATE = NB_UDP_ST;
        }
      }
      else
      {
        APP_STATE = NB_UDP_CR;
      }
    }
  }
  if(key & KEY_RIGHT)
  {
    printf("key_right press\r\n");
    if(seq_record & NB_SEQ_UDP_CR)
    {
      //表示UDP已经创建好，可以进行数据收发工作

      APP_STATE = NB_UDP_REG;
      
    }
    else
    {
      //给出提示
      LCD_Print("S3 Create UDP",NULL);
    }
    
  }
}

//******************************************************************************
// fn : NB_MsgreportCb
//
// brief : NB模块消息上报回调
//
// param : 
//
// return : none
int  NB_MsgreportCb(msg_types_t types,int len,char* msg)
{
  volatile uint16_t pwm_values = 0;
  switch(types)
  {
  case MSG_INIT:
    {
      printf("\r\nINIT=%s\r\n",msg);
      LCD_Print("Init=%s",msg);
      if(*msg == 'S')
      {
        HAL_LED_SET(LED_INDEX_1,0);
        Display_print0(dispHandle,1,0,"NET=ON");
        APP_STATE = NB_SIGN;
        seq_record |= NB_SEQ_INIT;  //记录初始化成功
        LCD_Print("S2 to get info",NULL);
        LCD_Print("S3 to Create UDP",NULL);
      }
      else
      {
        printf("Reinit NB with S1\r\n");
         LCD_Print("Reinit NB...",NULL);
      }
    }
    break;
  case MSG_IMSI:
    {
      printf("\r\nIMSI=%s\r\n",msg);
      LCD_Print(msg,NULL);
    }
    break;
  case MSG_REG:
    {
      Display_print1(dispHandle,1,0,"NET=%s",(*msg) == 1 ?"ON":"0FF");
    }
    break;
  case MSG_SIGN:
    {
      Display_print1(dispHandle,1,8,"%sdbm",msg);
    }
    break;
  case MSG_MODULE_INFO:
    {
      printf("\r\nMinfo=%s\r\n",msg);
      LCD_Print("Info=%s",msg);
    }
    break;
  case MSG_MID:
    {
      printf("\r\nMID=%s\r\n",msg);
      LCD_Print(msg,NULL);
    }
    break;
  case MSG_MMODEL:
    {
      printf("\r\nModel=%s\r\n",msg);
      LCD_Print(msg,NULL);
    }
    break;
  case MSG_MREV:
    {
      printf("\r\nREV=%s\r\n",msg);
      LCD_Print(msg,NULL);
    }
    break;
  case MSG_BAND:
    {
      printf("\r\nFreq=%s\r\n",msg);
      LCD_Print("Freq=%s",msg);
    }
    break;
  case MSG_IMEI:
    {
      printf("\r\nIMEI=%s\r\n",msg);
      //保存IMEI信息
      NB_Module_IMEI = (uint8_t*)msg;
      LCD_Print(msg,NULL);
    }
    break;
  case MSG_UDP_CREATE:
    {
      printf("\r\nUDP_CR=%s\r\n",msg);
      LCD_Print("UDP=%s",msg);
      if(*msg == 'S')  //"S" 表示创建成功，'F'表示失败
      {
        seq_record |= NB_SEQ_UDP_CR;  //记录初始化成功
        
        printf("S4 to register iotxx Cloud\r\n");
        LCD_Print("S4 reg Packet",NULL);
      }
      else
      {
        printf("Please, recreate udp\r\n");
        LCD_Print("Recreate UDP",NULL);
      }
    }
    break;
  case MSG_UDP_CLOSE:
    {
      printf("\r\nUDP_CL=%s\r\n",msg);

      if(*msg == 'S')
      {
        //清除UDP 创建成功标志
        seq_record &= ~NB_SEQ_UDP_CR;
      }
    }
    break;
  case MSG_UDP_SEND:
    {
      printf("\r\nUDP_SEND=%s\r\n",msg);
      LCD_Print("Send=%s",msg);
    }
    break;
  case MSG_UDP_RECE:
    {
      printf("\r\nUDP_RECE=%s\r\n",msg);
      LCD_Print(msg,NULL);
      if(msg[0] == 'C' || msg[0] == 'c' || sizeof(msg) == 2){
          if(msg[1] == '1')
            {
                LCD_Print("lamp open",NULL);
                is_rgb_pwm = 1;
            }else if (msg[1] == '0'){
                LCD_Print("lamp close",NULL);
                is_rgb_pwm = 0;
                user_pwm_setvalue(97);
            }
      }else if (msg[0] == 'S' || msg[0] == 's'){
        if(sizeof(msg) == 3){
            is_rgb_pwm = 0;
          user_pwm_setvalue(99);
          pwm_values = msg[1]-48;
          printf("\r\nssssssss %d\r\n",pwm_values);
          user_pwm_setvalue(pwm_values);
        }else if(sizeof(msg) == 4){
          is_rgb_pwm = 0;
          user_pwm_setvalue(99);
          uint16_t aa = msg[1]-48;
          uint16_t bb = msg[2]-48;
          printf("\r\nssssssss %d\r\n",pwm_values);
          pwm_values = aa+aa+aa+aa+aa+aa+aa+aa+aa+aa+bb;
          printf("\r\nssssssss %d\r\n",aa);
          printf("\r\nssssssss %d\r\n",bb);
          printf("\r\nssssssss %d\r\n",pwm_values);
          user_pwm_setvalue(pwm_values);
        }
          
      }
    }
    break;
  case MSG_COAP:
    {
      printf("\r\nCOAP=%s\r\n",msg);
    }
    break;
  case MSG_COAP_SEND:
    {
      printf("\r\nCOAP_SENT=%s\r\n",msg);
    }
    break;
    
  case MSG_COAP_RECE:
    {
      printf("\r\nCOAP_RECE=%s\r\n",msg);
    }
    break;
  default :
    {
      break;
    }
  }
  return 0;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
