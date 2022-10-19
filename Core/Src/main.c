/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEACT_SPEED 2.5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
struct Motor
{
	uint16_t Encode;
	float T_speed;
	float error_speed;
	float error_add;
	float error_last;
	float real_speed;
	int32_t T_duty;
	
}M1,M2,M3,M4;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Stop(void);

void qianjingezi(uint32_t gezi_num);
void car_run(int32_t LQ, int32_t RQ, int32_t LH, int32_t RL);  //左前 右前 左后 右后
void car_control(float x, float y, float w);            //X轴 Y轴 角速度

void zuozhuan(void);
void youzhuan(void);
void CeJu(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
enum RUN_FLAG{
    STPO_FLAG = 0,
    QIANJIN,
    ZUOZHUAN,
    YOUZHUAN,
    HOUTUI
}run_flag;
uint32_t RUN_FLAG = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);   //M2
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);    //M3
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);	
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);     //M4
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);     //M1
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);	
	
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
    //qianjingezi(5);
     HAL_TIM_Base_Start_IT(&htim10);
	  car_control(3,0,0);

		while(1);
		
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#include "stdlib.h"

void car_run(int32_t LQ, int32_t RQ, int32_t LH, int32_t RH)
{
    if(LQ >= 0)
    {
        TIM8->CCR1 = LQ;        //M3  			前左轮
        TIM8->CCR2 = 0;
    }
    else
    {
        TIM8->CCR1 = 0;         //M3  			前左轮
        TIM8->CCR2 = abs(LQ);
    }
    
    if(RQ >= 0)
    {
        TIM8->CCR3 = 0;        //M4  			前右轮
        TIM8->CCR4 = RQ;
    }
    else
    {
        TIM8->CCR3 = abs(RQ);  //M4  			前右轮
        TIM8->CCR4 = 0;
    }
    if(LH >= 0)
    {
        TIM5->CCR3 = LH;        //M2  			后左轮
        TIM5->CCR4 = 0;
    }
    else
    {
        TIM5->CCR3 = 0;         //M2  			后左轮
        TIM5->CCR4 = abs(LH);
    }
    if(RH >= 0)
    {
        TIM5->CCR1 = 0;        	//M1  			后右轮
        TIM5->CCR2 = RH;
    }
    else
    {
        TIM5->CCR1 = abs(RH);   //M1  			后右轮
        TIM5->CCR2 = 0;
    }
}

void car_control(float x, float y, float w)
{
    M3.T_speed = x + y - w;
    M4.T_speed = x - y + w;
    M2.T_speed = x - y - w;
    M1.T_speed = x + y + w;
    
    car_run( M3.T_speed, M4.T_speed,M2.T_speed, M1.T_speed);
}


void Stop(void)
{	  
    HAL_TIM_Base_Stop_IT(&htim10); 
    run_flag = STPO_FLAG;
    
    TIM5->CCR1 = 1000;
    TIM5->CCR2 = 1000;
    TIM5->CCR3 = 1000;
    TIM5->CCR4 = 1000;
    
    TIM8->CCR1 = 1000;
    TIM8->CCR2 = 1000;
    TIM8->CCR3 = 1000;
    TIM8->CCR4 = 1000;  
}

uint32_t gezi_flag = 0;
void qianjingezi(uint32_t gezi_num)
{
    HAL_TIM_Base_Start_IT(&htim10);
    run_flag = QIANJIN;
    while(gezi_num)
    { 
        HAL_Delay(300);
        gezi_flag = 0;
        while(gezi_flag == 0)
                HAL_Delay(1);
        gezi_num --;
    }
    Stop();
}

uint32_t xuanzhuan_flag = 0;
void zuozhuan()
{
    run_flag = ZUOZHUAN;
    car_control(0, 0, 2.5);
    HAL_Delay(300);
    HAL_TIM_Base_Start_IT(&htim10); 
    xuanzhuan_flag = 1;
    while(xuanzhuan_flag == 0)
        HAL_Delay(1);
    HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
    Stop();
}

void youzhuan()
{
    run_flag = YOUZHUAN;
    car_control(0, 0, -2.5);
    HAL_Delay(300);
    HAL_TIM_Base_Start_IT(&htim10); 
    xuanzhuan_flag = 1;
    while(xuanzhuan_flag == 0)
        HAL_Delay(1);
    HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
    Stop();
}

void Xunji()
{
	
//	static uint32_t heixian_flag;
    uint32_t qian[7]; int32_t qian_num , qian_flag = 0;
    qian[0]=HAL_GPIO_ReadPin(qian_1_GPIO_Port,qian_1_Pin);
    qian[1]=HAL_GPIO_ReadPin(qian_2_GPIO_Port,qian_2_Pin);
    qian[2]=HAL_GPIO_ReadPin(qian_3_GPIO_Port,qian_3_Pin);
    qian[3]=HAL_GPIO_ReadPin(qian_4_GPIO_Port,qian_4_Pin);
    qian[4]=HAL_GPIO_ReadPin(qian_5_GPIO_Port,qian_5_Pin);
    qian[5]=HAL_GPIO_ReadPin(qian_6_GPIO_Port,qian_6_Pin);
		qian[6]=HAL_GPIO_ReadPin(qian_7_GPIO_Port,qian_7_Pin);
    
    qian_num = qian[0] +qian[1] +qian[2] +qian[3] +qian[4] +qian[5] +qian[6];
    if ( qian_num == 1)             //如果只有一个循迹在线上
    {
        for(uint8_t i = 0; i<=6; i++)
        {
            if(qian[i] == 1)
                qian_flag = i;       //记录第几个在线上
        }
    }
	
	
		uint32_t hou[7]; int32_t hou_num , hou_flag = 0;
		hou[6]=HAL_GPIO_ReadPin(hou_1_GPIO_Port,hou_1_Pin);
    hou[5]=HAL_GPIO_ReadPin(hou_2_GPIO_Port,hou_2_Pin);
    hou[4]=HAL_GPIO_ReadPin(hou_3_GPIO_Port,hou_3_Pin);
    hou[3]=HAL_GPIO_ReadPin(hou_4_GPIO_Port,hou_4_Pin);
    hou[2]=HAL_GPIO_ReadPin(hou_5_GPIO_Port,hou_5_Pin);
    hou[1]=HAL_GPIO_ReadPin(hou_6_GPIO_Port,hou_6_Pin);
    hou[0]=HAL_GPIO_ReadPin(hou_7_GPIO_Port,hou_7_Pin);
    
    hou_num = hou[0] +hou[1] +hou[2] +hou[3] +hou[4] +hou[5] +hou[6];
    if ( hou_num == 1)             //如果只有一个循迹在线上
    {
        for(uint8_t i = 0; i<=6; i++)
        {
            if(hou[i] == 1)
                hou_flag = i;       //记录第几个在线上
        }
    }
    
    uint32_t zuo[7]; int32_t zuo_num , zuo_flag = 0;
    
		zuo[6]=HAL_GPIO_ReadPin(zuo_1_GPIO_Port,zuo_1_Pin);
    zuo[5]=HAL_GPIO_ReadPin(zuo_2_GPIO_Port,zuo_2_Pin);
    zuo[4]=HAL_GPIO_ReadPin(zuo_3_GPIO_Port,zuo_3_Pin);
    zuo[3]=HAL_GPIO_ReadPin(zuo_4_GPIO_Port,zuo_4_Pin);
    zuo[2]=HAL_GPIO_ReadPin(zuo_5_GPIO_Port,zuo_5_Pin);
    zuo[1]=HAL_GPIO_ReadPin(zuo_6_GPIO_Port,zuo_6_Pin);
    zuo[0]=HAL_GPIO_ReadPin(zuo_7_GPIO_Port,zuo_7_Pin);
    zuo_num = zuo[0] +zuo[1] +zuo[2] +zuo[3] +zuo[4] +zuo[5] +zuo[6];
    if ( zuo_num == 1)             //如果只有一个循迹在线上
    {
        for(uint8_t i = 0; i<=6; i++)
        {
            if(zuo[i] == 1)
                zuo_flag = i;       //记录第几个在线上
        }
    }
    
    uint32_t you[7]; int32_t you_num , you_flag = 0;
    you[0]=HAL_GPIO_ReadPin(you_1_GPIO_Port,you_1_Pin);
    you[1]=HAL_GPIO_ReadPin(you_2_GPIO_Port,you_2_Pin);
    you[2]=HAL_GPIO_ReadPin(you_3_GPIO_Port,you_3_Pin);
    you[3]=HAL_GPIO_ReadPin(you_4_GPIO_Port,you_4_Pin);
    you[4]=HAL_GPIO_ReadPin(you_5_GPIO_Port,you_5_Pin);
    you[5]=HAL_GPIO_ReadPin(you_6_GPIO_Port,you_6_Pin);
	you[6]=HAL_GPIO_ReadPin(you_7_GPIO_Port,you_7_Pin);
    
    you_num = you[0] +you[1] +you[2] +you[3] +you[4] +you[5] +you[6];
    if ( you_num == 1)             //如果只有一个循迹在线上
    {
        for(uint8_t i = 0; i<=6; i++)
        {
            if(you[i] == 1)
                you_flag = i;       //记录第几个在线上
        }
    }
    

    
    if(run_flag == QIANJIN)
    {
/******************单独前循迹纠偏*******************/

        if(qian[3] == 1)        
            car_control(DEACT_SPEED, 0, 0);
        if(qian[2] == 1 && qian[4] == 0)
            car_control(DEACT_SPEED, -0.3, 0.2);
        else if(qian[4] == 1 && qian[2] == 0)
            car_control(DEACT_SPEED, 0.3, -0.2);
        
        if(qian[1] == 1 && qian[5] == 0)
            car_control(DEACT_SPEED - 0.3, -0.5, 0.3);
        if(qian[5] == 1 && qian[1] == 0)
            car_control(DEACT_SPEED - 0.3, 0.5, -0.3);
        
        if(qian[0] == 1 && qian[6] == 0)
            car_control(DEACT_SPEED - 0.5, -0.5, 0.54);
        if(qian[6] == 1 && qian[0] == 0)
            car_control(DEACT_SPEED - 0.5, 0.5, -0.4);
        
        if((qian[2] == 1 && qian[3] == 1 && qian[4] == 1) )   //前后循迹在线上则不纠偏
        {
            car_control(DEACT_SPEED, 0, 0);
        }
        if((qian[2] == 1 && qian[3] == 1 && qian[4] == 1))      //小车在十字中心处
        {
            gezi_flag = 1;
        }

/************************************************/ 
        
/******************前后双循迹纠偏*******************/
//      int32_t y_fix, w_fix, rec_speed;
//      if (qian_num == 1 && hou_num ==1)
//      {
//        if (qian_flag + hou_flag >= 5 && qian_flag + hou_flag <= 7)
//            y_fix = 0;
//        if (qian_flag + hou_flag < 5 && qian_flag + hou_flag >= 3)
//            y_fix = -50;
//        if (qian_flag + hou_flag > 7 && qian_flag + hou_flag <= 9)
//            y_fix = 50;
//        if (qian_flag + hou_flag < 3)
//            y_fix = 100;
//        if (qian_flag + hou_flag > 9)
//            y_fix = -100;
//        
//        if (qian_num - hou_num == 0)
//            w_fix = 0;
//        if (qian_num - hou_num > 0 && qian_num - hou_num <= 2)
//            w_fix = -50;
//        if (qian_num - hou_num < 0 && qian_num - hou_num >= -2)
//            w_fix = 50;
//        if (qian_num - hou_num > 2)
//				{
//            w_fix = -100;
//						rec_speed = -100;
//				}
//        if (qian_num - hou_num < -2)
//				{
//            w_fix = 100;
//						rec_speed = -100;
//				}
//        car_control(DEACT_SPEED + rec_speed, y_fix, w_fix);
//      }
//      else
//          car_control(DEACT_SPEED, 0, 0);
//      if((qian[2] == 1 && qian[3] == 1 && qian[4] == 1) || (hou[2] == 1 && hou[3] == 1 && hou[4] == 1))   //前后循迹在线上则不纠偏
//      {
//          car_control(DEACT_SPEED, 0, 0);
//      }
//      if((zuo[2] == 1 || zuo[3] == 1 || zuo[4] == 1) && (you[2] == 1 || you[3] == 1 || you[4] == 1))      //小车在十字中心处
//      {
//        gezi_flag = 1;
//      }

/************************************************/  
    }
    else if(run_flag == ZUOZHUAN || run_flag == YOUZHUAN)
    {
        if((zuo[3] == 1 || zuo[2] == 1)  &&  (you[3] == 1 ||you[3] == 1 ))      //小车与十垂直
        {
            xuanzhuan_flag = 1;
        }
    }
}





#define KP 150
#define KI 125
#define KD 25
		
void PID()
{

		M1.error_speed = M1.T_speed - M1.real_speed;
		M1.error_add += M1.error_speed;
		M1.T_duty = M1.error_speed * KP + M1.error_add * KI + (M1.error_speed - M1.error_last) * KD;
		M1.error_last = M1.error_speed;
	
		M2.error_speed = M2.T_speed - M2.real_speed;
		M2.error_add += M2.error_speed;
		M2.T_duty = M2.error_speed * KP + M2.error_add * KI + (M2.error_speed - M2.error_last) * KD;
		M2.error_last = M2.error_speed;
	
		M3.error_speed = M3.T_speed - M3.real_speed;
		M3.error_add += M3.error_speed;
		M3.T_duty = M3.error_speed * KP + M3.error_add * KI + (M3.error_speed - M3.error_last) * KD;
		M3.error_last = M3.error_speed;
	
		M4.error_speed = M4.T_speed - M4.real_speed;
		M4.error_add += M4.error_speed;
		M4.T_duty = M4.error_speed * KP + M4.error_add * KI + (M4.error_speed - M4.error_last) * KD;
		M4.error_last = M4.error_speed;
	
		car_run(M3.T_duty,M4.T_duty,M2.T_duty,M1.T_duty);
}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&htim10))
    {
    Xunji();
	static uint32_t flag = 0;
	if(flag<10)
	flag++;
	else
	{
	flag=0;
	CeJu();
	PID();
	}
        
    }
}



void CeJu()
{
	M1.Encode=__HAL_TIM_GET_COUNTER(&htim1);
	M2.Encode=__HAL_TIM_GET_COUNTER(&htim2);
	M3.Encode=__HAL_TIM_GET_COUNTER(&htim3);
	M4.Encode=__HAL_TIM_GET_COUNTER(&htim4);
	
	__HAL_TIM_SET_COUNTER(&htim1,0);
	__HAL_TIM_SET_COUNTER(&htim2,0);
	__HAL_TIM_SET_COUNTER(&htim3,0);
	__HAL_TIM_SET_COUNTER(&htim4,0);
	
	if (M1.Encode<32768)
	{
	  M1.real_speed=-M1.Encode/1340.0*6*3.14;
		
	}
	else 
	{
	   M1.real_speed=-(M1.Encode-65536)/1340.0*6*3.14;
	}			
	
	if (M2.Encode<32768)
	{
	  M2.real_speed=-M2.Encode/1340.0*6*3.14;
	}
	else 
	{
	   M2.real_speed=-(M2.Encode-65536)/1340.0*6*3.14;
	}			

	if (M3.Encode<32768)
	{
	  M3.real_speed=M3.Encode/1340.0*6*3.14;
	}
	else 
	{
	   M3.real_speed=(M3.Encode-65536)/1340.0*6*3.14;
	}			
	
	if (M4.Encode<32768)
	{
	  M4.real_speed=-M4.Encode/1340.0*6*3.14;
	}
	else 
	{
	   M4.real_speed=-(M4.Encode-65536)/1340.0*6*3.14;
	}			
	
   printf("SPEED1=%f\r\n",M1.real_speed);
   printf("SPEED2=%f\r\n",M2.real_speed);
   printf("SPEED3=%f\r\n",M3.real_speed);
   printf("SPEED4=%f\r\n",M4.real_speed);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
