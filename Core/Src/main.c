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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LQ_FIX 0
#define RQ_FIX 0
#define LH_FIX 0
#define RH_FIX 0

#define DEACT_SPEED 700

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void qianjin(uint32_t speed, uint32_t left_duty, uint32_t right_duty);
void houtui(uint32_t speed);
void Zuo_Guai(uint32_t speed);
void You_Guai(uint32_t speed);
void Zuo_WeiYi(uint32_t speed);
void You_WeiYi(uint32_t speed);
void Stop(void);

void qianjingezi(uint32_t gezi_num);
void car_run(int32_t LQ, int32_t RQ, int32_t LH, int32_t RL);  //左前 右前 左后 右后
void car_control(int32_t x, int32_t y, int32_t w);             //X轴 Y轴 角速度

void 		zuozhuan();
	void			youzhuan();
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);   //M2
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);    //M3
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);	
	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);     //M4
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);     //M1
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
		qianjingezi(6);
		zuozhuan();
		qianjingezi(6);
		
		zuozhuan();
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
        __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1, LQ);   	        //M3  			前左轮
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2, 0);
    }
    else
    {
        __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1, 0);   	        //M3  			前左轮
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2, abs(LQ));
    }
    if(RQ >= 0)
    {
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3, 0);    			//M4   			前右轮
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4, RQ);	  
    }
    else
    {
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3, abs(RQ));    	//M4   			前右轮
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4, 0);	  
    }
    if(LH >= 0)
    {
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3, LH);  	        //M2        后左轮    
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4, 0); 
    }
    else
    {
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3, 0);  	        //M2        后左轮    
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4, abs(LH));  
    }
    if(RH >= 0)
    {
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1, 0);    			//M1     		后右轮
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2, RH);	
    }
    else
    {
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1, abs(RH));        //M1     		后右轮
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2, 0);	
    }
}

void car_control(int32_t x, int32_t y, int32_t w)
{
    int32_t lq, rq, lh, rh;
    lq = x + y - w + LQ_FIX;
    rq = x - y + w + LQ_FIX;
    lh = x - y - w + LQ_FIX;
    rh = x + y + w + LQ_FIX;
    
    car_run(lq, rq, lh, rh);
}


void Stop(void)
{	  
        HAL_TIM_Base_Stop_IT(&htim10); 
        run_flag = STPO_FLAG;
        __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,1000);  //M2           
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,1000); 
		
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,1000);   //M3
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,1000);
	
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,1000);    //M4
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,1000);	  
		
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,1000);    //M1
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,1000);                
		
}

uint32_t gezi_flag = 0;
void qianjingezi(uint32_t gezi_num)
{
		while(gezi_num)
		{
				HAL_TIM_Base_Start_IT(&htim10); 
				run_flag = QIANJIN;
				car_control(DEACT_SPEED, 0, 0);
				gezi_flag = 0;
				HAL_Delay(300);
				while(gezi_flag == 0)
						HAL_Delay(1);
				gezi_num --;
				Stop();
				HAL_Delay(1000);
				
		}
    Stop();

}

uint32_t xuanzhuan_flag = 0;
void zuozhuan()
{
    run_flag = ZUOZHUAN;
    car_control(0, 0, 700);
    HAL_Delay(200);
    HAL_TIM_Base_Start_IT(&htim10); 
    while(xuanzhuan_flag == 0)
        HAL_Delay(1);
    HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
    Stop();
}

void youzhuan()
{
    run_flag = YOUZHUAN;
    car_control(0, 0, -700);
    HAL_Delay(200);
    HAL_TIM_Base_Start_IT(&htim10); 
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
            car_control(DEACT_SPEED, -50, 20);
        else if(qian[4] == 1 && qian[2] == 0)
            car_control(DEACT_SPEED, 50, -20);
        
        if(qian[1] == 1 && qian[5] == 0)
            car_control(DEACT_SPEED - 50, -100, 50);
        if(qian[5] == 1 && qian[1] == 0)
            car_control(DEACT_SPEED - 50, 100, -50);
        
        if(qian[0] == 1 && qian[6] == 0)
            car_control(DEACT_SPEED - 100, -100, 70);
        if(qian[6] == 1 && qian[0] == 0)
            car_control(DEACT_SPEED - 100, 100, -70);
        
        if((qian[2] == 1 && qian[3] == 1 && qian[4] == 1) )   //前后循迹在线上则不纠偏
        {
            car_control(DEACT_SPEED, 0, 0);
        }
        if(qian[2] == 1 && qian[3] == 1 && qian[4] == 1)      //小车在十字中心处
        {

            gezi_flag = 1;
        }

/************************************************/ 
        
/******************前后双循迹纠偏*******************/
//      int32_t y_fix, w_fix;
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
//            w_fix = -100;
//        if (qian_num - hou_num < -2)
//            w_fix = 100;
//        car_control(DEACT_SPEED, y_fix, w_fix);
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
        if((zuo[2] == 1 || zuo[3] == 1 || zuo[4] == 1) && (you[2] == 1 || you[3] == 1 || you[4] == 1))      //小车与十垂直
        {
            xuanzhuan_flag = 1;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&htim10))
    {
        Xunji();
        
    }
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
