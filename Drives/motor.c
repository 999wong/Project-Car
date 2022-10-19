 #include"stdio.h"   
 
		
void CeJu()
{
		uint16_t  Encode_1;
		uint16_t  Encode_2;
		uint16_t  Encode_3;
		uint16_t  Encode_4;
		float M1_Speed,M2_Speed,M3_Speed,M4_Speed;
	
		Encode_1=__HAL_TIM_GET_COUNTER(&htim1);
		Encode_2=__HAL_TIM_GET_COUNTER(&htim2);
		Encode_3=__HAL_TIM_GET_COUNTER(&htim3);
		Encode_4=__HAL_TIM_GET_COUNTER(&htim4);
		
		__HAL_TIM_SET_COUNTER(&htim1,0);
		__HAL_TIM_SET_COUNTER(&htim2,0);
		__HAL_TIM_SET_COUNTER(&htim3,0);
		__HAL_TIM_SET_COUNTER(&htim4,0);
		
		printf("Encode_1:   %d   \n",Encode_1);
		printf("Encode_2:   %d   \n",Encode_2);
		printf("Encode_3:   %d   \n",Encode_3);
		printf("Encode_4:   %d   \n",Encode_4);
		
		HAL_Delay(1000);
		
		if (Encode_1<32768)
		{
		  M1_Speed=Encode_1/1340.0*6*3.14;
			
		}
		else 
    {
		   M1_Speed=(Encode_1-65536)/1340.0*6*3.14;
			
		}			
		
		if (Encode_2<32768)
		{
		  M2_Speed=-Encode_2/1340.0*6*3.14;
			
		}
		else 
    {
		   M2_Speed=-(Encode_2-65536)/1340.0*6*3.14;
			
		}			
	
		if (Encode_3<32768)
		{
		  M3_Speed=Encode_3/1340.0*6*3.14;
			
		}
		else 
    {
		   M3_Speed=(Encode_3-65536)/1340.0*6*3.14;
			
		}			
		
		if (Encode_4<32768)
		{
		  M4_Speed=Encode_4/1340.0*6*3.14;
			
		}
		else 
    {
		   M4_Speed=(Encode_4-65536)/1340.0*6*3.14;
			
		}			
		
	   
		printf("M1_位移:   %f    cm \n",M1_Speed);
	  printf("M2_位移:   %f    cm \n",M2_Speed);
		printf("M3_位移:   %f    cm \n",M3_Speed);
		printf("M4_位移:   %f    cm \n",M4_Speed);
		
	}

	
	
		
		
		
		
		
		
		