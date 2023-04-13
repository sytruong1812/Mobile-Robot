/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "stdio.h"
#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/// lidar
typedef enum _status_t
{
  // for transmittion
  TX_IDLE,
  TX_BUSY,
  TX_DONE,
  // for receiving
  RX_IDLE,
  RX_BUSY,
  RX_DONE,
  // for memory copy
  MEM_IDLE,
  MEM_BUSY,
  MEM_DONE
}status_t;

typedef struct _buffer_t
{
  uint8_t *buf;
  uint32_t size;  
}buffer_t;

#define BUFFER_SIZE 16

// receiving buffers
uint8_t rxBuf_1[BUFFER_SIZE];
uint8_t rxBuf_2[BUFFER_SIZE];
// switched receiving buffer list
buffer_t rxList[2] = { {rxBuf_1, sizeof(rxBuf_1)}, {rxBuf_2, sizeof(rxBuf_2)} };
// index for list switching
uint8_t rxIdx = 0;
// receiving flag
volatile status_t rxFlag = RX_IDLE;

// transmit buffers
uint8_t txBuf_1[BUFFER_SIZE];
uint8_t txBuf_2[BUFFER_SIZE];
// switched tramsmit buffer list
buffer_t txList[2] = { {txBuf_1, sizeof(txBuf_1)}, {txBuf_2, sizeof(txBuf_2)} };
// index for list switching
uint8_t txIdx = 0;
// transmitting flag
volatile status_t txFlag = TX_IDLE;

// memory buffer
volatile status_t memFlag = MEM_IDLE;

//#define DEBUG_ON

// start scan command
uint8_t scan[]={0xA5,0x20};
/// lidar

int gioihan=0;
int s[5]={452,56,348,100,0};			//Push cho wheel right
int s1[5]={452,-56,348,100,0};		//push cho wheel left
float v_trai=0;
float v_phai=0;
int vitris[5];
int vitris1[5];
unsigned short MAX_tangtoc=2500;
unsigned short MIN_tangtoc=2000;
unsigned short MAX_giamtoc=2000;
unsigned short MIN_giamtoc=1600;
int l=0;
int l1=0;
int f=0;
int l2=0;
int j=0;
int i=0;
int k=0;
int a=0;
int votlo=60;
float gocqueo_trai=0;
float gocqueo_phai=0;
float heso_kc=0.00072;
float phi_trai_truoc=0;
float phi_trai_sau=0;
float phi_phai_truoc=0;
float phi_phai_sau=0;
float R=3.25;
float phi_trai=0;
float phi_phai=0;
float w_trai=0;
float w_phai=0;
float w_save_trai=0;
float w_save_phai=0;
float x_save=0;
float y_save=0;
float x[5];
float y[5];
float v=0;
float theta=0;
float theta1;
float theta2;
char Rx_data[1];
char buff[50];
float tyletruyen=1/30;
float beta_truc_trai=0;
float beta_truc_phai=0;
float beta_banhxich_trai=0;
float beta_banhxich_phai=0;
int soxunghientai_banhtrai=0;
int soxunghientai_banhphai=0;
float L_trai=0;
float L_phai=0;
float L_giua=0;
int soxungcap_queotrai=0;
int soxungcap_queophai=0;
unsigned short sovongcanquay_trai=0;
unsigned short sovongcanquay_phai=0;

int encoder1=0;
int encoder2=0;
int encodervitri=0;
int last_encoder1=0;
int last_encoder2=0;
unsigned short a1=0;
unsigned short a2=0;
unsigned short a11=0;
unsigned short a21=0;
uint8_t receive_data='0';
uint8_t receive_data1[10]="sang";
uint32_t adc_val[2];
float vol_val[2];
int c=0;
int c1=0;
int biennho=0;
int biennho1=0;
int encoder_xung=0;
int encoder_xung1=0;
unsigned short MAX=1800;
unsigned short MIN =1500;
unsigned short tocdodat=1500;
unsigned short tocdothuc=0;
unsigned short tocdodat1=1500;
unsigned short tocdothuc1=0;
int vitridat=0;
int vitridat1=0;
unsigned int vitrithuc=0;
unsigned int vitrithuc1=0;
long last=0;
long last1=0;
long last2=0;
float sailech=0;
////pid lien tuc
unsigned short output =0;
unsigned short lastoutput =0;
float Kp=0.05;
float Kd=0.016485;
float Ki=0.005;
long T=25;
int PIDlientuc_vantoc(int output, int tocdodat, int tocdothuc);
unsigned short error1 =0;
unsigned short last_error1 =0;
#ifdef __GNUC__
			#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
			#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE		
{
	HAL_UART_Transmit(&huart2,(uint8_t *)&ch,1,0xFFFF);
	return ch;
}
// pid roi rac
int PIDroirac_vantoc_trai(int output_roirac, int tocdodat, int tocdothuc);
unsigned short output_roirac =0;
unsigned short lastoutput1_roirac =0;
float Kp_roirac=0.03;
float Kd_roirac=0.00009;
float Ki_roirac=0.000045;
float alpha=0;
float beta=0;
float gama=0;
long T1=15;
unsigned short error1_roirac =0;
unsigned short error1_roirac1 =0;
unsigned short error1_roirac2 =0;

//// pid roi rac
//int PIDroirac_vantoc_phai(int output_roirac2, int tocdodat1, int tocdothuc1);
//unsigned short output_roirac2 =0;
//unsigned short lastoutput1_roirac1 =0;
//float Kp_roirac1=0.015;
//float Kd_roirac1=0.0005;
//float Ki_roirac1=0.00008;
//float alpha11=0;
//float beta11=0;
//float gama11=0;
//unsigned short error1_roirac3 =0;
//unsigned short error1_roirac4 =0;
//unsigned short error1_roirac5 =0;

// pid vi tri và pid van toc
int PIDroirac_vantoc_phai(int output_roirac1, int tocdodat1, int tocdothuc1);
unsigned short output_roirac1 =0;
unsigned short lastoutput1_roirac1 =0;
float Kp_roirac1=0.03;
float Kd_roirac1=0.00009;
float Ki_roirac1=0.000045;
float alpha1=0;
float beta1=0;
float gama1=0;
long T2=15;
unsigned short error1_vitri =0;
unsigned short error1_vitri1 =0;
unsigned short error1_vitri2 =0;
unsigned short tinhtoan=0;
unsigned short tinhtoan1=0;
unsigned short tinhtoans=0;
unsigned short tinhtoan1s=0;
// pid roi rac bu
int PIDroirac_bu(int output_bu, int output_roirac, int output_vitri);
unsigned short output_bu =0;
unsigned short lastoutput1_bu =0;
float Kp_bu=0.05;
float Kd_bu=0.008;
float Ki_bu=0.0009;
float alpha_bu=0;
float beta_bu=0;
float gama_bu=0;
long T3=15;
unsigned short error1_bu =0;
unsigned short error1_bu1 =0;
unsigned short error1_bu2 =0;

// ENCODER
void HAL_TIM_CaptureCallback_Encoder(TIM_HandleTypeDef *htim1,TIM_HandleTypeDef *htim2)
{
		encoder2=__HAL_TIM_GET_COUNTER(htim1);
		encoder1=__HAL_TIM_GET_COUNTER(htim2);
}
void HAL_TIM_CaptureCallback_Encoder_STOP(TIM_HandleTypeDef *htim1,TIM_HandleTypeDef *htim2)
{
		__HAL_TIM_SET_COUNTER(htim1,0);
		__HAL_TIM_SET_COUNTER(htim2,0);
}
// ADC
void ADC_Giatri()
{
		HAL_ADC_Start(&hadc1);
		for(i=0;i<29;i++) c++;
		adc_val[0]=HAL_ADC_GetValue(&hadc1);
		for(i=0;i<57;i++) c++;
		adc_val[1]=HAL_ADC_GetValue(&hadc1);

		vol_val[0]=adc_val[0]*3.3/4096;
		vol_val[1]=adc_val[1]*3.3/4096;
}
// DIEU KHIEN DONG CO
typedef enum
{
	MOTOR_STOP,
	MOTOR_CW,	//cung chieu kim dong ho
	MOTOR_CCW	//nguoc chieu kim dong ho
}MotorState;

void pwm_set_duty(TIM_HandleTypeDef *tim, uint16_t channel, uint8_t duty)
{
	uint16_t ccr = (uint16_t)duty*(tim->Instance->ARR)/100;
	switch(channel)
	{
		case TIM_CHANNEL_1:
			tim->Instance->CCR1 = ccr;
			break;
		case TIM_CHANNEL_2:
			tim->Instance->CCR2 = ccr;
			break;
		case TIM_CHANNEL_3:
			tim->Instance->CCR3 = ccr;
			break;
		case TIM_CHANNEL_4:
			tim->Instance->CCR4 = ccr;
			break;
	}
}
// KIEU PUL/DIR
//speed 0->100 phan tram
void control_motor1(MotorState state, uint8_t speed)
{
	switch(state)
	{
		case MOTOR_STOP:
			HAL_GPIO_WritePin(MOTOR1_IO_GPIO_Port,MOTOR1_IO_Pin,GPIO_PIN_RESET);
//			TIM1->CCR1 = 0;
			pwm_set_duty(&htim4,TIM_CHANNEL_1,0);
			break;
		case MOTOR_CW:
			HAL_GPIO_WritePin(MOTOR1_IO_GPIO_Port,MOTOR1_IO_Pin,GPIO_PIN_RESET);
//			TIM1->CCR1 = (uint16_t)speed*999/100;
			pwm_set_duty(&htim4,TIM_CHANNEL_1,speed);
			break;
		case MOTOR_CCW:
			HAL_GPIO_WritePin(MOTOR1_IO_GPIO_Port,MOTOR1_IO_Pin,GPIO_PIN_SET);
//			TIM1->CCR1 = (uint16_t)speed*999/00;
			pwm_set_duty(&htim4,TIM_CHANNEL_1,speed);
			break;
	}
}
void control_motor2(MotorState state, uint8_t speed)
{
	switch(state)
	{
		case MOTOR_STOP:
			HAL_GPIO_WritePin(MOTOR2_IO_GPIO_Port,MOTOR2_IO_Pin,GPIO_PIN_RESET);
//			TIM1->CCR1 = 0;
			pwm_set_duty(&htim4,TIM_CHANNEL_2,0);
			break;
		case MOTOR_CW:
			HAL_GPIO_WritePin(MOTOR2_IO_GPIO_Port,MOTOR2_IO_Pin,GPIO_PIN_RESET);
//			TIM1->CCR1 = (uint16_t)speed*999/100;
			pwm_set_duty(&htim4,TIM_CHANNEL_2,speed);
			break;
		case MOTOR_CCW:
			HAL_GPIO_WritePin(MOTOR2_IO_GPIO_Port,MOTOR2_IO_Pin,GPIO_PIN_SET);
//			TIM1->CCR1 = (uint16_t)speed*999/100;
			pwm_set_duty(&htim4,TIM_CHANNEL_2,speed);
			break;
	}

}
// KIEU 2 KENH PUL
typedef enum
{
	MOTOR_STOP_2CHANEL,
	MOTOR_CW_CCW_2CHANEL
}MotorState_2CHANEL;
void control_motor1_2chanel_1(MotorState_2CHANEL state_2CHANEL, uint8_t speed1, uint8_t speed2)
{
	switch(state_2CHANEL)
	{
		case MOTOR_STOP_2CHANEL:
			HAL_GPIO_WritePin(enapwm_GPIO_Port,enapwm_Pin,GPIO_PIN_SET);
//			TIM1->CCR1 = 0;
			pwm_set_duty(&htim4,TIM_CHANNEL_1,0);
			pwm_set_duty(&htim4,TIM_CHANNEL_2,0);
			break;
		case MOTOR_CW_CCW_2CHANEL:
			HAL_TIM_CaptureCallback_Encoder(&htim2,&htim5);		
			HAL_GPIO_WritePin(enapwm_GPIO_Port,enapwm_Pin,GPIO_PIN_SET);
//			TIM1->CCR1 = 0;
			pwm_set_duty(&htim4,TIM_CHANNEL_1,speed1);
			pwm_set_duty(&htim4,TIM_CHANNEL_2,speed2);
			break;
	}
}
void control_motor1_2chanel_2(MotorState_2CHANEL state_2CHANEL, uint8_t speed1, uint8_t speed2)
{
	switch(state_2CHANEL)
	{
		case MOTOR_STOP_2CHANEL:
			HAL_GPIO_WritePin(enapwm_GPIO_Port,enapwm_Pin,GPIO_PIN_SET);
//			TIM1->CCR1 = 0;
			pwm_set_duty(&htim3,TIM_CHANNEL_3,0);
			pwm_set_duty(&htim3,TIM_CHANNEL_4,0);
			break;
		case MOTOR_CW_CCW_2CHANEL:
			HAL_TIM_CaptureCallback_Encoder(&htim2,&htim5);		
			HAL_GPIO_WritePin(enapwm_GPIO_Port,enapwm_Pin,GPIO_PIN_SET);
//			TIM1->CCR1 = 0;
			pwm_set_duty(&htim3,TIM_CHANNEL_1,speed1);
			pwm_set_duty(&htim3,TIM_CHANNEL_2,speed2);
			break;
	}
}
void donghoc()
{
		w_save_phai=w_phai;
		w_save_trai=w_trai;
//		w_save_trai=w_trai[0]+w_trai[1]+w_trai[2];
//		w_save_phai=w_phai[0]+w_phai[1]+w_phai[2];
		v=(float)(R*(w_save_phai+w_save_trai)/2);					//CT 2.1
		theta=(float)(R*(w_save_phai-w_save_trai)/37);   //CT 2.6
		if(((s[i]>0&&s1[j]>0)||(s[i]<0&&s1[j]<0))&&l==0)
		{
			x[k]=v*cos(theta);			//CT 2.4
			y[k]=v*sin(theta);			//CT 2.5
		}
		if(((s[i]>0&&s1[j]>0)||(s[i]<0&&s1[j]<0))&&l==2)
		{
			y[k]=v*cos(theta);
			x[k]=v*sin(theta);
			if((vitrithuc==vitridat)&&(vitrithuc1==vitridat1)) l=0;
		}
		if(((s[i]>0&&s1[j]>0)||(s[i]<0&&s1[j]<0))&&l==1)
		{
			y[k]=-v*cos(theta);
			x[k]=v*sin(theta);
			if((vitrithuc==vitridat)&&(vitrithuc1==vitridat1)) l=0;
		}
		if(s[i]>0&&s1[j]<0)
		{
			y[k]=-v*cos(theta);
			x[k]=v*sin(theta);
			l=1;
		}
		if(s[i]<0&&s1[j]>0)
		{
			y[k]=v*cos(theta);
			x[k]=v*sin(theta);
			l=2;
		}
		x_save=x[0]+x[1]+x[2]+x[3]+x[4];
		y_save=y[0]+y[1]+y[2]+y[3]+y[4];
		
//		w_save_trai=w_trai[0]+w_trai[1]+w_trai[2];
//		w_save_phai=w_phai[0]+w_phai[1]+w_phai[2];
//		v=(float)(R*(w_save_trai+w_save_phai)/2);
//		theta=(float)(R*(w_save_trai-w_save_phai)/37);
//		x=v*cos(theta);
//		y=v*sin(theta);
	
//			v=(float)((R*(w_phai+w_trai))/2);
//			L_giua = (float)((L_phai+L_trai)/2);
//			theta1 = (float)((L_phai-L_trai)/(18.5))*180/3.14;
//			theta2 = (float)theta1 - theta;
//			theta = theta2;
//			x = (float)L_giua*cos(theta);
//			y = (float)L_giua*sin(theta);
		
//int s[5]={452,56,348,100,0};			//Push cho wheel right
//int s1[5]={452,-56,348,100,0};		//Push cho wheel left
				soxungcap_queotrai=(s[i]*360)/0.45;			
				soxungcap_queophai=(s1[j]*360)/0.45;
//CT 3.6 tính nguoc lai soxungcap và tinh dong hoc gocqueo   
				gocqueo_trai=(float)(soxungcap_queotrai/4.5924)*((9*3.25)/(5*18.85))/38.1;			
				gocqueo_phai=(float)(soxungcap_queophai/4.5924)*((9*3.25)/(5*18.85))/38.1;
	
}
void stop_motor()
{
	HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
}
void start_motor()
	{
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
}
typedef enum
{
	MOTOR_sovong_1,
	MOTOR_sovong_2
}MotorState_sovong;

int PIDroirac_vantoc_trai(int output_roirac, int tocdodat, int tocdothuc)
{
	if(tocdodat>tocdothuc) error1_roirac=abs(tocdodat)-abs(tocdothuc);
	if(tocdodat<tocdothuc) error1_roirac=abs(tocdothuc)-abs(tocdodat);

	alpha=2*T1*Kp_roirac+ Ki_roirac*T1*T1+ 2*Kd_roirac;
	beta=T1*T1*Ki_roirac - 4*Kd_roirac- 2*T1*Kp_roirac;
	gama= 2*Kd_roirac;
	
	output_roirac=(alpha*error1_roirac+beta*error1_roirac1+gama*error1_roirac2+2*T1*lastoutput1_roirac)/(2*T1);
	
	lastoutput1_roirac=output_roirac;
	error1_roirac1=error1_roirac;
	error1_roirac2=error1_roirac1;
		
	if(output_roirac<votlo)
		{
			if(output_roirac<=0)
			{
				output_roirac=0;
			}
			else
			{
				output_roirac=output_roirac;
			}
			return output_roirac;
		}
		else if(output_roirac==0)
		{
				output_roirac=10;
				return output_roirac;
		}
		else
		{
			error1_roirac=(tocdodat-tocdothuc)/20;
		}
	}

int PIDroirac_vantoc_phai(int output_roirac1, int tocdodat1, int tocdothuc1)
{
	if(tocdodat1>tocdothuc1) error1_vitri=abs(tocdodat1)-abs(tocdothuc1);
	if(tocdodat1<tocdothuc1) error1_vitri=abs(tocdothuc1)-abs(tocdodat1);
	
	alpha=(2*T2*Kp_roirac1)+ (Ki_roirac1*T2*T2)+ (2*Kd_roirac1);
	beta=(T2*T2*Ki_roirac1) - (4*Kd_roirac1)- (2*T2*Kp_roirac1);
	gama= 2*Kd_roirac1;
	output_roirac1=(alpha*error1_vitri + beta*error1_vitri1 + gama*error1_vitri2+2*T2*lastoutput1_roirac1)/(2*T2);
	lastoutput1_roirac1=output_roirac1;
	error1_vitri1=error1_vitri;
	error1_vitri2=error1_vitri1;
	if(output_roirac1<votlo)
	{
		if(output_roirac1<=0)
		{
			output_roirac1=0;
		}
		else
		{
			output_roirac1=output_roirac1;
		}
	return output_roirac1;
	}
	else if(output_roirac1==0)
	{
			output_roirac1=10;
			return output_roirac1;
	}
	else
	{
		error1_vitri=(tocdodat1-tocdothuc1)/20;
	}

}
void PID_DONGCOTRAI_timer_2_4(int gocquay_mongmuon1, char chieuquay_DC1)
{
	if(HAL_GetTick() - last >= T1)
    {
        if(chieuquay_DC1=='t')
        {
            vitridat=gocquay_mongmuon1*360;
            last = HAL_GetTick();
            encoder_xung=__HAL_TIM_GET_COUNTER(&htim2);				
            c=encoder_xung+c;
            vitrithuc=c*0.45;       //(1xung = độ)0.45 = 360/800  (PPR = 800 -> 1 vòng  = 800 xung) đổi vòng ra độ
            tinhtoan=vitridat*0.1;
            __HAL_TIM_SET_COUNTER(&htim2,0);
        
            tocdothuc=(encoder_xung)* (60 * (1000/T1)) / (4*200);
        
            output_roirac=PIDroirac_vantoc_trai(output_roirac, tocdodat, tocdothuc);
            if(output_roirac>=0)
            {
                if((vitrithuc<(vitridat-tinhtoan))&&vitrithuc<vitridat)
                {
                        tocdodat=tocdodat+5;
                        MAX=MAX+5;
                        MIN=MIN+5;
                        if(MAX>=MAX_tangtoc) MAX=MAX_tangtoc;
                        if(MIN>=MIN_tangtoc) MIN=MIN_tangtoc;
                        if(tocdodat>=MAX) tocdodat=MAX;
                        control_motor1(MOTOR_CW,output_roirac);		
                }
                if((vitrithuc>=(vitridat-tinhtoan))&&(vitrithuc<vitridat))
                {
                        tocdodat=tocdodat-5;
                        MAX=MAX-5;
                        MIN=MIN-5;
                        if(MAX<=MAX_giamtoc) MAX=MAX_giamtoc;
                        if(MIN<=MIN_giamtoc) MIN=MIN_giamtoc;
                        if(tocdodat<=MIN) tocdodat=MIN;
                        control_motor1(MOTOR_CW,output_roirac);
                }
                if(vitrithuc>=(vitridat-10))
                {
                    
                    control_motor1(MOTOR_CW,1);			
                }		
                soxunghientai_banhtrai=vitrithuc/0.45;
                L_trai=(float)soxunghientai_banhtrai*heso_kc;
                phi_trai=(float)((180*L_trai)/(3.14*R));	
                w_trai=(float)(((phi_trai)/15)*(0.0333*0.7857));
            }
        
            else
            {
                control_motor1(MOTOR_CW,1);		; // de dong co khong chay
            }
        }
                
        if(chieuquay_DC1=='n')
        {
            vitridat=gocquay_mongmuon1*360;
            last =HAL_GetTick();
            encoder_xung=__HAL_TIM_GET_COUNTER(&htim2);				
            c=encoder_xung+c;
            vitrithuc=c*-0.45;
            tinhtoan=vitridat*0.1;
            __HAL_TIM_SET_COUNTER(&htim2,0);
            tocdothuc=((abs(encoder_xung))* (60 * (1000/T1))) / (4*200);
            output_roirac=PIDroirac_vantoc_trai(output_roirac, tocdodat, tocdothuc);

            if(output_roirac>=0)
            {
                if((vitrithuc<(vitridat-tinhtoan))&&vitrithuc<vitridat)
                {
                    tocdodat=tocdodat+5;
                    MAX=MAX+5;
                    MIN=MIN+5;
                    if(MAX>=MAX_tangtoc) MAX=MAX_tangtoc;
                    if(MIN>=MIN_tangtoc) MIN=MIN_tangtoc;
                    if(tocdodat>=MAX) tocdodat=MAX;
                    control_motor1(MOTOR_CCW,output_roirac);		
                }
                if((vitrithuc>=(vitridat-tinhtoan))&&(vitrithuc<vitridat))
                {
                    tocdodat=tocdodat-5;
                    MAX=MAX-5;
                    MIN=MIN-5;
                    if(MAX<=MAX_giamtoc) MAX=MAX_giamtoc;
                    if(MIN<=MIN_giamtoc) MIN=MIN_giamtoc;
                    if(tocdodat<=MIN) tocdodat=MIN;
                    control_motor1(MOTOR_CCW,output_roirac);
                }

                if(vitrithuc>=(vitridat-10))
                {
                    control_motor1(MOTOR_CW,1);			
                }		
                soxunghientai_banhtrai=vitrithuc/0.45;
                L_trai=(float)soxunghientai_banhtrai*-heso_kc;
                phi_trai=(float)((180*L_trai)/(3.14*R));	
                w_trai=(float)(((phi_trai)/15)*(0.0333*0.7857));
            }
            else
            {
                control_motor1(MOTOR_CW,1); // de dong co khong chay
            }
            
        }
    }
}
void PID_DONGCOPHAI_timer_3_4(int gocquay_mongmuon_2,char chieuquay_DC2)
{
	if(HAL_GetTick() - last1 >=T2)
			{
				if(chieuquay_DC2=='t')
				{
						vitridat1=gocquay_mongmuon_2*360;
						last1 =HAL_GetTick();
						encoder_xung1=__HAL_TIM_GET_COUNTER(&htim5);				
						c1=encoder_xung1+c1;
						vitrithuc1=c1*0.45;
						tinhtoan1=vitridat1*0.1;
						__HAL_TIM_SET_COUNTER(&htim5,0);
						tocdothuc1=((encoder_xung1)* (60 * (1000/T2))) / (4*200);
						output_roirac1=PIDroirac_vantoc_phai(output_roirac1, tocdodat1, tocdothuc1);
						if(output_roirac1>=0)
						{
								if((vitrithuc1<(vitridat1-tinhtoan1))&&vitrithuc1<vitridat1)
								{
										tocdodat1=tocdodat1+5;
										MAX=MAX+5;
										MIN=MIN+5;
										if(MAX>=MAX_tangtoc) MAX=MAX_tangtoc;
										if(MIN>=MIN_tangtoc) MIN=MIN_tangtoc;
										if(tocdodat1>=MAX) tocdodat1=MAX;
									control_motor2(MOTOR_CW,output_roirac1);		
								}
								if((vitrithuc1>=(vitridat1-tinhtoan1))&&(vitrithuc1<vitridat1))
								{
										tocdodat1=tocdodat1-5;
										MAX=MAX-5;
										MIN=MIN-5;
										if(MAX<=MAX_giamtoc) MAX=MAX_giamtoc;
										if(MIN<=MIN_giamtoc) MIN=MIN_giamtoc;
										if(tocdodat1<=MIN) tocdodat1=MIN;
										control_motor2(MOTOR_CW,output_roirac1);		
								}
								if(vitrithuc1>=(vitridat1-10))
								{
									control_motor2(MOTOR_CW,1);		

								}
								soxunghientai_banhphai=vitrithuc1/0.45;
								L_phai=(float)soxunghientai_banhphai*heso_kc;
								phi_phai=(float)((180*L_phai)/(3.14*R));
								w_phai=(float)(((phi_phai)/15)*(0.0333*0.7857));
						}
						else
						{
							control_motor2(MOTOR_CW,1);	; // de dong co khong chay
						}
						
				}
				if(chieuquay_DC2=='n')
				{
					vitridat1=gocquay_mongmuon_2*360;
					last1 =HAL_GetTick();
					encoder_xung1=__HAL_TIM_GET_COUNTER(&htim5);				
					c1=encoder_xung1+c1;
					vitrithuc1=c1*-0.45;
					tinhtoan1=vitridat1*0.1;
					__HAL_TIM_SET_COUNTER(&htim5,0);
					tocdothuc1=((abs(encoder_xung1))* (60 * (1000/T2))) / (4*200);
					output_roirac1=PIDroirac_vantoc_phai(output_roirac1, tocdodat1, tocdothuc1);
					if(output_roirac1>=0)
					{
									if((vitrithuc1<(vitridat1-tinhtoan1))&&vitrithuc1<vitridat1)
							{
										tocdodat1=tocdodat1+5;
										MAX=MAX+5;
										MIN=MIN+5;
										if(MAX>=MAX_tangtoc) MAX=MAX_tangtoc;
										if(MIN>=MIN_giamtoc) MIN=MIN_giamtoc;
										if(tocdodat1>=MAX) tocdodat1=MAX;
										control_motor2(MOTOR_CCW,output_roirac1);		
							}
							if((vitrithuc1>=(vitridat1-tinhtoan1))&&(vitrithuc1<vitridat1))
							{
										tocdodat1=tocdodat1-5;
										MAX=MAX-5;
										MIN=MIN-5;
										if(MAX<=MAX_giamtoc) MAX=MAX_giamtoc;
										if(MIN<=MIN_giamtoc) MIN=MIN_giamtoc;
										if(tocdodat1<=MIN) tocdodat1=MIN;
										control_motor2(MOTOR_CCW,output_roirac1);		
							}
							if(vitrithuc1>=(vitridat1-10))
								{
									control_motor2(MOTOR_CW,1);		
								}
							soxunghientai_banhphai=vitrithuc1/0.45;
							L_phai=(float)soxunghientai_banhphai*-heso_kc;
							phi_phai=(float)((180*L_phai)/(3.14*R));
							w_phai=(float)(((phi_phai)/15)*(0.0333*0.7857));
					}
					else
					{
						control_motor2(MOTOR_CW,1);	 // de dong co khong chay
					}
				
				}
			}
}
int PIDroirac_bu(int output_bu, int vitrithuc, int vitrithuc1)
{
	if(vitrithuc>vitrithuc1) error1_bu=abs(vitrithuc)-abs(vitrithuc1);		//Ham abs() tra ve gia tri tuyet doi abs(-6)=6
	if(vitrithuc<vitrithuc1) error1_bu=abs(vitrithuc1)-abs(vitrithuc);
	
	alpha_bu=2*T3*Kp_bu+ Ki_bu*T3*T3+ 2*Kd_bu;
	beta_bu=T3*T3*Ki_bu - 4*Kd_bu- 2*T2*Kp_bu;
	gama_bu= 2*Kd_bu;
	output_bu=(alpha_bu*error1_bu+beta_bu*error1_bu1+gama_bu*error1_bu2+2*T3*lastoutput1_bu)/(2*T3);
	
	lastoutput1_bu=output_bu;
	error1_bu1=error1_bu;
	error1_bu2=error1_bu1;
	if(output_bu>500)
	{
		output_bu=500;
	}
	else if(output_bu<=0)
	{
		output_bu=0;
	}
	else
	{
		output_bu=output_bu;
		
	}
	return output_bu;
}
void busailechdonghoc()
{
		if(vitridat==vitridat1)
		{
			output_bu=PIDroirac_bu(output_bu,vitrithuc, vitrithuc1);
			if(vitris[i]>vitris1[j])
			{
				tocdodat=tocdodat-output_bu;
				tocdodat1=tocdodat1+output_bu;
				if(tocdodat1>=MAX) tocdodat1=MAX;			//MAX = 1800, MIN = 1500
				if(tocdodat1<=MIN) tocdodat1=MIN;
				if(tocdodat>=MAX) tocdodat=MAX;
				if(tocdodat<=MIN) tocdodat=MIN;
			}
			else
			{
				tocdodat=tocdodat+output_bu;
				tocdodat1=tocdodat1-output_bu;
				if(tocdodat1>=MAX) tocdodat1=MAX;
				if(tocdodat1<=MIN) tocdodat1=MIN;
				if(tocdodat>=MAX) tocdodat=MAX;
				if(tocdodat<=MIN) tocdodat=MIN;
			}
		}
}
void queophai(int gocqueo)
{
	  start_motor();
		soxungcap_queotrai=(((5*18.85*(gocqueo*30*1.27))/(9*3.25))*4.5924);
		soxungcap_queophai=(((5*18.85*(gocqueo*30*1.27))/(9*3.25))*4.5924);
		sovongcanquay_trai=((soxungcap_queotrai*0.45)/360);
		sovongcanquay_phai=((soxungcap_queophai*0.45)/360);
		PID_DONGCOTRAI_timer_2_4(sovongcanquay_trai,'t');
		PID_DONGCOPHAI_timer_3_4(sovongcanquay_phai,'n');
	}
void queotrai(int gocqueo1)
{
	start_motor();
	soxungcap_queotrai=((((5*18.85*(gocqueo1*30*1.27))/9*3.25)));
	soxungcap_queophai=((((5*18.85*(gocqueo1*30*1.27))/9*3.25)));
	sovongcanquay_trai=((soxungcap_queotrai*0.45)/360);
	sovongcanquay_phai=((soxungcap_queophai*0.45)/360);
	PID_DONGCOTRAI_timer_2_4(sovongcanquay_trai,'n');
	PID_DONGCOPHAI_timer_3_4(sovongcanquay_phai,'t');
}

void dithang(int quangduong1)
{
	start_motor();
	soxungcap_queotrai=quangduong1/heso_kc;
	soxungcap_queophai=quangduong1/heso_kc;
	sovongcanquay_trai=((soxungcap_queotrai*0.45)/360);
	sovongcanquay_phai=((soxungcap_queophai*0.45)/360);
	PID_DONGCOTRAI_timer_2_4(sovongcanquay_trai,'t');
	PID_DONGCOPHAI_timer_3_4(sovongcanquay_phai,'t');
}
void dilui(int quangduong2)
{
	start_motor();
	soxungcap_queotrai=quangduong2/heso_kc;
	soxungcap_queophai=quangduong2/heso_kc;
	sovongcanquay_trai=((soxungcap_queotrai*0.45)/360);
	sovongcanquay_phai=((soxungcap_queophai*0.45)/360);
	PID_DONGCOTRAI_timer_2_4(sovongcanquay_trai,'n');
	PID_DONGCOPHAI_timer_3_4(sovongcanquay_phai,'n');
	
}
void dung()
{
	c=0;
	c1=0;
	vitrithuc=0;
	vitrithuc1=0;
	HAL_Delay(100);
}
void reset11()
{
	c=0;
	c1=0;
	vitrithuc=0;
	vitrithuc1=0;
	tocdodat=0;
	tocdodat1=0;
	tocdothuc=0;
	tocdothuc1=0;
	output_roirac=0;
	output_roirac1=0;

}

	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{ 
  // set transmittion complete flag
  txFlag = TX_IDLE;  
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // switch the buffer index
  uint8_t next = rxIdx ^ 0x01;
  // start next receiving
  HAL_UART_Receive_DMA(&huart1, rxList[next].buf, rxList[next].size);
  
  // set current receiving buffer status flag
  rxFlag = RX_DONE;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	htim4.Instance -> CCR1=0; // de dong co khong chay
	htim4.Instance -> CCR2=0; // de dong co khong chay
	last=HAL_GetTick();
	last1=HAL_GetTick();
	last2=HAL_GetTick();
	HAL_UART_Transmit(&huart1,(uint8_t*)scan,2,0xFFFF);
  //HAL_UART_Transmit(&huart2, "--start--\r\n", 11, 0xFFFF);
  HAL_UART_Receive_DMA(&huart1, rxList[rxIdx].buf, rxList[rxIdx].size);

  /* USER CODE BEGIN WHILE */
  while (1)
  {
/*................TINH TOAN DONG HOC................*/
				// queophai(90);
				// HAL_UART_Transmit(&huart2, (uint8_t*)Rx_data,1,1000);
				// HAL_UART_Receive_IT(&huart2, (uint8_t*)Rx_data,1);
				// if(Rx_data[0]=='F') 
				// {
				// 	start_motor();
				// 	control_motor2(MOTOR_CW,50);	
				// 	control_motor1(MOTOR_CW,50);	
				// }
				// if(Rx_data[0]=='B') 
				// {
				// 	start_motor();
				// 	control_motor2(MOTOR_CCW,50);	
				// 	control_motor1(MOTOR_CCW,50);	
				// }
				// if(Rx_data[0]=='L') 
				// {
				// 	start_motor();
				// 	control_motor2(MOTOR_CW,50);
				// 	control_motor1(MOTOR_CCW,0);					
				// }
				// if(Rx_data[0]=='R') 
				// {
				// 	start_motor();
				// 	control_motor1(MOTOR_CW,50);
				// 	control_motor2(MOTOR_CW,0);
				// }
				// if(Rx_data[0]=='S') 
				// {
				// 	control_motor1(MOTOR_CW,0);
				// 	control_motor2(MOTOR_CW,0);
				// 	dung();
				// 	reset11();
				// }				
				// tocdodat=1700;
				// tocdodat1=1700;
				
/*......... CHUONG TRINH CHINH .............*/ 
				    if(s[i]>0)
					{
						PID_DONGCOTRAI_timer_2_4(s[i],'t');
						l1=s[i]*360;
					}
					if(s1[j]>0)
					{
						PID_DONGCOPHAI_timer_3_4(s1[j],'t');
						l2=s1[j]*360;
					}
					if(s[i]<0)
					{
						PID_DONGCOTRAI_timer_2_4(s[i]*-1,'n');
						l1=s[i]*-360;
					}
					if(s1[j]<0)
					{
						PID_DONGCOPHAI_timer_3_4(s1[j]*-1,'n');
						l2=s1[j]*-360;
					}
					if(a==1)
					{
							j++;
							i++;
							k++;	
							a=0;
					}
					donghoc();
					int len = sprintf(buff,"%.4f,%.4f\n",x_save,y_save);
					HAL_UART_Transmit(&huart2,(uint8_t *)buff,len,300);
					busailechdonghoc();
					vitris[i]=vitrithuc;
					vitris1[j]=vitrithuc1;
					if(vitris[i]>=(l1)&&vitris1[j]>=(l2))
					{
						if(i<gioihan&&j<gioihan)
						{						
							vitridat=0;
							vitrithuc=0;
							vitridat1=0;
							vitrithuc1=0;
							tocdodat=MIN;
							tocdodat1=MIN;
							c=0;
							c1=0;	
							a=1;
							for(f=0;f<=100;f++) 
							{
								PID_DONGCOTRAI_timer_2_4(0,'t');
								PID_DONGCOPHAI_timer_3_4(0,'t');
								HAL_Delay(1);
							}
							
						}
						if(i==gioihan&&j==gioihan)
						{
							stop_motor();
							break;
						}						
					}
		
/*//PID KHONG BU	
				if(s[i]>0)
					{
						PID_DONGCOTRAI_timer_2_4(s[i],'t');
						l1=s[i]*360;
					}
					if(s1[j]>0)
					{
						PID_DONGCOPHAI_timer_3_4(s1[j],'t');
						l2=s1[j]*360;
					}
					if(s[i]<0)
					{
						PID_DONGCOTRAI_timer_2_4(s[i]*-1,'n');
						l1=s[i]*-360;
					}
					if(s1[j]<0)
					{
						PID_DONGCOPHAI_timer_3_4(s1[j]*-1,'n');
						l2=s1[j]*-360;
					}
					if(a==1)
					{
							j++;
							i++;
							k++;	
							a=0;
					}
					donghoc();
					int len1 = sprintf(buff,"%.4f,%.4f\n",x_save,y_save);
					HAL_UART_Transmit(&huart2,(uint8_t *)buff,len,300);
					//busailechdonghoc();
					vitris[i]=vitrithuc;
					vitris1[j]=vitrithuc1;
					if(vitris[i]>=(l1)&&vitris1[j]>=(l2))
					{
						if(i<gioihan&&j<gioihan)
						{						
							vitridat=0;
							vitrithuc=0;
							vitridat1=0;
							vitrithuc1=0;
							tocdodat=MIN;
							tocdodat1=MIN;
							c=0;
							c1=0;	
							a=1;
							for(f=0;f<=100;f++) 
							{
								PID_DONGCOTRAI_timer_2_4(0,'t');
								PID_DONGCOPHAI_timer_3_4(0,'t');
								HAL_Delay(1);
							}
							
						}
						if(i==gioihan&&j==gioihan)
						{
							stop_motor();
							break;
						}						
					}
*/

/*// Read Lidar
control_motor1(MOTOR_CCW,80);
*/

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MOTOR1_IO_Pin|MOTOR2_IO_Pin|enapwm_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOTOR1_IO_Pin MOTOR2_IO_Pin enapwm_Pin */
  GPIO_InitStruct.Pin = MOTOR1_IO_Pin|MOTOR2_IO_Pin|enapwm_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
