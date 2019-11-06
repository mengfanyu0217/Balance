#include "stm32f10x.h"
#include "sys.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
u8 Flag_Stop=0,delay_50,delay_flag,Flash_Send,PID_Send;                   //ֹͣ��־λ 50ms��׼��ʱ��־λ
int Encoder,Encoder_Key,Target_Position=10000,Target_Velocity=40; //���������������
int Moto;                                             //���PWM���� Ӧ��Motor�� ��Moto�¾�	
int Voltage;                                          //��ص�ѹ������صı���
float Position_KP=40,Position_KI=0.1,Position_KD=200,Velocity_KP=5,Velocity_KI=5;      //PIDϵ��
float Amplitude_PKP=2,Amplitude_PKI=0.1,Amplitude_PKD=3,Amplitude_VKP=1,Amplitude_VKI=1; //PID������ز���
float Menu_MODE=1,Menu_PID=1;  //PID��ر�־λ
float Angle_Balance;  //�Ƕ�
u8 Flag_MODE=1;   
int main(void)
  { 
		delay_init();	    	            //=====��ʱ������ʼ��	
		uart_init(128000);	            //=====���ڳ�ʼ��Ϊ
		JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
		JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���
		LED_Init();                     //=====��ʼ���� LED ���ӵ�Ӳ���ӿ�
	  KEY_Init();                     //=====������ʼ��
		MY_NVIC_PriorityGroupConfig(2);	//=====�����жϷ���
    MiniBalance_PWM_Init(7199,0);   //=====��ʼ��PWM 10KHZ������������� �����ʼ������ӿ� 
		uart3_init(9600);               //=====����3��ʼ��
    Encoder_Init_TIM4();            //=====��ʼ��������
		Adc_Init();                     //=====adc��ʼ��
    IIC_Init();                     //=====IIC��ʼ��
    MPU6050_initialize();           //=====MPU6050��ʼ��	
    DMP_Init();                     //=====��ʼ��DMP 
    OLED_Init();                    //=====OLED��ʼ��	    
		while(Flag_MODE)                //=====���û�ѡ��  ����ģʽ
		{
		  oled_show_once();               //=====��ʱ��ʾOLED
			if(TIM4->CNT>10500)Menu_MODE=0; //�ٶ�ģʽ  ���ת������
			if(TIM4->CNT<9500) Menu_MODE=1; //λ��ģʽ  ��ǰת������
		  if(TIM4->CNT>10500||TIM4->CNT<9500)Flag_MODE=0,OLED_Clear(),TIM4->CNT=0;
		}
		 Encoder_Init_TIM2();            //=====�������ӿ�
	  MiniBalance_EXTI_Init();        //=====MPU6050 5ms��ʱ�жϳ�ʼ��
    while(1) // ��ѭ��
	   {
				delay_flag=1;	              //===50ms�жϾ�׼��ʱ��־λ
				oled_show();                //===��ʾ����	  	
        DataScope();			           //===��λ��
				while(delay_flag);          //===50ms�жϾ�׼��ʱ  ��Ҫ�ǲ�����ʾ��λ����Ҫ�ϸ��50ms��������   	
	   } 
}

