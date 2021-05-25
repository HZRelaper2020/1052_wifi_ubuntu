/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   ledӦ�ú����ӿ�
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  i.MXRT1052������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"  
	
#include "./led/bsp_led.h"   

 /**
  * @brief  ��ʼ������LED��IO
  * @param  ��
  * @retval ��
  */
void LED_GPIO_Config(void)
{	
	  
		/* ����Ϊ���ģʽ��Ĭ�ϸߵ�ƽ����ʹ���жϣ�����ͨ��GPIO_PinInit������������ */
    gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 1, kGPIO_NoIntmode};
		
		/* �������ŵĸ���ģʽ */
		IOMUXC_SetPinMux(
		CORE_BOARD_LED_IOMUXC,    /* ����Ϊ��ͨIO */
		0U);                         /* �����û�ʧ�ܣ������ŵ�Ĭ�������ƽ */
		
		/*�������Ź���*/
		IOMUXC_SetPinConfig(
		CORE_BOARD_LED_IOMUXC,        /* ����˵�� : */
		0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
																							 Drive Strength Field: R0/6
																							 Speed Field: medium(100MHz)
																							 Open Drain Enable Field: Open Drain Disabled
																							 Pull / Keep Enable Field: Pull/Keeper Enabled
																							 Pull / Keep Select Field: Keeper
																							 Pull Up / Down Config. Field: 100K Ohm Pull Down
																							 Hyst. Enable Field: Hysteresis Disabled */
		
    /* ����˵�� : */
		/* ת������: ת���������������ߵ͵�ƽ�л��������½�ʱ���ٶȵ������
			����ǿ��: R0/6 ������Ϊ���ʱ��Ч ��
			�ٶ����� : medium(100MHz)
			��©����: �ر� ����©����̬�������������ã���I2C ��
			��/����������: ʹ��
			��/������ѡ��: ������
			����/����ѡ��: 100Kŷķ����
			�ͻ�������: �ر� ��������ʱ��Ч��ʩ���ش�������ʹ�ܺ���Թ�������������*/


		
		/* ��ʼ�� LED GPIO. */
    GPIO_PinInit(CORE_BOARD_LED_GPIO, CORE_BOARD_LED_GPIO_PIN, &led_config);


}

/*********************************************END OF FILE**********************/
