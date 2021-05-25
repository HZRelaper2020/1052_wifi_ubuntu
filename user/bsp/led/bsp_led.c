/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   led应用函数接口
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  i.MXRT1052开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"  
	
#include "./led/bsp_led.h"   

 /**
  * @brief  初始化控制LED的IO
  * @param  无
  * @retval 无
  */
void LED_GPIO_Config(void)
{	
	  
		/* 配置为输出模式，默认高电平，不使用中断，后面通过GPIO_PinInit函数加载配置 */
    gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 1, kGPIO_NoIntmode};
		
		/* 设置引脚的复用模式 */
		IOMUXC_SetPinMux(
		CORE_BOARD_LED_IOMUXC,    /* 配置为普通IO */
		0U);                         /* 若配置化失败，该引脚的默认输入电平 */
		
		/*设置引脚功能*/
		IOMUXC_SetPinConfig(
		CORE_BOARD_LED_IOMUXC,        /* 配置说明 : */
		0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
																							 Drive Strength Field: R0/6
																							 Speed Field: medium(100MHz)
																							 Open Drain Enable Field: Open Drain Disabled
																							 Pull / Keep Enable Field: Pull/Keeper Enabled
																							 Pull / Keep Select Field: Keeper
																							 Pull Up / Down Config. Field: 100K Ohm Pull Down
																							 Hyst. Enable Field: Hysteresis Disabled */
		
    /* 配置说明 : */
		/* 转换速率: 转换速率慢（调整高低电平切换上升和下降时间速度的设置项）
			驱动强度: R0/6 （仅作为输出时有效 ）
			速度配置 : medium(100MHz)
			开漏配置: 关闭 （开漏高阻态常用于总线配置，如I2C ）
			拉/保持器配置: 使能
			拉/保持器选择: 保持器
			上拉/下拉选择: 100K欧姆下拉
			滞回器配置: 关闭 （仅输入时有效，施密特触发器，使能后可以过滤输入噪声）*/


		
		/* 初始化 LED GPIO. */
    GPIO_PinInit(CORE_BOARD_LED_GPIO, CORE_BOARD_LED_GPIO_PIN, &led_config);


}

/*********************************************END OF FILE**********************/
