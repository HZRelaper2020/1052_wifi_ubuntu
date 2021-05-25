#ifndef __LED_H
#define	__LED_H

#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CORE_BOARD_LED_GPIO 				GPIO1
#define CORE_BOARD_LED_GPIO_PIN 		(9U)
#define CORE_BOARD_LED_IOMUXC			IOMUXC_GPIO_AD_B0_09_GPIO1_IO09

#define CORE_BOARD_DELAY_COUNT 		800000//0

/** the macro definition to trigger the led on or off 
  * 1 - off
  *0 - on
  */
#define ON  1
#define OFF 0

/* 使用标准的固件库控制IO*/
#define CORE_BOARD_LED(a)	if (a)	\
					GPIO_PinWrite(CORE_BOARD_LED_GPIO, CORE_BOARD_LED_GPIO_PIN, 0U);\
					else		\
					GPIO_PinWrite(CORE_BOARD_LED_GPIO, CORE_BOARD_LED_GPIO_PIN, 1U);



/* 直接操作寄存器的方法控制IO */
#define digitalHi(p,i)		 {p->DR |= (1U << i);}	 //输出为高电平		
#define digitalLo(p,i)		 	{p->DR &= ~(1U << i);}//输出低电平
#define digitalToggle(p,i) {p->DR ^= (1U<<i);} //输出反转状态


/* 定义控制IO的宏 */
#define CORE_BOARD_LED_TOGGLE		 digitalToggle(CORE_BOARD_LED_GPIO,CORE_BOARD_LED_GPIO_PIN)
#define CORE_BOARD_LED_OFF		   digitalHi(CORE_BOARD_LED_GPIO,CORE_BOARD_LED_GPIO_PIN)
#define CORE_BOARD_LED_ON			   digitalLo(CORE_BOARD_LED_GPIO,CORE_BOARD_LED_GPIO_PIN)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void LED_GPIO_Config(void);

#endif /* __LED_H */
