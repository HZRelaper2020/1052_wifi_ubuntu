/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

#include "stm32f2xx.h"
#include "stm32f2xx_gpio.h"
#include "wiced_constants.h"
#include <stdio.h>
#include "../../platform/command_console_platform.h"

struct breakout_pins_setting_t
{
    uint16_t          breakout_pin_number;
    int               useful; /**< non-zero if we can actually use this pin for output, else zero */

    /** The below values are meaningless if useful is false/zero */
    GPIO_TypeDef *    port;
    uint16_t          pin;
    GPIOMode_TypeDef  type;
    GPIOOType_TypeDef otype;
    GPIOPuPd_TypeDef  pupd;
};

const struct breakout_pins_setting_t const breakout_gpio_settings[] =
{
    {  1, WICED_FALSE, GPIOA, GPIO_Pin_4,  GPIO_Mode_AN,  GPIO_OType_PP, GPIO_PuPd_UP }, /* ignored */
    {  2, WICED_FALSE, GPIOA, GPIO_Pin_4,  GPIO_Mode_AN,  GPIO_OType_PP, GPIO_PuPd_UP }, /* ignored */
    {  3, WICED_TRUE,  GPIOA, GPIO_Pin_1,  GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP },
    {  4, WICED_TRUE,  GPIOA, GPIO_Pin_2,  GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP },
    {  5, WICED_TRUE,  GPIOA, GPIO_Pin_3,  GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP },
    {  6, WICED_TRUE,  GPIOB, GPIO_Pin_11, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP },
    {  7, WICED_TRUE,  GPIOB, GPIO_Pin_12, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP },
    {  8, WICED_FALSE, GPIOA, GPIO_Pin_4,  GPIO_Mode_AN,  GPIO_OType_PP, GPIO_PuPd_UP }, /* ignored */
    {  9, WICED_FALSE, GPIOA, GPIO_Pin_0,  GPIO_Mode_AN,  GPIO_OType_PP, GPIO_PuPd_UP }, /* ignored */
    { 10, WICED_FALSE, GPIOA, GPIO_Pin_4,  GPIO_Mode_AN,  GPIO_OType_PP, GPIO_PuPd_UP }, /* ignored */
    { 11, WICED_FALSE, GPIOA, GPIO_Pin_9,  GPIO_Mode_AN,  GPIO_OType_PP, GPIO_PuPd_UP }, /* ignored */
    { 12, WICED_FALSE, GPIOA, GPIO_Pin_10, GPIO_Mode_AN,  GPIO_OType_PP, GPIO_PuPd_UP }, /* ignored */
    { 13, WICED_FALSE, GPIOA, GPIO_Pin_7,  GPIO_Mode_AN,  GPIO_OType_PP, GPIO_PuPd_UP }, /* ignored */
    { 14, WICED_FALSE, GPIOA, GPIO_Pin_6,  GPIO_Mode_AN,  GPIO_OType_PP, GPIO_PuPd_UP }, /* ignored */
    { 15, WICED_FALSE, GPIOA, GPIO_Pin_5,  GPIO_Mode_AN,  GPIO_OType_PP, GPIO_PuPd_UP }, /* ignored */
    { 16, WICED_FALSE, GPIOA, GPIO_Pin_4,  GPIO_Mode_AN,  GPIO_OType_PP, GPIO_PuPd_UP }, /* ignored */
    { 17, WICED_FALSE, GPIOA, GPIO_Pin_4,  GPIO_Mode_AN,  GPIO_OType_PP, GPIO_PuPd_UP }, /* ignored */
    { 18, WICED_FALSE, GPIOA, GPIO_Pin_4,  GPIO_Mode_AN,  GPIO_OType_PP, GPIO_PuPd_UP }  /* ignored */
};

const unsigned int breakout_settings_size = ( sizeof( breakout_gpio_settings ) / sizeof( struct breakout_pins_setting_t ) );
const unsigned int sizeof_breakout_pins_setting_t = sizeof( struct breakout_pins_setting_t );


inline uint16_t breakout_pins_get_pin_number( struct breakout_pins_setting_t *settings )
{
    return ( settings->breakout_pin_number );
}

inline int breakout_pins_gpio_init( struct breakout_pins_setting_t *breakout_pin )
{
    GPIO_InitTypeDef gpio_init_structure;

    /* If we can't utilize this pin for output then there is no point setting it up... */
    if ( !breakout_pin->useful )
    {
        return 0;
    }

    gpio_init_structure.GPIO_Pin   = breakout_pin->pin;
    gpio_init_structure.GPIO_Speed = ( breakout_pin->type == GPIO_Mode_AIN ) ? (GPIOSpeed_TypeDef) 0 : GPIO_Speed_50MHz;
    gpio_init_structure.GPIO_Mode  = breakout_pin->type;
    gpio_init_structure.GPIO_OType = breakout_pin->otype;
    gpio_init_structure.GPIO_PuPd  = breakout_pin->pupd;
    GPIO_Init( breakout_pin->port, &gpio_init_structure );
    return 1;
}

inline void breakout_pins_gpio_setbits( struct breakout_pins_setting_t *breakout_pin )
{
    GPIO_SetBits( breakout_pin->port, breakout_pin->pin );
}

inline void breakout_pins_gpio_resetbits( struct breakout_pins_setting_t *breakout_pin )
{
    GPIO_ResetBits( breakout_pin->port, breakout_pin->pin );
}
