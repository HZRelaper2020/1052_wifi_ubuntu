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

#ifndef INCLUDED_WPL_POWER_EVENTS_H_
#define INCLUDED_WPL_POWER_EVENTS_H_

#ifdef __cplusplus
extern "C" {
#endif

event_lookup_table_entry_t wpl_event_list_table[] =
{
    /* { proc-id, event-id, event-desc } */
    //MCU Events
    { EVENT_PROC_ID_MCU,        EVENT_ID_POWERSTATE,      EVENT_DESC_POWER_ACTIVE1                },
    { EVENT_PROC_ID_MCU,        EVENT_ID_POWERSTATE,      EVENT_DESC_POWER_ACTIVE2                },
    { EVENT_PROC_ID_MCU,        EVENT_ID_POWERSTATE,      EVENT_DESC_POWER_DEEPSLEEP              },
    { EVENT_PROC_ID_MCU,        EVENT_ID_POWERSTATE,      EVENT_DESC_POWER_PDS              },

    //MCU Serial Events
    { EVENT_PROC_ID_MCU,        EVENT_ID_UART,            EVENT_DESC_UART_IDLE                    },
    { EVENT_PROC_ID_MCU,        EVENT_ID_UART,            EVENT_DESC_UART_TX                      },
    { EVENT_PROC_ID_MCU,        EVENT_ID_UART,            EVENT_DESC_UART_RX                      },

    //MCU SPI Events
    { EVENT_PROC_ID_MCU,        EVENT_ID_SPI_1,           EVENT_DESC_SPI_OFF                    },
    { EVENT_PROC_ID_MCU,        EVENT_ID_SPI_1,           EVENT_DESC_SPI_IDLE                      },
    { EVENT_PROC_ID_MCU,        EVENT_ID_SPI_1,           EVENT_DESC_SPI_READ                      },
    { EVENT_PROC_ID_MCU,        EVENT_ID_SPI_1,           EVENT_DESC_SPI_WRITE                      },

    { EVENT_PROC_ID_BT,        EVENT_ID_BT_DATA,       EVENT_DESC_BT_POWER_IDLE                  },
    { EVENT_PROC_ID_BT,        EVENT_ID_BT_DATA,       EVENT_DESC_BT_POWER_TX                    },
    { EVENT_PROC_ID_BT,        EVENT_ID_BT_DATA,       EVENT_DESC_BT_POWER_RX                    },
    { EVENT_PROC_ID_BT,        EVENT_ID_BT_DATA,       EVENT_DESC_BT_POWER_SLEEP                    },

#if WPL_ENABLE_WIFI_LOG
     //Wi-Fi Events
    { EVENT_PROC_ID_WIFI,       EVENT_ID_WIFI_DATA,       EVENT_DESC_WIFI_IDLE                    },
    { EVENT_PROC_ID_WIFI,       EVENT_ID_WIFI_DATA,       EVENT_DESC_WIFI_BAND                    },
    { EVENT_PROC_ID_WIFI,       EVENT_ID_WIFI_DATA,       EVENT_DESC_WIFI_BW                      },
    { EVENT_PROC_ID_WIFI,       EVENT_ID_WIFI_DATA,       EVENT_DESC_WIFI_PMMODE                  },
    { EVENT_PROC_ID_WIFI,       EVENT_ID_WIFI_DATA,       EVENT_DESC_WIFI_RATE_TYPE               },
    { EVENT_PROC_ID_WIFI,       EVENT_ID_WIFI_DATA,       EVENT_DESC_WIFI_RATE0                   },
    { EVENT_PROC_ID_WIFI,       EVENT_ID_WIFI_DATA,       EVENT_DESC_WIFI_RATE1                   },
    { EVENT_PROC_ID_WIFI,       EVENT_ID_WIFI_DATA,       EVENT_DESC_WIFI_RATE2                   },
    { EVENT_PROC_ID_WIFI,       EVENT_ID_WIFI_DATA,       EVENT_DESC_WIFI_RATE3                   },
    { EVENT_PROC_ID_WIFI,       EVENT_ID_WIFI_DATA,       EVENT_DESC_WIFI_RATE4                   },
    { EVENT_PROC_ID_WIFI,       EVENT_ID_WIFI_DATA,       EVENT_DESC_WIFI_RATE5                   },
    { EVENT_PROC_ID_WIFI,       EVENT_ID_WIFI_DATA,       EVENT_DESC_WIFI_RATE6                   },
    { EVENT_PROC_ID_WIFI,       EVENT_ID_WIFI_DATA,       EVENT_DESC_WIFI_RATE7                   },
    { EVENT_PROC_ID_WIFI,       EVENT_ID_WIFI_DATA,       EVENT_DESC_WIFI_RATE8                   },
    { EVENT_PROC_ID_WIFI,       EVENT_ID_WIFI_DATA,       EVENT_DESC_WIFI_RATE9                   },
#endif

    //MCU function profile Events
    { EVENT_PROC_ID_MCU,        EVENT_ID_PROFILING,       EVENT_DESC_FUNC_IDLE                    },
    { EVENT_PROC_ID_MCU,        EVENT_ID_PROFILING,       EVENT_DESC_FUNC_TIME                    },


};


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ifndef INCLUDED_WPL_POWER_EVENTS_H_ */
