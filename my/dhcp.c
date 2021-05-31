//#include "scan.h"

#include <stdio.h>
#include <string.h>
#include "wwd_management.h"
#include "wwd_wifi.h"
#include "wwd_debug.h"
#include "wwd_events.h"
#include "wwd_assert.h"
#include "wwd_buffer.h"
#include "wwd_network.h"
#include "RTOS/wwd_rtos_interface.h"
#include "platform/wwd_platform_interface.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "wiced_utilities.h"
#include "wwd_buffer_interface.h"
/******************************************************
 *               静态函数声明
 ******************************************************/
err_t
ethernet_input(struct pbuf *p, struct netif *netif);

/******************************************************
 *               变量定义
 ******************************************************/
#define CIRCULAR_RESULT_BUFF_SIZE 40


/******************************************************
 *              功能定义
 ******************************************************/

/**
 * Main Scan app
 */

//#define AP_SSID              "nwztest"// "test_rk3288" //"Relaper-H3C-2.4G"    /* 路由名称 */
//#define AP_PASS              "123456789" //"hzrelaper"         /* 路由密码 */
#define AP_SEC               WICED_SECURITY_WPA2_MIXED_PSK  /* 路由加密 */

#define COUNTRY              WICED_COUNTRY_AUSTRALIA    /* 选择城市 据说澳大利亚的信号更强一些，这里选择为澳大利亚 */

static struct netif wiced_if;
static struct dhcp netif_dhcp;

static ip4_addr_t ipaddr, netmask, gw;

static wiced_ssid_t ap_ssid =
{
    .length = 0,
};

void dhcp_main(uint8_t* ssid,uint8_t* password)
{
	wwd_result_t result;

    /* 初始化 Wiced */
    WPRINT_APP_INFO(("Starting Wiced v" WICED_VERSION "\n"));

    wwd_buffer_init(NULL);

    result = wwd_management_wifi_on( COUNTRY );
    if ( result != WWD_SUCCESS )
    {
        WPRINT_APP_INFO(("Error %d while starting WICED!\n", result));
    }

#if 0
    for (int i=0;i<200000;i++){
	    vTaskDelay(500);
	    if (ssid != NULL){
	    result = wwd_ensure_wlan_bus_is_up();
	    PRINTF("\n%d result:%d\n",i,result);
	    }
    }
#endif

    /*尝试加入Wi-Fi网络 */
    WPRINT_APP_INFO(("Joining : %s \n",ssid));

    ap_ssid.length = strlen(ssid);
    strcpy(ap_ssid.value,ssid);
    
    while ( wwd_wifi_join( &ap_ssid, AP_SEC, password, strlen(password), NULL, WWD_STA_INTERFACE ) != WWD_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to join  : %s    .. retrying\n",ssid));
    }
    WPRINT_APP_INFO(("Successfully joined : %s \n",ssid));

    ip4_addr_set_zero( &gw );
    ip4_addr_set_zero( &ipaddr );
    ip4_addr_set_zero( &netmask );

//    PRINTF("1 wiced_if:0x%x dhcp:0x%x\n",&wiced_if,wiced_if.client_data[0]);
    if ( NULL == netif_add( &wiced_if, &ipaddr, &netmask, &gw, (void*) WWD_STA_INTERFACE, ethernetif_init, ethernet_input ) )
    {
        WPRINT_APP_ERROR(( "Could not add network interface\n" ));
        return;
    }

     netif_set_up( &wiced_if );

    WPRINT_APP_INFO(("Obtaining IP address via DHCP\n"));
    dhcp_set_struct( &wiced_if, &netif_dhcp );
  //  PRINTF("3 wiced_if:0x%x dhcp:0x%x\n",&wiced_if,wiced_if.client_data[0]);
    if (dhcp_start( &wiced_if )){
	    WPRINT_APP_INFO(("start dhcp failed\n"));
	    return;
    }

    while ( netif_dhcp.state != 10 )
    {
    /* 等待 */
        sys_msleep( 10 );
    }
  //  WPRINT_APP_INFO(("dhcp ok\n"));
    WPRINT_APP_INFO( ( "Network ready IP: %s\n", ip4addr_ntoa(netif_ip4_addr(&wiced_if))));
 //   PRINTF("2 wiced_if:0x%x dhcp:0x%x\n",&wiced_if,wiced_if.client_data[0]);

    while(0)
    {

    }
    
}

