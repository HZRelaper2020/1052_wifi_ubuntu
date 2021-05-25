/**
  ******************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   i.MX RT + FreeRTOS + LWIP + AP6181（wifi）
  ******************************************************************
  * @attention
  *
  * 实验平台:野火  i.MXRT1052开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************
  */
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "lwip/opt.h"
#include "lwip/icmp.h"
#include "lwip/inet_chksum.h"
#include "lwip/sockets.h"
#include "lwip/mem.h"
#include "lwip/inet.h"
#include "netif/etharp.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "lwip/prot/dhcp.h"
#include "wwd_network.h"
#include "wwd_management.h"
#include "wwd_wifi.h"
#include "wwd_debug.h"
#include "wwd_assert.h"
#include "platform/wwd_platform_interface.h"
#include "RTOS/wwd_rtos_interface.h"
#include "wwd_buffer_interface.h"

#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "./led/bsp_led.h" 
/******************************************************
 *                      Macros
 ******************************************************/
/** @cond */
#define MAKE_IPV4_ADDRESS(a, b, c, d)          ((((uint32_t) (a)) << 24) | (((uint32_t) (b)) << 16) | (((uint32_t) (c)) << 8) | ((uint32_t) (d)))

/* 在发行版中，所有UART打印都被抑制，从而消除
了removef和malloc依赖，从而显着减少了内存使用*/
#ifndef DEBUG
#undef  WPRINT_APP_INFO
#define WPRINT_APP_INFO(args) printf args
#undef  WPRINT_APP_ERROR
#define WPRINT_APP_ERROR(args) printf args
#endif

/** @endcond */

/******************************************************
 *                    配置
 ******************************************************/
#define AP_SSID              "Relaper-H3C-2.4G"    /* 路由名称 */
#define AP_PASS              "hzrelaper"         /* 路由密码 */
#define AP_SEC               WICED_SECURITY_WPA2_MIXED_PSK  /* 路由加密 */

#define COUNTRY              WICED_COUNTRY_AUSTRALIA    /* 选择城市 据说澳大利亚的信号更强一些，这里选择为澳大利亚 */
#define USE_DHCP             WICED_TRUE     /* 是否使用 DHCP */
#define IP_ADDR              MAKE_IPV4_ADDRESS( 192, 168,   5,  128 )  /* 如果USE_DHCP为WICED_TRUE，则不需要 */
#define GW_ADDR              MAKE_IPV4_ADDRESS( 192, 168,   5,   1 )   /* 如果USE_DHCP为WICED_TRUE，则不需要 */
#define NETMASK              MAKE_IPV4_ADDRESS( 255, 255, 255,   0 )    /* 如果USE_DHCP为WICED_TRUE，则不需要 */
/* #define PING_TARGET          MAKE_IPV4_ADDRESS( 192, 168,   1, 2 ) */  /* 如果要ping特定IP而不是网关，请取消注释*/

#define PING_RCV_TIMEOUT     (1000)    /** ping接收超时-以毫秒为单位*/
#define PING_DELAY           (1000)    /** ping响应/超时与下一次ping发送之间的延迟-以毫秒为单位 */
#define PING_ID              (0xAFAF)
#define PING_DATA_SIZE       (32)      /** ping 其他数据大小*/
#define JOIN_TIMEOUT         (10000)   /** 加入无线网络的超时时间（以毫秒为单位）= 10秒*/
#define APP_THREAD_STACKSIZE (5120)


/******************************************************
 *               静态函数声明
 ******************************************************/
extern void netio_init(void);
static void ping_prepare_echo( struct icmp_echo_hdr *iecho, uint16_t len );
static err_t ping_recv( int socket_hnd );
static err_t ping_send( int s, ip4_addr_t *addr );
static void tcpip_init_done( void * arg );
static void startup_thread( void *arg );
static void app_main( void );

/******************************************************
 *               变量定义
 ******************************************************/

static uint16_t     ping_seq_num;
static struct netif wiced_if;
static TaskHandle_t  startup_thread_handle;

static const wiced_ssid_t ap_ssid =
{
    .length = sizeof(AP_SSID)-1,
    .value  = AP_SSID,
};

static void delay(int num) {
	while (num-- > 0)
		;
}

static void wifi_reset() {
	// wifi off
//	GPIO_PinWrite(GPIO1, 2, 0);
//	delay(1 * 1000 * 1000);
//	// wifi on
//	GPIO_PinWrite(GPIO1, 2, 1);
//	delay(1 * 1000 * 1000);
}
/*
配置wifi lwip信息
*/
void Config_WIFI_LwIP_Info()
{
		ip4_addr_t ipaddr, netmask, gw;
    ip4_addr_t target;
    int socket_hnd;
    wwd_time_t send_time;
    int recv_timeout = PING_RCV_TIMEOUT;
    wwd_result_t result;
		
    /* 初始化 Wiced */
    WPRINT_APP_INFO(("Starting Wiced v" WICED_VERSION "\n"));

    wwd_buffer_init( NULL);
    result = wwd_management_wifi_on( COUNTRY );
    if ( result != WWD_SUCCESS )
    {
        WPRINT_APP_INFO(("Error %d while starting WICED!\n", result));
    }

    /*尝试加入Wi-Fi网络 */
    WPRINT_APP_INFO(("Joining : " AP_SSID "\n"));
    while ( wwd_wifi_join( &ap_ssid, AP_SEC, (uint8_t*) AP_PASS, sizeof( AP_PASS ) - 1, NULL, WWD_STA_INTERFACE ) != WWD_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to join  : " AP_SSID "   .. retrying\n"));
    }
    WPRINT_APP_INFO(("Successfully joined : " AP_SSID "\n"));

    /* 设置IP配置*/
    if ( USE_DHCP == WICED_TRUE )
    {
        ip4_addr_set_zero( &gw );
        ip4_addr_set_zero( &ipaddr );
        ip4_addr_set_zero( &netmask );
    }
    else
    {
        ipaddr.addr  = htonl( IP_ADDR );
        gw.addr      = htonl( GW_ADDR );
        netmask.addr = htonl( NETMASK );
    }

    if ( NULL == netif_add( &wiced_if, &ipaddr, &netmask, &gw, (void*) WWD_STA_INTERFACE, ethernetif_init, ethernet_input ) )
    {
        WPRINT_APP_ERROR(( "Could not add network interface\n" ));
        return;
    }

    netif_set_up( &wiced_if );

    if ( USE_DHCP == WICED_TRUE )
    {
        struct dhcp netif_dhcp;
        WPRINT_APP_INFO(("Obtaining IP address via DHCP\n"));
        dhcp_set_struct( &wiced_if, &netif_dhcp );
        dhcp_start( &wiced_if );
        while ( netif_dhcp.state != DHCP_STATE_BOUND )
        {
            /* 等待 */
            sys_msleep( 10 );
        }
    }

    WPRINT_APP_INFO( ( "Network ready IP: %s\n", ip4addr_ntoa(netif_ip4_addr(&wiced_if))));
		    /*打开本地套接字进行ping */
    if ( ( socket_hnd = lwip_socket( AF_INET, SOCK_RAW, IP_PROTO_ICMP ) ) < 0 )
    {
        WPRINT_APP_ERROR(( "unable to create socket for Ping\n" ));
        return;
    }

    /* 在本地套接字上设置接收超时，以便ping会超时. */
    lwip_setsockopt( socket_hnd, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout, sizeof( recv_timeout ) );

#ifdef PING_TARGET
        target.addr = htonl( PING_TARGET );
#else /* ifdef PING_TARGET */
        target.addr = netif_ip4_gw(&wiced_if)->addr;
#endif /* ifdef PING_TARGET */

//    WPRINT_APP_INFO(("Pinging: %s\n", ip4addr_ntoa( &target )));
		netio_init();
		
}
/******************************************************
 *               功能定义
 ******************************************************/

/**
 * Main Ping app
 *
 * 初始化Wiced，加入无线网络，然后一直ping指定的IP地址。
 */

static void app_main( void )
{
   
		Config_WIFI_LwIP_Info();
		
    /* Loop forever */
    while ( 1 )
    {
        err_t result;
#if 0
        /* 发送ping*/
        if ( ping_send( socket_hnd, &target ) != ERR_OK )
        {
            WPRINT_APP_ERROR(( "Unable to send Ping\n" ));
            return;
        }

        /* 记录时间ping已发送 */
        send_time = host_rtos_get_time( );

        /* 等待ping回复 */
        result = ping_recv( socket_hnd );
        if ( ERR_OK == result )
        {
            WPRINT_APP_INFO(("Pinginging Reply %dms\n", (int)( host_rtos_get_time( ) - send_time ) ));
        }
        else
        {
            WPRINT_APP_INFO(("Ping timeout\n"));
        }
#endif 
        /*等待直到下一次ping的时间 */
        sys_msleep( PING_DELAY );
    }

    /* 关机代码-由于无限循环而未使用 */
#if 0
    WPRINT_APP_INFO(("Closing down\n"));
    lwip_close( socket_hnd );
    if ( USE_DHCP == WICED_TRUE )
    {
        dhcp_stop( &wiced_if );
    }
    netif_set_down( &wiced_if );
    netif_remove( &wiced_if );
    wwd_wifi_leave( );
    if ( WWD_SUCCESS != wwd_management_deinit( ) )
    {
        WPRINT_APP_ERROR(("WICED de-initialization failed\n"));
    }
#endif /* if 0 */
}



/**
 *  准备回显ICMP请求数据包的内容
 *
 *  @param iecho  : 指向icmp_echo_hdr结构的指针，将在其中构造ICMP数据包
 *  @param len    : 传递给iecho参数的数据包缓冲区的字节长度
 *
 */

static void ping_prepare_echo( struct icmp_echo_hdr *iecho, uint16_t len )
{
    int i;

    ICMPH_TYPE_SET( iecho, ICMP_ECHO );
    ICMPH_CODE_SET( iecho, 0 );
    iecho->chksum = 0;
    iecho->id = PING_ID;
    iecho->seqno = htons( ++ping_seq_num );

    /* 用一些数据填充额外的数据缓冲区 */
    for ( i = 0; i < PING_DATA_SIZE; i++ )
    {
        ( (char*) iecho )[sizeof(struct icmp_echo_hdr) + i] = i;
    }

    iecho->chksum = inet_chksum( iecho, len );
}



/**
 *  发送一个Ping
 *
 * 使用指定的套接字将ICMP回显请求（Ping）发送到指定的IP地址。
 *
 *  @param socket_hnd :将通过其发送ping请求的本地套接字的句柄
 *  @param addr       :将向其发送ping请求的IP地址
 *
 *  @return 如果成功发送，则返回ERR_OK；如果内存不足，则返回ERR_MEM；否则，返回ERR_VAL
 */
static err_t ping_send( int socket_hnd, ip4_addr_t *addr )
{
    int err;
    struct icmp_echo_hdr *iecho;
    struct sockaddr_in to;
    size_t ping_size = sizeof(struct icmp_echo_hdr) + PING_DATA_SIZE;

    /* 为数据包分配内存*/
    if ( !( iecho = (struct icmp_echo_hdr*) mem_malloc( ping_size ) ) )
    {
        return ERR_MEM;
    }

    /* 构造ping请求 */
    ping_prepare_echo( iecho, ping_size );

    /*发送ping请求 */
    to.sin_len = sizeof( to );
    to.sin_family = AF_INET;
    to.sin_addr.s_addr = addr->addr;

    err = lwip_sendto( socket_hnd, iecho, ping_size, 0, (struct sockaddr*) &to, sizeof( to ) );

    /* 释放 包 */
    mem_free( iecho );

    return ( err ? ERR_OK : ERR_VAL );
}


/**
 *  收到Ping回复
 *  等待使用指定的套接字接收ICMP回显答复（Ping答复）。 将序列号和ID号与上
 *  次发送的ping进行比较，如果匹配，则返回ERR_OK，表示有效的ping响应。
 *  @param socket_hnd : 本地套接字的句柄，通过该句柄将收到ping答复
 *
 *  @return  如果收到有效答复，则为ERR_OK，否则为ERR_TIMEOUT
 */
static err_t ping_recv( int socket_hnd )
{
    char buf[64];
    int fromlen, len;
    struct sockaddr_in from;
    struct ip_hdr *iphdr;
    struct icmp_echo_hdr *iecho;

    while ( ( len = lwip_recvfrom( socket_hnd, buf, sizeof( buf ), 0, (struct sockaddr*) &from, (socklen_t*) &fromlen ) ) > 0 )
    {
        if ( len >= (int) ( sizeof(struct ip_hdr) + sizeof(struct icmp_echo_hdr) ) )
        {
            iphdr = (struct ip_hdr *) buf;
            iecho = (struct icmp_echo_hdr *) ( buf + ( IPH_HL( iphdr ) * 4 ) );

            if ( ( iecho->id == PING_ID ) &&
                 ( iecho->seqno == htons( ping_seq_num ) ) &&
                 ( ICMPH_TYPE( iecho ) == ICMP_ER ) )
            {
                return ERR_OK; /*收到回声回复-成功返回 */
            }
        }
    }

    return ERR_TIMEOUT; /* 超时前未收到有效的回声回复 */
}

static void BOARD_USDHCClockConfiguration(void)
{
    /*将系统pll PFD2分数分频器配置为18*/
    CLOCK_InitSysPfd(kCLOCK_Pfd0, 0x12U);
    /* 配置USDHC时钟源和分频器*/
    CLOCK_SetDiv(kCLOCK_Usdhc2Div, 0U);
    CLOCK_SetMux(kCLOCK_Usdhc2Mux, 1U);
}



/**
 * 主函数
 */
// .h
void aaa();
// .c
void aaa(){
}

int main( void )
{
    gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    /* 初始化内存管理单元 */
    BOARD_ConfigMPU();
    /* 初始化开发板引脚 */
    BOARD_InitPins();
    /* 初始化开发板时钟 */
    BOARD_BootClockRUN();
    /* 初始化SDIO时钟 */
    BOARD_USDHCClockConfiguration();
    /* 初始化调试串口 */
    BOARD_InitDebugConsole();
		wifi_reset();
    /* 初始化LED */
    LED_GPIO_Config();
    /* 设置SDIO中断优先级 */
    NVIC_SetPriority(USDHC2_IRQn, 5U);

    IOMUXC_EnableMode(IOMUXC_GPR, kIOMUXC_GPR_ENET1TxClkOutputDir, true);
    /* 初始化PHY RESET引脚 */
    GPIO_PinInit(GPIO1, 9, &gpio_config);
    /* 初始化PHY INT引脚 */
    GPIO_PinInit(GPIO1, 10, &gpio_config);
    /* 在复位之前ENET_INT输出高电平 */
    GPIO_WritePinOutput(GPIO1, 10, 1);
    GPIO_WritePinOutput(GPIO1, 9, 0);
    GPIO_WritePinOutput(GPIO1, 9, 1);
    /*创建一个初始线程 */
    xTaskCreate(startup_thread, "app_thread", APP_THREAD_STACKSIZE/sizeof( portSTACK_TYPE ), NULL, DEFAULT_THREAD_PRIO, &startup_thread_handle);

    /* 启动FreeRTOS调度程序-此调用永远不会返回*/
    vTaskStartScheduler( );

    /* 除非vTaskStartScheduler中出现错误，否则永远不要到这里 */
    WPRINT_APP_ERROR(("Main() function returned - error" ));
    return 0;
}



/**
 *  初始线程功能-启动LwIP并调用app_main
 * 该函数使用tcpip_init函数启动LwIP，然后等待信号量，
 * 直到LwIP通过调用回调@ref tcpip_init_done指示其已
 * 启动。完成后，将调用应用程序的@ref app_main函数。
 * @param arg :  未使用-符合线程功能原型所需
 */

static void startup_thread( void *arg )
{
    SemaphoreHandle_t lwip_done_sema;
    UNUSED_PARAMETER( arg);

    WPRINT_APP_INFO( ( "\nPlatform " PLATFORM " initialised\n" ) );
    WPRINT_APP_INFO( ( "Started FreeRTOS" FreeRTOS_VERSION "\n" ) );
    WPRINT_APP_INFO( ( "Starting LwIP " LwIP_VERSION "\n" ) );

    /* 创建信号量以在LwIP完成初始化时发出信号 */
    lwip_done_sema = xSemaphoreCreateCounting( 1, 0 );
    if ( lwip_done_sema == NULL )
    {
        /*无法创建信号量 */
        WPRINT_APP_ERROR(("Could not create LwIP init semaphore"));
        return;
    }

    /*初始化LwIP，提供回调函数和回调信号量 */
    tcpip_init( tcpip_init_done, (void*) &lwip_done_sema );
    xSemaphoreTake( lwip_done_sema, portMAX_DELAY );
    vQueueDelete( lwip_done_sema );

    /* 运行主应用程序功能 */
    app_main( );

    /* 清理此启动线程*/
    vTaskDelete( startup_thread_handle );
}



/**
 *  LwIP初始化完成回调
 * @param arg : the handle for the semaphore to post (cast to a void pointer)
 */

static void tcpip_init_done( void * arg )
{
    SemaphoreHandle_t * lwip_done_sema = (SemaphoreHandle_t *) arg;
    xSemaphoreGive( *lwip_done_sema );
}

