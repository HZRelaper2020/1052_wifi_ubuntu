/**
  ******************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   i.MX RT + FreeRTOS + LWIP + AP6181��wifi��
  ******************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  i.MXRT1052������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
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

/* �ڷ��а��У�����UART��ӡ�������ƣ��Ӷ�����
��removef��malloc�������Ӷ����ż������ڴ�ʹ��*/
#ifndef DEBUG
#undef  WPRINT_APP_INFO
#define WPRINT_APP_INFO(args) printf args
#undef  WPRINT_APP_ERROR
#define WPRINT_APP_ERROR(args) printf args
#endif

/** @endcond */

/******************************************************
 *                    ����
 ******************************************************/
#define AP_SSID              "Relaper-H3C-2.4G"    /* ·������ */
#define AP_PASS              "hzrelaper"         /* ·������ */
#define AP_SEC               WICED_SECURITY_WPA2_MIXED_PSK  /* ·�ɼ��� */

#define COUNTRY              WICED_COUNTRY_AUSTRALIA    /* ѡ����� ��˵�Ĵ����ǵ��źŸ�ǿһЩ������ѡ��Ϊ�Ĵ����� */
#define USE_DHCP             WICED_TRUE     /* �Ƿ�ʹ�� DHCP */
#define IP_ADDR              MAKE_IPV4_ADDRESS( 192, 168,   5,  128 )  /* ���USE_DHCPΪWICED_TRUE������Ҫ */
#define GW_ADDR              MAKE_IPV4_ADDRESS( 192, 168,   5,   1 )   /* ���USE_DHCPΪWICED_TRUE������Ҫ */
#define NETMASK              MAKE_IPV4_ADDRESS( 255, 255, 255,   0 )    /* ���USE_DHCPΪWICED_TRUE������Ҫ */
/* #define PING_TARGET          MAKE_IPV4_ADDRESS( 192, 168,   1, 2 ) */  /* ���Ҫping�ض�IP���������أ���ȡ��ע��*/

#define PING_RCV_TIMEOUT     (1000)    /** ping���ճ�ʱ-�Ժ���Ϊ��λ*/
#define PING_DELAY           (1000)    /** ping��Ӧ/��ʱ����һ��ping����֮����ӳ�-�Ժ���Ϊ��λ */
#define PING_ID              (0xAFAF)
#define PING_DATA_SIZE       (32)      /** ping �������ݴ�С*/
#define JOIN_TIMEOUT         (10000)   /** ������������ĳ�ʱʱ�䣨�Ժ���Ϊ��λ��= 10��*/
#define APP_THREAD_STACKSIZE (5120)


/******************************************************
 *               ��̬��������
 ******************************************************/
extern void netio_init(void);
static void ping_prepare_echo( struct icmp_echo_hdr *iecho, uint16_t len );
static err_t ping_recv( int socket_hnd );
static err_t ping_send( int s, ip4_addr_t *addr );
static void tcpip_init_done( void * arg );
static void startup_thread( void *arg );
static void app_main( void );

/******************************************************
 *               ��������
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
����wifi lwip��Ϣ
*/
void Config_WIFI_LwIP_Info()
{
		ip4_addr_t ipaddr, netmask, gw;
    ip4_addr_t target;
    int socket_hnd;
    wwd_time_t send_time;
    int recv_timeout = PING_RCV_TIMEOUT;
    wwd_result_t result;
		
    /* ��ʼ�� Wiced */
    WPRINT_APP_INFO(("Starting Wiced v" WICED_VERSION "\n"));

    wwd_buffer_init( NULL);
    result = wwd_management_wifi_on( COUNTRY );
    if ( result != WWD_SUCCESS )
    {
        WPRINT_APP_INFO(("Error %d while starting WICED!\n", result));
    }

    /*���Լ���Wi-Fi���� */
    WPRINT_APP_INFO(("Joining : " AP_SSID "\n"));
    while ( wwd_wifi_join( &ap_ssid, AP_SEC, (uint8_t*) AP_PASS, sizeof( AP_PASS ) - 1, NULL, WWD_STA_INTERFACE ) != WWD_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to join  : " AP_SSID "   .. retrying\n"));
    }
    WPRINT_APP_INFO(("Successfully joined : " AP_SSID "\n"));

    /* ����IP����*/
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
            /* �ȴ� */
            sys_msleep( 10 );
        }
    }

    WPRINT_APP_INFO( ( "Network ready IP: %s\n", ip4addr_ntoa(netif_ip4_addr(&wiced_if))));
		    /*�򿪱����׽��ֽ���ping */
    if ( ( socket_hnd = lwip_socket( AF_INET, SOCK_RAW, IP_PROTO_ICMP ) ) < 0 )
    {
        WPRINT_APP_ERROR(( "unable to create socket for Ping\n" ));
        return;
    }

    /* �ڱ����׽��������ý��ճ�ʱ���Ա�ping�ᳬʱ. */
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
 *               ���ܶ���
 ******************************************************/

/**
 * Main Ping app
 *
 * ��ʼ��Wiced�������������磬Ȼ��һֱpingָ����IP��ַ��
 */

static void app_main( void )
{
   
		Config_WIFI_LwIP_Info();
		
    /* Loop forever */
    while ( 1 )
    {
        err_t result;
#if 0
        /* ����ping*/
        if ( ping_send( socket_hnd, &target ) != ERR_OK )
        {
            WPRINT_APP_ERROR(( "Unable to send Ping\n" ));
            return;
        }

        /* ��¼ʱ��ping�ѷ��� */
        send_time = host_rtos_get_time( );

        /* �ȴ�ping�ظ� */
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
        /*�ȴ�ֱ����һ��ping��ʱ�� */
        sys_msleep( PING_DELAY );
    }

    /* �ػ�����-��������ѭ����δʹ�� */
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
 *  ׼������ICMP�������ݰ�������
 *
 *  @param iecho  : ָ��icmp_echo_hdr�ṹ��ָ�룬�������й���ICMP���ݰ�
 *  @param len    : ���ݸ�iecho���������ݰ����������ֽڳ���
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

    /* ��һЩ��������������ݻ����� */
    for ( i = 0; i < PING_DATA_SIZE; i++ )
    {
        ( (char*) iecho )[sizeof(struct icmp_echo_hdr) + i] = i;
    }

    iecho->chksum = inet_chksum( iecho, len );
}



/**
 *  ����һ��Ping
 *
 * ʹ��ָ�����׽��ֽ�ICMP��������Ping�����͵�ָ����IP��ַ��
 *
 *  @param socket_hnd :��ͨ���䷢��ping����ı����׽��ֵľ��
 *  @param addr       :�����䷢��ping�����IP��ַ
 *
 *  @return ����ɹ����ͣ��򷵻�ERR_OK������ڴ治�㣬�򷵻�ERR_MEM�����򣬷���ERR_VAL
 */
static err_t ping_send( int socket_hnd, ip4_addr_t *addr )
{
    int err;
    struct icmp_echo_hdr *iecho;
    struct sockaddr_in to;
    size_t ping_size = sizeof(struct icmp_echo_hdr) + PING_DATA_SIZE;

    /* Ϊ���ݰ������ڴ�*/
    if ( !( iecho = (struct icmp_echo_hdr*) mem_malloc( ping_size ) ) )
    {
        return ERR_MEM;
    }

    /* ����ping���� */
    ping_prepare_echo( iecho, ping_size );

    /*����ping���� */
    to.sin_len = sizeof( to );
    to.sin_family = AF_INET;
    to.sin_addr.s_addr = addr->addr;

    err = lwip_sendto( socket_hnd, iecho, ping_size, 0, (struct sockaddr*) &to, sizeof( to ) );

    /* �ͷ� �� */
    mem_free( iecho );

    return ( err ? ERR_OK : ERR_VAL );
}


/**
 *  �յ�Ping�ظ�
 *  �ȴ�ʹ��ָ�����׽��ֽ���ICMP���Դ𸴣�Ping�𸴣��� �����кź�ID������
 *  �η��͵�ping���бȽϣ����ƥ�䣬�򷵻�ERR_OK����ʾ��Ч��ping��Ӧ��
 *  @param socket_hnd : �����׽��ֵľ����ͨ���þ�����յ�ping��
 *
 *  @return  ����յ���Ч�𸴣���ΪERR_OK������ΪERR_TIMEOUT
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
                return ERR_OK; /*�յ������ظ�-�ɹ����� */
            }
        }
    }

    return ERR_TIMEOUT; /* ��ʱǰδ�յ���Ч�Ļ����ظ� */
}

static void BOARD_USDHCClockConfiguration(void)
{
    /*��ϵͳpll PFD2������Ƶ������Ϊ18*/
    CLOCK_InitSysPfd(kCLOCK_Pfd0, 0x12U);
    /* ����USDHCʱ��Դ�ͷ�Ƶ��*/
    CLOCK_SetDiv(kCLOCK_Usdhc2Div, 0U);
    CLOCK_SetMux(kCLOCK_Usdhc2Mux, 1U);
}



/**
 * ������
 */
// .h
void aaa();
// .c
void aaa(){
}

int main( void )
{
    gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    /* ��ʼ���ڴ����Ԫ */
    BOARD_ConfigMPU();
    /* ��ʼ������������ */
    BOARD_InitPins();
    /* ��ʼ��������ʱ�� */
    BOARD_BootClockRUN();
    /* ��ʼ��SDIOʱ�� */
    BOARD_USDHCClockConfiguration();
    /* ��ʼ�����Դ��� */
    BOARD_InitDebugConsole();
		wifi_reset();
    /* ��ʼ��LED */
    LED_GPIO_Config();
    /* ����SDIO�ж����ȼ� */
    NVIC_SetPriority(USDHC2_IRQn, 5U);

    IOMUXC_EnableMode(IOMUXC_GPR, kIOMUXC_GPR_ENET1TxClkOutputDir, true);
    /* ��ʼ��PHY RESET���� */
    GPIO_PinInit(GPIO1, 9, &gpio_config);
    /* ��ʼ��PHY INT���� */
    GPIO_PinInit(GPIO1, 10, &gpio_config);
    /* �ڸ�λ֮ǰENET_INT����ߵ�ƽ */
    GPIO_WritePinOutput(GPIO1, 10, 1);
    GPIO_WritePinOutput(GPIO1, 9, 0);
    GPIO_WritePinOutput(GPIO1, 9, 1);
    /*����һ����ʼ�߳� */
    xTaskCreate(startup_thread, "app_thread", APP_THREAD_STACKSIZE/sizeof( portSTACK_TYPE ), NULL, DEFAULT_THREAD_PRIO, &startup_thread_handle);

    /* ����FreeRTOS���ȳ���-�˵�����Զ���᷵��*/
    vTaskStartScheduler( );

    /* ����vTaskStartScheduler�г��ִ��󣬷�����Զ��Ҫ������ */
    WPRINT_APP_ERROR(("Main() function returned - error" ));
    return 0;
}



/**
 *  ��ʼ�̹߳���-����LwIP������app_main
 * �ú���ʹ��tcpip_init��������LwIP��Ȼ��ȴ��ź�����
 * ֱ��LwIPͨ�����ûص�@ref tcpip_init_doneָʾ����
 * ��������ɺ󣬽�����Ӧ�ó����@ref app_main������
 * @param arg :  δʹ��-�����̹߳���ԭ������
 */

static void startup_thread( void *arg )
{
    SemaphoreHandle_t lwip_done_sema;
    UNUSED_PARAMETER( arg);

    WPRINT_APP_INFO( ( "\nPlatform " PLATFORM " initialised\n" ) );
    WPRINT_APP_INFO( ( "Started FreeRTOS" FreeRTOS_VERSION "\n" ) );
    WPRINT_APP_INFO( ( "Starting LwIP " LwIP_VERSION "\n" ) );

    /* �����ź�������LwIP��ɳ�ʼ��ʱ�����ź� */
    lwip_done_sema = xSemaphoreCreateCounting( 1, 0 );
    if ( lwip_done_sema == NULL )
    {
        /*�޷������ź��� */
        WPRINT_APP_ERROR(("Could not create LwIP init semaphore"));
        return;
    }

    /*��ʼ��LwIP���ṩ�ص������ͻص��ź��� */
    tcpip_init( tcpip_init_done, (void*) &lwip_done_sema );
    xSemaphoreTake( lwip_done_sema, portMAX_DELAY );
    vQueueDelete( lwip_done_sema );

    /* ������Ӧ�ó����� */
    app_main( );

    /* ����������߳�*/
    vTaskDelete( startup_thread_handle );
}



/**
 *  LwIP��ʼ����ɻص�
 * @param arg : the handle for the semaphore to post (cast to a void pointer)
 */

static void tcpip_init_done( void * arg )
{
    SemaphoreHandle_t * lwip_done_sema = (SemaphoreHandle_t *) arg;
    xSemaphoreGive( *lwip_done_sema );
}

