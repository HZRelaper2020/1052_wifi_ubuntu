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
#include "lwip/sockets.h"
#include "wiced_utilities.h"
#include "wwd_buffer_interface.h"


#if 0
int tcp_send_test(uint8_t* ip,uint16_t port,int delay)
{
	int sk = socket(AF_INET,SOCK_STREAM,0);
	if (sk < 0){
		ERROR("create socket failed\n");
		return -1;
	}

	struct sockaddr_in addr;
	memset(&addr,0,sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = inet_addr(ip);

	if (connect(sk,(struct sockaddr*)&addr,sizeof(addr))){
		ERROR("connect failed\n");
		return -1;
	}

	int times = 0;
	char buf[20];
	while(1){
		times++;
		snprintf(buf,sizeof(buf),"%08d",times);
		if (send(sk,buf,8,0) != 8){
			ERROR("send size failed\n");
			break;
		}
		vTaskDelay(delay);
	}
}

int tcp_echo_test(uint16_t port)
{
	int sk = socket(AF_INET,SOCK_STREAM,0);
	if (sk < 0){
		ERROR("create socket failed\n");
		return -1;
	}

	struct sockaddr_in addr;
	memset(&addr,0,sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = INADDR_ANY;

	if (bind(sk,(struct sockaddr*)&addr,sizeof(addr))){
		ERROR("bind address failed\n");
		return -1;
	}
	
	PRINTF("start listen %d...\n",port);

	if (listen(sk,4)){
		ERROR("listen socket failed\n");
		return -1;
	}


	char buf[200];

	while(1){
		socklen_t  addrlen = 0;
		memset(&addr,0,sizeof(addr));
		int client = accept(sk,(struct sockaddr*)&addr,&addrlen);
		if (client < 0){
			ERROR("accept socket failed\n");
			continue;
		}
		
		PRINTF("recv from %s:%d\n",inet_ntoa(addr.sin_addr),ntohs(addr.sin_port));
		while(1){
			int recvlen = recv(client,buf,sizeof(buf),0);
			if (recvlen < 0){
				ERROR("recv socket failed\n");
				break;
			}else if (recvlen == 0){
				break;
			}else if (recvlen > 0){
				if (send(client,buf,recvlen,0)){
					ERROR("send socket buf failed\n");
					break;
				}
			}

		}
		closesocket(client);
		
	}

	return 0;

_exit:
	closesocket(sk);
	return -1;
}
#endif
