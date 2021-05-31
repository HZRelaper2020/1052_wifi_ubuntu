# wifi模块 ubuntu端，做成.a文件给eclipse调用


### 存在问题
* Wifi/WICED/WWD/internal/wwd_thread.c   362行 增加 host_rtos_delay_milliseconds(10)  不然死循环
