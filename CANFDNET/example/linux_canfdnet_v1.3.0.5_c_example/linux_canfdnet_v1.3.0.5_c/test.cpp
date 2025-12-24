#include <stdio.h>
#include <string.h>
#include <thread>
#include <unistd.h>  
#include "./src/CANFDNET.h"
#include "./src/common.h"
#include "./src/zlgcan/zlgcan.h"

#define msleep(ms)  usleep((ms)*1000)
#define MAX_CHANNELS  4     // 通道数量

// 线程传输结构体
typedef struct {
    DEVICE_HANDLE device_handle;    // 设备句柄
    CHANNEL_HANDLE cHandle;         // 通道句柄
    int index;      // 通道号
    int total;      // 接收总数
} RX_CTX;

int rx_thd_run = 1;

// CAN报文       
void construct_can_frame(ZCAN_Transmit_Data& can_data, canid_t id, UINT delay = 0)
{
	memset(&can_data, 0, sizeof(can_data));
	can_data.transmit_type = 0;                 // 正常发送
	can_data.frame.can_id = MAKE_CAN_ID(id, 0, 0, 0);
	can_data.frame.can_dlc = 8;                 // 长度 0-8
	// can_data.frame.__pad |= TX_ECHO_FLAG;	// 发送回显

    if(delay != 0){ 
        can_data.frame.__pad = 0x80;            // 0x80 设置延时标志位，单位ms
        can_data.frame.__res0 = delay;          // 帧间隔低字节
        can_data.frame.__res1 = 0x0;            // 帧间隔高字节
    }

	for (int i = 0; i < 8; ++i)                 // 数据
		can_data.frame.data[i] = i;
}

// CANFD报文
void construct_canfd_frame(ZCAN_TransmitFD_Data& canfd_data, canid_t id, UINT delay = 0)
{
	memset(&canfd_data, 0, sizeof(canfd_data));
	canfd_data.transmit_type = 0;                   // 正常发送
	canfd_data.frame.can_id = MAKE_CAN_ID(id, 0, 0, 0);
	canfd_data.frame.len = 64;                      // 长度
	// canfd_data.frame.flags |= TX_ECHO_FLAG;		// 发送回显

	if(delay != 0){
        canfd_data.frame.flags = 0x80;          // 0x80 设置延时标志位，单位ms
	    canfd_data.frame.__res0 = delay;        // 帧间隔低字节
	    canfd_data.frame.__res1 = 0x0;          // 帧间隔高字节
    }

	for (int i = 0; i < 8; ++i)                 // 数据
		canfd_data.frame.data[i] = i;
}

// 接收线程
void * rx_thread(void *data)
{
    RX_CTX *ch_ctx = (RX_CTX *)data;
    DEVICE_HANDLE device_handle = ch_ctx->device_handle;
    CHANNEL_HANDLE cHandle = ch_ctx->cHandle;
    int chn_idx = ch_ctx->index;
    ZCAN_Receive_Data can_data[100];
    ZCAN_ReceiveFD_Data canfd_data[100];

    memset(can_data, 0, sizeof(can_data));
    memset(canfd_data, 0, sizeof(canfd_data));
    int len = 10;
    
    while(rx_thd_run){
        UINT count = ZCAN_GetReceiveNum(cHandle, 0);    // 0-CAN
        if (rx_thd_run && count > 0){
            int rcount = ZCAN_Receive(cHandle, can_data, len, 10);
            for(int i = 0; i < rcount; ++i){
                printf("chn: %d ", chn_idx);
                printf("ID: %x CAN ", can_data[i].frame.can_id);
                printf("%s Data: ", can_data[i].frame.__pad & TX_ECHO_FLAG ? "Tx" : "Rx");

                for(int j = 0; j < can_data[i].frame.can_dlc; ++j)
                    printf("%x ", can_data[i].frame.data[j]);
                printf("\n");
            }
        }

        UINT count_fd = ZCAN_GetReceiveNum(cHandle, 1);         // 1-CANFD
        if (rx_thd_run && count_fd > 0){
            int rcount = ZCAN_ReceiveFD(cHandle, canfd_data, len, 10);
            for(int i = 0; i < rcount; ++i){
                printf("chn: %d ", chn_idx);
                printf("ID: %x ", canfd_data[i].frame.can_id);
                printf("%s ", canfd_data[i].frame.flags & CANFD_BRS ? "CANFD加速" : "CANFD");
                printf("%s Data: ", canfd_data[i].frame.flags & TX_ECHO_FLAG ? "Tx" : "Rx");

                for(int j = 0; j < canfd_data[i].frame.len; ++j)
                    printf("%x ", canfd_data[i].frame.data[j]);
                printf("\n");
            }
        }
        msleep(10);
    }
    // printf("Channle %d thread exit\n", ch_ctx->index);
    pthread_exit(0);
}

int main()
{    
    pthread_t rx_threads[MAX_CHANNELS];     // 接收线程
    RX_CTX rx_ctx[MAX_CHANNELS];            // 线程上下文

    UINT devtype = ZCAN_CANFDNET_400U_TCP;  // 设备类型
    CHANNEL_HANDLE ch[MAX_CHANNELS] = {};   // 通道句柄

    // 打开设备
    DEVICE_HANDLE device_handle = ZCAN_OpenDevice(devtype, 0, 0);    // 设备句柄
    if (device_handle == INVALID_DEVICE_HANDLE) {
        printf("Open device failed! \n");
        return 0;
    }
    printf("Open device successfully!\n");

    

    // 循环初始化
    for (int i = 0; i < MAX_CHANNELS; ++i) {
        ZCAN_CHANNEL_INIT_CONFIG config;
        memset(&config, 0, sizeof(config));
        UINT val = 0;

        ch[i] = ZCAN_InitCAN(device_handle, i, &config);
        if (ch[i] == INVALID_CHANNEL_HANDLE) {
            printf("Init CAN %d failed!\n", i);
            return 0;
        }

        //// 合并接收 0-关闭 1-打开
        if(i == 0){
            val = 0;
            ZCAN_SetReference(devtype, 0, i, SETREF_SET_DATA_RECV_MERGE, &val);     
        }

        // ip地址，工作模式，目标端口号
        val = 0;
        ZCAN_SetReference(devtype, 0, i, CMD_TCP_TYPE, &val);                   // TCP
        ZCAN_SetReference(devtype, 0, i, CMD_DESIP, (void*)"192.168.0.178");    // IP
        val = 8000;
        ZCAN_SetReference(devtype, 0, i, CMD_DESPORT, &val);                    // 端口

        if (ZCAN_StartCAN(ch[i]) != STATUS_OK) {
            printf("Start CAN %d failed!\n", i);
            goto end;
        }

        rx_ctx[i].device_handle = device_handle;
        rx_ctx[i].cHandle = ch[i];
        rx_ctx[i].index = i;
        rx_ctx[i].total = 0;
        pthread_create(&rx_threads[i], NULL, rx_thread, &rx_ctx[i]);
    }
    
// 普通发送
#if 1
    // CAN 报文
    int send_count;
	ZCAN_Transmit_Data trans_data[10];
	memset(trans_data, 0, sizeof(trans_data));
	for (int i = 0; i < 10; ++i) {
		construct_can_frame(trans_data[i], 0x111);
	}
	send_count = ZCAN_Transmit(ch[0], trans_data, 10);
	printf("ZCAN_Transmit %d\n", send_count);

	// CANFD 报文
	ZCAN_TransmitFD_Data trans_fd_data[10];
	memset(trans_fd_data, 0, sizeof(trans_fd_data));
	for (int i = 0; i < 10; ++i) {
		construct_canfd_frame(trans_fd_data[i], 0x222);
	}
	send_count = ZCAN_TransmitFD(ch[0], trans_fd_data, 10);
	printf("ZCAN_TransmitFD %d\n", send_count);
#endif

    // 阻塞等待
    getchar();
    rx_thd_run = 0;
end:
    // 复位通道，关闭设备
    for (int i = 0; i < MAX_CHANNELS; ++i) {
        ZCAN_ResetCAN(ch[i]);
        pthread_join(rx_threads[i], NULL);
    }
    ZCAN_CloseDevice(device_handle);
    return 0;
}


