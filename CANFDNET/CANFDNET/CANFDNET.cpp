// CANFDNET.cpp : CANFD application entry point

#include "stdafx.h"
#include "zlgcan.h"
#include <iostream>
#include <thread>
#include <ctime>
#include <cstdio>


// Cross-platform sprintf
#ifdef LINUX_BUILD
#define sprintf_s(buf, size, ...) snprintf(buf, size, __VA_ARGS__)
#endif


// Print mutex
CRITICAL_SECTION print_mutex;


// ��ӡ������
#define LOCKED_BLOCK(code) do { \
    EnterCriticalSection(&print_mutex); \
    code; \
    LeaveCriticalSection(&print_mutex); \
} while(0)


// �߳����б�ʶ
int g_thd_run = 1;


// ʾ�� ����CAN����       
void Construct_CAN_Frame(ZCAN_Transmit_Data& can_data, canid_t id, UINT delay = 0)
{
	memset(&can_data, 0, sizeof(can_data));
	can_data.frame.can_id = MAKE_CAN_ID(id, 0, 0, 0);
	can_data.frame.can_dlc = 8;             // ���ݳ���
	can_data.transmit_type = 0;             // ��������
	// can_data.frame.__pad |= TX_ECHO_FLAG;	// ���ͻ���

	if (delay > 0){
		can_data.frame.__pad |= 0x80;			// ������ʱ��־λ ��0x80Ϊ1ms���ȣ�0xC0Ϊ0.1ms����
		can_data.frame.__res0 = LOBYTE(delay);	// ֡������ֽ�
		can_data.frame.__res1 = HIBYTE(delay);	// ֡������ֽ�
	}

	for (int i = 0; i < 8; ++i){
		can_data.frame.data[i] = i;
	}
}


// ʾ�� ����CANFD����
void Construct_CANFD_Frame(ZCAN_TransmitFD_Data& canfd_data, canid_t id, UINT delay = 0)
{
	memset(&canfd_data, 0, sizeof(canfd_data));
	canfd_data.frame.can_id = id;
	canfd_data.frame.len = 64;				// ���ݳ���
	canfd_data.transmit_type = 0;			// ��������
	canfd_data.frame.flags |= CANFD_BRS;	// CANFD���� 0x1
	// canfd_data.frame.flags |= TX_ECHO_FLAG;	// ���ͻ��� 0x20

	if (delay > 0){
		canfd_data.frame.flags |= 0x80;				// ������ʱ��־λ ��0x80Ϊ1ms���ȣ�0xC0Ϊ0.1ms����
		canfd_data.frame.__res0 = LOBYTE(delay);	// ֡������ֽ�
		canfd_data.frame.__res1 = HIBYTE(delay);	// ֡������ֽ�
	}

	for (int i = 0; i < 64; ++i) {
		canfd_data.frame.data[i] = i;
	}
}


// ʾ�� ����ϲ�����CAN/CANFD����
void Construct_Data_Frame(ZCANDataObj& data, int ch, canid_t id, bool is_fd, UINT delay = 0) {
	memset(&data, 0, sizeof(data));
	data.dataType = ZCAN_DT_ZCAN_CAN_CANFD_DATA;
	data.chnl = ch;

	ZCANCANFDData& can_data = data.data.zcanCANFDData;
	can_data.frame.can_id = MAKE_CAN_ID(id, 0, 0, 0);
	can_data.frame.len = is_fd ? 64 : 8;
	can_data.flag.unionVal.transmitType = 0;   // ��������
	// can_data.flag.unionVal.txEchoRequest = 1;  // ���÷��ͻ���
	can_data.flag.unionVal.frameType = is_fd ? 1 : 0;
	can_data.flag.unionVal.txDelay = ZCAN_TX_DELAY_UNIT_MS; // ���з�����ʱ��λ ms
	can_data.timeStamp = delay;                             //  ������ʱʱ�䣬���ֵ 65535
	for (int i = 0; i < can_data.frame.len; i++)
		can_data.frame.data[i] = i;
}


// �����������߳�
void thread_busload(DEVICE_HANDLE dev, int chn_idx)
{
	char path[24] = {};
	sprintf_s(path, 24, "%d/get_bus_usage/1", chn_idx);

	while (g_thd_run) {
		BusUsage* pUse = (BusUsage*)ZCAN_GetValue(dev, path);
		if (NULL != pUse)
			printf("����������: %d%%��%d֡/��\n", pUse->nBusUsage / 100, pUse->nFrameCount);
		//else
		//	printf("NULL\n");
		Sleep(500);
	}
}


// Receive thread
void thread_task(CHANNEL_HANDLE chn)
{
	ZCAN_Receive_Data canData[100] = {};
	ZCAN_ReceiveFD_Data canfdData[100] = {};
#ifdef LINUX_BUILD
	int chn_idx = (uintptr_t)chn & 0x000000FF;   // Channel index
#else
	int chn_idx = (unsigned int)chn & 0x000000FF;	// ͨ����
#endif

	while (g_thd_run)
	{
		if (ZCAN_GetReceiveNum(chn, 0))		// 0-CAN
		{
			uint32_t ReceiveNum = ZCAN_Receive(chn, canData, 100, 10);
			for (int i = 0; i < ReceiveNum; i++) {
				LOCKED_BLOCK({
					printf("CHN:%d [%lld] CAN  ", chn_idx, canData[i].timestamp);		// ͨ����ʱ���

					printf(IS_EFF(canData[i].frame.can_id) ? "��չ֡ " : "��׼֡ ");		// ֡����
					//printf(IS_RTR(canData[i].frame.can_id) ? " Զ��֡ " : " ����֡ ");	// ֡��ʽ
					printf(IS_TX_ECHO(canData[i].frame.__pad) ? "Tx " : "Rx ");	// ����

					printf("ID:0x%X Data: ", GET_ID(canData[i].frame.can_id));			// ID 
					for (int j = 0; j < canData[i].frame.can_dlc; j++){					// ����
						printf("%02X ", canData[i].frame.data[j]);
					}
					printf("\n");
				});
			}
		}

		if (ZCAN_GetReceiveNum(chn, 1))		// 1-CANFD
		{
			uint32_t ReceiveNum = ZCAN_ReceiveFD(chn, canfdData, 100, 10);
			for (int i = 0; i < ReceiveNum; i++)
			{
				LOCKED_BLOCK({
					printf("CHN:%d [%lld] CANFD", chn_idx, canfdData[i].timestamp);			// ͨ����ʱ���
					printf((canfdData[i].frame.flags & 0x01) == 0x01 ? "���� " : "");		// ����

					printf(IS_EFF(canfdData[i].frame.can_id) ? " ��չ֡ " : " ��׼֡ ");		// ֡����
					//printf(IS_RTR(canfdData[i].frame.can_id) ? " Զ��֡ " : " ����֡ ");	// ֡��ʽ
					printf(IS_TX_ECHO(canfdData[i].frame.flags) ? " Tx " : " Rx ");	// ����

					printf("ID:0x%X Data: ", GET_ID(canfdData[i].frame.can_id));			// ID
					for (int j = 0; j < canfdData[i].frame.len; j++) {						// ����
						printf("%02x ", canfdData[i].frame.data[j]);
					}
					printf("\n");
				});
			}
		}
		Sleep(10);
	}
}


// �ϲ������߳�
void thread_task_merge(DEVICE_HANDLE dev)
{
	ZCANDataObj dataObj[100];

	while (g_thd_run) {
		uint32_t RecNum = ZCAN_GetReceiveNum(dev, 2);  // �ϲ�����
		if (RecNum > 0) {
			uint32_t ReceiveNum = ZCAN_ReceiveData(dev, dataObj, 100, 10);
			for (int i = 0; i < ReceiveNum; i++) {
				if (dataObj[i].dataType == 1) {  // CAN / CANFD
					ZCANCANFDData canfdData = dataObj[i].data.zcanCANFDData;
					LOCKED_BLOCK({
						printf("CHN:%d [%llu] ", dataObj[i].chnl, (unsigned long long)canfdData.timeStamp);			// ͨ����ʱ���
						printf((canfdData.flag.rawVal & 1) == 0 ? "CAN" : "CANFD");				// CAN CANFD
						printf((canfdData.frame.flags & 1) == 1 ? "���� " : "");					// ����

						printf(IS_EFF(canfdData.frame.can_id) ? " ��չ֡ " : " ��׼֡ ");			// ֡����
						//printf(IS_RTR(canfdData.frame.can_id) ? " Զ��֡ " : " ����֡ ");			// ֡��ʽ
						printf(((canfdData.flag.rawVal & (1 << 9)) != 0) ? " TX " : " RX ");	// ����

						printf("ID:0x%X ", GET_ID(canfdData.frame.can_id));						// ID
						for (int j = 0; j < canfdData.frame.len; j++) {							// ����
							printf("%02x ", canfdData.frame.data[j]);
						}
						printf("\n");
					});
				}
			}
		}
		Sleep(10);
	}
}


// ����ʾ��
void Send_test(DEVICE_HANDLE dev, CHANNEL_HANDLE chn){
	const int send_num = 10;
	int send_count = 0;

	// CAN
	ZCAN_Transmit_Data canData[send_num];
	memset(canData, 0, sizeof(canData));
	for (int i = 0; i < send_num; ++i) {
		Construct_CAN_Frame(canData[i], i);
	}
	send_count = ZCAN_Transmit(chn, canData, send_num);

	LOCKED_BLOCK({
		printf("\n���� %d ��CAN����\n", send_count);
	});

	// CANFD
	ZCAN_TransmitFD_Data canfdData[send_num];
	memset(canfdData, 0, sizeof(canfdData));
	for (int i = 0; i < send_num; ++i) {
		Construct_CANFD_Frame(canfdData[i], i);
	}
	send_count = ZCAN_TransmitFD(chn, canfdData, send_num);

	LOCKED_BLOCK({
		printf("\n���� %d ��CANFD����\n", send_count);
	});

	Sleep(10);
	// �ϲ�����
	ZCANDataObj dataObj[10] = {};
	for (int i = 0; i < 10; ++i)
		Construct_Data_Frame(dataObj[i], 0, i, false);
	int ret_count = ZCAN_TransmitData(dev, dataObj, 10);		// �豸���

	LOCKED_BLOCK({
		printf("\n���� %d ���ϲ�����\n", send_count);
	});
}


// ��ʱ����
void AutoSend_test(DEVICE_HANDLE dev)
{
	// CAN
	ZCAN_AUTO_TRANSMIT_OBJ auto_can;
	memset(&auto_can, 0, sizeof(auto_can));
	auto_can.index = 0;								// �������������ֲ�ͬ�Ķ�ʱ����
	auto_can.enable = 1;							// 1-ʹ�� 0-�ر�
	auto_can.interval = 500;						// ���ڣ���λms
	Construct_CAN_Frame(auto_can.obj, 0x100);				// ���챨�ģ��ͷ��͵ķ�ʽһ��
	if (0 == ZCAN_SetValue(dev, "0/auto_send", &auto_can)){
		printf("���ö�ʱ���� CAN ʧ��\n");
	}
	else{
		printf("���ö�ʱ���� CAN �ɹ�\n");
	}

	// CANFD
	ZCANFD_AUTO_TRANSMIT_OBJ auto_canfd;
	memset(&auto_canfd, 0, sizeof(auto_canfd));
	auto_canfd.index = 1;
	auto_canfd.enable = 1;
	auto_canfd.interval = 1000;
	Construct_CANFD_Frame(auto_canfd.obj, 0x200);
	if (0 == ZCAN_SetValue(dev, "0/auto_send_canfd", &auto_canfd)){
		printf("���ö�ʱ���� CANFD ʧ��\n");
	}
	else{
		printf("���ö�ʱ���� CANFD �ɹ�\n");
	}

	if (0 == ZCAN_SetValue(dev, "0/apply_auto_send", "0")){
		printf("������ʱ����ʧ��\n");
	}
	else{
		printf("������ʱ���ͳɹ�\n");
	}

	//// ��ʱ���͹����У����Ը����޸�
	//Sleep(2000);
	//memset(&auto_can, 0, sizeof(auto_can));
	//auto_can.index = 0;
	//auto_can.enable = 1;
	//auto_can.interval = 1000;
	//Construct_CAN_Frame(auto_can.obj, 0x555);
	//ZCAN_SetValue(dev, "0/auto_send", (const char*)&auto_can);			// ���ö�ʱ����

	//if (0 == ZCAN_SetValue(dev, "0/apply_auto_send", "0")){
	//	printf("���Ƕ�ʱ����ʧ��\n");
	//}
	//else{
	//	printf("����������ʱ���ͳɹ�\n");
	//}
}


// ���з���
void QueueSend_test(DEVICE_HANDLE dev, CHANNEL_HANDLE chn)
{
	// ���з���
	int free_count = *(int*)ZCAN_GetValue(dev, "0/get_device_available_tx_count/1"); // ���û���
	printf("���з��Ϳ��л���: %d\n", free_count);

	// CAN
	free_count = *(int*)ZCAN_GetValue(dev, "0/get_device_available_tx_count/1");
	if (free_count > 50) {
		ZCAN_Transmit_Data tran_data[50] = {};
		memset(tran_data, 0, sizeof(tran_data));
		for (int i = 0; i < 50; ++i)
			Construct_CAN_Frame(tran_data[i], 0x50, 100);	// ������з��ͱ��ģ�����������������
		int ret_count = ZCAN_Transmit(chn, tran_data, 50);
	}

	// CANFD
	free_count = *(int*)ZCAN_GetValue(dev, "0/get_device_available_tx_count/1");
	if (free_count > 50) {
		ZCAN_TransmitFD_Data tranfd_data[50] = {};
		memset(tranfd_data, 0, sizeof(tranfd_data));
		for (int i = 0; i < 50; ++i)
			Construct_CANFD_Frame(tranfd_data[i], 0x150, 100);
		int ret_count = ZCAN_TransmitFD(chn, tranfd_data, 50);
	}

	// �ϲ�����
	free_count = *(int*)ZCAN_GetValue(dev, "0/get_device_available_tx_count/1");
	if (free_count > 50) {
		ZCANDataObj tran_data[50] = {};
		for (int i = 0; i < 50; ++i)
			Construct_Data_Frame(tran_data[i], 0, i, false, 100);
		int ret_count = ZCAN_TransmitData(dev, tran_data, 50);
	}

	// �ӳ�һ��ʱ��󣬻�ȡһ�¶��з��ͻ���
	printf("�س���ն��з���\n");
	getchar();
	free_count = *(int*)ZCAN_GetValue(dev, "0/get_device_available_tx_count/1"); // ���û���
	printf("���з��Ϳ��л���: %d\n", free_count);

	// ��ն��з���
	ZCAN_SetValue(dev, "0/clear_delay_send_queue", "0");
	printf("��ն��з���\n");

}


// ��̬���� (ֻ��CANFDNET-400U��CANFDNET-800U֧��)
void Dynamic_config_test(DEVICE_HANDLE dev, int chnidx)
{
	std::string key;
	std::string value;
	ZCAN_DYNAMIC_CONFIG_DATA CfgData;

	// �ٲ�������
	memset(&CfgData, 0, sizeof(ZCAN_DYNAMIC_CONFIG_DATA));
	key = ZCAN_DYNAMIC_CONFIG_CAN_NOMINALBAUD;					
	sprintf_s(CfgData.key, key.length(), key.c_str(), chnidx);
	memcpy(CfgData.value, "500000", sizeof(CfgData.value));
	if (0 == ZCAN_SetValue(dev, "0/add_dynamic_data", (void *)&CfgData)) {
		printf("��̬���á��ٲ������ʡ�ʧ��\n");
		return;
	}

	// ����������
	memset(&CfgData, 0, sizeof(ZCAN_DYNAMIC_CONFIG_DATA));
	key = ZCAN_DYNAMIC_CONFIG_CAN_DATABAUD;						
	sprintf_s(CfgData.key, key.length(), key.c_str(), chnidx);
	memcpy(CfgData.value, "2000000", sizeof(CfgData.value));
	if (0 == ZCAN_SetValue(dev, "0/add_dynamic_data", (void *)&CfgData)) {
		printf("��̬���á����������ʡ�ʧ��\n");
		return;
	}

	// �ն˵���
	memset(&CfgData, 0, sizeof(ZCAN_DYNAMIC_CONFIG_DATA));
	key = ZCAN_DYNAMIC_CONFIG_CAN_USERES;
	sprintf_s(CfgData.key, key.length(), key.c_str(), chnidx);
	memcpy(CfgData.value, "1", sizeof(CfgData.value));				// 0-�ر� 1-��
	if (0 == ZCAN_SetValue(dev, "0/add_dynamic_data", (void *)&CfgData)) {
		printf("��̬���á��ն˵��衱ʧ��\n");
		return;
	}

	int ref = 0;	//0-��̬���ã����粻���棩 1-�־�����
	if (0 == ZCAN_SetValue(dev, "0/apply_dynamic_data", (void *)&ref))		//ʹ�ܶ�̬������Ч��
		printf("��̬����ʧ��\n");
	else
		printf("��̬���óɹ�\n");
}


// ��ʼ��CANFDNETͨ������ͨ����ʽ��ͨ��ָ������������飩
bool Init_chn_CANFDNET_ALL(DEVICE_HANDLE dev, int workMode, const char* ip, const char* work_port, const char* local_port, int chnmax, CHANNEL_HANDLE* outChn) {
	// ����ģʽ����
	if (workMode == 0) {
		if (0 == ZCAN_SetValue(dev, "0/work_mode", "0")) {			// PCΪ�ͻ���
			printf("���ù���ģʽʧ��\n");
			return NULL;
		}
		if (0 == ZCAN_SetValue(dev, "0/ip", ip)) {					// Ŀ��IP��ַ��Ҳ�����豸��IP��ַ
			printf("����IP��ַʧ��\n");
			return NULL;
		}
		if (0 == ZCAN_SetValue(dev, "0/work_port", work_port)) {	// Ŀ��˿ڣ����ù������豸�ġ����ض˿ڡ���
			printf("���ñ��ض˿�ʧ��\n");
			return NULL;
		}
	}
	else if (workMode == 1) {
		if (0 == ZCAN_SetValue(dev, "0/work_mode", "1")) {			// PC��Ϊ������
			printf("���ù���ģʽʧ��\n");
			return NULL;
		}
		if (0 == ZCAN_SetValue(dev, "0/local_port", local_port)) {	// ���ض˿ڣ���Ӧ���ù������豸�ġ�Ŀ��˿ڡ���
			printf("���ñ��ض˿�ʧ��\n");
			return NULL;
		}
	}
	else if (workMode == 2) {										// UDP
		if (0 == ZCAN_SetValue(dev, "0/local_port", local_port)) {	// ���ض˿�
			printf("���ñ��ض˿�ʧ��\n");
			return NULL;
		}
		if (0 == ZCAN_SetValue(dev, "0/ip", ip)) {					// Ŀ��IP��ַ
			printf("����IP��ַʧ��\n");
			return NULL;
		}
		if (0 == ZCAN_SetValue(dev, "0/work_port", work_port)) {	// Ŀ��˿�
			printf("���ñ��ض˿�ʧ��\n");
			return NULL;
		}
	}

	// UDPģʽ����Ҫ

	// ��ʼ��ͨ������
	ZCAN_CHANNEL_INIT_CONFIG config;
	memset(&config, 0, sizeof(config));

	// ѭ������ͨ��
	for (int i = 0; i < chnmax; i++) {
		outChn[i] = ZCAN_InitCAN(dev, i, &config);		// InitCAN����Ϊ�˻�ȡ�ٿؾ��
		if (outChn[i] == INVALID_CHANNEL_HANDLE) {
			printf("��ʼ��ͨ�� %d ʧ��\n", i);
			return NULL;
		}
		if (ZCAN_StartCAN(outChn[i]) == STATUS_ERR) {	// �����Ľ�������
			printf("����ͨ�� %d ʧ��\n", i);
			return NULL;
		}
	}
	return true;
}


// ��ʼ��CANFDNETͨ������ͨ����ʽ
CHANNEL_HANDLE Init_chn_CANFDNET_ONE(DEVICE_HANDLE dev, int workMode, const char* ip, const char* work_port, const char* local_port, int chnIdx)
{
	// ����ģʽ����
	if (workMode == 0) {
		if (0 == ZCAN_SetValue(dev, "0/work_mode", "0")) {			// PCΪ�ͻ���
			printf("���ù���ģʽʧ��\n");
			return NULL;
		}
		if (0 == ZCAN_SetValue(dev, "0/ip", ip)) {					// Ŀ��IP��ַ��Ҳ�����豸��IP��ַ
			printf("����IP��ַʧ��\n");
			return NULL;
		}
		if (0 == ZCAN_SetValue(dev, "0/work_port", work_port)) {	// Ŀ��˿ڣ����ù������豸�ġ����ض˿ڡ���
			printf("���ñ��ض˿�ʧ��\n");
			return NULL;
		}
	}
	else if (workMode == 1) {
		if (0 == ZCAN_SetValue(dev, "0/work_mode", "1")) {			// PC��Ϊ������
			printf("���ù���ģʽʧ��\n");
			return NULL;
		}
		if (0 == ZCAN_SetValue(dev, "0/local_port", local_port)) {	// ���ض˿ڣ���Ӧ���ù������豸�ġ�Ŀ��˿ڡ���
			printf("���ñ��ض˿�ʧ��\n");
			return NULL;
		}
	}
	else if (workMode == 2) {										// UDP
		if (0 == ZCAN_SetValue(dev, "0/local_port", local_port)) {	// ���ض˿�
			printf("���ñ��ض˿�ʧ��\n");
			return NULL;
		}
		if (0 == ZCAN_SetValue(dev, "0/ip", ip)) {					// Ŀ��IP��ַ
			printf("����IP��ַʧ��\n");
			return NULL;
		}
		if (0 == ZCAN_SetValue(dev, "0/work_port", work_port)) {	// Ŀ��˿�
			printf("���ñ��ض˿�ʧ��\n");
			return NULL;
		}
	}
	
	// ��ʼ��ͨ��
	CHANNEL_HANDLE chn = NULL;
	ZCAN_CHANNEL_INIT_CONFIG config;		
	memset(&config, 0, sizeof(config));			// ����Ҫ����κ�����
	chn = ZCAN_InitCAN(dev, chnIdx, &config);	// �����ӵ�����£�ͨ���� chnIdx ҲҪ��Ӧ
	if (chn == INVALID_CHANNEL_HANDLE) {
		printf("��ʼ��ͨ��ʧ��\n");
		return NULL;
	}

	// ����ͨ��
	if (ZCAN_StartCAN(chn) == STATUS_ERR) {
		printf("����ͨ��ʧ��\n");
		return NULL;
	}

	return chn;
}


#ifdef LINUX_BUILD
int main(int argc, char* argv[])
#else
int _tmain(int argc, _TCHAR* argv[])
#endif
{
	InitializeCriticalSection(&print_mutex);

	const int chnMax = 2;				// ͨ������
	std::thread thd_busload;			// �����������߳�
	std::thread thd_handle[chnMax];		// �����߳�
	DEVICE_HANDLE dev[chnMax] = {};		// �豸���
	CHANNEL_HANDLE chn[chnMax] = {};	// ͨ�����
	bool isMergeRec = 0;				// �Ƿ�ϲ�����

#if 1
	// �����ӵķ�ʽ���������ù����У���ѡ�ˡ�CAN0 CAN1...�� ��ͬһ�����������߿ͻ��ˡ�

	// ���豸
	dev[0] = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, 0, 0);
	if (dev[0] == INVALID_DEVICE_HANDLE) {
		printf("Failed to open device\n");
#ifdef LINUX_BUILD
		printf("Press Enter to continue...\n");
		getchar();
#else
		system("pause");
#endif
		return 0;
	}

	// ��ʼ��ͨ���������ӣ�PC��Ϊ��������ʱ�������ʱ2-3��֮����ȥ�����շ�������
	if (Init_chn_CANFDNET_ALL(dev[0], 0, "192.168.1.5", "8002", " ",chnMax, chn)){	// �豸���������ģʽ��IP���˿ڣ�ͨ��������ͨ����������ַ
		printf("�ɹ���ʼ��ͨ��\n");
	}
	else{
		printf("��ʼ��ʧ�ܣ��س��˳�\n");
		goto end;
	}
	Sleep(2000);
	// �����̵߳Ĵ���
	for (int i = 0; i < chnMax; i++) {
		// �رջ��߿����ϲ����գ�ֻ��Ҫ����һ��
		if (i == 0){
			const char* merge_value = isMergeRec ? "1" : "0";
			if (ZCAN_SetValue(dev[0], "0/set_device_recv_merge", merge_value) == STATUS_ERR) {
				printf("���ý��պϲ�(%s)ʧ��\n", merge_value);
			}
		}

		// �����̵߳Ĵ���
		if (isMergeRec && i == 0){
			thd_handle[i] = std::thread(thread_task_merge, dev[0]);	// һ���߳�ȥ���ռ���
		}
		else{
			thd_handle[i] = std::thread(thread_task, chn[i]);		// ÿ��ͨ��һ���߳̽���
		}
	}
#else
	// �����ӵķ�ʽ���������ù����У�ֻ��ѡ��"CANx" һ��ͨ����Ӧһ�����������߿ͻ��ˡ�

	// ��ʼ�����̴Ӵ��豸��ʼ
	std::string ports[8] = { "8002", "8001", "8002", "8003" };
	for (int i = 0; i < chnMax; i++){
		dev[i] = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, i, 0);
		if (dev[i] == INVALID_DEVICE_HANDLE) {
			printf("Failed to open device\n");
#ifdef LINUX_BUILD
			printf("Press Enter to continue...\n");
			getchar();
#else
			system("pause");
#endif
			return 0;
		}

		chn[i] = Init_chn_CANFDNET_ONE(dev[i], 0, "192.168.1.5", ports[i].c_str(), " ", i);
		if (chn[i] == NULL){
			printf("��ʼ��ʧ��\n");
			goto end;
		}
		else{
			printf("�ɹ���ʼ��ͨ��\n");
		}

		// �رջ��߿����ϲ����գ�ֻ��Ҫ����һ��
		const char* merge_value = isMergeRec ? "1" : "0";
		if (ZCAN_SetValue(dev[i], "0/set_device_recv_merge", merge_value) == STATUS_ERR) {
			printf("���ý��պϲ�(%s)ʧ��\n", merge_value);
		}

		// �����̵߳Ĵ���
		if (isMergeRec){
			thd_handle[i] = std::thread(thread_task_merge, dev[i]);	// һ���߳�ȥ���ռ���
		}
		else{
			thd_handle[i] = std::thread(thread_task, chn[i]);		// ÿ��ͨ��һ���߳̽���
		}
	}

#endif

	//// ��̬���ã�ֻ��CANFDNET-400U��CANFDNET-800U��֧�֣�����Ҫ������ͨ����ʹ��
	//Dynamic_config_test(dev[0], 0);


	//// ���������ʣ���Ҫ������ͨ����ʱ��ʹ��������������ز�����
	//thd_busload = std::thread(thread_busload, dev[0], 1);


	// ����ʾ��
	Send_test(dev[0], chn[0]);


	//// ��ʱ����ʾ��
	//AutoSend_test(dev[0]);


	//// ���з���ʾ��
	//QueueSend_test(dev[0], chn[0]);

end:
	getchar();							// �س�����
	ZCAN_SetValue(dev[0], "0/clear_auto_send", "0");		// �����ʱ����

	g_thd_run = 0;						// �߳����б�ʶ
	if (thd_busload.joinable())			// �����������߳�
		thd_busload.join();

	for (int i = 0; i < chnMax; i++) {
		if (thd_handle[i].joinable())	// �����߳�
			thd_handle[i].join();

		ZCAN_ResetCAN(chn[i]);			// �ر�ͨ��
	}
	ZCAN_CloseDevice(dev[0]);			// �ر��豸

	// system("pause");
	DeleteCriticalSection(&print_mutex);
	return 0;
}



