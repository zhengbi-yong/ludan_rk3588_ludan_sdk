#include "TcpClient.h"
#include "log.h"
#include <string.h>
#include "common.h"
#include <cassert>
#include <iostream>
#include "NetPacket.h"

#ifndef WIN32
#include <time.h>
#include <sys/time.h>
#endif

static const int socket_recv_buf_size = 4 * 1024 * 1024;
static const int socket_send_buf_size = 4 * 1024 * 1024;

static const int app_recv_buf_size = 64 * 1024;

TcpClient::TcpClient(const char* svrIp, unsigned short svrPort, UINT devType)
: m_svrIp(svrIp)
, m_svrPort(svrPort)
, m_devType(devType)
{
    memset(&m_socket, 0, sizeof(m_socket));
    m_socket.handle = (~0);

    m_recvbuf = new char[app_recv_buf_size];
    assert(m_recvbuf != 0);
}

TcpClient::~TcpClient()
{
    Stop();
    delete[] m_recvbuf;
    m_recvbuf = nullptr;
}

bool TcpClient::Start()
{
    if (!ConnectServer()) {
        return false;
    }

    return true;
}

bool TcpClient::IsStart()
{
    return m_bStarted;
}

bool TcpClient::ConnectServer()
{
    if (m_svrIp.empty()) {
		LOG_ERR("TCP connect %s:%d failed, empty dest ip!\n", m_svrIp.c_str(), m_svrPort);
        return false;
    }

    if (cc_socket_is_connect(&m_socket)) {
        DisconnectServer();
    }

    if (cc_socket_init(&m_socket) != CC_SOCKET_OK) {
		LOG_ERR("Init socket failed!\n");
        return false;
    }

    cc_socket_set_recv_snd_buf(&m_socket, socket_recv_buf_size, socket_send_buf_size);

    LOG_INFO("TCP connect %s:%d ...\n", m_svrIp.c_str(), m_svrPort);

    if (cc_socket_connect(&m_socket, m_svrIp.c_str(), m_svrPort) != CC_SOCKET_OK) {
		LOG_ERR("TCP connect failed!\n");
        cc_socket_close(&m_socket);
        return false;
    }

    LOG_INFO("TCP connect done.\n");

    // run recv thread
	m_stopRecvLoop = false;
	m_recvThd = std::thread(std::bind(&TcpClient::TcpRecvLoop, this));

#ifdef WIN32
    thread_set_priority(m_recvThd, THREAD_PRIORITY_TIME_CRITICAL);
#endif

    m_bStarted = true;
    return true;
}

bool TcpClient::DisconnectServer()
{
	m_stopRecvLoop = true;
	
    if (cc_socket_is_connect(&m_socket)) {
        cc_socket_close(&m_socket);
    }

#ifdef WIN32
	LARGE_INTEGER nFreq;
	LARGE_INTEGER nBeginTime;
	LARGE_INTEGER nEndTime;
	QueryPerformanceFrequency(&nFreq);//获取时钟周期
	QueryPerformanceCounter(&nBeginTime);
#endif

	if (m_recvThd.joinable()) {
		m_recvThd.join();
	}

#ifdef WIN32
	QueryPerformanceCounter(&nEndTime);
	double ddf = (double)nFreq.QuadPart;
	double ddt = (double)(nEndTime.QuadPart - nBeginTime.QuadPart);
	double time = ddt / ddf;
	// std::cout << __FUNCTION__ << "^^^^ joinable time is: " << time * 1000 << "ms" << std::endl;
	// LOG_ALWAYS("%s^^^^ DisconnectServer time is: %fms", __FUNCTION__, time * 1000);
#endif

    m_bStarted = false;

    return true;
}

bool TcpClient::ReconnectServer()
{
	LOG_INFO("TCP reconnect %s:%d ...\n", m_svrIp.c_str(), m_svrPort);
	
    if (cc_socket_is_connect(&m_socket)) {
        cc_socket_close(&m_socket);
    }
	
	if (cc_socket_init(&m_socket) != CC_SOCKET_OK) {
		LOG_ERR("Init socket failed!\n");
        return false;
    }

    cc_socket_set_recv_snd_buf(&m_socket, socket_recv_buf_size, socket_send_buf_size);

	if (cc_socket_connect(&m_socket, m_svrIp.c_str(), m_svrPort) != CC_SOCKET_OK) {
		LOG_ERR("Failed to reconnect!\n");
        cc_socket_close(&m_socket);
        return false;
    }

    LOG_INFO("TCP reconnect done.\n");

	return true;
}

void TcpClient::TcpRecvLoop()
{
#ifdef WIN32
	LOG_ALWAYS("TcpClient TcpRecvLoop thread,tid = %u", GetCurrentThreadId());
#endif
	while (!m_stopRecvLoop)
	{
        memset(m_recvbuf, 0, app_recv_buf_size);
        int len = cc_socket_recv(&m_socket, m_recvbuf, app_recv_buf_size);
        //LOG_INFO("TCP recv %d bytes\n", len);
        if (len > 0) {
			if (m_recvCb) {
				char src_addr[64];
				memset(src_addr, 0, sizeof(src_addr));
				snprintf(src_addr, sizeof(src_addr) - 1, "%s:%d", m_socket.ip, m_socket.port);
                m_recvCb(src_addr, m_recvbuf, len);
            }
		}
		else if (len < 0) {
			// tcp disconnected, reconnect
#ifdef WIN32
            LOG_WARN("TCP disconneced, try reconnect\n");
#endif
			while (1) {
				if (m_stopRecvLoop) {
					break;
				}
				if (ReconnectServer()) {
					if (m_devType == ZCAN_CANFDNET_TCP || m_devType == ZCAN_CANFDNET_UDP || m_devType == ZCAN_CANFDNET_400U_TCP ||
						m_devType == ZCAN_CANFDNET_400U_UDP || m_devType == ZCAN_CANFDNET_100U_TCP || m_devType == ZCAN_CANFDNET_100U_UDP ||
						m_devType == ZCAN_CANFDNET_800U_TCP || m_devType == ZCAN_CANFDNET_800U_UDP)
					{	// 同步设备时间
					char bufTime[50];
					int nLen;
#ifdef WIN32
						SYSTEMTIME stTime;
						GetLocalTime(&stTime);
						nLen = sprintf_s(bufTime, "%04d-%02d-%02dT%02d:%02d:%02d.%03d", stTime.wYear, stTime.wMonth, stTime.wDay, stTime.wHour, stTime.wMinute, stTime.wSecond, stTime.wMilliseconds);
#else
						struct timeval tv;
						struct tm* ptm;
						char time_string[40];
						gettimeofday(&tv, NULL);
						ptm = localtime(&(tv.tv_sec));
						strftime(bufTime, sizeof(bufTime), "%Y-%m-%dT%H:%M:%S.000", ptm);
						nLen = strlen(bufTime);
#endif
						PacketDataRequest request;
						request.nRequest = PACKET_REQUEST_SET_SYNC_DEV_CLOCK;
						request.nTID = 0;
						request.vecData.assign(&bufTime[0], &bufTime[0] + nLen);
						Packet packet = CPacketEncoder::BuildPacketRequestEx(0, request);
						SendData(reinterpret_cast<const char*>(packet.data()), packet.size());
					}
					break;
				}
				//cc_sleep(3000);
				cc_sleep(200);
			}
		}
		else {
			cc_sleep(1);
		}
	}

	LOG_INFO("TCP recv loop exit. \n");
}

bool TcpClient::Stop()
{
    return DisconnectServer();
}

bool TcpClient::SendData(const char* data, unsigned int size) 
{
	LOG_INFO("TCP send %d bytes ...\n", size);

	if (!cc_socket_is_connect(&m_socket)) {
		LOG_ERR("TCP send failed, not connect!\n");
		return false;
	}

    std::lock_guard<std::recursive_mutex> lk(m_txMutex);
	if (cc_socket_send(&m_socket, data, size) != size) {
		LOG_ERR("TCP send failed!\n");
		return false;
	}

	return true;
}