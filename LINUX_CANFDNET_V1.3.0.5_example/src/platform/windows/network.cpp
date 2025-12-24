#include "network.h"
#include <string.h>
#include <stdio.h>
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib")
#include "log.h"
#include <mstcpip.h>
#include <Ws2tcpip.h>

static const char* GetErrorStr(int err)
{
    static char str[100];
    sprintf(str, "err=%d", err);
    return str;
}

class WinsockInitHelper
{
private:
    WinsockInitHelper()
    {
        WSADATA wsa_data;
        m_nRetCode = WSAStartup(MAKEWORD(2, 2), &wsa_data);
    };

    ~WinsockInitHelper()
    {
        if (0 == m_nRetCode)
        {
            WSACleanup();
        }
    }

public:
    static WinsockInitHelper& getInstance()
    {
        static WinsockInitHelper s_obj;
        return s_obj;
    }

    int GetRetCode()
    {
        return m_nRetCode;
    }

private:
    int m_nRetCode;
};

cc_socketerr cc_socket_init(cc_socket *s, bool btcp /*= true*/)
{
    if (WinsockInitHelper::getInstance().GetRetCode() != 0)
    {
        return CC_SOCKET_ERR;
    }

    memset(s, 0, sizeof(cc_socket));

    SOCKET hSocket = socket(AF_INET, (btcp ? SOCK_STREAM : SOCK_DGRAM), 0);
    if (hSocket == INVALID_SOCKET) {
        return CC_SOCKET_ERR;
    }

    s->handle = (cc_socket_handle)hSocket;
    return CC_SOCKET_OK;
}

static cc_socketerr cc_socket_keep_alive(cc_socket *s)
{
    int socktype = -1;
    int socktypelen = sizeof(socktype);
    if (getsockopt(s->handle, SOL_SOCKET, SO_TYPE, (char*)&socktype, &socktypelen) == SOCKET_ERROR)
    {
        LOG_ERR("Get socket type failed, %s!\n", GetErrorStr(WSAGetLastError()));
        return CC_SOCKET_ERR;
    }
    if (socktype != SOCK_STREAM){
        LOG_ERR("socket type(%d) no need set keepalive!\n", socktype);
        return CC_SOCKET_OK;
    }

    int open = 1;
    if (setsockopt(s->handle, SOL_SOCKET, SO_KEEPALIVE, (char*)&open, sizeof(open)) == SOCKET_ERROR) {
        LOG_ERR("TCP enable keep alive failed, %s!\n", GetErrorStr(WSAGetLastError()));
        return CC_SOCKET_ERR;
    }

    tcp_keepalive in = {0};
    tcp_keepalive out = {0};
    unsigned long uReturn = 0;
    in.onoff = 1;
    in.keepalivetime = 10000;       // 10s
    in.keepaliveinterval = 1000;    // 1s
    // The number of keep-alive probes cannot be changed and is set to 10.
    if (WSAIoctl(s->handle, SIO_KEEPALIVE_VALS,
        (LPVOID)&in, sizeof(in), (LPVOID)&out, 
        sizeof(out), &uReturn, NULL, NULL) == SOCKET_ERROR) {
            LOG_ERR("TCP set heartbeat failed, %s!\n", GetErrorStr(WSAGetLastError()));
            return CC_SOCKET_ERR;
        }

    LOG_INFO("TCP set heartbeat time:%ds interval:%ds.\n", in.keepalivetime/1000 , in.keepaliveinterval/1000);
    return CC_SOCKET_OK;
}

static cc_socketerr cc_socket_tcp_set_nodelay(cc_socket *s, int nodelay)
{
    int socktype = -1;
    int socktypelen = sizeof(socktype);
    if (getsockopt(s->handle, SOL_SOCKET, SO_TYPE, (char*)&socktype, &socktypelen) == SOCKET_ERROR)
    {
        LOG_ERR("Get socket type failed, %s!\n", GetErrorStr(WSAGetLastError()));
        return CC_SOCKET_ERR;
    }
    if (socktype != SOCK_STREAM){
        LOG_ERR("socket type(%d) no need set tcp nodelay!\n", socktype);
        return CC_SOCKET_OK;
    }

    int val = nodelay;
    if (setsockopt(s->handle, IPPROTO_TCP, TCP_NODELAY, (char*)&val, sizeof(val)) == SOCKET_ERROR) {
        LOG_ERR("TCP TCP_NODELAY failed, %s!\n", GetErrorStr(WSAGetLastError()));
        return CC_SOCKET_ERR;
    }

    LOG_INFO("TCP set no delay\n");
    return CC_SOCKET_OK;
}

static cc_socketerr cc_socket_tcp_set_reuseaddr(cc_socket *s, int reuseaddr)
{
    int socktype = -1;
    int socktypelen = sizeof(socktype);
    if (getsockopt(s->handle, SOL_SOCKET, SO_TYPE, (char*)&socktype, &socktypelen) == SOCKET_ERROR)
    {
        LOG_ERR("Get socket type failed, %s!\n", GetErrorStr(WSAGetLastError()));
        return CC_SOCKET_ERR;
    }
    if (socktype != SOCK_STREAM){
        LOG_ERR("socket type(%d) no need set tcp reuseaddr!\n", socktype);
        return CC_SOCKET_OK;
    }

    int val = reuseaddr;
    if (setsockopt(s->handle, SOL_SOCKET, SO_REUSEADDR, (char*)&val, sizeof(val)) == SOCKET_ERROR) {
        LOG_ERR("TCP SO_REUSEADDR failed, %s!\n", GetErrorStr(WSAGetLastError()));
        return CC_SOCKET_ERR;
    }

	if (setsockopt(s->handle, SOL_SOCKET, SO_DONTLINGER, (char*)&val, sizeof(val)) == SOCKET_ERROR) {
        LOG_ERR("TCP SO_REUSEADDR failed, %s!\n", GetErrorStr(WSAGetLastError()));
        return CC_SOCKET_ERR;
    }

    LOG_INFO("TCP set reuse addr\n");
    return CC_SOCKET_OK;
}

static cc_socketerr cc_socket_tcp_set_quickack(cc_socket *s)
{
#define SIO_TCP_SET_ACK_FREQUENCY2   _WSAIOW(IOC_VENDOR, 23)

    int freq = 1;
    unsigned long uReturn = 0;
    if (WSAIoctl(s->handle, SIO_TCP_SET_ACK_FREQUENCY2, (LPVOID)&freq, sizeof(freq), NULL, 0, &uReturn, NULL, NULL) == SOCKET_ERROR) {
        LOG_ERR("TCP set ack freq failed, %s!\n", GetErrorStr(WSAGetLastError()));
        return CC_SOCKET_ERR;
    }

    LOG_INFO("TCP set quickack\n");
    return CC_SOCKET_OK;
}

cc_socketerr cc_socket_connect(cc_socket *s, const char* ip, unsigned short port, uint32_t timeoutMs)
{
    strcpy(s->ip, ip);
    s->port = port;

    struct sockaddr_in addr_bc;
    addr_bc.sin_family = AF_INET;
    addr_bc.sin_port = htons(port);
    addr_bc.sin_addr.S_un.S_addr = inet_addr(ip);

    //set socket to nonblocking to set connect timeout
    bool socketblocking = true;
    unsigned long iMode = 1; // 0: blocking; 1:non-blocking
    int iRet = ioctlsocket(s->handle, FIONBIO, &iMode);
    if (iRet != 0){
        LOG_ERR("Set socket to nonbloking failed, %s!\n", GetErrorStr(WSAGetLastError()));
    }
    else{
        socketblocking = false;
    }

    int ret = connect(s->handle, (SOCKADDR*)&addr_bc, sizeof(SOCKADDR));
    if (socketblocking) {
        if (ret != 0) {
            LOG_ERR("TCP connect failed, %s!\n", GetErrorStr(WSAGetLastError()));
            return CC_SOCKET_ERR;
        }
    }
    else {
        if (ret != 0) {
            int nLastErr = WSAGetLastError();
            if (nLastErr != WSAEWOULDBLOCK) {
                iMode = 0; // 0: blocking; 1:non-blocking
                if (ioctlsocket(s->handle, FIONBIO, &iMode) != 0)
                {
                    LOG_ERR("Set socket back to blocking mode failed, %s!\n", GetErrorStr(WSAGetLastError()));
                }
                LOG_ERR("TCP non-blocking connect failed, %s!\n", GetErrorStr(nLastErr));
                return CC_SOCKET_ERR;
            }
            else {
                TIMEVAL timeout;
                timeout.tv_sec = 3;
                timeout.tv_usec = 0;
                fd_set Write, Err;
                FD_ZERO(&Write);
                FD_ZERO(&Err);
                FD_SET(s->handle, &Write);
                FD_SET(s->handle, &Err);
                int selret = select(0, nullptr, &Write, &Err, &timeout);
                if (selret == 0) {
                    //Time out
                    LOG_ERR("socket connect timeout\n");
                    return CC_SOCKET_ERR;
                }
                else if (selret == SOCKET_ERROR) {
                    LOG_ERR("select failed, %s!\n", GetErrorStr(WSAGetLastError()));
                    return CC_SOCKET_ERR;
                }
                else {
                    if (FD_ISSET(s->handle, &Err)) {
                        int errcode = 0;
                        int optlen = sizeof(errcode);
                        getsockopt(s->handle, SOL_SOCKET, SO_ERROR, (char*)&errcode, &optlen);
                        LOG_ERR("socket select errfd, %s!\n", GetErrorStr(errcode));
                        return CC_SOCKET_ERR;
                    }
                    if (!FD_ISSET(s->handle, &Write)) {
                        LOG_ERR("socket write fd not set!\n");
                        return CC_SOCKET_ERR;
                    }
                }
            }
        }

        iMode = 0; // 0: blocking; 1:non-blocking
        if (ioctlsocket(s->handle, FIONBIO, &iMode) != 0)
        {
            LOG_ERR("Set socket back to blocking mode failed, %s!\n", GetErrorStr(WSAGetLastError()));
        }
    }

    struct sockaddr_in addr_local, addr_rmt;
    memset(&addr_local, 0, sizeof(addr_local));
    memset(&addr_rmt, 0, sizeof(addr_rmt));
    int nlenlocal = sizeof(addr_local), nlenrmt = sizeof(addr_rmt);
    getsockname(s->handle, (SOCKADDR*)&addr_local, &nlenlocal);
    getpeername(s->handle, (SOCKADDR*)&addr_rmt, &nlenrmt);
    LOG_INFO("socket %s:%d connect to %s:%d\n", inet_ntoa(addr_local.sin_addr), cc_ntohs(addr_local.sin_port), inet_ntoa(addr_rmt.sin_addr), cc_ntohs(addr_rmt.sin_port));

    // set heartbeat 
    cc_socket_keep_alive(s);

    // set tcp nodelay
    cc_socket_tcp_set_nodelay(s, 1);

    // set quick ack
    cc_socket_tcp_set_quickack(s);

    return CC_SOCKET_OK;
}

bool cc_socket_is_connect(cc_socket *s)
{
    return s->handle != INVALID_SOCKET;
}

cc_socketerr cc_socket_getsockopt(cc_socket *s, int level, int optname, char* optval, int* optlen)
{
    if (getsockopt(s->handle, level, optname, optval, optlen) == SOCKET_ERROR) {
        LOG_ERR("cc_socket_getsockopt failed, %s!\n", GetErrorStr(WSAGetLastError()));
        return CC_SOCKET_ERR;
    }
    return CC_SOCKET_OK;
}

cc_socketerr cc_socket_setsockopt(cc_socket *s, int level, int optname, const char* optval, int optlen)
{
    if (setsockopt(s->handle, level, optname, optval, optlen) == SOCKET_ERROR) {
        LOG_ERR("cc_socket_setsockopt failed, %s!\n", GetErrorStr(WSAGetLastError()));
        return CC_SOCKET_ERR;
    }
    return CC_SOCKET_OK;
}

cc_socketerr cc_socket_close(cc_socket *s)
{
    struct sockaddr_in addr_local, addr_rmt;
    memset(&addr_local, 0, sizeof(addr_local));
    memset(&addr_rmt, 0, sizeof(addr_rmt));
    int nlenlocal = sizeof(addr_local), nlenrmt = sizeof(addr_rmt);
    getsockname(s->handle, (SOCKADDR*)&addr_local, &nlenlocal);
    getpeername(s->handle, (SOCKADDR*)&addr_rmt, &nlenrmt);
    LOG_INFO("socket %s:%d <--> %s:%d close!\n", inet_ntoa(addr_local.sin_addr), cc_ntohs(addr_local.sin_port), inet_ntoa(addr_rmt.sin_addr), cc_ntohs(addr_rmt.sin_port));
	shutdown((SOCKET)s->handle, SD_BOTH);
	closesocket((SOCKET)s->handle);
    memset(s, 0, sizeof(cc_socket));
    s->handle = INVALID_SOCKET;
    return CC_SOCKET_OK;
}

cc_socketerr cc_socket_bind(cc_socket *s, unsigned short port)
{
    s->port = port;

    struct sockaddr_in addr_bc;
	addr_bc.sin_family = AF_INET;
	addr_bc.sin_port = htons(port);
	addr_bc.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(s->handle, (struct sockaddr *)&addr_bc, sizeof(addr_bc)) == SOCKET_ERROR) {
        LOG_ERR("socket bind to port:%d failed, %s!\n", port, GetErrorStr(WSAGetLastError()));
        return CC_SOCKET_ERR;
    } 

    strcpy(s->ip, inet_ntoa(addr_bc.sin_addr));
    return CC_SOCKET_OK;
}

cc_socketerr cc_socket_listen(cc_socket *s, int maxnum)
{
    if (listen(s->handle, maxnum) == SOCKET_ERROR) {
        LOG_ERR("TCP listen failed, %s!\n", GetErrorStr(WSAGetLastError()));
        return CC_SOCKET_ERR;
    }
    
    return CC_SOCKET_OK;
}

cc_socketerr cc_socket_accept(cc_socket *s, cc_socket *clt)
{
    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    int len = sizeof(addr);
    SOCKET asock = accept(s->handle, (LPSOCKADDR)&addr, &len);
    if (asock == INVALID_SOCKET) {
        return CC_SOCKET_ERR;
    }

    memset(clt, 0, sizeof(cc_socket));
    clt->handle = asock;
    strcpy(clt->ip, inet_ntoa(addr.sin_addr));
    clt->port = ntohs(addr.sin_port);

    // set heartbeat
    cc_socket_keep_alive(clt);

    // set tcp nodelay
    cc_socket_tcp_set_nodelay(clt, 1);

    // set quick ack
    cc_socket_tcp_set_quickack(clt);

    // set reuse addr
    cc_socket_tcp_set_reuseaddr(clt, 1);

    return CC_SOCKET_OK;
}

int cc_socket_select(cc_socket *s, const std::vector<cc_socket> *readSet, 
    std::vector<cc_socket> *setted, int tmoMs)
{
    if (!readSet || readSet->size() == 0) {
        return 0;
    }

    fd_set rd;
    FD_ZERO(&rd);

    int maxHdl = 0;
    if (readSet) {
        for (unsigned int i=0; i<readSet->size(); i++) {
            FD_SET(readSet->at(i).handle, &rd);
            if (readSet->at(i).handle > maxHdl) {
                maxHdl = readSet->at(i).handle;
            }
        }
    }

    timeval tv = {tmoMs / 1000, tmoMs % 1000 * 1000};
    
    int ret = select(maxHdl + 1, &rd, nullptr, nullptr, tmoMs < 0 ? nullptr : &tv);
    if (ret == 0) {
        // the time limit expired
    }
    else if (ret < 0) {
        // an error occurred
        LOG_ERR("TCP select error, %s\n", GetErrorStr(WSAGetLastError()));
    }
    else {
        for (unsigned int i=0; i<readSet->size(); i++) {
            if (FD_ISSET(readSet->at(i).handle, &rd)) {
                if (setted) {
                    setted->push_back(readSet->at(i));
                }
            }
        }
    }

    return ret;
}

int cc_socket_send(cc_socket *s, const char* buf, unsigned int size)
{
    return send((SOCKET)s->handle, buf, size, 0);
}

int cc_socket_recv(cc_socket *s, char* buf, unsigned int size)
{
    int len = recv((SOCKET)s->handle, buf, size, 0);
    int err = WSAGetLastError();
    if (len == 0) {
        // connect closed
        return -1;
    }
    else if (len < 0) {
        if (err != WSAECONNABORTED) {
            LOG_ERR("socket recv failed, %s!\n", GetErrorStr(err));
        }
        return -1;
    }

    return len;
}

int cc_socket_send_to(cc_socket *s, const char* buf, unsigned int size, cc_socket *to)
{
    struct sockaddr_in addr_bc;
    addr_bc.sin_family = AF_INET;
    addr_bc.sin_port = htons(to->port);
    addr_bc.sin_addr.S_un.S_addr = inet_addr(to->ip);

    int ret = sendto((SOCKET)s->handle, buf, size, 0, (SOCKADDR*)&addr_bc, sizeof(SOCKADDR));
    if (ret < 0)
    {
        int err = WSAGetLastError();
        LOG_ERR("socket sent to failed, %s!\n", GetErrorStr(err));
    }
    return ret;
}

int cc_socket_recv_from(cc_socket *s, char* buf, unsigned int size, cc_socket *clt)
{
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    int addrlen = sizeof(addr);
    int len = recvfrom((SOCKET)s->handle, buf, size, 0, (LPSOCKADDR)&addr, &addrlen);
    int err = WSAGetLastError();
    if (len == 0) {
        // connect closed
        return -1;
    }
    else if (len < 0) {
        if (err != WSAECONNABORTED) {
            LOG_ERR("recv from failed, %s!\n", GetErrorStr(err));
        }
        return -1;
    }

    if (clt)
    {
        memset(clt, 0, sizeof(cc_socket));
        clt->handle = (~0);
        strcpy(clt->ip, inet_ntoa(addr.sin_addr));
        clt->port = ntohs(addr.sin_port);
    }

    return len;
}

cc_socketerr cc_socket_add_membership(cc_socket *s, const char* groupip, int loopback)
{
    IP_MREQ mreq;
    memset(&mreq, 0, sizeof(mreq));
    mreq.imr_interface.s_addr = cc_htonl(INADDR_ANY);
    mreq.imr_multiaddr.s_addr = inet_addr(groupip);
    if (CC_SOCKET_ERR == cc_socket_setsockopt(s, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq)))
    {
        LOG_ERR("UDP add membership to multicast addr : %s failed\n", groupip);
        return CC_SOCKET_ERR;
    }
    LOG_ERR("UDP add membership to multicast addr : %s success\n", groupip);

    if (CC_SOCKET_ERR == cc_socket_setsockopt(s, IPPROTO_IP, IP_MULTICAST_LOOP, (char*)&loopback, sizeof(loopback)))
    {
        LOG_ERR("UDP set multicast loopback to %d failed\n", loopback);
        return CC_SOCKET_ERR;
    }

    return CC_SOCKET_OK;
}

cc_socketerr cc_socket_drop_membership(cc_socket *s, const char* groupip)
{
    IP_MREQ mreq;
    memset(&mreq, 0, sizeof(mreq));
    mreq.imr_interface.s_addr = cc_htonl(INADDR_ANY);
    mreq.imr_multiaddr.s_addr = inet_addr(groupip);
    if (CC_SOCKET_ERR == cc_socket_setsockopt(s, IPPROTO_IP, IP_DROP_MEMBERSHIP, (char*)&mreq, sizeof(mreq)))
    {
        LOG_ERR("UDP drop membership to multicast addr : %s failed\n", groupip);
        return CC_SOCKET_ERR;
    }
    return CC_SOCKET_OK;
}

unsigned long cc_inet_addr(const char* cp)
{
    return inet_addr(cp);
}

unsigned long cc_htonl(unsigned long hostlong)
{
    return htonl(hostlong);
}

unsigned long cc_ntohl(unsigned long netlong)
{
    return ntohl(netlong);
}

unsigned short cc_htons(unsigned short hostshort)
{
    return htons(hostshort);
}

unsigned short cc_ntohs(unsigned short netshort)
{
    return ntohs(netshort);
}

unsigned long long cc_htonll(unsigned long long hostll)
{
    static const int num = 42;
    if (*reinterpret_cast<const char *>(&num) == num)
    {
        const unsigned long high_part = cc_htonl(static_cast<unsigned long>(hostll >> 32));
        const unsigned long low_part = cc_htonl(static_cast<unsigned long>(hostll & 0xFFFFFFFFLL));

        return (static_cast<unsigned long long>(low_part) << 32) | high_part;
    }
    else
    {
        return hostll;
    }
}

unsigned long long cc_ntohll(unsigned long long netll)
{
    static const int num = 42;
    if (*reinterpret_cast<const char *>(&num) == num)
    {
        const unsigned long high_part = cc_ntohl(static_cast<unsigned long>(netll >> 32));
        const unsigned long low_part = cc_ntohl(static_cast<unsigned long>(netll & 0xFFFFFFFFLL));

        return (static_cast<unsigned long long>(low_part) << 32) | high_part;
    }
    else
    {
        return netll;
    }
}

void cc_socket_set_recv_snd_buf(cc_socket *s, int recv, int send)
{
    int nRecv = 0, nRecvDef = 0, nRecvSet=0;
    int nSend = 0, nSendDef = 0, nSendSet=0;
    int nLen = sizeof(nRecv);
    cc_socketerr err;

    //query recv buffer size
    err = cc_socket_getsockopt(s, SOL_SOCKET, SO_RCVBUF, (char*)&nRecvDef, &nLen);

    //set recv buffer
    nRecv = recv;
    nLen = sizeof(nRecv);
    err = cc_socket_setsockopt(s, SOL_SOCKET, SO_RCVBUF, (char*)&nRecv, nLen);

    //query recv buffer size
    nLen = sizeof(nRecv);
    err = cc_socket_getsockopt(s, SOL_SOCKET, SO_RCVBUF, (char*)&nRecvSet, &nLen);
    LOG_INFO("sock recv buf, default:%d changeto:%d now:%d\n", nRecvDef, nRecv, nRecvSet);

    //query send buffer size
    nLen = sizeof(nSend);
    err = cc_socket_getsockopt(s, SOL_SOCKET, SO_SNDBUF, (char*)&nSendDef, &nLen);

    //set send buffer
    nSend = send;
    nLen = sizeof(nSend);
    err = cc_socket_setsockopt(s, SOL_SOCKET, SO_SNDBUF, (char*)&nSend, nLen);

    nLen = sizeof(nSend);
    err = cc_socket_getsockopt(s, SOL_SOCKET, SO_SNDBUF, (char*)&nSendSet, &nLen);
    LOG_INFO("sock send buf, default:%d changeto:%d now:%d\n", nSendDef, nSend, nSendSet);
}