#include "network.h"
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <errno.h>
#include "log.h"
#include <netinet/tcp.h>
#include <fcntl.h>


typedef struct sockaddr SOCKADDR;
typedef struct sockaddr_in SOCKADDR_IN;

#define INVALID_SOCKET	(-1)
#define SOCKET_ERROR	(-1)

cc_socketerr cc_socket_init(cc_socket *s, bool btcp)
{
    memset(s, 0, sizeof(cc_socket));
    s->handle = INVALID_SOCKET;

    int hSocket = socket(AF_INET, (btcp ? SOCK_STREAM : SOCK_DGRAM), 0);
    if (hSocket == INVALID_SOCKET) {
        return CC_SOCKET_ERR;
    }
    
    s->handle = (cc_socket_handle)hSocket;
    
    // defalut is block mode no timeout
    // set block mode timeout
    // struct timeval tmo;
    // tmo.tv_sec = timeout / 1000;
    // tmo.tv_usec = (timeout % 1000) * 1000;
    // socklen_t len = sizeof(tmo);
    // cc_socket_setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tmo, len);
    // cc_socket_setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tmo, len);

    return CC_SOCKET_OK;
}

static cc_socketerr cc_socket_keep_alive(cc_socket *s)
{
    int socktype = -1;
    socklen_t socktypelen = sizeof(socktype);
    if (getsockopt(s->handle, SOL_SOCKET, SO_TYPE, (char*)&socktype, &socktypelen) == SOCKET_ERROR) {
        LOG_ERR("Get socket type failed, %s!\n", strerror(errno));
        return CC_SOCKET_ERR;
    }
    if (socktype != SOCK_STREAM){
        LOG_ERR("socket type(%d) no need set keepalive!\n", socktype);
        return CC_SOCKET_OK;
    }

    int open = 1;
    if (setsockopt(s->handle, SOL_SOCKET, SO_KEEPALIVE, (char*)&open, sizeof(open)) == SOCKET_ERROR) {
        LOG_ERR("TCP enable keep alive failed, %s\n", strerror(errno));
        return CC_SOCKET_ERR;
    }

    // send first probe after interval. 
    int idle = 10;     // s
    if (setsockopt(s->handle, IPPROTO_TCP, TCP_KEEPIDLE, (char*)&idle, sizeof(idle)) == SOCKET_ERROR) {
        LOG_ERR("TCP set keepalive time failed, %s\n", strerror(errno));
        return CC_SOCKET_ERR;
    }

    int interval = 1;   // s
    if (setsockopt(s->handle, IPPROTO_TCP, TCP_KEEPINTVL, (char*)&interval, sizeof(interval)) == SOCKET_ERROR) {
        LOG_ERR("TCP set keepalive interval failed, %s\n", strerror(errno));
        return CC_SOCKET_ERR;
    }

    // number of probes without got a reply 
    int cnt = 10;
    if (setsockopt(s->handle, IPPROTO_TCP, TCP_KEEPCNT, (char*)&cnt, sizeof(cnt)) == SOCKET_ERROR) {
        LOG_ERR("TCP set keepalive count failed, %s\n", strerror(errno));
        return CC_SOCKET_ERR;
    }

    LOG_INFO("TCP set heartbeat %ds. \n", idle + interval * cnt);
    return CC_SOCKET_OK;
}

cc_socketerr cc_socket_connect(cc_socket *s, const char* ip, unsigned short port, uint32_t timeoutMs)
{
    strcpy(s->ip, ip);
    s->port = port;

    struct sockaddr_in addr_bc;
    addr_bc.sin_family = AF_INET;
    addr_bc.sin_port = htons(port);
    addr_bc.sin_addr.s_addr = inet_addr(ip);

    // NonBlock mode
    int oldopt = fcntl(s->handle, F_GETFL);
    fcntl(s->handle, F_SETFL, oldopt | O_NONBLOCK);

    int ret = connect(s->handle, (SOCKADDR*)&addr_bc, sizeof(SOCKADDR));
    if (ret != 0) {
        if (errno != EINPROGRESS) {
            LOG_ERR("TCP connect failed, %d - %s\n", errno, strerror(errno));
            fcntl(s->handle, F_SETFL, oldopt);
            return CC_SOCKET_ERR;
        }
        else {
            fd_set writeFds;
            FD_ZERO(&writeFds);
            FD_SET(s->handle, &writeFds);
            timeval tv;
            tv.tv_sec = timeoutMs / 1000;
            tv.tv_usec = (timeoutMs % 1000) * 1000;
            int ret = select(s->handle + 1, NULL, &writeFds, NULL, &tv);
            if (ret <= 0 || !FD_ISSET(s->handle, &writeFds)) {
                LOG_ERR("connect failed, ret=%d \n", ret);
                fcntl(s->handle, F_SETFL, oldopt);
                return CC_SOCKET_ERR;
            }
            else {
                int err=0;
                socklen_t errLen = sizeof(errno);
                if (getsockopt(s->handle, SOL_SOCKET, SO_ERROR, &err, &errLen)) {
                    LOG_ERR("getsockopt error! \n");
                    return CC_SOCKET_ERR;
                }
                if (err == ETIMEDOUT) {
                    LOG_ERR("connect timeout! \n");
                    return CC_SOCKET_ERR;
                }
                if (err == ECONNREFUSED) {
                    LOG_ERR("connect refused! \n");
                    return CC_SOCKET_ERR;
                }
            }
        }
    }

    // restore  
    fcntl(s->handle, F_SETFL, oldopt);

    // set heartbeat 
    cc_socket_keep_alive(s);

    return CC_SOCKET_OK;
}

bool cc_socket_is_connect(cc_socket *s)
{
    return s->handle != INVALID_SOCKET;
}

cc_socketerr cc_socket_getsockopt(cc_socket *s, int level, int optname, char* optval, int* optlen)
{
    if (getsockopt(s->handle, level, optname, optval, (socklen_t*)optlen) == SOCKET_ERROR) {
        LOG_ERR("cc_socket_getsockopt failed, %s!\n", strerror(errno));
        return CC_SOCKET_ERR;
    }
    return CC_SOCKET_OK;
}

cc_socketerr cc_socket_setsockopt(cc_socket *s, int level, int optname, const char* optval, int optlen)
{
    if (setsockopt(s->handle, level, optname, optval, optlen) == SOCKET_ERROR) {
        LOG_ERR("cc_socket_setsockopt failed, %s!\n", strerror(errno));
        return CC_SOCKET_ERR;
    }
    return CC_SOCKET_OK;
}

cc_socketerr cc_socket_close(cc_socket *s)
{
    if (s->handle != INVALID_SOCKET) {
        // stop recv 
        shutdown(s->handle, SHUT_RDWR);
        close(s->handle);
    }
    
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
    // inet_aton("192.168.28.223", &addr_bc.sin_addr); 

	if (bind(s->handle, (struct sockaddr *)&addr_bc, sizeof(addr_bc)) == SOCKET_ERROR) {
        LOG_ERR("TCP bind failed, %s!\n", strerror(errno));
        return CC_SOCKET_ERR;
    } 

    strcpy(s->ip, inet_ntoa(addr_bc.sin_addr));
    return CC_SOCKET_OK;
}

cc_socketerr cc_socket_listen(cc_socket *s, int maxnum)
{
    if (listen(s->handle, maxnum) == SOCKET_ERROR) {
        LOG_ERR("TCP listen failed, %s!\n", strerror(errno));
        return CC_SOCKET_ERR;
    }
    
    return CC_SOCKET_OK;
}

cc_socketerr cc_socket_accept(cc_socket *s, cc_socket *clt)
{
    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    int len = sizeof(addr);
    int asock = accept(s->handle, (struct sockaddr*)&addr, (socklen_t*)&len);
    if (asock == INVALID_SOCKET) {
        return CC_SOCKET_ERR;
    }

    memset(clt, 0, sizeof(cc_socket));
    clt->handle = asock;
    strcpy(clt->ip, inet_ntoa(addr.sin_addr));
    clt->port = ntohs(addr.sin_port);

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
        LOG_ERR("TCP select error, %s\n", strerror(errno));
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
    return send(s->handle, buf, size, 0);
}

int cc_socket_recv(cc_socket *s, char* buf, unsigned int size)
{
    int len = recv(s->handle, buf, size, 0);
    int err = errno; 
    if (len == 0) {
        // connect closed
        return -1;
    }
    else if (len < 0) {
        if (err != ECONNABORTED) {
            LOG_ERR("TCP recv failed, %d - %s\n", err, strerror(err));
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
    addr_bc.sin_addr.s_addr = inet_addr(to->ip);
    int len = sizeof(addr_bc);
    int ret = sendto(s->handle, buf, size, 0, (struct sockaddr*)&addr_bc, (socklen_t)len);
    if (ret < 0)
    {
        int err = errno;
        LOG_ERR("socket sent to failed, %s!\n", strerror(err));
    }
    return ret;
}

int cc_socket_recv_from(cc_socket *s, char* buf, unsigned int size, cc_socket *clt)
{
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    int addrlen = sizeof(addr);
    int len = recvfrom(s->handle, buf, size, 0, (struct sockaddr*)&addr, (socklen_t*)&addrlen);
    int err = errno;
    if (len == 0) {
        // connect closed
        return -1;
    }
    else if (len < 0) {
        if (err != ECONNABORTED) {
            LOG_ERR("socket recv failed, %s!\n", strerror(err));
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
cc_socketerr cc_socket_add_membership(cc_socket *s, const char* groupip, int loopback)
{
    return CC_SOCKET_ERR;
}

cc_socketerr cc_socket_drop_membership(cc_socket *s, const char* groupip)
{
    return CC_SOCKET_ERR;
}

void cc_socket_set_recv_snd_buf(cc_socket *s, int recv, int send)
{
    // not use
    return;

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
