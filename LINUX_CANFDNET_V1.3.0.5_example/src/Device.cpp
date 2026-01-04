#include "Device.h"
#include "common.h"
#include "log.h"
#include <memory>
#include <string.h>
#include <math.h>
#include <cassert>
#include "network.h"
#include "cJSON/cJSON.h"
#include "service/TcpClient.h"
#include "service/TcpServer.h"
#include "service/UdpService.h"

#ifdef WIN32
#include <Windows.h>
#else 
#include <time.h>
#include <sys/time.h>
#endif


#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

using namespace std;

const int REQUEST_TIMEOUT = 1500;           //�������ʱʱ��


std::string UtilGetcJSONString(cJSON* obj)
{
    std::string result;
    if (obj)
    {
        if (cJSON_IsString(obj))
        {
            result = obj->valuestring;
        }
    }
    return result;
}

double UtilGetcJSONNumber(cJSON* obj)
{
    if (obj)
    {
        if (cJSON_IsNumber(obj))
        {
            return obj->valuedouble;
        }
    }
    return 0.0f;
}

int UtilJsonByteArray(cJSON* msgData, BYTE* pData, int nLen)
{
    int nJsonArrayLen = 0;
    if (msgData && cJSON_IsArray(msgData))
    {
        nJsonArrayLen = cJSON_GetArraySize(msgData);
        nJsonArrayLen = min(nJsonArrayLen, nLen);
        for (int i = 0; i<nJsonArrayLen && i<nLen; ++i)
        {
            pData[i] = (BYTE)UtilGetcJSONNumber(cJSON_GetArrayItem(msgData, i));
        }
    }

    return nJsonArrayLen;
}

USHORT UtilVersionString2USHORT(const char* version)
{
    //version : V1.03.09
    CANFDNetVersion stVersion;
    std::string strVersion;
    for (size_t i = 0; i < strlen(version); i++)
    {
        char v = version[i];
        if (isdigit(v) || v == '.' )
        {
            strVersion.append(1, v);
        }
    }

    std::string strArray[3];
    size_t nIndex = strVersion.find('.');
    if (nIndex != std::string::npos)
    {
        strArray[0].assign(strVersion.begin(), strVersion.begin() + nIndex);
        size_t nIndex1 = strVersion.find('.', nIndex + 1);
        if (nIndex1 != std::string::npos)
        {
            strArray[1].assign(strVersion.begin() + nIndex + 1, strVersion.begin() + nIndex1);
            strArray[2].assign(strVersion.begin() + nIndex1 + 1, strVersion.end());
        }
        else
        {
            strArray[1].assign(strVersion.begin() + nIndex1 + 1, strVersion.end());
            strArray[2] = "00";
        }
    }
    else
    {
        strArray[0] = strVersion;
        strArray[1] = "00";
        strArray[2] = "00";
    }

    stVersion.st3Version.nType = 1;
    stVersion.st3Version.nMajor = atoi(strArray[0].c_str());
    stVersion.st3Version.nMinor = atoi(strArray[1].c_str());
    stVersion.st3Version.nRevision = atoi(strArray[2].c_str());

    return stVersion.nRawVersion;
}

#include <sstream>
std::string UtilStringAppendIndex(const char* strSrc, int nIndex)
{
    std::string str(strSrc);
    std::ostringstream strStream;
    strStream << nIndex;
    return str + strStream.str();
}

static struct DeviceInfo
{
    UINT            devType;
    UINT            devChnlCount;   //CANͨ������
    UINT            devLINCount;    //LINͨ������
    UINT            devTCP;         //��ʶ tcp or udp�� 1��tcp�� 0��udp
    UINT            devTxEcho;      //�豸�Ƿ�֧�ַ��ͻ��ԣ�ZCANPRO��֧�ַ��ͻ��ԵĻ�ͬʱʹ��ͨ���ϲ�����
    const char*     devName;
} s_devinfo[] = {
    /* DevType                   CHNL    LIN     TCP     TxEcho  Name*/
    { ZCAN_CANFDNET_TCP,         2,      0,      1,      1,      "CANFDNET-200U-TCP" },
    { ZCAN_CANFDNET_UDP,         2,      0,      0,      1,      "CANFDNET-200U-UDP" },
    { ZCAN_CANFDWIFI_TCP,        1,      0,      1,      1,      "CANFDWIFI-100U-TCP" },
    { ZCAN_CANFDWIFI_UDP,        1,      0,      0,      1,      "CANFDWIFI-100U-UDP" },
    { ZCAN_CANFDNET_400U_TCP,    4,      0,      1,      1,      "CANFDNET-400U-TCP" },
    { ZCAN_CANFDNET_400U_UDP,    4,      0,      0,      1,      "CANFDNET-400U-UDP" },
    { ZCAN_CANFDNET_100U_TCP,    1,      0,      1,      1,      "CANFDNET-100mini-TCP" },
    { ZCAN_CANFDNET_100U_UDP,    1,      0,      0,      1,      "CANFDNET-100mini-UDP" },
    { ZCAN_CANFDNET_800U_TCP,    8,      0,      1,      1,      "CANFDNET-800U-TCP" },
    { ZCAN_CANFDNET_800U_UDP,    8,      0,      0,      1,      "CANFDNET-800U-UDP" },
    { ZCAN_CANFDDTU_400_TCP,     4,      4,      1,      1,      "CANFDDTU-400-TCP" },
    { ZCAN_CANFDDTU_400_UDP,     4,      4,      0,      1,      "CANFDDTU-400-UDP" },
    { ZCAN_CANFDWIFI_200U_TCP,   2,      0,      1,      1,      "CANFDWIFI-200U-TCP" },
    { ZCAN_CANFDWIFI_200U_UDP,   2,      0,      0,      1,      "CANFDWIFI-200U-UDP" },
	{ ZCAN_CANFDDTU_800ER_TCP,   8,      4,      1,      1,      "CANFDDTU-800ER-TCP" },
	{ ZCAN_CANFDDTU_800ER_UDP,   8,      4,      0,      1,      "CANFDDTU-800ER-UDP" },
	{ ZCAN_CANFDDTU_800EWGR_TCP, 8,      4,      1,      1,      "CANFDDTU-800EWGR-TCP" },
	{ ZCAN_CANFDDTU_800EWGR_UDP, 8,      4,      0,      1,      "CANFDDTU-800EWGR-UDP" },
	{ ZCAN_CANFDDTU_600EWGR_TCP, 6,      0,      1,      1,      "CANFDDTU-600EWGR-TCP" },
	{ ZCAN_CANFDDTU_600EWGR_UDP, 6,      0,      0,      1,      "CANFDDTU-600EWGR-UDP" },
	{ ZCAN_CANFDDTU_CASCADE_TCP, 30, 4, 1, 1, "CANFDDTU_CASCADE_TCP" },
	{ ZCAN_CANFDDTU_CASCADE_UDP, 30, 4, 0, 1, "CANFDDTU_CASCADE_UDP" },
};

Device::Device(UINT devType, UINT devIdx, UINT Reserved /*= 0*/)
    : m_devType(devType), m_devIdx(devIdx), m_srcPort(Reserved), m_frmMerge(CAN_DATA_QUEUE_BUFF_SIZE * 2)
{
    LOG_INFO("devtype:%d devIndex:%d SrcPort:%d\n", devType, devIdx, Reserved);
    m_service = nullptr;
    m_bDevInfoUpdated = false;
    memset(m_protocolVersion, 0, sizeof(m_protocolVersion));
    memset(&m_stDevInfo, 0, sizeof(m_stDevInfo));
    m_stDevInfo.dr_Version = 0x0100;
    m_stDevInfo.in_Version = 0x0100;
    for (size_t i = 0; i < sizeof(s_devinfo)/sizeof(s_devinfo[0]); i++)
    {
        if (devType == s_devinfo[i].devType)
        {
            m_devChnlCount = s_devinfo[i].devChnlCount;
            m_devLINCount = s_devinfo[i].devLINCount;
            m_devName = s_devinfo[i].devName;
            m_devTCP = s_devinfo[i].devTCP;

            m_stDevInfo.can_Num = s_devinfo[i].devChnlCount;
            memcpy(m_stDevInfo.str_hw_Type, m_devName.c_str(), min(m_devName.length(), (size_t)40));

            m_bRecvMerge = false;

            #ifdef WIN32
			LOG_ALWAYS("tid = %u devtname:%s chnl:%d TCP:%d RecvMerge:%d\n", GetCurrentThreadId(), m_devName.c_str(), m_devChnlCount, m_devTCP, m_bRecvMerge);
            #endif
            
            break;
        }
    }

    m_bitChnlStarted.reset();
    memset(m_chnlStatus, 0, sizeof(m_chnlStatus));
    memset(&m_statistic, 0, sizeof(m_statistic));
    memset(&m_busUsage, 0, sizeof(m_busUsage));
    m_bTxEcho = false;

    m_bEchoResponse = true;

#ifdef WIN32
    m_hDecoderData = CreateEvent(NULL, FALSE, FALSE, NULL);
#endif
}

Device::~Device()
{
    Stop();
	m_bRecvMerge = false;
#ifdef WIN32
    CloseHandle(m_hDecoderData);
    m_hDecoderData = NULL;
#endif
}

bool Device::GetReference(UINT chnIdx, UINT refType, void* pData)
{
    switch(refType)
    { 
    case CMD_DESIP:
        strcpy((char*)pData, m_destIp.c_str());
        break;
    case CMD_DESPORT:
        *(UINT*)pData = m_destPort;
        break;
    case CMD_TCP_TYPE:
        *(UINT*)pData = m_workMode;
        break;
    case CMD_SRCPORT:
        *(UINT*)pData = m_srcPort;
        break;
    case CMD_CLIENT:
        {
            REMOTE_CLIENT *pRClient = (REMOTE_CLIENT*)pData;
            TcpServer *svr = dynamic_cast<TcpServer*>(m_service);
            if (!pRClient || pRClient->iIndex < 0 
                || !svr || pRClient->iIndex >= (int)svr->GetClients()->size()) {
                return false;
            }
            const cc_socket* sck = svr->GetClient(pRClient->iIndex);
            if (!sck) {
                return false;
            }
            // Linux sizeof(HANDLE)=8, Windows sizeof(HANDLE)=4
            pRClient->hClient = 0;
            memcpy(&pRClient->hClient, &sck->handle, sizeof(sck->handle));
            pRClient->port = sck->port;
            strcpy(pRClient->szip, sck->ip);
        }
        break;
    case CMD_CLIENT_COUNT: 
        {
            TcpServer *svr = dynamic_cast<TcpServer*>(m_service);
            if (svr) {
                *(UINT*)pData = svr->GetClients()->size();
            }
            else {
                *(UINT*)pData = 0;
            }
        }
        break;
    case CMD_DISCONN_CLINET:
        return false;
    case SETGETREF_VIRIFY_DEVICE:
        {
            return true;
        }
        break;
    case SETGETREF_VIRIFY_DEVICE_BY_PASS:
        {
            VerifyDeviceData* data = (VerifyDeviceData*)pData;
            if (data)
            {
                return VerifyDeviceByPass(*data);
            }
            return false;
        }
        break;
    case SETGETREF_UDP_MULTICAST_LOOPBACK:
        *(int*)pData = m_udpmulticastloopback;
        break;
    case GETREF_GET_TX_TIMESTAMP:
        {
            if (chnIdx < m_devChnlCount && pData != NULL)
            {
                TxTimeStamp* pVal = reinterpret_cast<TxTimeStamp*>(pData);
                UINT nRetCount = GetTxEchoTimeStamp(chnIdx, *pVal);
                pVal->nBufferTimeStampCount = nRetCount;
                return false;
            }
            else
            {
                return false;
            }
        }
        break;
    case GETREF_GET_BUS_USAGE:
        {
            if (chnIdx < m_devChnlCount && pData != NULL)
            {
                BusUsage* pVal = reinterpret_cast<BusUsage*>(pData);
                return GetBusUsage(chnIdx, *pVal);
            }
            else
            {
                return false;
            }
        }
        break;
    case GETREF_GET_DELAY_SEND_AVALIABLE_COUNT:
        {
            if (chnIdx < m_devChnlCount && pData != NULL)
            {
                UINT* pVal = reinterpret_cast<UINT*>(pData);
                bool bRet = GetDelaySendAvailableTxCount(chnIdx, *pVal);
                return bRet;
            }
            return false;
        }
        break;
    case GETREF_GET_DEV_CAN_AUTO_SEND_COUNT:
        {
            if (chnIdx < m_devChnlCount && pData != NULL)
            {
                UINT* pVal = reinterpret_cast<UINT*>(pData);
                bool bRet = GetDevAutoSendListCount(chnIdx, false, *pVal);
                return bRet;
            }
            return false;
        }
        break;
    case GETREF_GET_DEV_CAN_AUTO_SEND_DATA:
        {
            if (chnIdx < m_devChnlCount && pData != NULL)
            {
                bool bRet = GetDevAutoSendListData(chnIdx, false, pData);
                return bRet;
            }
            return false;
        }
        break;
    case GETREF_GET_DEV_CANFD_AUTO_SEND_COUNT:
        {
            if (chnIdx < m_devChnlCount && pData != NULL)
            {
                UINT* pVal = reinterpret_cast<UINT*>(pData);
                bool bRet = GetDevAutoSendListCount(chnIdx, true, *pVal);
                return bRet;
            }
            return false;
        }
        break;
    case GETREF_GET_DEV_CANFD_AUTO_SEND_DATA:
        {
            if (chnIdx < m_devChnlCount && pData != NULL)
            {
                bool bRet = GetDevAutoSendListData(chnIdx, true, pData);
                return bRet;
            }
            return false;
        }
        break;
    case GETREF_GET_DEV_STATE_SYS_INFO:
        {
            if (pData != NULL)
            {
                ZCAN_RAW_DATA* pVal = (ZCAN_RAW_DATA*)pData;
                std::string strResult;
                if (GetDevState(PACKET_TYPE_DEV_STATE_PARAM_SYS_INFO, strResult))
                {
                    pVal->nResultLen = strResult.length() + 1;
                    memcpy(pVal->pData, strResult.c_str(), min(pVal->nResultLen, pVal->nDataLen));
                    return true;
                }
            }
            return false;
        }
        break;
    case GETREF_GET_DEV_STATE_CAN_INFO:
        {
            if (pData != NULL)
            {
                ZCAN_RAW_DATA* pVal = (ZCAN_RAW_DATA*)pData;
                std::string strResult;
                if (GetDevState(PACKET_TYPE_DEV_STATE_PARAM_CAN_INFO, strResult))
                {
                    pVal->nResultLen = strResult.length() + 1;
                    memcpy(pVal->pData, strResult.c_str(), min(pVal->nResultLen, pVal->nDataLen));
                    return true;
                }
            }
            return false;
        }
        break;
    case GETREF_GET_DEV_STATE_RECORDER_INFO:
        {
            if (pData != NULL)
            {
                ZCAN_RAW_DATA* pVal = (ZCAN_RAW_DATA*)pData;
                std::string strResult;
                if (GetDevState(PACKET_TYPE_DEV_STATE_PARAM_RECORDER_INFO, strResult))
                {
                    pVal->nResultLen = strResult.length() + 1;
                    memcpy(pVal->pData, strResult.c_str(), min(pVal->nResultLen, pVal->nDataLen));
                    return true;
                }
            }
            return false;
        }
        break;
    case GETREF_GET_DEV_STATE_NET_INFO:
        {
            if (pData != NULL)
            {
                ZCAN_RAW_DATA* pVal = (ZCAN_RAW_DATA*)pData;
                std::string strResult;
                if (GetDevState(PACKET_TYPE_DEV_STATE_PARAM_NET_INFO, strResult))
                {
                    pVal->nResultLen = strResult.length() + 1;
                    memcpy(pVal->pData, strResult.c_str(), min(pVal->nResultLen, pVal->nDataLen));
                    return true;
                }
            }
            return false;
        }
        break;
    case GETREF_GET_DEV_GPS_COUNT:
        {
            if (pData != NULL)
            {
                UINT* pVal = reinterpret_cast<UINT*>(pData);
                *pVal = GetGPSRecvNum();
                return true;
            }
            return false;
        }
        break;
    case GETREF_GET_DEV_GPS_DATA:
        {
            if (pData != NULL)
            {
                ZCAN_GPS_DATA* pVal = reinterpret_cast<ZCAN_GPS_DATA*>(pData);
                pVal->nRet = GetGPSData(pVal->pData, pVal->nFrameCount, pVal->nWaitTime);
                return true;
            }
            return false;
        }
        break;
    case GETREF_GET_LIN_TX_FIFO_TOTAL:
        {
            if (chnIdx < m_devLINCount && pData != NULL)
            {
                UINT* pVal = reinterpret_cast<UINT*>(pData);
                LINFIFOInfo fifoInfo;
                bool bRet = GetLINTxFIFOInfo(chnIdx, fifoInfo);
                if (bRet)
                {
                    *pVal = fifoInfo.nFIFOSize;
                }
                return bRet;
            }
            return false;
        }
        break;
    case GETREF_GET_LIN_TX_FIFO_AVAILABLE:
        {
            if (chnIdx < m_devLINCount && pData != NULL)
            {
                UINT* pVal = reinterpret_cast<UINT*>(pData);
                LINFIFOInfo fifoInfo;
                bool bRet = GetLINTxFIFOInfo(chnIdx, fifoInfo);
                if (bRet)
                {
                    *pVal = fifoInfo.nFIFOAvailable;
                }
                return bRet;
            }
            return false;
        }
        break;
    case GETREF_GET_DATA_RECV_MERGE:
        {
            if (pData != NULL)
            {
                UINT* pVal = reinterpret_cast<UINT*>(pData);
                *pVal = m_bRecvMerge;
                return true;
            }
            return false;
        }
        break;
    case GETREF_GET_TX_ECHO_ENABLE:
        {
            if (pData != NULL)
            {
                UINT* pVal = reinterpret_cast<UINT*>(pData);
                *pVal = m_bTxEcho;
                return true;
            }
            return false;
        }
        break;
    default:
        return false;
    }

    return true;
}

bool Device::SetReference(UINT chnIdx, UINT refType, void* pData)
{
    switch(refType)
    {
    case CMD_DESIP:
        m_destIp = (char*)pData;
        LOG_ALWAYS("Set dest ip:%s", m_destIp.c_str());
        break;
    case CMD_DESPORT:
        m_destPort = (USHORT)*(UINT*)pData;
        // LOG_ALWAYS("Set dest port:%d", m_destPort);
        break;
    case CMD_SRCPORT:
        m_srcPort = (USHORT)*(UINT*)pData;
        LOG_ALWAYS("Set src port:%d", m_srcPort);
        break;
    case CMD_TCP_TYPE:
        m_workMode = *(UINT*)pData;
        LOG_ALWAYS("Set work mode:%s", m_workMode == TCP_SERVER ? "Server" : "Client");
        break;
    case CMD_DISCONN_CLINET:
        {
            if (m_workMode != TCP_SERVER) {
                return false;
            }

            REMOTE_CLIENT* pClient =  (REMOTE_CLIENT*)pData;
            TcpServer *svr = dynamic_cast<TcpServer*>(m_service);
            if (!pClient || !svr) {
                return false;
            }

            // Linux sizeof(HANDLE)=8, Windows sizeof(HANDLE)=4
            int hdl = pClient->hClient;
            cc_socket_handle sckh;
            memcpy(&sckh, &hdl, sizeof(sckh));    
            return svr->DisconnectClient(sckh);
        }
        break;
    case SETGETREF_VIRIFY_DEVICE:
        {
            return true;
        }
        break;
    case SETGETREF_UDP_MULTICAST_LOOPBACK:
        m_udpmulticastloopback = *(int*)pData;
        break;
    case SETGETREF_VIRIFY_DEVICE_BY_PASS:
        {
            VerifyDeviceData* data = (VerifyDeviceData*)pData;
            if (data)
            {
                return VerifyDeviceByPass(*data);
            }
            return false;
        }
        break;
    case SETREF_ADD_TIMER_SEND_CAN_DIRECT:
        {
            if (chnIdx < m_devChnlCount && pData != nullptr)
            {
                ZCAN_AUTO_TRANSMIT_OBJ* pObj = reinterpret_cast<ZCAN_AUTO_TRANSMIT_OBJ*>(pData);
                if (pObj)
                {
                    return SetAutoSend(chnIdx, pObj);
                }
            }
            return false;
        }
        break;
    case SETREF_ADD_TIMER_SEND_CANFD_DIRECT:
        {
            if (chnIdx < m_devChnlCount && pData != nullptr)
            {
                ZCANFD_AUTO_TRANSMIT_OBJ* pObj = reinterpret_cast<ZCANFD_AUTO_TRANSMIT_OBJ*>(pData);
                if (pObj)
                {
                    return SetAutoSend(chnIdx, pObj);
                }
            }
            return false;
        }
        break;
    case SETREF_ADD_TIMER_SEND_CAN:
        {
            if (chnIdx < m_devChnlCount && pData != nullptr)
            {
                ZCAN_AUTO_TRANSMIT_OBJ* pObj = reinterpret_cast<ZCAN_AUTO_TRANSMIT_OBJ*>(pData);
                if (pObj)
                {
                    return AddAutoSend(chnIdx, pObj);
                }
            }
            return false;
        }
        break;
    case SETREF_ADD_TIMER_SEND_CANFD:
        {
            if (chnIdx < m_devChnlCount && pData != nullptr)
            {
                ZCANFD_AUTO_TRANSMIT_OBJ* pObj = reinterpret_cast<ZCANFD_AUTO_TRANSMIT_OBJ*>(pData);
                if (pObj)
                {
                    return AddAutoSend(chnIdx, pObj);
                }
            }
            return false;
        }
        break;
    case SETREF_APPLY_TIMER_SEND:
        {
            if (chnIdx < m_devChnlCount)
            {
                return ApplyAutoSend(chnIdx);
            }
            return false;
        }
        break;
    case SETREF_CLEAR_TIMER_SEND:
        {
            if (chnIdx < m_devChnlCount)
            {
                return ClearAutoSend(chnIdx);
            }
            return false;
        }
        break;
    case SETREF_SET_DEV_TIMESTAMP:
        {
            if (pData != nullptr)
            {
                UINT64 nTime = *(UINT64*)pData;
                return SetDevTimeStamp(nTime);
            }
            return false;
        }
        break;
    case SETREF_SET_TX_ECHO_ENABLE:
        {
            if (pData != NULL)
            {
                UINT* pVal = reinterpret_cast<UINT*>(pData);
                m_bTxEcho = *pVal > 0;
                return true;
            }
            return false;
        }
        break;
    case SETREF_CLEAR_DELAY_SEND_QUEUE:
        if (chnIdx < m_devChnlCount)
        {
            return ClearDelaySendQueue(chnIdx);
        }
        break;
    case SETREF_SET_DATA_RECV_MERGE:
    {
        if (pData != NULL)
        {
            UINT* pVal = reinterpret_cast<UINT*>(pData);
            m_bRecvMerge = *pVal > 0;
            return true;
        }
        return false;
    }
	case SETREF_ADD_DYNAMIC_BY_JSON:
    {
        if (pData != NULL)
        {
			ZCAN_DYNAMIC_CONFIG_DATA* pObj = reinterpret_cast<ZCAN_DYNAMIC_CONFIG_DATA*>(pData);
			if (pObj)
			{
				return addDynamicCfg(chnIdx, pObj);
			}
        }
        return false;
    }
	case SETREF_APPLY_DYNAMIC_BY_JSON:
	{
		if (chnIdx < m_devChnlCount)
		{
			UINT* pVal = reinterpret_cast<UINT*>(pData);
			return ApplyDynamicCfg(chnIdx, *pVal);
		}
		return false;
	}
    break;
	case SETREF_ADD_FILTER:
	{
		if (chnIdx < m_devChnlCount && pData != nullptr)
		{
			CANFDNETFilterItem* pFilterItem = reinterpret_cast<CANFDNETFilterItem*>(pData);
            pFilterItem->nChn = chnIdx;
			if (m_vecFilterItems[chnIdx].size() < CANFDNET_FILTER_COUNT_MAX)
			{
				m_vecFilterItems[chnIdx].push_back(*pFilterItem);
				return true;
			}
		 }
		return false;
	}
		break;
	case SETREF_APPLY_FILTER:
	{
		if (chnIdx < m_devChnlCount)
		{
			bool bRet = ApplyCANFilter();
			return bRet;
		}
		return false;
	}
		break;
	case SETREF_CLEAR_FILTER:
	{
		if (chnIdx < m_devChnlCount)
		{
			m_vecFilterItems[chnIdx].clear();
			bool bRet = ClearCANFilter(chnIdx);
			return bRet;
		}
		return false;
	}
		break;
	case SETREF_SET_DYNAMIC_BY_DATA:
	{
		if (chnIdx < m_devChnlCount && pData != nullptr)
		{
			ZCAN_DYNAMIC_CONFIG* pObj = reinterpret_cast<ZCAN_DYNAMIC_CONFIG*>(pData);
			if (pObj)
			{
				return setDynamicCfg(chnIdx, pObj);
			}
		}
		return false;
	}
		break;
    default:
        return false;
    }

    return true;
}

ULONG Device::GetReceiveNum(UINT chnIdx, BYTE type)
{
    //type:TYPE_CAN TYPE_CANFD TYPE_ALL_DATA
    if (m_bRecvMerge || type == TYPE_ALL_DATA)
    {
        return m_frmMerge.size();
    }
    else if (chnIdx < m_devChnlCount)
    {
        if (type == TYPE_CANFD)
        {
            return m_frmCANFD[chnIdx].size();
        }
        else if (type == TYPE_CAN)
        {
            return m_frmCAN[chnIdx].size();
        }
    }
    return 0;
}

void Device::ClearBuffer(UINT chnIdx)
{
    if (chnIdx < m_devChnlCount)
    {
        LOG_INFO("ClearBuffer Chnl %d: CAN:%d CANFD:%d Err:%d\n", chnIdx, m_frmCAN[chnIdx].size(), m_frmCANFD[chnIdx].size(), m_errQueue[chnIdx].size());
        m_frmCAN[chnIdx].clear();
        m_frmCANFD[chnIdx].clear();
        m_errQueue[chnIdx].clear();
		m_frmMerge.clear();//2023.03.28�¼���պϲ����ջ�������
        memset(&m_chnlStatus[chnIdx], 0, sizeof(ZCAN_CHANNEL_STATUS));

        m_devAutoSendList.chnl[chnIdx].vecCAN.clear();
        m_devAutoSendList.chnl[chnIdx].vecCANFD.clear();
    }
}

ULONG Device::Transmit(UINT chnIdx, const ZCAN_Transmit_Data* pSend, ULONG len)
{
    ULONG sentCnt = 0;
    VecPackets vecPackets;
    if (chnIdx >= m_devChnlCount || pSend == nullptr || len == 0)
    {
        return sentCnt;
    }
    vecPackets = CPacketEncoder::BuildPacketCAN(chnIdx, pSend, len, m_bTxEcho);
    for (auto& packet : vecPackets)
    {
        if (packet.size() != SendData(packet.data(), packet.size()))
        {
            m_statistic.chnl[chnIdx].nTxCAN += sentCnt;
            return sentCnt;
        }
        sentCnt += CPacketUtil::GetPacketFrameCount(packet);
    }
    m_statistic.chnl[chnIdx].nTxCAN += sentCnt;
    return sentCnt;
}


ULONG Device::TransmitFD(UINT chnIdx, const ZCAN_TransmitFD_Data* pSend, ULONG len)
{
    ULONG sentCnt = 0;
    VecPackets vecPackets;
    if (chnIdx >= m_devChnlCount || pSend == nullptr || len == 0)
    {
        return sentCnt;
    }
	//m_bTxEcho = false;
    vecPackets = CPacketEncoder::BuildPacketCANFD(chnIdx, pSend, len, m_bTxEcho);
    for (auto& packet : vecPackets)
    {
        if (packet.size() != SendData(packet.data(), packet.size()))
        {
            m_statistic.chnl[chnIdx].nTxCANFD += sentCnt;
            return sentCnt;
        }
        sentCnt += CPacketUtil::GetPacketFrameCount(packet);
    }
    m_statistic.chnl[chnIdx].nTxCANFD += sentCnt;
    return sentCnt;
}

ULONG Device::TransmitData(const ZCANDataObj* pSend, ULONG len)
{
    ULONG sentCnt = 0;
    VecPackets vecPackets;
    if (pSend == nullptr || len == 0)
    {
        return sentCnt;
    }
    vecPackets = CPacketEncoder::BuildPacketMerge( pSend, len);
    for (auto& packet : vecPackets)
    {
        if (packet.size() != SendData(packet.data(), packet.size()))
        {
            m_statistic.nTxMerge += sentCnt;
            return sentCnt;
        }
        sentCnt += CPacketUtil::GetPacketFrameCount(packet);
    }
    m_statistic.nTxMerge += sentCnt;
    return sentCnt;
}

ULONG Device::Receive(UINT chnIdx, ZCAN_Receive_Data* pReceive, ULONG len, int waitTime/* = -1*/)
{
    if (chnIdx >= m_devChnlCount || pReceive == nullptr || len == 0)
    {
        return 0;
    }

    CanDataQueue<PacketDataCAN>& frmdata = m_frmCAN[chnIdx];

#ifdef WIN32
    frmdata.WaitEvent(waitTime);
#else
    const int per_sleep_ms = 1;
    if (waitTime > 0)
    {
        int sleepTimes = (int)ceil(waitTime / (float)per_sleep_ms);
        while (sleepTimes--)
        {
            if (frmdata.size() <= 0)
            {
                cc_sleep(per_sleep_ms);
            }
            else
            {
                break;
            }
        }
    }
    else if (waitTime < 0)
    {
        while (true)
        {
            if (frmdata.size() <= 0)
            {
                cc_sleep(per_sleep_ms);
            }
            else
            {
                break;
            }
        }
    }
#endif

    memset(pReceive, 0, len*sizeof(ZCAN_Receive_Data));
    frmdata.lock();
    ULONG actLen = frmdata.size() > len ? len : frmdata.size();
    for (ULONG i = 0; i < actLen; i++)
    {
        CPacketUtil::CANFrame2ZCANRx(frmdata.front(), pReceive[i], nullptr);
        frmdata.pop();
    }
    frmdata.unlock();

    //LOG_INFO("Receive Chnl:%d, count:%d\n", chnIdx, actLen);

    return actLen;
}

ULONG Device::ReceiveFD(UINT chnIdx, ZCAN_ReceiveFD_Data* pReceive, ULONG len, int waitTime/* = -1*/)
{
    if (chnIdx >= m_devChnlCount || pReceive == nullptr || len == 0)
    {
        return 0;
    }

    CanDataQueue<PacketDataCANFD>& frmdata = m_frmCANFD[chnIdx];

#ifdef WIN32
    frmdata.WaitEvent(waitTime);
#else
    const int per_sleep_ms = 1;
    if (waitTime > 0)
    {
        int sleepTimes = (int)ceil(waitTime / (float)per_sleep_ms);
        while (sleepTimes--)
        {
            if (frmdata.size() <= 0)
            {
                cc_sleep(per_sleep_ms);
            }
            else
            {
                break;
            }
        }
    }
    else if (waitTime < 0)
    {
        while (true)
        {
            if (frmdata.size() <= 0)
            {
                cc_sleep(per_sleep_ms);
            }
            else
            {
                break;
            }
        }
    }
#endif

    memset(pReceive, 0, len*sizeof(ZCAN_ReceiveFD_Data));

    frmdata.lock();
    ULONG actLen = frmdata.size() > len ? len : frmdata.size();
    for (ULONG i = 0; i < actLen; i++)
    {
        CPacketUtil::CANFrame2ZCANRx(frmdata.front(), pReceive[i], nullptr, false);
        frmdata.pop();
    }
    frmdata.unlock();

    //LOG_INFO("ReceiveFD Chnl:%d, count:%d\n", chnIdx, actLen);

    return actLen;
}

ULONG Device::ReceiveData(ZCANDataObj* pReceive, ULONG len, int waitTime /*= -1*/)
{
    if (pReceive == nullptr || len == 0)
    {
        return 0;
    }

    CanDataQueue<ZCANDataObj>& frmdata = m_frmMerge;

#ifdef WIN32
    frmdata.WaitEvent(waitTime);
#else
    const int per_sleep_ms = 1;
    if (waitTime > 0)
    {
        int sleepTimes = (int)ceil(waitTime / (float)per_sleep_ms);
        while (sleepTimes--)
        {
            if (frmdata.size() <= 0)
            {
                cc_sleep(per_sleep_ms);
            }
            else
            {
                break;
            }
        }
    }
    else if (waitTime < 0)
    {
        while (true)
        {
            if (frmdata.size() <= 0)
            {
                cc_sleep(per_sleep_ms);
            }
            else
            {
                break;
            }
        }
    }
#endif

    memset(pReceive, 0, len*sizeof(ZCANDataObj));

    frmdata.lock();
    ULONG actLen = frmdata.size() > len ? len : frmdata.size();
    for (ULONG i = 0; i < actLen; i++)
    {
        pReceive[i] = frmdata.front();
        frmdata.pop();
    }
    frmdata.unlock();

    //LOG_INFO("ReceiveData count:%d\n", actLen);
    m_statistic.nAppRxMerge += actLen;

    return actLen;
}

bool Device::GetChnlErrorInfo(UINT chnIdx, ZCAN_CHANNEL_ERR_INFO* pError)
{
    if (nullptr == pError || chnIdx >= m_devChnlCount)
    {
        return false;
    }

    memset(pError, 0, sizeof(ZCAN_CHANNEL_ERR_INFO));
    CanDataQueue<ZCAN_CHANNEL_ERR_INFO>& frmdata = m_errQueue[chnIdx];
    frmdata.lock();
    if (frmdata.size() > 0)
    {
        *pError = frmdata.front();
        frmdata.pop();
    }
    frmdata.unlock();
    return true;
}

bool Device::GetChnlStatus(UINT chnIdx, ZCAN_CHANNEL_STATUS * pCANStatus)
{
    if (pCANStatus == nullptr || chnIdx >= m_devChnlCount)
    {
        return false;
    }

    *pCANStatus = m_chnlStatus[chnIdx];
    return true;
}

bool Device::GetDeviceInfo(ZCAN_DEVICE_INFO * pInfo)
{
    if (nullptr != pInfo)
    {
        if (!m_bDevInfoUpdated)
        {
            GetDeviceInfo(m_stDevInfo);
        }

        memcpy(pInfo, &m_stDevInfo, sizeof(ZCAN_DEVICE_INFO));
        return true;
    }
    return false;
}

bool Device::SetAutoSend(UINT chnIdx, const ZCAN_AUTO_TRANSMIT_OBJ* pObj)
{
    if (nullptr == pObj || chnIdx >= m_devChnlCount)
    {
        return false;
    }

    if (pObj->index >= DEV_AUTO_SEND_INDEX_MAX)
    {
        return false;
    }

    Packet packet = CPacketEncoder::BuildPacketAutosend(chnIdx, pObj, m_bTxEcho);
    if (packet.size() == SendData(packet.data(), packet.size()))
    {
        return true;
    }
    return false;
}

bool Device::SetAutoSend(UINT chnIdx, const ZCANFD_AUTO_TRANSMIT_OBJ* pObj)
{
    if (nullptr == pObj || chnIdx >= m_devChnlCount)
    {
        return false;
    }

    if (pObj->index >= DEV_AUTO_SEND_INDEX_MAX)
    {
        return false;
    }

    Packet packet = CPacketEncoder::BuildPacketAutosend(chnIdx, pObj, m_bTxEcho);
    if (packet.size() == SendData(packet.data(), packet.size()))
    {
        return true;
    }
    return false;
}

bool Device::SetAutoSend(UINT chnIdx, VecAutoSendData& vecAutoSend)
{
    if (chnIdx >= m_devChnlCount)
    {
        return false;
    }

    if (vecAutoSend.size() > 0)
    {
        VecPackets vecPackets;
        vecPackets = CPacketEncoder::BuildPacketAutosend(chnIdx, vecAutoSend, m_bTxEcho);
        for (auto& packet : vecPackets)
        {
            if (packet.size() != SendData(packet.data(), packet.size()))
            {
                return false;
            }
        }
    }
    return true;
}

unsigned int Device::SendData(const BYTE* data, unsigned int len)
{
    const char* cdata = reinterpret_cast<const char*>(data);
    if (nullptr == data ||
        len == 0 ||
        !m_service ||
        !m_service->SendData(cdata, len))
    {
        m_statistic.nTxBytesFailed += len;
        return 0;
    }
    m_statistic.nTxBytesSuccess += len;
    return len;
}

bool Device::VerifyDeviceByPass(VerifyDeviceData& data)
{
    bool bRet = false;
    BYTE session = ++m_authIncIndex;
    Packet packet = CPacketEncoder::BuildPacketAUTH(true, reinterpret_cast<const BYTE*>(data.inData), sizeof(data.inData), session);
    LOG_INFO("Auth Data Send Request :%d\n", session);
    if ( packet.size() == SendData(packet.data(), packet.size()) )
    {
        //wait for response
        AuthDeviceData_PTR authData = std::make_shared<AuthDeviceData>();
        if (authData)
        {
            memcpy(&authData->data, &data, sizeof(VerifyDeviceData));
            {
                std::unique_lock<std::mutex> lk_(m_authMapMutex);
                m_authDataMap[session] = authData;
            }
            std::mutex mutex_;
            std::unique_lock<std::mutex> ulk(mutex_);
            if (authData->cv.wait_for(ulk, std::chrono::milliseconds(REQUEST_TIMEOUT)) == std::cv_status::no_timeout)
            {
                memcpy(data.OutData, authData->data.OutData,  sizeof(data.OutData));
                bRet = true;
                LOG_INFO("Auth Data Copyed\n");
            }
            else
            {
                LOG_ERR("Auth Data Response Timeout\n");
            }

            //Clear result
            {
                std::lock_guard<std::mutex> lk_(m_authMapMutex);
                m_authDataMap.erase(session);
            }
        }
    }
    return bRet;
}

bool Device::StartCAN(UINT chnIdx)
{
    if (chnIdx >= m_devChnlCount)
    {
        return false;
    }
    if (m_bitChnlStarted.any())
    {
        m_bitChnlStarted.set(chnIdx);
        return true;
    }

    if (CreateAndStartService())
    {
        m_bitChnlStarted.set(chnIdx);

        //TCP�ͻ���ģʽ,UDPģʽ�����豸����ʱ��ȡһ���豸��Ϣ
        if (!m_devTCP || TCP_SERVER != m_workMode)
        {
            GetDeviceInfo(m_stDevInfo);
        }
		if (m_devType == ZCAN_CANFDNET_TCP || m_devType == ZCAN_CANFDNET_UDP || m_devType == ZCAN_CANFDNET_400U_TCP ||
			m_devType == ZCAN_CANFDNET_400U_UDP || m_devType == ZCAN_CANFDNET_100U_TCP || m_devType == ZCAN_CANFDNET_100U_UDP ||
			m_devType == ZCAN_CANFDNET_800U_TCP || m_devType == ZCAN_CANFDNET_800U_UDP)
		{
			SyncDevClock();//ͬ���豸ʱ��
		}
        LOG_ALWAYS("Device start success!");
        return true;
    }
    else
    {
        m_bitChnlStarted.reset(chnIdx);
    }
    return false;
}

bool Device::CreateAndStartService()
{
    if (!m_service)
    {
#ifndef BUILD_LIB_UDP
        if (m_devTCP)
        {
            if (m_workMode == TCP_SERVER)
            {
                //tcp server
				//LOG_ALWAYS("tid = %u, Tcp server mode. listen on:%d\n", *(unsigned int *)&std::this_thread::get_id(), m_srcPort);
                m_service = new TcpServer(m_srcPort, DEV_MAX_CLIENTS);
            }
            else
            {
                //tcp client 
				//LOG_ALWAYS("tid = %u, Tcp client mode. connect to:%s:%d\n", *(unsigned int *)&std::this_thread::get_id(), m_destIp.c_str(), m_destPort);
                m_service = new TcpClient(m_destIp.c_str(), m_destPort, m_devType);
            }
        }
        else
        {
			//LOG_ALWAYS("tid = %u, UDP mode. %d to:%s:%d\n", *(unsigned int *)&std::this_thread::get_id(), m_srcPort, m_destIp.c_str(), m_destPort);
            m_service = new UdpService(m_srcPort, m_destIp.c_str(), m_destPort, m_udpmulticastloopback);
        }
#else
        {
            LOG_ALWAYS("UDP mode. %d to:%s:%d\n", m_srcPort, m_destIp.c_str(), m_destPort);
            m_service = new UdpService(m_srcPort, m_destIp.c_str(), m_destPort, m_udpmulticastloopback);
        }
#endif

        if (!m_service)
        {
            return false;
        }

        m_service->SetRecvCallback(std::bind(&Device::DealReceivedData, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }

    if (m_service->IsStart())
    {
        return true;
    }
    else
    {
        if (m_service->Start())
        {
            StartDataProcessThread();
            m_tpDevStartTickCount = std::chrono::steady_clock::now();

            return true;
        }
    }

    return false;
}

bool Device::StopCAN(UINT chnIdx)
{
    if (chnIdx >= m_devChnlCount)
    {
        return false;
    }

    m_bitChnlStarted.reset(chnIdx);

    if (m_bitChnlStarted.any())
    {
        return true;
    }

    return _StopAndClear();
}

bool Device::Stop()
{
    m_bitChnlStarted.reset();

    return _StopAndClear();
}

#include <iostream>
bool Device::_StopAndClear()
{
    LOG_INFO("_StopAndClear+++\n");
    if (!m_service)
    {
        LOG_INFO("_StopAndClear0---\n");
        return false;
    }

#ifdef WIN32
	LARGE_INTEGER nFreq;
	LARGE_INTEGER nBeginTime;
	LARGE_INTEGER nEndTime;
	QueryPerformanceFrequency(&nFreq);//��ȡʱ������
	QueryPerformanceCounter(&nBeginTime);
#endif

    m_service->Stop();

#ifdef WIN32
	QueryPerformanceCounter(&nEndTime);
	double ddf = (double)nFreq.QuadPart;
	double ddt = (double)(nEndTime.QuadPart - nBeginTime.QuadPart);
	double time = ddt / ddf;
	// std::cout << __FUNCTION__ << "^^^^ StopCAN time is: " << time * 1000 << "ms" << std::endl;
	//LOG_ALWAYS("%s^^^^ DisconnectServer time is: %fms", __FUNCTION__, time * 1000);
#endif

	StopDataProcessThread();

	{
        std::lock_guard<std::mutex> lk_(m_recvDecodersMutex);
        for (auto& decoder : m_vecDecoders)
        {
            delete decoder;
            decoder = nullptr;
        }
        m_vecDecoders.clear();
    }

    LOG_ALWAYS("Device TxBytes:%I64u TxFailedBytes:%I64u RxBytes:%I64u", m_statistic.nTxBytesSuccess, m_statistic.nTxBytesFailed, m_statistic.nRxBytes);

    //CAN
    for (size_t i = 0; i < m_devChnlCount; i++)
    {
        //Clear can buffer
        ClearBuffer(i);

        //auto send data
        m_autoSendData[i].clear();

        //log chnl statistic
        LOG_ALWAYS("Chnl %d: TxCAN:%I64u(S:%I64u) TxCANFD:%I64u(S:%I64u) RxCAN:%I64u(D:%I64u) RxCANFD:%I64u(D:%I64u) Err:%I64u TxEcho:%I64u RxMerge:%I64u(D:%I64u) AppRxMerge:%I64u", 
            i, m_statistic.chnl[i].nTxCAN, m_statistic.chnl[i].nTxCANSim, m_statistic.chnl[i].nTxCANFD, m_statistic.chnl[i].nTxCANFDSim,
            m_statistic.chnl[i].nRxCAN, m_statistic.chnl[i].nRxCANDrop, m_statistic.chnl[i].nRxCANFD, m_statistic.chnl[i].nRxCANFDDrop,
            m_statistic.chnl[i].nError, m_statistic.chnl[i].nTxEcho, m_statistic.chnl[i].nRxMerge, m_statistic.chnl[i].nRxMergeDrop, m_statistic.chnl[i].nAppRxMerge);
    }

    //LIN
    for (size_t i = 0; i < m_devLINCount; i++)
    {
        m_frmLIN[i].clear();
    }

    //Other
    {
        m_frmGPS.clear();
        m_frmMerge.clear();
    }

    {
        std::lock_guard<std::mutex> lk(m_authMapMutex);
        m_authDataMap.clear();
    }

    {
        std::lock_guard<std::mutex> lk(m_pktRequestMutex);
        m_pktRequestDataMap.clear();
    }

    delete m_service;
    m_service = nullptr;

    memset(&m_statistic, 0, sizeof(m_statistic));

	//ֻ��closedevice��ʱ��Ż���� resetcan/resetlin�����
    //m_bTxEcho = false;
    //m_bRecvMerge = false;

    //Clear Dev State
    m_stateSysInfo.bNewData = false;
    m_stateSysInfo.strState.clear();
    m_stateCANInfo.bNewData = false;
    m_stateCANInfo.strState.clear();
    m_stateRecorderInfo.bNewData = false;
    m_stateRecorderInfo.strState.clear();
    m_stateNetInfo.bNewData = false;
    m_stateNetInfo.strState.clear();

    m_bDevInfoUpdated = false;
    m_stDevInfo.fw_Version = 0;
    m_stDevInfo.hw_Version = 0;
    memset(m_stDevInfo.str_Serial_Num, 0, sizeof(m_stDevInfo.str_Serial_Num));
    memcpy(m_stDevInfo.str_hw_Type, m_devName.c_str(), min(m_devName.length(), (size_t)40));

    LOG_INFO("_StopAndClear1---\n");
    return true;
}

bool Device::InitLIN(UINT chnIdx, PZCAN_LIN_INIT_CONFIG pLINInitConfig)
{
    if (chnIdx < DEV_LIN_COUNT_MAX && pLINInitConfig != NULL)
    {
#if 0
        m_linConfig[chnIdx].IsMaster = (pLINInitConfig->linMode == LIN_MODE_MASTER);
        m_linConfig[chnIdx].nEnable = 0;
        m_linConfig[chnIdx].stFeature.rawVal = 0;
        m_linConfig[chnIdx].stFeature.unionVal.bEnhancedChksum = !!(pLINInitConfig->linFlag & LIN_FLAG_CHK_ENHANCE);
        m_linConfig[chnIdx].stFeature.unionVal.bVarDLCDisabled = !(pLINInitConfig->linFlag & LIN_FLAG_VAR_DLC);
        m_linConfig[chnIdx].nBaud = pLINInitConfig->linBaud;
        return true;
#endif
		m_linConfig[chnIdx].IsMaster = pLINInitConfig->linMode;
		m_linConfig[chnIdx].nEnable = 0;
		m_linConfig[chnIdx].stFeature.rawVal = 0;
		//������Ϣ��chkSumMode��������ϢchkSumMode��һ�¡��˴�����ת������
		if (pLINInitConfig->chkSumMode == CLASSIC_CHKSUM)
		{
			m_linConfig[chnIdx].stFeature.unionVal.bEnhancedChksum = 0;
		}
		if (pLINInitConfig->chkSumMode == ENHANCE_CHKSUM)
		{
			m_linConfig[chnIdx].stFeature.unionVal.bEnhancedChksum = 1;
		}
		if (pLINInitConfig->chkSumMode == AUTOMATIC)
		{
			m_linConfig[chnIdx].stFeature.unionVal.bEnhancedChksum = 2;
		}
		//m_linConfig[chnIdx].stFeature.unionVal.bVarDLCDisabled = !(pLINInitConfig->linFlag & LIN_FLAG_VAR_DLC);
		m_linConfig[chnIdx].nBaud = pLINInitConfig->linBaud;

		return true;
    }
    return false;
}

UINT Device::StartLIN(UINT chnIdx)
{
    if (chnIdx >= m_devLINCount)
    {
        return false;
    }
    UINT nLINIndex = DEV_CHNL_COUNT_MAX + chnIdx;
	if (m_bitChnlStarted.test(nLINIndex))
	{
		return true;
	}

    if (CreateAndStartService())
    {
        //TCP�ͻ���ģʽ,UDPģʽ�����豸����ʱ��ȡһ���豸��Ϣ
        if (!m_devTCP || TCP_SERVER != m_workMode)
        {
            GetDeviceInfo(m_stDevInfo);
        }

        //Start
        m_linConfig[chnIdx].nEnable = 1;
        bool bRet = SetLINConfig(chnIdx, m_linConfig[chnIdx]);
        if (bRet)
        {
            m_bitChnlStarted.set(nLINIndex);
            LOG_ALWAYS("Device LIN :%d start success!", chnIdx);
        }
        else
        {
            LOG_ALWAYS("Device LIN :%d start failed!", chnIdx);
        }
		if (m_devType == ZCAN_CANFDNET_TCP || m_devType == ZCAN_CANFDNET_UDP || m_devType == ZCAN_CANFDNET_400U_TCP ||
			m_devType == ZCAN_CANFDNET_400U_UDP || m_devType == ZCAN_CANFDNET_100U_TCP || m_devType == ZCAN_CANFDNET_100U_UDP ||
			m_devType == ZCAN_CANFDNET_800U_TCP || m_devType == ZCAN_CANFDNET_800U_UDP)
		{
			SyncDevClock();//ͬ���豸ʱ��
		}
        return bRet;
    }
    else
    {
        m_bitChnlStarted.reset(nLINIndex);
    }
    return false;
}

UINT Device::ResetLIN(UINT chnIdx)
{
    if (chnIdx >= m_devChnlCount)
    {
        return false;
    }

    m_linConfig[chnIdx].nEnable = 0;
    bool bRet = SetLINConfig(chnIdx, m_linConfig[chnIdx]);
    LOG_ALWAYS("Device LIN :%d Reset result:%d!", chnIdx, bRet);

    UINT nLINIndex = DEV_CHNL_COUNT_MAX + chnIdx;
    m_bitChnlStarted.reset(nLINIndex);

    if (m_bitChnlStarted.any())
    {
        return true;
    }

    return _StopAndClear();
}

UINT Device::TransmitLIN(UINT chnIdx, PZCAN_LIN_MSG pSend, UINT Len)
{
    if (chnIdx >= m_devLINCount || pSend == nullptr || Len == 0 || !m_linConfig[chnIdx].IsMaster)
    {
        return 0;
    }

    return SendLINMsgs(chnIdx, pSend, Len);
}

UINT Device::GetLINReceiveNum(UINT chnIdx)
{
    if (chnIdx < m_devLINCount)
    {
        return m_frmLIN[chnIdx].size();
    }
    return 0;
}

UINT Device::ReceiveLIN(UINT chnIdx, PZCAN_LIN_MSG pReceive, UINT Len, int WaitTime)
{
    if (chnIdx >= m_devLINCount || pReceive == nullptr || Len == 0)
    {
        return 0;
    }

    CanDataQueue<PacketDataLIN>& frmdata = m_frmLIN[chnIdx];

#ifdef WIN32
    frmdata.WaitEvent(WaitTime);
#else
    const int per_sleep_ms = 1;
    if (WaitTime > 0)
    {
        int sleepTimes = (int)ceil(WaitTime / (float)per_sleep_ms);
        while (sleepTimes--)
        {
            if (frmdata.size() <= 0)
            {
                cc_sleep(per_sleep_ms);
            }
            else
            {
                break;
            }
        }
    }
    else if (WaitTime < 0)
    {
        while (true)
        {
            if (frmdata.size() <= 0)
            {
                cc_sleep(per_sleep_ms);
            }
            else
            {
                break;
            }
        }
    }
#endif

    memset(pReceive, 0, Len*sizeof(ZCAN_LIN_MSG));

    frmdata.lock();
    UINT actLen = frmdata.size() > Len ? Len : frmdata.size();
    for (ULONG i = 0; i < actLen; i++)
    {
        CPacketUtil::PacketLIN2ZLIN(frmdata.front(), pReceive[i], nullptr);
        frmdata.pop();
    }
    frmdata.unlock();

    //LOG_INFO("ReceiveLIN Chnl:%d, count:%d\n", chnIdx, actLen);

    return actLen;
}

UINT Device::SetLINSlaveMsg(UINT chnIdx, PZCAN_LIN_MSG pSend, UINT nMsgCount)
{
    //��վģʽʱֱ�ӷ��ͣ���Ϊ���ô�վ��Ӧ��Ϣ
    if (chnIdx < m_devLINCount && m_linConfig[chnIdx].IsMaster == 0)
    {
        return SendLINMsgs(chnIdx, pSend, nMsgCount);
    }
    return false;
}

UINT Device::ClearLINSlaveMsg(UINT chnIdx, BYTE* pLINID, UINT nIDCount)
{
    //Э����ֻ���������ͨ����lin��վ��Ӧ��������Ҫ�������id����Ӧ
    //ֱ���ڴ�վ�з��Ͷ�Ӧ��id�������ݳ�����Ϊ0��Ϊ�����Ӧid����Ӧ
	/*if (chnIdx < m_devLINCount && m_linConfig[chnIdx].IsMaster == 0)
	{
	if (pLINID == nullptr || nIDCount == 0)
	{
	return ClearLINAsSlaveResponse(chnIdx);
	}
	else
	{
	std::vector<ZCAN_LIN_MSG> vecLINMsg;
	vecLINMsg.resize(nIDCount);
	for (UINT i = 0; i < nIDCount; ++i)
	{
	memset(&vecLINMsg[i], 0, sizeof(ZCAN_LIN_MSG));
	vecLINMsg[i].ID = pLINID[i];
	vecLINMsg[i].DataLen = 0;
	}
	return SendLINMsgs(chnIdx, vecLINMsg.data(), vecLINMsg.size());
	}
	}*/
    return false;
}

UINT Device::SendLINMsgs(UINT chnIdx, PZCAN_LIN_MSG pSend, UINT Len)
{
    UINT sentCnt = 0;
    VecPackets vecPackets;
    if (chnIdx >= m_devLINCount || pSend == nullptr || Len == 0)
    {
        return sentCnt;
    }

    vecPackets = CPacketEncoder::BuildPacketLIN(chnIdx, pSend, Len);
    for (auto& packet : vecPackets)
    {
        if (packet.size() != SendData(packet.data(), packet.size()))
        {
            m_statistic.chnl[chnIdx].nTxLIN += sentCnt;
            return sentCnt;
        }
        sentCnt += CPacketUtil::GetPacketFrameCount(packet);
    }
    m_statistic.chnl[chnIdx].nTxLIN += sentCnt;
    return sentCnt;
}

UINT Device::SetLINSubscribe(UINT chnIdx, PZCAN_LIN_SUBSCIBE_CFG pSend, UINT nMsgCount)
{
	UINT sentCnt = 0;
	VecPackets vecPackets;
	if (chnIdx >= m_devLINCount || pSend == nullptr || nMsgCount == 0)
	{
		return sentCnt;
	}

	vecPackets = CPacketEncoder::BuildPacketLIN(chnIdx, pSend, nMsgCount);
	for (auto& packet : vecPackets)
	{
		if (packet.size() != SendData(packet.data(), packet.size()))
		{
			m_statistic.chnl[chnIdx].nTxLIN += sentCnt;
			return sentCnt;
		}
		sentCnt += CPacketUtil::GetPacketFrameCount(packet);
	}
	m_statistic.chnl[chnIdx].nTxLIN += sentCnt;
	return sentCnt;
}

UINT Device::SetLINPublish(UINT chnIdx, PZCAN_LIN_PUBLISH_CFG pSend, UINT nMsgCount)
{
	UINT sentCnt = 0;
	VecPackets vecPackets;
	if (chnIdx >= m_devLINCount || pSend == nullptr || nMsgCount == 0)
	{
		return sentCnt;
	}

	vecPackets = CPacketEncoder::BuildPacketLIN(chnIdx, pSend, nMsgCount);
	for (auto& packet : vecPackets)
	{
		if (packet.size() != SendData(packet.data(), packet.size()))
		{
			m_statistic.chnl[chnIdx].nTxLIN += sentCnt;
			return sentCnt;
		}
		sentCnt += CPacketUtil::GetPacketFrameCount(packet);
	}
	m_statistic.chnl[chnIdx].nTxLIN += sentCnt;
	return sentCnt;
}

bool Device::AddAutoSend(UINT chnIdx, const ZCAN_AUTO_TRANSMIT_OBJ* pObj)
{
    if (chnIdx < m_devChnlCount && pObj && pObj->index < DEV_AUTO_SEND_INDEX_MAX)
    {
        VecAutoSendData& vecAutoSend = m_autoSendData[chnIdx];
        AutoSendData* pAutoSendData = GetAutoSendIndex(vecAutoSend, pObj->index);
        if (pAutoSendData != nullptr)
        {
            ZCANAutoSend2AutoSend(pObj, *pAutoSendData);
        }
        else
        {
            AutoSendData autoSend;
            ZCANAutoSend2AutoSend(pObj, autoSend);
            vecAutoSend.push_back(autoSend);
        }

        return true;
    }
    return false;
}

bool Device::AddAutoSend(UINT chnIdx, const ZCANFD_AUTO_TRANSMIT_OBJ* pObj)
{
    if (chnIdx < m_devChnlCount && pObj && pObj->index < DEV_AUTO_SEND_INDEX_MAX)
    {
        VecAutoSendData& vecAutoSend = m_autoSendData[chnIdx];
        AutoSendData* pAutoSendData = GetAutoSendIndex(vecAutoSend, pObj->index);
        if (pAutoSendData != nullptr)
        {
            ZCANAutoSend2AutoSend(pObj, *pAutoSendData);
        }
        else
        {
            AutoSendData autoSend;
            ZCANAutoSend2AutoSend(pObj, autoSend);
            vecAutoSend.push_back(autoSend);
        }

        return true;
    }
    return false;
}

bool Device::ApplyAutoSend(UINT chnIdx)
{
    if (chnIdx < m_devChnlCount)
    {
        VecAutoSendData& vecAutoSend = m_autoSendData[chnIdx];
        return SetAutoSend(chnIdx, vecAutoSend);
    }
    return false;
}

bool Device::ClearAutoSend(UINT chnIdx)
{
    if (chnIdx < m_devChnlCount)
    {
        VecAutoSendData& vecAutoSend = m_autoSendData[chnIdx];
        for (auto & item : vecAutoSend)
        {
            item.data.enable = 0;
        }

        bool bret = SetAutoSend(chnIdx, vecAutoSend);
        vecAutoSend.clear();
        return bret;
    }
    return false;
}

AutoSendData* Device::GetAutoSendIndex(VecAutoSendData& vecAutoSend, UINT autoSendIndex)
{
    for (auto& item : vecAutoSend)
    {
        if (item.data.index == autoSendIndex)
        {
            return &item;
        }
    }
    return nullptr;
}

void Device::ZCANAutoSend2AutoSend(const ZCAN_AUTO_TRANSMIT_OBJ* pObj, AutoSendData& autoSend)
{
    memset(&autoSend, 0, sizeof(autoSend));
    autoSend.iscanfd = 0;
    autoSend.sendcount = 0;
    memcpy(&autoSend.data, pObj, sizeof(ZCAN_AUTO_TRANSMIT_OBJ));
}

void Device::ZCANAutoSend2AutoSend(const ZCANFD_AUTO_TRANSMIT_OBJ* pObj, AutoSendData& autoSend)
{
    memset(&autoSend, 0, sizeof(autoSend));
    autoSend.iscanfd = 1;
    autoSend.sendcount = 0;
    memcpy(&autoSend.data, pObj, sizeof(ZCANFD_AUTO_TRANSMIT_OBJ));
}

bool Device::IsDevStartLessThan(long long ms)
{
    std::chrono::steady_clock::time_point tp_recv = std::chrono::steady_clock::now();
    std::chrono::milliseconds time_span = std::chrono::duration_cast<std::chrono::milliseconds>(tp_recv - m_tpDevStartTickCount);
    return time_span.count() <= ms;
}

bool Device::SetDevTimeStamp(UINT64 nTimeStampUs)
{
    PacketTypeREQParam reqParam;
    reqParam.rawValue = 0;
    reqParam.unionValue.nReqVal = PACKET_TYPE_PARAM_DEV_REQ_RES_TIMESTAMP;
    UINT64 nTimeNetOrder = cc_htonll(nTimeStampUs);
    BYTE* data = (BYTE*)&nTimeNetOrder;
    Packet packet = CPacketEncoder::BuildPacketDevReqRes(true, data, sizeof(nTimeNetOrder), reqParam.rawValue);
    return SendData(packet.data(), packet.size()) == packet.size();
}

UINT Device::GetTxEchoTimeStamp(UINT chnIdx, TxTimeStamp& rTxTimeStamp)
{
    if (chnIdx >= m_devChnlCount || rTxTimeStamp.pTxTimeStampBuffer == nullptr || rTxTimeStamp.nBufferTimeStampCount == 0)
    {
        return 0;
    }

    return 0;
}

bool Device::GetBusUsage(UINT chnIdx, BusUsage& rBusUsage)
{
    if (chnIdx < m_devChnlCount && m_busUsage.newdata[chnIdx])
    {
        rBusUsage.nBusUsage = m_busUsage.chnl[chnIdx].nBusUsage;
        rBusUsage.nChnl = m_busUsage.chnl[chnIdx].nChnl;
        rBusUsage.nFrameCount = m_busUsage.chnl[chnIdx].nFrameCount;
        rBusUsage.nReserved = m_busUsage.chnl[chnIdx].nReserved;
        rBusUsage.nTimeStampBegin = m_busUsage.chnl[chnIdx].nTimeStampBegin;
        rBusUsage.nTimeStampEnd = m_busUsage.chnl[chnIdx].nTimeStampEnd;

        m_busUsage.newdata[chnIdx] = false;
        return true;
    }
    return false;
}

bool Device::GetDelaySendAvailableTxCount(UINT chnIdx, UINT& nCount)
{
    if (chnIdx < m_devChnlCount)
    {
        std::string strDelayInfo;
        if (GetDelaySendInfo(chnIdx, strDelayInfo))
        {
            cJSON* pInfo = cJSON_Parse(strDelayInfo.c_str());
            if (pInfo)
            {
                UINT nSize = (UINT)UtilGetcJSONNumber(cJSON_DetachItemFromObjectCaseSensitive(pInfo, "Size"));
                nCount = (UINT)UtilGetcJSONNumber(cJSON_DetachItemFromObjectCaseSensitive(pInfo, "Remain"));
                cJSON_Delete(pInfo);
                return true;
            }
        }
    }
    return false;
}

bool Device::GetDevAutoSendListCount(UINT chnIdx, bool bCANFD, UINT& nCount)
{
    if (chnIdx < m_devChnlCount)
    {
        std::chrono::steady_clock::time_point tp_now = std::chrono::steady_clock::now();
        std::chrono::milliseconds time_span = std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - m_devAutoSendList.chnl[chnIdx].m_tpRecv);
        if (time_span.count() > REQUEST_TIMEOUT)
        {
            std::vector<PacketDataAutoSend> vecAutoSend;
            if (!GetDevAutoSendList(chnIdx, vecAutoSend))
            {
                return false;
            }

            m_devAutoSendList.chnl[chnIdx].vecCAN.clear();
            m_devAutoSendList.chnl[chnIdx].vecCANFD.clear();
            m_devAutoSendList.chnl[chnIdx].m_tpRecv = std::chrono::steady_clock::now();
            ZCAN_AUTO_TRANSMIT_OBJ zCAN;
            ZCANFD_AUTO_TRANSMIT_OBJ zCANFD;
            int nChnl;
            for (auto& item : vecAutoSend)
            {
                if (item.bEnable)
                {
                    if (item.pktData.canHead.frameInfo.unionVal.bFD)
                    {
                        CPacketUtil::AutoSendFrame2ZCANAutoSend(item, zCANFD, nChnl);
                        m_devAutoSendList.chnl[chnIdx].vecCANFD.push_back(zCANFD);
                    }
                    else
                    {
                        CPacketUtil::AutoSendFrame2ZCANAutoSend(item, zCAN, nChnl);
                        m_devAutoSendList.chnl[chnIdx].vecCAN.push_back(zCAN);
                    }
                }
            }
        }

        if (bCANFD)
        {
            nCount = m_devAutoSendList.chnl[chnIdx].vecCANFD.size();
        }
        else
        {
            nCount = m_devAutoSendList.chnl[chnIdx].vecCAN.size();
        }
        return true;
    }
    return false;
}

bool Device::GetDevAutoSendListData(UINT chnIdx, bool bCANFD, void* pData)
{
    if (chnIdx < m_devChnlCount && pData != nullptr)
    {
        int nCount = 0, nSize=0;
        if (bCANFD)
        {
            nCount = m_devAutoSendList.chnl[chnIdx].vecCANFD.size();
            nSize = sizeof(ZCANFD_AUTO_TRANSMIT_OBJ) * nCount;
            memcpy(pData, m_devAutoSendList.chnl[chnIdx].vecCANFD.data(), nSize);
        }
        else
        {
            nCount = m_devAutoSendList.chnl[chnIdx].vecCAN.size();
            nSize = sizeof(ZCAN_AUTO_TRANSMIT_OBJ) * nCount;
            memcpy(pData, m_devAutoSendList.chnl[chnIdx].vecCAN.data(), nSize);
        }
        return true;
    }
    return false;
}

bool Device::RequestDevData(PacketDataRequest& devRequest, PacketDataResponse& devResponse, bool autoProtocol)
{
    bool bRet = false;
    BYTE nSeq = CreatePktRequestSeq();
    uint32_t nTID = CreatePktTID();

    // �ɵ��������TIDֻ��һ���ֽ�
    if (!autoProtocol || !IsSupportRequestResponseEx()) {
        nTID &= 0xFF;
    }

    devRequest.nTID = nTID;
    DevRequestResponse_PTR data = std::make_shared<DevRequestResponse>();
    if (data)
    {
        data->nSeq = nSeq;
        data->nTID = nTID;
        data->requestData = devRequest;
    }
    else
    {
        return false;
    }

    uint64_t key = GetDevRequestKey(nSeq, nTID);
    Packet packet;
    if (autoProtocol && IsSupportRequestResponseEx()) {
        packet = CPacketEncoder::BuildPacketRequestEx(nSeq, devRequest);
    }
    else {
        packet = CPacketEncoder::BuildPacketRequest(nSeq, devRequest);
    }
    LOG_INFO("request:%d, seq:%d, tid:%u\n", devRequest.nRequest, nSeq, nTID);

	//wait for response
	{
		std::unique_lock<std::mutex> lk_(m_pktRequestMutex);
		m_pktRequestDataMap[key] = data;
	}
    if (packet.size() == SendData(packet.data(), packet.size()))
    {
        std::mutex mutex_;
        std::unique_lock<std::mutex> ulk(mutex_);
		if (data->cv.wait_for(ulk, std::chrono::milliseconds(REQUEST_TIMEOUT), [&data]{
			return data->responsed; }))
			{
			devResponse = data->responseData;
			bRet = true;
            LOG_INFO("response data copyed, seq:%d tid:%u/%u\n", nSeq, nTID, devResponse.nTID);
        }
        else
        {
            LOG_ERR("response timeout\n");
        }

        //Clear result
        {
            std::lock_guard<std::mutex> lk_(m_pktRequestMutex);
            m_pktRequestDataMap.erase(key);
        }
    }
    else
    {
        LOG_ERR("send data error\n");
    }
    return bRet;
}

uint64_t Device::GetDevRequestKey(BYTE pktSeq, uint32_t nTID)
{
    pktSeq = pktSeq % 8;
    return ((uint64_t)pktSeq << 32) | nTID;
}
void Device::GenGetDevInfoRequest(PacketDataRequest& devRequest)
{
    devRequest.nRequest = PACKET_REQUEST_GET_DEVICE_INFO;
    devRequest.nTID = 0;
    devRequest.vecData.clear();
}

void Device::GenGetDevStateRequest(BYTE nType, PacketDataRequest& devRequest)
{
    devRequest.nRequest = PACKET_REQUEST_GET_DEVICE_STATE;
    devRequest.nTID = 0;
    char requestJson[64];
    memset(requestJson, 0, sizeof(requestJson));
    int nLen = snprintf(requestJson, 64, "{\r\n\"Type\":%d\r\n}", nType) + 1;
    BYTE* pData = (BYTE*)&requestJson[0];
    devRequest.vecData.assign(pData, pData + nLen);
}

void Device::GenGetAutoSendListRequest(UINT chnIdx, UINT nOffset, UINT nRequestCount, PacketDataRequest& devRequest)
{
    devRequest.nRequest = PACKET_REQUEST_GET_DEVICE_AUTOSEND_LIST;
    devRequest.nTID = 0;
    char requestJson[200];
    memset(requestJson, 0, sizeof(requestJson));
    int nLen = snprintf(requestJson, 200, "{\r\n\"Chn\":%d,\r\n\"Offset\":%d,\r\n\"Num\":%d\r\n}", chnIdx, nOffset, nRequestCount) + 1;
    BYTE* pData = (BYTE*)&requestJson[0];
    devRequest.vecData.assign(pData, pData + nLen);
}

void Device::GenGetDevDelaySendRequest(UINT chnIdx, PacketDataRequest& devRequest)
{
    devRequest.nRequest = PACKET_REQUEST_GET_DEVICE_DELAY_SEND_STATE;
    devRequest.nTID = 0;
    char requestJson[64];
    memset(requestJson, 0, sizeof(requestJson));
    int nLen = snprintf(requestJson, 64, "{\r\n\"Chn\":%d\r\n}", chnIdx) + 1;
    BYTE* pData = (BYTE*)&requestJson[0];
    devRequest.vecData.assign(pData, pData + nLen);
}

void Device::GenClearDelaySendRequest(UINT chnIdx, PacketDataRequest& devRequest)
{
    devRequest.nRequest = PACKET_REQUEST_CLEAR_DELAY_SEND_QUEUE;
    devRequest.nTID = 0;
    char requestJson[64];
    memset(requestJson, 0, sizeof(requestJson));
    int nLen = snprintf(requestJson, 64, "{\r\n\"Chn\":%d\r\n}", chnIdx) + 1;
    BYTE* pData = (BYTE*)&requestJson[0];
    devRequest.vecData.assign(pData, pData + nLen);
}

bool Device::GetDevInfo(std::string& strDevInfo)
{
    PacketDataRequest request;
    PacketDataResponse response;
    GenGetDevInfoRequest(request);
    // ֻ��ʹ�þɵ���������ȥ��ȡ, �����ݹ�����
    if (RequestDevData(request, response, false))
    {
        if (response.nResult)
        {
            std::string strResult(response.vecData.begin(), response.vecData.end());
            strDevInfo = strResult.c_str();
        }
        return response.nResult > 0;
    }
    return false;
}

bool Device::GetDevState(BYTE nType, std::string& strDevState)
{
    PacketDataRequest request;
    PacketDataResponse response;
    GenGetDevStateRequest(nType, request);
    if (RequestDevData(request, response))
    {
        if (response.nResult)
        {
            std::string strResult(response.vecData.begin(), response.vecData.end());
            strDevState = strResult.c_str();
        }
        return response.nResult > 0;
    }
    return false;
}

bool Device::GetDevAutoSendList(UINT chnIdx, std::vector<PacketDataAutoSend>& vecAutoSend)
{
    //����֡�������ƣ�һ֡��ʱ����ռ��88�ֽڣ�һ����̫��֡���԰�װ(1460-6-1)/88 = 16 ֡
    UINT nTotalCount = DEV_AUTO_SEND_INDEX_MAX;
    UINT nReaded = 0;
    PacketDataRequest request;
    PacketDataResponse response;
    vecAutoSend.clear();
    std::vector<PacketDataAutoSend> vecAutoSendOnce;
    while (nReaded < nTotalCount)
    {
        //����һ��ͨѶ��Ҫ�ڵ�������ɴ��䣬һ�λ�ȡ������ᵼ����Ӧ���ݳ������������ݳ���
        //��������һ�λ�ȡ����Ŀ��������Ϊ4
        UINT nReadOnce = min((UINT)4, nTotalCount - nReaded);
        GenGetAutoSendListRequest(chnIdx, nReaded, nReadOnce, request);
        if (RequestDevData(request, response))
        {
            if (response.nResult)
            {
                //response�е�vecData������ʱ���ĵ�json�ַ���
                std::string strResult(response.vecData.begin(), response.vecData.end());
                UINT nCount = GetAutoSendListFromJsonStr(chnIdx, nReaded, nReadOnce, strResult, vecAutoSendOnce);
                vecAutoSend.insert(vecAutoSend.end(), vecAutoSendOnce.begin(), vecAutoSendOnce.end());
                nReaded += nCount;
                if (0 == nCount || nCount < nReadOnce)
                {
                    break;
                }
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    assert(nReaded == vecAutoSend.size());
    return true;
}

bool Device::GetDelaySendInfo(UINT chnIdx, std::string& strDelaySendInfo)
{
    PacketDataRequest request;
    PacketDataResponse response;
    GenGetDevDelaySendRequest(chnIdx, request);
    if (RequestDevData(request, response))
    {
        if (response.nResult)
        {
            std::string strResult(response.vecData.begin(), response.vecData.end());
            strDelaySendInfo = strResult.c_str();
        }
        return response.nResult > 0;
    }
    return false;
}

bool Device::ClearDelaySendQueue(UINT chnIdx)
{
    if (chnIdx < m_devChnlCount)
    {
        PacketDataRequest request;
        PacketDataResponse response;
        GenClearDelaySendRequest(chnIdx, request);
        if (RequestDevData(request, response))
        {
            return response.nResult > 0;
        }
        return false;
    }
    return false;
}

bool Device::GetDevInfoFromJsonStr(const std::string& strDevInfo, ZCAN_DEVICE_INFO * pInfo)
{
    LOG_INFO("Device info string:%s\n", strDevInfo.c_str());
    if (pInfo == nullptr)
    {
        return false;
    }

    cJSON* pDevInfo = cJSON_Parse(strDevInfo.c_str());
    if (pDevInfo)
    {
        std::string strModel = UtilGetcJSONString(cJSON_GetObjectItemCaseSensitive(pDevInfo, "Model"));
        std::string strFirmware = UtilGetcJSONString(cJSON_GetObjectItemCaseSensitive(pDevInfo, "Firmware"));
        std::string strHardware = UtilGetcJSONString(cJSON_GetObjectItemCaseSensitive(pDevInfo, "Hardware"));
        std::string strSN = UtilGetcJSONString(cJSON_GetObjectItemCaseSensitive(pDevInfo, "SN"));
        
        // 20201105��ͨѶЭ��汾��V1.10����ڴ��ֶ�
        std::string protoVer = UtilGetcJSONString(cJSON_GetObjectItemCaseSensitive(pDevInfo, "ProtoVer"));   
        memset(m_protocolVersion, 0, sizeof(m_protocolVersion));
        std::replace(protoVer.begin(), protoVer.end(), 'V', '0');
        std::replace(protoVer.begin(), protoVer.end(), 'v', '0');
        std::vector<std::string> vers = cc_split(protoVer, ".");
        if (vers.size() >= 2) {
            m_protocolVersion[0] = atoi(vers[0].c_str());
            m_protocolVersion[1] = atoi(vers[1].c_str());
        }

        memset(pInfo, 0, sizeof(ZCAN_DEVICE_INFO));
        pInfo->can_Num = m_devChnlCount;
        memcpy(pInfo->str_hw_Type, strModel.c_str(), min(strModel.length(), sizeof(pInfo->str_hw_Type)));
        memcpy(pInfo->str_Serial_Num,  strSN.c_str(), min(strSN.length(), sizeof(pInfo->str_Serial_Num)));
        pInfo->dr_Version = 0x0100;
        pInfo->in_Version = 0x0100;
        pInfo->fw_Version = UtilVersionString2USHORT(strFirmware.c_str());
        pInfo->hw_Version = UtilVersionString2USHORT(strHardware.c_str());

		//2023.03.23�¼�can linͨ������ȡ
		UINT nCanChnNum = (UINT)UtilGetcJSONNumber(cJSON_GetObjectItemCaseSensitive(pDevInfo, "CanChnNum"));
		UINT nLinChnNum = (UINT)UtilGetcJSONNumber(cJSON_GetObjectItemCaseSensitive(pDevInfo, "LinChnNum"));
		pInfo->can_Num = nCanChnNum;
		pInfo->reserved[0] = nLinChnNum;

        cJSON_Delete(pDevInfo);
        return true;
    }

    return false;
}

UINT Device::GetAutoSendListFromJsonStr(UINT chnIdx, UINT nOffset, UINT nRequestCount, const std::string& strAutoSendJson, std::vector<PacketDataAutoSend>& vecAutoSend)
{
    cJSON* pAutoSendMsgs = cJSON_Parse(strAutoSendJson.c_str());
    if (pAutoSendMsgs)
    {
        vecAutoSend.clear();
        PacketDataAutoSend item;
        for (UINT i = nOffset; i<nOffset + nRequestCount; ++i)
        {
            std::string key = UtilStringAppendIndex("Msg", i);
            cJSON* pSendItem = cJSON_GetObjectItemCaseSensitive(pAutoSendMsgs, key.c_str());
            if (pSendItem)
            {
                GetAutoSendFromJson(chnIdx, i, pSendItem, item);
                vecAutoSend.push_back(item);
            }
            else
            {
                break;
            }
        }

        cJSON_Delete(pAutoSendMsgs);
    }

    return vecAutoSend.size();
}

void Device::GetAutoSendFromJson(UINT nChnIdx, UINT nIndex, cJSON* config, PacketDataAutoSend& autoSend)
{
    if (!config) return;
    memset(&autoSend, 0, sizeof(autoSend));

    //20201106 Э��汾����V1.10֮�����ӷ������ڵ�λ�ֶΣ��޲������߲���ֵΪ0��ʾ��λ��ms��ֵΪ1��ʾʱ�䵥λΪ100us(0.1ms)
    int nPeriodUnit = (int)UtilGetcJSONNumber(cJSON_GetObjectItemCaseSensitive(config, "PeriodUnit"));
    nPeriodUnit = !!nPeriodUnit;
    int nPeriodValue = (int)UtilGetcJSONNumber(cJSON_GetObjectItemCaseSensitive(config, "Period"));
    autoSend.stFlag.unionValue.nUseDataTimestamp = nPeriodUnit;
    autoSend.nPeriodMs = nPeriodUnit == 0 ? nPeriodValue : nPeriodValue/10;
    autoSend.pktData.canHead.nTimestamp = nPeriodValue;

    autoSend.nIndex = (BYTE)nIndex;
    autoSend.bEnable = (BYTE)UtilGetcJSONNumber(cJSON_GetObjectItemCaseSensitive(config, "Enable"));
    autoSend.nRepeatCount = (USHORT)UtilGetcJSONNumber(cJSON_GetObjectItemCaseSensitive(config, "Cnt"));

    autoSend.pktData.canHead.nID = (int)UtilGetcJSONNumber(cJSON_GetObjectItemCaseSensitive(config, "MsgID"));
    autoSend.pktData.canHead.frameInfo.rawVal = (int)UtilGetcJSONNumber(cJSON_GetObjectItemCaseSensitive(config, "MsgFlag"));
    autoSend.pktData.canHead.nChnl = (BYTE)nChnIdx;
    cJSON* msgData = cJSON_GetObjectItemCaseSensitive(config, "MsgData");
    autoSend.pktData.canHead.nDataLen = UtilJsonByteArray(msgData, &autoSend.pktData.canData[0], DATA_LEN_CANFD);
}

UINT Device::GetGPSRecvNum()
{
    return m_frmGPS.size();
}

UINT Device::GetGPSData(ZCAN_GPS_FRAME* pReceive, UINT len, int waitTime)
{
    if (pReceive == nullptr || len == 0)
    {
        return 0;
    }

    CanDataQueue<PacketDataGPS>& frmGPS = m_frmGPS;

    const int per_sleep_ms = 10;
    if (waitTime > 0)
    {
        int sleepTimes = (int)ceil(waitTime / (float)per_sleep_ms);
        while (sleepTimes--)
        {
            if (frmGPS.size() <= 0)
            {
                cc_sleep(per_sleep_ms);
            }
            else
            {
                break;
            }
        }
    }
    else if (waitTime < 0)
    {
        while (true)
        {
            if (frmGPS.size() <= 0)
            {
                cc_sleep(per_sleep_ms);
            }
            else
            {
                break;
            }
        }
    }

    frmGPS.lock();
    UINT actLen = frmGPS.size() > len ? len : frmGPS.size();
    for (ULONG i = 0; i < actLen; i++)
    {
        CPacketUtil::PacketGPS2ZGPS(frmGPS.front(), pReceive[i]);
        frmGPS.pop();
    }
    frmGPS.unlock();

    return actLen;
}

bool Device::GetDeviceInfo(ZCAN_DEVICE_INFO& devInfo)
{
    std::string strDevInfo;
    if (GetDevInfo(strDevInfo))
    {
        m_bDevInfoUpdated = GetDevInfoFromJsonStr(strDevInfo, &devInfo);
        return m_bDevInfoUpdated;
    }
    return false;
}

bool Device::ClearLINAsSlaveResponse(UINT chnIdx)
{
    if (chnIdx < m_devLINCount)
    {
        PacketDataRequest request;
        PacketDataResponse response;
        GenClearLINAsSlaveResponseRequest(chnIdx, request);
        if (RequestDevData(request, response))
        {
            return response.nResult > 0;
        }
        return false;
    }
    return false;
}

void Device::GenClearLINAsSlaveResponseRequest(UINT chnIdx, PacketDataRequest& devRequest)
{
    devRequest.nRequest = PACKET_REQUEST_CLEAR_LIN_AS_SLAVE_RESPONSE;
    devRequest.nTID = 0;
    char requestJson[64];
    memset(requestJson, 0, sizeof(requestJson));
    int nLen = snprintf(requestJson, 64, "{\r\n\"Chn\":%d\r\n}", chnIdx) + 1;
    BYTE* pData = (BYTE*)&requestJson[0];
    devRequest.vecData.assign(pData, pData + nLen);
}

bool Device::GetLINTxFIFOInfo(UINT chnIdx, LINFIFOInfo& fifoInfo)
{
    if (chnIdx < m_devLINCount)
    {
        PacketDataRequest request;
        PacketDataResponse response;
        GenGetLINTxFIFOInfoRequest(chnIdx, request);
        if (RequestDevData(request, response))
        {
            if (response.nResult)
            {
                std::string strResult(response.vecData.begin(), response.vecData.end());
                return GetFIFOInfoFromJsonStr(strResult, fifoInfo);
            }
            return response.nResult > 0;
        }
        return false;
    }
    return false;
}

void Device::GenGetLINTxFIFOInfoRequest(UINT chnIdx, PacketDataRequest& devRequest)
{
    devRequest.nRequest = PACKET_REQUEST_GET_LIN_FIFO_INFO;
    devRequest.nTID = 0;
    char requestJson[64];
    memset(requestJson, 0, sizeof(requestJson));
    int nLen = snprintf(requestJson, 64, "{\r\n\"Chn\":%d\r\n}", chnIdx) + 1;
    BYTE* pData = (BYTE*)&requestJson[0];
    devRequest.vecData.assign(pData, pData + nLen);
}

bool Device::GetFIFOInfoFromJsonStr(std::string& strFIFOInfo, LINFIFOInfo& fifoInfo)
{
    LOG_INFO("Device LIN FIFO info string:%s\n", strFIFOInfo.c_str());

    cJSON* pInfo = cJSON_Parse(strFIFOInfo.c_str());
    if (pInfo)
    {
        fifoInfo.nChnl = (UINT)UtilGetcJSONNumber(cJSON_GetObjectItemCaseSensitive(pInfo, "Chn"));
        fifoInfo.nFIFOSize = (UINT)UtilGetcJSONNumber(cJSON_GetObjectItemCaseSensitive(pInfo, "Size"));
        fifoInfo.nFIFOAvailable = (UINT)UtilGetcJSONNumber(cJSON_GetObjectItemCaseSensitive(pInfo, "Remain"));

        cJSON_Delete(pInfo);
        return true;
    }
    return false;
}

bool Device::SetLINConfig(UINT chnIdx,const LINConfig& linConfig)
{
    if (chnIdx < m_devLINCount)
    {
        PacketDataRequest request;
        PacketDataResponse response;
        GenSetLINConfigRequest(chnIdx, linConfig, request);
        if (RequestDevData(request, response))
        {
            if (response.nResult)
            {
                std::string strResult(response.vecData.begin(), response.vecData.end());
                return true;
            }
            return response.nResult > 0;
        }
        return false;
    }
    return false;
}

void Device::GenSetLINConfigRequest(UINT chnIdx, const LINConfig& linConfig, PacketDataRequest& devRequest)
{
    devRequest.nRequest = PACKET_REQUEST_SET_CONFIG;
    devRequest.nTID = 0;

    cJSON* pChnlObj = cJSON_CreateObject();
    cJSON_AddNumberToObject(pChnlObj, "Enable", linConfig.nEnable);
    cJSON_AddNumberToObject(pChnlObj, "IsMaster", linConfig.IsMaster);
    cJSON_AddNumberToObject(pChnlObj, "Baudrate", linConfig.nBaud);
    cJSON_AddNumberToObject(pChnlObj, "Feature", linConfig.stFeature.rawVal);

    char chnlKey[10] = { 0 };
    snprintf(chnlKey, 10, "LIN%d", chnIdx);
    cJSON* pChnlConfig = cJSON_CreateObject();
    cJSON_AddItemToObject(pChnlConfig, chnlKey, pChnlObj);

    //20210818,��λ����Ҫ��LIN��̬����ʱ����������LIN�ڵ�
    cJSON* pLINConfig = cJSON_CreateObject();
    cJSON_AddItemToObject(pLINConfig, "LIN", pChnlConfig);

    char* pStr = cJSON_Print(pLINConfig);
    std::string strConfig(pStr);
    devRequest.vecData.assign(strConfig.c_str(), strConfig.c_str() + strConfig.length());
    cJSON_free(pStr);
    cJSON_Delete(pLINConfig);
}

void Device::StartDataProcessThread()
{
    // run data process thread
    m_bThdRun = true;
    m_DataProcessThd = std::thread(std::bind(&Device::DataProcessThdLoop, this));
	
#ifdef WIN32
    thread_set_priority(m_DataProcessThd, THREAD_PRIORITY_TIME_CRITICAL);
#endif
}

void Device::StopDataProcessThread()
{
    LOG_INFO("StopDataProcessThread +++\n");
    m_bThdRun = false;
    if (m_DataProcessThd.joinable())
    {
        m_DataProcessThd.join();
    }
    LOG_INFO("StopDataProcessThread ---\n");
}

CPacketDecoder* Device::GetDecoder(const char* rmt)
{
    std::unique_lock<std::mutex> lk_(m_recvDecodersMutex);
    for (auto* decoder : m_vecDecoders)
    {
        if (decoder->GetDecoderName().compare(rmt) == 0)
        {
            return decoder;
        }
    }

    CPacketDecoder* pNewDeocder = new CPacketDecoder(rmt);
    m_vecDecoders.push_back(pNewDeocder);
    return pNewDeocder;
}

void Device::DealReceivedData(const char* src_addr, const char* data, unsigned int len)
{
    //remote endpoint, such as "192.168.0.25:3005"
    std::string rmt_ep(src_addr);
    m_statistic.nRxBytes += len;
    CPacketDecoder* decoder = GetDecoder(src_addr);
    if (decoder)
    {
        decoder->AppendData(data, len);

#ifdef WIN32
        SetEvent(m_hDecoderData);
#endif
    }
    else
    {
        LOG_ERR("No valid decoder for %s!\n", src_addr);
    }
}

void Device::DataProcessThdLoop()
{
#ifdef WIN32
	if (m_workMode == TCP_SERVER) {
		LOG_ALWAYS("Device TcpServer DataProcessThdLoop thread,tid = %u", GetCurrentThreadId());
	} else {
		LOG_ALWAYS("Device TcpClient DataProcessThdLoop thread,tid = %u", GetCurrentThreadId());
	}
#endif

    LOG_INFO("Device DataProcessThread loop run\n");
    CPacketDecoder* decoder = nullptr;
    std::vector<CPacketDecoder*> vecDecoders;
    bool bProcessed = false;
    VecPackets vecPackets;
    while (m_bThdRun)
    {
        {
            vecDecoders.clear();
            std::unique_lock<std::mutex> lk_(m_recvDecodersMutex);
            vecDecoders = m_vecDecoders;
        }

        bProcessed = false;
        for (auto i : vecDecoders )
        {
            if (i->GetPackets(vecPackets) > 0)
            {
                ProcessPackets(vecPackets);
                bProcessed = bProcessed ? bProcessed : vecPackets.size() > 0;
            }
        }

        if (!bProcessed)
        {
#ifdef WIN32
            WaitForSingleObject(m_hDecoderData, 1);
#else
            cc_sleep(1);
#endif
        }
    }

    LOG_INFO("Device DataProcessThread loop exit\n");
}

void Device::ProcessPackets(VecPackets& vecPackets)
{
    //LOG_INFO("ProcessPackets begin, count:%d\n", vecPackets.size());
    for (auto& packet : vecPackets)
    {
        BYTE pktType = CPacketUtil::GetPacketFrameType(packet);
        if (PACKET_TYPE_CAN == pktType)
        {
            std::vector<PacketDataCAN> vecFrame;
            if (CPacketUtil::GetPacketDataCAN(packet, vecFrame) > 0)
            {
                HandleCANFrame(vecFrame);
            }
            else
            {
                LOG_ERR("PACKET_TYPE_CAN No Valid Data!\n");
            }
        }
        else if (PACKET_TYPE_CANFD == pktType)
        {
            std::vector<PacketDataCANFD> vecFrame;
            if (CPacketUtil::GetPacketDataCANFD(packet, vecFrame) > 0)
            {
                HandleCANFDFrame(vecFrame);
            }
            else
            {
                LOG_ERR("PACKET_TYPE_CANFD No Valid Data!");
            }
        }
        else if (PACKET_TYPE_AUTH_REQ == pktType)
        {
            FrameData vecFrame;
            BYTE pktTypeParam = 0;
            if (CPacketUtil::GetPacketDataAUTH(packet, vecFrame, pktTypeParam) > 0)
            {
                HandleAuthDataRequest(vecFrame, pktTypeParam);
            }
            else
            {
                LOG_ERR("PACKET_TYPE_AUTH_REQ No Valid Data!\n");
            }
        }
        else if (PACKET_TYPE_AUTH_RES == pktType)
        {
            FrameData vecFrame;
            BYTE pktTypeParam = 0;
            if (CPacketUtil::GetPacketDataAUTH(packet, vecFrame, pktTypeParam) > 0)
            {
                HandleAuthDataResponse(vecFrame, pktTypeParam);
            }
            else
            {
                LOG_ERR("PACKET_TYPE_AUTH_RES No Valid Data!\n");
            }
        }
        else if (PACKET_TYPE_CAN_AUTO_SEND == pktType)
        {
            LOG_INFO("Recvied Autosend data, drop packet!\n");
        }
        else if (PACKET_TYPE_BUS_USAGE == pktType)
        {
            PacketDataBusUsage pktBusUsage;
            if (CPacketUtil::GetPacketDataBusUsage(packet, pktBusUsage) > 0)
            {
                HandleBusUsageFrame(pktBusUsage);
            }
            else
            {
                LOG_ERR("PACKET_TYPE_BUS_USAGE No Valid Data!\n");
            }
        }
        else if (PACKET_TYPE_DEV_REQ == pktType)
        {
            //�豸�������󣬿����ģ����Ӧ
            FrameData vecFrame;
            BYTE pktTypeParam = 0;
            if (CPacketUtil::GetPacketDataDevReq(packet, vecFrame, pktTypeParam) > 0)
            {
                HandleDevReqFrame(vecFrame, pktTypeParam);
            }
            else
            {
                LOG_ERR("PACKET_TYPE_DEV_REQ No Valid Data!\n");
            }
        }
        else if (PACKET_TYPE_DEV_RES == pktType)
        {
            //�ⷢ�������豸������Ӧ
            FrameData vecFrame;
            BYTE pktTypeParam = 0;
            if (CPacketUtil::GetPacketDataDevRes(packet, vecFrame, pktTypeParam) > 0)
            {
                HandleDevResFrame(vecFrame, pktTypeParam);
            }
            else
            {
                LOG_ERR("PACKET_TYPE_DEV_RES No Valid Data!\n");
            }
        }
        else if (PACKET_TYPE_REQUEST_RESPONSE == pktType)
        {
            //�ͻ��������豸����
            PacketTypeRequestResponseParam pktTypeParam;
            pktTypeParam.rawValue = CPacketUtil::GetPacketFrameTypeParam(packet);
            if (pktTypeParam.unionValue.bResponse)
            {
                //��Ӧ��,�豸���ص�����
                PacketDataResponse responseData;
                if (CPacketUtil::GetPacketDataResponse(packet, responseData) > 0)
                {
                    HandleResponseData(pktTypeParam.unionValue.nSeq, responseData);
                }
            }
            else
            {
                //��������Լ���ΪServerʱ���ͻ��˹�����������
                PacketDataRequest requestData;
                if (CPacketUtil::GetPacketDataRequest(packet, requestData) > 0)
                {
                    HandleRequestData(pktTypeParam.unionValue.nSeq, requestData);
                }
            }
        }
        else if (PACKET_TYPE_REQUEST_RESPONSE_EX == pktType)
        {
            //�ͻ��������豸����
            PacketTypeRequestResponseParam pktTypeParam;
            pktTypeParam.rawValue = CPacketUtil::GetPacketFrameTypeParam(packet);
            if (pktTypeParam.unionValue.bResponse)
            {
                //��Ӧ��,�豸���ص�����
                PacketDataResponse responseData;
                if (CPacketUtil::GetPacketDataResponseEx(packet, responseData) > 0)
                {
                    HandleResponseDataEx(pktTypeParam.unionValue.nSeq, responseData);
                }
            }
            else
            {
                //��������Լ���ΪServerʱ���ͻ��˹�����������
                PacketDataRequest requestData;
                if (CPacketUtil::GetPacketDataRequestEx(packet, requestData) > 0)
                {
                    HandleRequestDataEx(pktTypeParam.unionValue.nSeq, requestData);
                }
            }
        }
        else if (PACKET_TYPE_DEV_STATE == pktType)
        {
            //�豸�����ϱ���״̬����
            FrameData vecFrame;
            BYTE pktTypeParam = CPacketUtil::GetPacketFrameTypeParam(packet);
            if (CPacketUtil::GetPacketDataDevState(packet, vecFrame, pktTypeParam) > 0)
            {
                HandleDevStateData(vecFrame, pktTypeParam);
            }
        }
        else if (PACKET_TYPE_GPS == pktType)
        {
            //gps����
            std::vector<PacketDataGPS> vecFrame;
            if (CPacketUtil::GetPacketDataGPS(packet, vecFrame) > 0)
            {
                HandleGPSFrame(vecFrame);
            }
            else
            {
                LOG_ERR("PACKET_TYPE_GPS No Valid Data!\n");
            }
        }
        else if (PACKET_TYPE_LIN == pktType)
        {
            //LIN����
            std::vector<PacketDataLIN> vecFrame;
            if (CPacketUtil::GetPacketDataLIN(packet, vecFrame) > 0)
            {
                HandleLINFrame(vecFrame);
            }
            else
            {
                LOG_ERR("PACKET_TYPE_LIN No Valid Data!\n");
            }
        }
        else
        {
            LOG_ERR("Unkown Packet Type, drop packet!\n");
        }
    }
    //LOG_INFO("ProcessPackets done\n");
}

void Device::HandleCANFrame(const std::vector<PacketDataCAN>& vecFrame)
{
    bool bStartfor1Second = IsDevStartLessThan(1000);
    for (auto & pkt : vecFrame)
    {
        if (pkt.canHead.nChnl < m_devChnlCount)
        {
            if (m_bitChnlStarted.test(pkt.canHead.nChnl) || bStartfor1Second)
            {
                //�ж��Ƿ�ZCANPRO���͵��������֡
                HandleEchoRequestFrame(pkt);

                if (m_bRecvMerge)
                {
                    CanDataQueue<ZCANDataObj>& canData = m_frmMerge;
                    ZCANDataObj dataObj;
                    if (CPacketUtil::PacketData2ZCANDataObj(pkt, dataObj))
                    {
                        canData.lock();
                        if (canData.push(dataObj))
                        {
                            m_statistic.nRxMergeDrop++;
                        }
                        canData.unlock();
                        m_statistic.nRxMerge++;
                    }
                }
                else
                {
                    ZCAN_CHANNEL_ERR_INFO errInfo{ 0 };
                    if (CPacketUtil::CANFrame2ErrFrame(pkt, errInfo))
                    {
                        CanDataQueue<ZCAN_CHANNEL_ERR_INFO>& errData = m_errQueue[pkt.canHead.nChnl];
                        errData.lock();
                        errData.push(errInfo);
                        errData.unlock();
                        m_chnlStatus[pkt.canHead.nChnl].regRECounter = errInfo.passive_ErrData[1];
                        m_chnlStatus[pkt.canHead.nChnl].regTECounter = errInfo.passive_ErrData[2];
                        m_statistic.chnl[pkt.canHead.nChnl].nError++;
                    }
                    else
                    {
                        CanDataQueue<PacketDataCAN>& canData = m_frmCAN[pkt.canHead.nChnl];
                        canData.lock();
                        if (canData.push(pkt))
                        {
                            m_statistic.chnl[pkt.canHead.nChnl].nRxCANDrop++;
                        }
                        canData.unlock();
                        m_statistic.chnl[pkt.canHead.nChnl].nRxCAN++;
                    }
                }

                //ͳ����Ϣ
                if (pkt.canHead.frameInfo.unionVal.nTx)
                {
                    m_statistic.chnl[pkt.canHead.nChnl].nTxEcho++;
                }
            }
            else
            {
                //LOG_ERR("CAN Data Ingored, Chnl:%d, started:%d StartLess1S:%d\n", pkt.canHead.nChnl, m_bitChnlStarted.test(pkt.canHead.nChnl), bStartfor1Second);
            }
        }
        else
        {
            //LOG_ERR("CAN Invalid pkt.canHead.nChnl : %d\n", pkt.canHead.nChnl);
        }
    }
}

void Device::HandleCANFDFrame(const std::vector<PacketDataCANFD>& vecFrame)
{
    bool bStartfor1Second = IsDevStartLessThan(1000);
    for (auto & pkt : vecFrame)
    {
        if (pkt.canHead.nChnl < m_devChnlCount)
        {
            if (m_bitChnlStarted.test(pkt.canHead.nChnl) || bStartfor1Second)
            {
                //�ж��Ƿ�ZCANPRO���͵��������֡
                HandleEchoRequestFrame(pkt);

                if (m_bRecvMerge)
                {
                    CanDataQueue<ZCANDataObj>& canData = m_frmMerge;
                    ZCANDataObj dataObj;
                    if (CPacketUtil::PacketData2ZCANDataObj(pkt, dataObj))
                    {
                        canData.lock();
                        if (canData.push(dataObj))
                        {
                            m_statistic.nRxMergeDrop++;
                        }
                        canData.unlock();
                        m_statistic.nRxMerge++;
                    }
                }
                else
                {
                    ZCAN_CHANNEL_ERR_INFO errInfo{ 0 };
                    if (CPacketUtil::CANFrame2ErrFrame(pkt, errInfo))
                    {
                        CanDataQueue<ZCAN_CHANNEL_ERR_INFO>& errData = m_errQueue[pkt.canHead.nChnl];
                        errData.lock();
                        errData.push(errInfo);
                        errData.unlock();
                        m_chnlStatus[pkt.canHead.nChnl].regRECounter = errInfo.passive_ErrData[1];
                        m_chnlStatus[pkt.canHead.nChnl].regTECounter = errInfo.passive_ErrData[2];
                        m_statistic.chnl[pkt.canHead.nChnl].nError++;
                    }
                    else
                    {
                        //CANFD���������CAN����
                        if (pkt.canHead.frameInfo.unionVal.bFD)
                        {
                            CanDataQueue<PacketDataCANFD>& canData = m_frmCANFD[pkt.canHead.nChnl];
                            canData.lock();
                            if (canData.push(pkt))
                            {
                                m_statistic.chnl[pkt.canHead.nChnl].nRxCANFDDrop++;
                            }
                            canData.unlock();
                            m_statistic.chnl[pkt.canHead.nChnl].nRxCANFD++;
                        }
                        else
                        {
                            PacketDataCAN canpkt;
                            CPacketUtil::CANFDFrame2CANFrame(pkt, canpkt);
                            CanDataQueue<PacketDataCAN>& canData = m_frmCAN[pkt.canHead.nChnl];
                            canData.lock();
                            if (canData.push(canpkt))
                            {
                                m_statistic.chnl[pkt.canHead.nChnl].nRxCANDrop++;
                            }
                            canData.unlock();
                            m_statistic.chnl[pkt.canHead.nChnl].nRxCAN++;
                        }
                    }
                }

                //ͳ����Ϣ
                if (pkt.canHead.frameInfo.unionVal.nTx)
                {
                    m_statistic.chnl[pkt.canHead.nChnl].nTxEcho++;
                }
            }
            else
            {
                //LOG_ERR("CANFD Data Ingored, Chnl:%d, started:%d StartLess1S:%d\n", pkt.canHead.nChnl, m_bitChnlStarted.test(pkt.canHead.nChnl), bStartfor1Second);
            }
        }
        else
        {
            //LOG_ERR("CANFD Invalid pkt.canHead.nChnl : %d\n", pkt.canHead.nChnl);
        }
    }
}

void Device::HandleGPSFrame(const std::vector<PacketDataGPS>& vecFrame)
{
    if (m_bRecvMerge)
    {
        CanDataQueue<ZCANDataObj>& canData = m_frmMerge;
        ZCANDataObj dataObj;
        canData.lock();
        for (auto & pkt : vecFrame)
        {
            CPacketUtil::PacketData2ZCANDataObj(pkt, dataObj);
            if (canData.push(dataObj))
            {
                m_statistic.nRxMergeDrop++;
            }
            m_statistic.nRxMerge++;
        }
        canData.unlock();
    }
    else
    {
        CanDataQueue<PacketDataGPS>& gpsData = m_frmGPS;
        gpsData.lock();
        for (auto & pkt : vecFrame)
        {
            gpsData.push(pkt);
            m_statistic.nGPS++;
        }
        gpsData.unlock();
    }
}

void Device::HandleLINFrame(const std::vector<PacketDataLIN>& vecFrame)
{
    if (m_bRecvMerge)
    {
        CanDataQueue<ZCANDataObj>& canData = m_frmMerge;
        ZCANDataObj dataObj;
        canData.lock();
        for (auto & pkt : vecFrame)
		{
			UINT nLINIndex = DEV_CHNL_COUNT_MAX + pkt.nChnl;
			if (!m_bitChnlStarted.test(nLINIndex))//linͨ��û������˵�����
			{
				continue;
			}
            CPacketUtil::PacketData2ZCANDataObj(pkt, dataObj);
            if (canData.push(dataObj))
            {
                m_statistic.nRxMergeDrop++;
            }
            m_statistic.nRxMerge++;
        }
        canData.unlock();
    }
    else
    {
		for (auto & pkt : vecFrame)
		{
			UINT nLINIndex = DEV_CHNL_COUNT_MAX + pkt.nChnl;
			if (!m_bitChnlStarted.test(nLINIndex))//linͨ��û������˵�����
			{
				continue;
			}
			if (pkt.nChnl < DEV_LIN_COUNT_MAX)
			{
				CanDataQueue<PacketDataLIN>& data = m_frmLIN[pkt.nChnl];
				data.lock();
				data.push(pkt);
				data.unlock();
				m_statistic.chnl[pkt.nChnl].nRxLIN++;
			}
		}
    }
}

void Device::HandleAuthDataResponse(const FrameData& vecAuth, BYTE pktTypeParam)
{
    std::unique_lock<std::mutex> lk_(m_authMapMutex);
    auto itr = m_authDataMap.find(pktTypeParam);
    if (itr != m_authDataMap.end())
    {
        AuthDeviceData_PTR authData = itr->second;
        lk_.unlock();
        memcpy(authData->data.OutData, vecAuth.data(), FRAME_LEN_AUTH);
        authData->cv.notify_all();
        LOG_INFO("Auth Data Response Recieved:%d\n", pktTypeParam);
    }
    else
    {
        LOG_ERR("Auth Data Response Dropped:%d\n", pktTypeParam);
    }
}

void Device::HandleAuthDataRequest(const FrameData& vecAuth, BYTE pktTypeParam)
{
    LOG_INFO("Auth Data Request Recieved:%d\n", pktTypeParam);
}

void Device::HandleBusUsageFrame(const PacketDataBusUsage& pktBusUsage)
{
    if (pktBusUsage.nChnl < m_devChnlCount)
    {
        m_busUsage.chnl[pktBusUsage.nChnl] = pktBusUsage;
        m_busUsage.newdata[pktBusUsage.nChnl] = true;
        //LOG_INFO("Chnl:%d BusUsage:%d Count:%d Time%I64u-%I64u\n", pktBusUsage.nChnl, pktBusUsage.nBusUsage, pktBusUsage.nFrameCount, pktBusUsage.nTimeStampBegin, pktBusUsage.nTimeStampEnd);
    }
	if (m_bRecvMerge)
	{
		//����ͨ�������ʻ���
		CanDataQueue<ZCANDataObj>& canData = m_frmMerge;
		ZCANDataObj dataObj;
		if (CPacketUtil::PacketData2ZCANDataObj(pktBusUsage, dataObj))
		{
			canData.lock();
			if (canData.push(dataObj))
			{
				m_statistic.nRxMergeDrop++;
			}
			canData.unlock();
			m_statistic.nRxMerge++;
		}
	}
}

void Device::HandleDevReqFrame(const FrameData& vecDevReq, BYTE pktTypeParam)
{
    PacketTypeREQParam reqParam;
    reqParam.rawValue = pktTypeParam;
    if (reqParam.unionValue.nReqVal == PACKET_TYPE_PARAM_DEV_REQ_RES_TIMESTAMP)
    {
        //�ͻ�ģ�ⷢ������ֱ�ӷ�����Ӧ��ʾ�ɹ���
        PacketTypeRESParam resParam;
        resParam.rawValue = 0;
        resParam.unionValue.nReqVal = reqParam.unionValue.nReqVal;
        resParam.unionValue.nResult = 1; // 1:��ʾִ�гɹ�
        Packet packet = CPacketEncoder::BuildPacketDevReqRes(false, nullptr, 0, pktTypeParam);
        SendData(packet.data(), packet.size());
    }
}

void Device::HandleDevResFrame(const FrameData& vecDevRes, BYTE pktTypeParam)
{
    PacketTypeRESParam param;
    param.rawValue = pktTypeParam;
    if (param.unionValue.nReqVal == PACKET_TYPE_PARAM_DEV_REQ_RES_TIMESTAMP)
    {
        // param.unionValue.nResult ��ʾ�豸��Ӧ�����1���ɹ���0��ʧ��
        // param.unionValue.nResult;
    }
}

void Device::HandleEchoRequestFrame(const PacketDataCAN& canFrame)
{
    LOG_INFO("CAN received, .chnl=%d .tx=%d .echo=%d\n", canFrame.canHead.nChnl, canFrame.canHead.frameInfo.unionVal.nTx, canFrame.canHead.frameInfo.unionVal.nEcho);
    if (m_bEchoResponse && canFrame.canHead.frameInfo.unionVal.nEcho)
    {
        Packet packet = CPacketEncoder::BuildPacketEchoResponse(canFrame);
        if (packet.size() == SendData(packet.data(), packet.size()))
        {
            m_statistic.chnl[canFrame.canHead.nChnl].nTxCANSim++;
        }
    }
}

void Device::HandleEchoRequestFrame(const PacketDataCANFD& canFrame)
{
    LOG_INFO("CANFD received, .chnl=%d .tx=%d .echo=%d\n", canFrame.canHead.nChnl, canFrame.canHead.frameInfo.unionVal.nTx, canFrame.canHead.frameInfo.unionVal.nEcho);
    if (m_bEchoResponse && canFrame.canHead.frameInfo.unionVal.nEcho)
    {
        Packet packet = CPacketEncoder::BuildPacketEchoResponse(canFrame);
        if (packet.size() == SendData(packet.data(), packet.size()))
        {
            if (canFrame.canHead.frameInfo.unionVal.bFD)
            {
                m_statistic.chnl[canFrame.canHead.nChnl].nTxCANFDSim++;
            }
            else
            {
                m_statistic.chnl[canFrame.canHead.nChnl].nTxCANSim++;
            }
        }
    }
}

//PktType 0x50����������Ӧ
void Device::HandleRequestData(BYTE nSeq, PacketDataRequest& requestData)
{
    //����յ������ͻ��˵�����ģ���豸������Ӧ
    Packet packet = CPacketEncoder::BuildPacketEmulateResponse(nSeq, requestData);
    SendData(packet.data(), packet.size());
}

//PktType 0x50����������Ӧ
void Device::HandleResponseData(BYTE nSeq, PacketDataResponse& responseData)
{
    std::unique_lock<std::mutex> lk_(m_pktRequestMutex);
    uint64_t key = GetDevRequestKey(nSeq, responseData.nTID);
    auto itr = m_pktRequestDataMap.find(key);
    if (itr != m_pktRequestDataMap.end())
    {
        DevRequestResponse_PTR pktRequest = itr->second;
        lk_.unlock();
        pktRequest->responseData = responseData;
		pktRequest->responsed = true;
        pktRequest->cv.notify_all();
        LOG_INFO("Dev Request Response Recieved, seq:%d tid:%d\n", nSeq, responseData.nTID);
    }
    else
    {
        LOG_ERR("Dev Request Response Dropped, seq:%d tid:%d \n", nSeq, responseData.nTID);
    }
}

//PktType 0x52����������Ӧ
void Device::HandleRequestDataEx(BYTE nSeq, PacketDataRequest& requestData)
{
    //����յ������ͻ��˵�����ģ���豸������Ӧ
    Packet packet = CPacketEncoder::BuildPacketEmulateResponseEx(nSeq, requestData);
    SendData(packet.data(), packet.size());
}

//PktType 0x52����������Ӧ
void Device::HandleResponseDataEx(BYTE nSeq, PacketDataResponse& responseData)
{
    std::unique_lock<std::mutex> lk_(m_pktRequestMutex);
    uint64_t key = GetDevRequestKey(nSeq, responseData.nTID);
    auto itr = m_pktRequestDataMap.find(key);
    if (itr != m_pktRequestDataMap.end())
    {
        DevRequestResponse_PTR pktRequest = itr->second;
        lk_.unlock();
        pktRequest->responseData = responseData;
        pktRequest->responsed = true;
        pktRequest->cv.notify_all();
        LOG_INFO("Dev Request Response Ex Recieved, seq:%d tid:%u\n", nSeq, responseData.nTID);
    }
    else
    {
        LOG_ERR("Dev Request Response Ex Dropped, seq:%d tid:%u \n", nSeq, responseData.nTID);
    }
}

//PktType 0x04 �豸״̬����
void Device::HandleDevStateData(const FrameData& vecDevState, BYTE pktTypeParam)
{
    switch (pktTypeParam)
    {
    case PACKET_TYPE_DEV_STATE_PARAM_SYS_INFO:
        {
            m_stateSysInfo.strState.assign(vecDevState.begin(), vecDevState.end());
            m_stateSysInfo.bNewData = true;
        }
        break;
    case PACKET_TYPE_DEV_STATE_PARAM_CAN_INFO:
        {
            m_stateCANInfo.strState.assign(vecDevState.begin(), vecDevState.end());
            m_stateCANInfo.bNewData = true;
        }
        break;
    case PACKET_TYPE_DEV_STATE_PARAM_RECORDER_INFO:
        {
            m_stateRecorderInfo.strState.assign(vecDevState.begin(), vecDevState.end());
            m_stateRecorderInfo.bNewData = true;
        }
        break;
    case PACKET_TYPE_DEV_STATE_PARAM_NET_INFO:
        {
            m_stateNetInfo.strState.assign(vecDevState.begin(), vecDevState.end());
            m_stateNetInfo.bNewData = true;
        }
        break;
    default:
        break;
    }

}

BYTE Device::CreatePktRequestSeq()
{
    std::lock_guard<std::mutex> lk(m_pktRequestSeqMutex);
    m_pktRequestSeq = (++m_pktRequestSeq) % 8;
    return m_pktRequestSeq;
}

uint32_t Device::CreatePktTID()
{
    std::lock_guard<std::mutex> lk(m_pktTIDMutex);
    m_pktTID++;
    return m_pktTID;
}

bool Device::UDSRequest(UINT reqID, const std::string& reqParam, const BYTE* reqData, UINT reqDataLen, 
                     std::string& respParam, std::vector<BYTE> &respData)
{
    PacketDataRequest request;
    request.nRequest = PACKET_REQUEST_DO_UDS_REQUEST;
    int reqParamLen = reqParam.length();
    request.vecData.resize(reqParamLen + 1 + reqDataLen, 0);
    memcpy(&request.vecData[0], reqParam.c_str(), reqParamLen);
    request.vecData[reqParamLen] = 0;
    if (reqData && reqDataLen > 0) {
        memcpy(&request.vecData[reqParamLen + 1], reqData, reqDataLen);
    }

    bool bRet = false;

    BYTE nSeq = CreatePktRequestSeq();
    uint32_t nTID = CreatePktTID();

    // �ɵ��������TIDֻ��һ���ֽ�
    if (!IsSupportRequestResponseEx()) {
        nTID &= 0xFF;
    }

    request.nTID = nTID;
    DevRequestResponse_PTR data = std::make_shared<DevRequestResponse>();
    if (data)
    {
        data->nSeq = nSeq;
        data->nTID = nTID;
        data->requestData = request;
    }
    else
    {
        LOG_ERR("memory is full! \n");
        return false;;
    }

    uint64_t key = GetDevRequestKey(nSeq, nTID);
    Packet packet;
    if (IsSupportRequestResponseEx()) {
        packet = CPacketEncoder::BuildPacketRequestEx(nSeq, request);
    }
    else {
        packet = CPacketEncoder::BuildPacketRequest(nSeq, request);
    }

    LOG_INFO("uds request: udsId:%d, seq:%d, tid:%u\n", reqID, nSeq, nTID);

	//wait for response
	{
		std::unique_lock<std::mutex> lk_(m_pktRequestMutex);
		m_pktRequestDataMap[key] = data;
	}
    if (packet.size() != SendData(packet.data(), packet.size())) {
        LOG_ERR("send data error! \n");
    }
    else {
        int maxTryCount = 200; // 300s
        while (maxTryCount--)
        {
            std::mutex mutex_;
            std::unique_lock<std::mutex> ulk(mutex_);
            if (!data->cv.wait_for(ulk, std::chrono::milliseconds(REQUEST_TIMEOUT), [&data]{
                return data->responsed; }))
				{
                // ��ʱ���ȡUDS��ǰ״̬��������������У�������ȴ�
                UDS_STATUS status = UDS_STATUS_IDLE;
                if (!GetUdsStatus(reqID, status)) {
                    LOG_ERR("GetUdsStatus failed\n");
                    break;
                }

                if (status == UDS_STATUS_REQUESTING) {
                    continue;
                }
                else {
                    if (!data->responsed) {
                        LOG_ERR("GetUdsStatus failed with status=%d \n", status);
                    }
                }
            }
            // ���յ���Ӧ����UDS��ǰ����������
            if (data->responsed) {
                if (data->responseData.nResult == 0) {
                    LOG_ERR("response result error\n");
                    break;
                }

                if (data->responseData.vecData.empty()) {
                    LOG_ERR("response data empty\n");
                    break;
                }

                size_t i=0;
                for (i=0; data->responseData.vecData.size(); i++) {
                    if (data->responseData.vecData[i] == 0) {
                        break;
                    }
                }

                if (i >= data->responseData.vecData.size()) {
                    LOG_ERR("response data format error\n");
                    break;
                }

                respParam = (char*)&data->responseData.vecData[0];
                respData.clear();
                if (i < data->responseData.vecData.size() - 1) {
                    respData.assign(data->responseData.vecData.begin() + i + 1, data->responseData.vecData.end());
                }

                bRet = true;
                LOG_INFO("response data, seq:%d tid:%u/%u\n", nSeq, nTID, data->responseData.nTID);
            }
            else {
                LOG_ERR("not received response\n");
            }
            break;
        }
    }

    //Clear result
    {
        std::lock_guard<std::mutex> lk_(m_pktRequestMutex);
        m_pktRequestDataMap.erase(key);
    }

    return bRet;
}

bool Device::UDSControl(const string& req, string &resp)
{
    PacketDataRequest request;
    PacketDataResponse response;
    request.nRequest = PACKET_REQUEST_DO_UDS_CONTROL;
    request.nTID = 0;
    request.vecData.assign(req.begin(), req.end());
    request.vecData.push_back(0);

    if (!RequestDevData(request, response)) {
        return false;
    }
    
    if (response.nResult == 0) {
        LOG_ERR("UDS_Control response result error\n");
        return false;
    }

    if (response.vecData.empty()) {
        resp = nullptr;
    }
    else {
        char* strResult = new char[response.vecData.size()]; 
        memcpy(strResult, (char*)&response.vecData[0], response.vecData.size());
        resp = strResult;
    }

    return true;
}

bool Device::GetUdsStatus(UINT reqId, UDS_STATUS &status)
{
    PacketDataRequest request;
    PacketDataResponse response;
    request.nRequest = PACKET_REQUEST_GET_UDS_STATUS;
    request.nTID = 0;

    cJSON* pObj = cJSON_CreateObject();
    cJSON_AddNumberToObject(pObj, "ReqID", reqId);
    char* pStr = cJSON_PrintUnformatted(pObj);
    std::string strConfig(pStr);
    request.vecData.assign(strConfig.c_str(), strConfig.c_str() + strConfig.length());
    request.vecData.push_back(0);
    cJSON_free(pStr);
    cJSON_Delete(pObj);

    if (!RequestDevData(request, response)) {
        return false;
    }
    
    if (response.nResult == 0) {
        LOG_ERR("GetUdsStatus result error\n");
        return false;
    }

    std::string strResult(response.vecData.begin(), response.vecData.end());
    cJSON* pInfo = cJSON_Parse(strResult.c_str());
    if (!pInfo) {
        LOG_ERR("GetUdsStatus response json parse error\n");
        return false;
    }
    
    status = (UDS_STATUS)(int)UtilGetcJSONNumber(cJSON_DetachItemFromObjectCaseSensitive(pInfo, "Status"));
    cJSON_Delete(pInfo);
    
    return true;
}

struct CAN_DYNAMIC_CONFIG
{
	std::string preKey;
	std::string Key;
};

static CAN_DYNAMIC_CONFIG g_canCfg[] =
{                             
	{ "_ENABLE", "Enable" },  
	{ "_TYPE", "Type" },
	{ "_MODE", "Mode" },
	{ "_CLK", "Clk" },
	{ "_TXATTEMPTS", "TxAttempts" },
	{ "_NOMINALBAUD", "NominalBaud" },
	{ "_DATABAUD", "DataBaud" },
	{ "_USERES", "UseRes" },
	{ "_SNDCFG_INTERVAL", "SndCfg.Interval" },
	{ "_SNDCFG_BUFSIZE", "SndCfg.BufSize"},
	{ "_SNDCFG_BUFMODE", "SndCfg.BufMode" },
	{ "_BUSRATIO_ENABLE", "BusRatio.Enable" },
	{ "_BUSRATIO_PERIOD", "BusRatio.Period" },
};

static CAN_DYNAMIC_CONFIG g_filterCfg[] =
{                             
	{ "_ERR", "Err" },  
	{ "_BEGINID", "BeginID" },
	{ "_ENDID", "EndID" },
	{ "_CHN", "Chn" },
	{ "_FD", "Fd" },
	{ "_EXT", "Ext" },
	{ "_RTR", "Rtr" },
	{ "_LEN", "Len" },
	{ "_BEGINTIME", "BeginTime" },
	{ "_ENDTIME", "EndTime" },
	{ "_FILTERDATA", "FilterData" },
	{ "_MASKDATA", "MaskData" },
};
bool Device::addDynamicCfg(UINT chnIdx, const ZCAN_DYNAMIC_CONFIG_DATA* pObj)
{
	if (chnIdx < m_devChnlCount && pObj)
	{
		std::string key(pObj->key);
		std::size_t found = key.find("DEVNAME");
		if (found != std::string::npos)
		{
			m_dynamicCfgData["DevName"] = pObj->value;
			return true;
		}
		// Filter
		found = key.find("FILTER");
		if (found != std::string::npos)
		{
			std::string str("Filter.");
			found = key.find("ENABLE");
			if (found != std::string::npos)
			{
				str.append("Enable");
				m_dynamicCfgData[str] = pObj->value;
				return true;
			}
			found = key.find("WHITELIST");
			if (found != std::string::npos)
			{
				str.append("Whitelist");
				m_dynamicCfgData[str] = pObj->value;
				return true;
			}
			str.append("Rules.Rule");
			for (int filterCount = 0; filterCount < 16; filterCount++)
			{
				std::string temp("FILTER" + to_string(filterCount));
				std::string s = str + to_string(filterCount);
				found = key.find(temp);
				if (found != std::string::npos)
				{
					int nSize = sizeof(g_filterCfg) / sizeof(g_filterCfg[0]);
					for (int index = 0; index < nSize; index++)
					{
						for (auto item : g_filterCfg)
						{
							string preKey;
							preKey.append(item.preKey);
							found = key.find(preKey);
							if (found != std::string::npos)
							{
								s.append(item.Key);
								m_dynamicCfgData[s] = pObj->value;
								break;
							}
						}
					}
				}
			}
			return true;
		}
		// CAN
		for (int i = 0; i < 8; i++)
		{
			std::string str("CAN.CAN");
			std::string temp("CAN");
			str.append(std::to_string(i));
			temp.append(std::to_string(i));
			std::size_t found = key.find(temp);
			if (found != std::string::npos)
			{
				str.append(".");
				int nSize = sizeof(g_canCfg) / sizeof(g_canCfg[0]);
				for (int index = 0; index < nSize; index++)
				{
					if (index >=0 &&index <=7)
					{
						string preKey;
						preKey.append(std::to_string(i));
						preKey.append(g_canCfg[index].preKey);
						std::size_t f = key.find(preKey);
						if (f != std::string::npos)
						{
							str.append(g_canCfg[index].Key);
							m_dynamicCfgData[str] = pObj->value;
							break;
						}
					}
					else
					{
						std::size_t f = key.find(g_canCfg[index].preKey);
						if (f != std::string::npos)
						{
							str.append(g_canCfg[index].Key);
							m_dynamicCfgData[str] = pObj->value;
							break;
						}
					}
				}
			}
		}	
	}
	return true;
}

struct BaudTiming
{
	UINT    nBaudVal;       //������ֵ
	UINT    nTSEG1;         //������ֵ��Ӧ��λ��ʱֵ
	UINT    nTSEG2;
	UINT    nSJW;
	UINT    nBRP;
};

// USBCANFD800(�Լ���ͬʱ��Ƶ�ʵ��豸)�õĲ����ʲ���
static struct BaudTiming kUSBCANFD800AbitBaudrate[] =
{
	{ 1000000, 31, 8, 8, 1 },  //80%
	{ 800000, 19, 5, 5, 2 },  //80%
	{ 500000, 31, 8, 8, 2 },  //80%
	{ 250000, 31, 8, 8, 4 },  //80%
	{ 125000, 31, 8, 8, 8 },  //80%
	{ 100000, 31, 8, 8, 10 }, //80%
	{ 50000, 31, 8, 8, 20 }, //80%
};

static struct BaudTiming kUSBCANFD800DbitBaudrate[] =
{
	{ 5000000, 5, 2, 2, 1 },   //75%
	{ 4000000, 7, 2, 2, 1 },   //80%
	{ 2000000, 15, 4, 4, 1 },  //80%
	{ 1000000, 15, 4, 4, 2 },  //80%
	{ 800000, 7, 2, 2, 5 },   //80%
	{ 500000, 7, 2, 2, 8 },  //80%
	{ 250000, 7, 2, 2, 16 },  //80%
	{ 125000, 7, 2, 2, 32 }, //80%
	{ 100000, 7, 2, 2, 40 }, //80%
};

// CANFDDTU-300(�Լ���ͬʱ��Ƶ�ʵ��豸)�õĲ����ʲ���
static struct BaudTiming kCANFDDTU300AbitBaudrate[] =
{
	{ 1000000, 63, 16, 16, 1 },  //80%
	{ 800000, 79, 20, 20, 1 },  //80%
	{ 500000, 127, 32, 32, 1 },  //80%
	{ 250000, 69, 10, 10, 4 },  //80%
	{ 125000, 69, 10, 10, 8 },  //80%
	{ 100000, 174, 25, 25, 4 }, //80%
	{ 50000, 174, 25, 25, 8 }, //80%
};

static struct BaudTiming kCANFDDTU300DbitBaudrate[] =
{
	{ 5000000, 11, 4, 4, 1 },   //75%
	{ 4000000, 14, 5, 5, 1 },   //80%
	{ 2000000, 14, 5, 5, 2 },  //80%
	{ 1000000, 14, 5, 5, 4 },  //80%
	{ 800000, 14, 5, 5, 5 },   //80%
	{ 500000, 15, 4, 4, 8 },  //80%
	{ 250000, 15, 4, 4, 16 },  //80%
	{ 125000, 15, 4, 4, 32 }, //80%
	{ 100000, 15, 4, 4, 40 }, //80%
};


bool Device::ApplyDynamicCfg(UINT chnIdx, UINT isPersist)
{
	if (chnIdx < m_devChnlCount)
	{
		cJSON* root = cJSON_CreateObject();
		std::map<string, cJSON*>m_list;
		for (auto &cfg : m_dynamicCfgData)
		{
			cJSON* sessionParamNode = { NULL };
			std::vector<std::string> tokens = cc_split(cfg.first, ".");
			if (tokens.empty()) {
				continue;
			}
			for (size_t i = 0; i < tokens.size(); i++) {
				if (tokens.size() == 1)
				{
					if (tokens[i] == "DevName")
					{
						cJSON_AddStringToObject(root, tokens[i].c_str(), cfg.second.c_str());
						continue;
					}
					cJSON_AddNumberToObject(root, tokens[i].c_str(), std::atoi(cfg.second.c_str()));
					continue;
				}

				std::map<string, cJSON*>::iterator iter;
				iter = m_list.find(tokens[i]);
				if (iter != m_list.end())
				{
					continue;
				}
				else
				{
					if (i == tokens.size() - 1)//��ײ��item
					{
						if (!CheckDynamicCfgIsBaud(tokens[i], cfg.second, m_list[tokens[i - 1]]))
						{
							cJSON_AddNumberToObject(m_list[tokens[i - 1]], tokens[i].c_str(), std::atoi(cfg.second.c_str()));
						}
						continue;
					}else{
						sessionParamNode = cJSON_CreateObject();
						m_list[tokens[i]] = sessionParamNode;
						if (i == 0)
						{
							cJSON_AddItemToObject(root, tokens[i].c_str(), sessionParamNode);
						}
						else
						{
							cJSON_AddItemToObject(m_list[tokens[i - 1]], tokens[i].c_str(), sessionParamNode);
						}
					}
				}
			}
		}
		m_dynamicCfgData.clear();
		char* pStr = cJSON_PrintUnformatted(root);
		std::string reqParam(pStr);
		LOG_ALWAYS("reqParam = %s", reqParam.c_str());
		cJSON_free(pStr);
		cJSON_Delete(root);
		return SendDynamicCfg(chnIdx, reqParam, isPersist);
	}
	return false;
}

bool Device::setDynamicCfg(UINT chnIdx, const ZCAN_DYNAMIC_CONFIG * pObj)
{
	if (chnIdx < m_devChnlCount)
	{
		cJSON* pConfig = cJSON_CreateObject();
		if (pConfig)
		{
			//FILTER
			if (pObj->dynamicConfigDataType == DYNAMIC_CONFIG_FILTER)
			{
				cJSON* pFilterConfig = _GenCANFDFilterCfgJson(pObj->data.filterCfg);
				cJSON_AddItemToObject(pConfig, "Filter", pFilterConfig);
			}
		}

		m_dynamicCfgData.clear();
		char* pStr = cJSON_PrintUnformatted(pConfig);
		std::string reqParam(pStr);
		LOG_ALWAYS("reqParam = %s", reqParam.c_str());
		cJSON_free(pStr);
		cJSON_Delete(pConfig);
		return SendDynamicCfg(chnIdx, reqParam, pObj->isPersist);
	}
	return false;
}

cJSON* Device::_GenCANFDFilterCfgJson(const CANFD_FILTER_CFG& rConfig)
{
	cJSON* config = cJSON_CreateObject();
	cJSON_AddNumberToObject(config, "Enable", rConfig.bEnable);
	cJSON_AddNumberToObject(config, "Whitelist", rConfig.enBlackWhiteList);
	cJSON* pRules = cJSON_CreateObject();
	for (int i = 0; i < CANFD_FILTER_COUNT_MAX; ++i)
	{
		std::string key("Rule" + to_string(i));
		cJSON_AddItemToObject(pRules, key.c_str(), _GenCANFDFilterRuleItemJson(rConfig.vecFilters[i]));
	}
	cJSON_AddItemToObject(config, "Rules", pRules);

	return config;
}

cJSON* Device::_GenCANFDFilterRuleItemJson(const CANFD_FILTER_RULE& rConfig)
{
	cJSON* config = cJSON_CreateObject();
	//err
	if (rConfig.presentFlag.unionValue.bErr)
	{
		cJSON_AddNumberToObject(config, "Err", rConfig.nErr);
	}
	//chnl
	if (rConfig.presentFlag.unionValue.bChnl)
	{
		cJSON_AddNumberToObject(config, "Chn", rConfig.nChnl);
	}
	//��ͬʱ����ERR��CAN/CANFD����ʱ( rConfig.presentFlag.unionValue.bErr == 0)��
	//����ֻ���մ���֡ʱ( rConfig.presentFlag.unionValue.bErr == 1 && rConfig.nErr != 0)
	//ֻ��Ҫͨ���������ɣ��������ɺ���������
	if (!rConfig.presentFlag.unionValue.bErr ||
		(rConfig.presentFlag.unionValue.bErr && rConfig.nErr))
	{
		return config;
	}

	//Fd
	if (rConfig.presentFlag.unionValue.bFD)
	{
		cJSON_AddNumberToObject(config, "Fd", rConfig.nFD);
	}
	//Ext
	if (rConfig.presentFlag.unionValue.bEXT)
	{
		cJSON_AddNumberToObject(config, "Ext", rConfig.nExt);
	}
	//Rtr
	if (rConfig.presentFlag.unionValue.bRTR)
	{
		cJSON_AddNumberToObject(config, "Rtr", rConfig.nRtr);
	}
	//Len
	if (rConfig.presentFlag.unionValue.bLen)
	{
		cJSON_AddNumberToObject(config, "Len", rConfig.nLen);
	}
	//time
	if (rConfig.presentFlag.unionValue.bTime)
	{
		cJSON_AddNumberToObject(config, "BeginTime", rConfig.nBeginTime);
		cJSON_AddNumberToObject(config, "EndTime", rConfig.nEndTime);
	}
	//id
	if (rConfig.presentFlag.unionValue.bID)
	{
		cJSON_AddNumberToObject(config, "BeginID", rConfig.nBeginID);
		cJSON_AddNumberToObject(config, "EndID", rConfig.nEndID);
	}
	//filtermask
	if (rConfig.presentFlag.unionValue.bFilterMask)
	{
		cJSON_AddItemToObject(config, "FilterData", _GenJsonByteArray(&rConfig.nFilterData[0], min(rConfig.nFilterDataLen, CANFD_DATA_LEN_MAX)));
		cJSON_AddItemToObject(config, "MaskData", _GenJsonByteArray(&rConfig.nMaskData[0], min(rConfig.nMaskDataLen, CANFD_DATA_LEN_MAX)));
	}

	return config;
}

cJSON* Device::_GenJsonByteArray(const BYTE* pByte, int nLen)
{
	int * pData = new int[nLen];
	for (int i = 0; i < nLen; ++i)
	{
		pData[i] = pByte[i];
	}
	cJSON* ret = cJSON_CreateIntArray(pData, nLen);
	delete[] pData;
	return ret;
}

bool Device::SendDynamicCfg(UINT chnIdx, std::string data, UINT isPersist)
{
	if (chnIdx < m_devChnlCount)
	{
		PacketDataRequest request;
		PacketDataResponse response;
		request.nRequest = (isPersist == 0) ? PACKET_REQUEST_SET_CONFIG : PACKET_REQUEST_SET_CONFIG_PERSIST;
		request.nTID = 0;
		request.vecData.assign(data.c_str(), data.c_str() + data.length());
		if (RequestDevData(request, response))
		{
			if (response.nResult)
			{
				std::string strResult(response.vecData.begin(), response.vecData.end());
				return true;
			}
			return response.nResult > 0;
		}
		return false;
	}
	return false;
}

bool Device::CheckDynamicCfgIsBaud(std::string cfg, std::string value, cJSON *obj)
{
    if (!m_bDevInfoUpdated) {
        if (GetDeviceInfo(m_stDevInfo) != true) {
            return false;
        }
    }

	if (cfg == "NominalBaud" || cfg == "DataBaud")
	{
        cJSON *sessionParamNode = cJSON_CreateObject();
        LOG_ALWAYS("str_hw_Type = %s", m_stDevInfo.str_hw_Type);
		std::string str_hw_type((char *)m_stDevInfo.str_hw_Type);
        str_hw_type = UpperString(str_hw_type);
		if (str_hw_type == "CANFDDTU-300UR-MINI" || 
			str_hw_type == "CANFDDTU-200UR" || 
			str_hw_type == "CANFDDTU-300EWGR" || 
			str_hw_type == "CANFDDTU-300ER") {
            if (cfg == "NominalBaud")
            {
                int size = sizeof(kCANFDDTU300AbitBaudrate) / sizeof(BaudTiming);
                for (int i = 0; i < size; i++)
                {
                    if (kCANFDDTU300AbitBaudrate[i].nBaudVal == std::atoi(value.c_str()))
                    {
                        cJSON_AddNumberToObject(sessionParamNode, "TSEG1", kCANFDDTU300AbitBaudrate[i].nTSEG1);
                        cJSON_AddNumberToObject(sessionParamNode, "TSEG2", kCANFDDTU300AbitBaudrate[i].nTSEG2);
                        cJSON_AddNumberToObject(sessionParamNode, "SJW", kCANFDDTU300AbitBaudrate[i].nSJW);
                        cJSON_AddNumberToObject(sessionParamNode, "BRP", kCANFDDTU300AbitBaudrate[i].nBRP);
                    }
                }
            }
            if (cfg == "DataBaud")
            {
                int size = sizeof(kCANFDDTU300DbitBaudrate) / sizeof(BaudTiming);
                for (int i = 0; i < size; i++)
                {
                    if (kCANFDDTU300DbitBaudrate[i].nBaudVal == std::atoi(value.c_str()))
                    {
                        cJSON_AddNumberToObject(sessionParamNode, "TSEG1", kCANFDDTU300DbitBaudrate[i].nTSEG1);
                        cJSON_AddNumberToObject(sessionParamNode, "TSEG2", kCANFDDTU300DbitBaudrate[i].nTSEG2);
                        cJSON_AddNumberToObject(sessionParamNode, "SJW", kCANFDDTU300DbitBaudrate[i].nSJW);
                        cJSON_AddNumberToObject(sessionParamNode, "BRP", kCANFDDTU300DbitBaudrate[i].nBRP);
                    }
                }
            }
        } else {
            if (cfg == "NominalBaud")
            {
                int size = sizeof(kUSBCANFD800AbitBaudrate) / sizeof(BaudTiming);
                for (int i = 0; i < size; i++)
                {
                    if (kUSBCANFD800AbitBaudrate[i].nBaudVal == std::atoi(value.c_str()))
                    {
                        cJSON_AddNumberToObject(sessionParamNode, "TSEG1", kUSBCANFD800AbitBaudrate[i].nTSEG1);
                        cJSON_AddNumberToObject(sessionParamNode, "TSEG2", kUSBCANFD800AbitBaudrate[i].nTSEG2);
                        cJSON_AddNumberToObject(sessionParamNode, "SJW", kUSBCANFD800AbitBaudrate[i].nSJW);
                        cJSON_AddNumberToObject(sessionParamNode, "BRP", kUSBCANFD800AbitBaudrate[i].nBRP);
                    }
                }
            }
            if (cfg == "DataBaud")
            {
                int size = sizeof(kUSBCANFD800DbitBaudrate) / sizeof(BaudTiming);
                for (int i = 0; i < size; i++)
                {
                    if (kUSBCANFD800DbitBaudrate[i].nBaudVal == std::atoi(value.c_str()))
                    {
                        cJSON_AddNumberToObject(sessionParamNode, "TSEG1", kUSBCANFD800DbitBaudrate[i].nTSEG1);
                        cJSON_AddNumberToObject(sessionParamNode, "TSEG2", kUSBCANFD800DbitBaudrate[i].nTSEG2);
                        cJSON_AddNumberToObject(sessionParamNode, "SJW", kUSBCANFD800DbitBaudrate[i].nSJW);
                        cJSON_AddNumberToObject(sessionParamNode, "BRP", kUSBCANFD800DbitBaudrate[i].nBRP);
                    }
                }
            }
        }
		cJSON_AddItemToObject(obj, cfg.c_str(), sessionParamNode);
		return true;
	}
	else
	{
		return false;
	}
}

bool Device::ClearCANFilter(uint8_t nCANIndex)
{
    m_vecFilterItems[nCANIndex].clear();
	return ApplyCANFilter();
}

bool Device::ApplyCANFilter()
{
    std::vector<CANFDNETFilterItem> filter_item;
    for (int i = 0; i < DEV_CHNL_COUNT_MAX; i++) {
        if (!m_vecFilterItems[i].empty()) {
            filter_item.insert(filter_item.end(), m_vecFilterItems[i].begin(), m_vecFilterItems[i].end());
        }
    }
    size_t nCount = filter_item.size();
	if (nCount > 0)
	{
		cJSON* root = cJSON_CreateObject();
		cJSON* filter = cJSON_CreateObject();

		cJSON_AddNumberToObject(filter, "Enable", 1);
		cJSON_AddNumberToObject(filter, "Whitelist", 1);//Ĭ���ǰ�����
		cJSON * rules = cJSON_CreateObject();
		cJSON_AddItemToObject(filter, "Rules", rules);
		int nValidFilterCount = min(nCount, CANFDNET_FILTER_COUNT_MAX);
		for (int i = 0; i < nValidFilterCount; i++)
		{
			cJSON * rule = cJSON_CreateObject();
			char str[10] = {0};
			snprintf(str, 10, "Rule%d", i);
			cJSON_AddItemToObject(rules, str, rule);

			cJSON_AddNumberToObject(rule, "Ext", filter_item[i].nFrameType);
			cJSON_AddNumberToObject(rule, "BeginID", filter_item[i].nStartID);
			cJSON_AddNumberToObject(rule, "EndID", filter_item[i].nEndID); 
			cJSON_AddNumberToObject(rule, "Chn", filter_item[i].nChn); 
		}
		cJSON_AddItemToObject(root, "Filter", filter);
		char* pStr = cJSON_PrintUnformatted(root);
		std::string reqParam(pStr);
		//LOG_ALWAYS("reqParam = %s\n", reqParam.c_str());
		cJSON_free(pStr);
		cJSON_Delete(root);
		return SendDynamicCfg(0, reqParam, 0);//Ĭ���Ƕ�̬����
	}
	else
	{
		cJSON* root = cJSON_CreateObject();
		cJSON* filter = cJSON_CreateObject();

		cJSON_AddNumberToObject(filter, "Enable", 0);
		cJSON_AddNumberToObject(filter, "Whitelist", 1);//Ĭ���ǰ�����
		cJSON_AddItemToObject(root, "Filter", filter);
		cJSON * rules = cJSON_CreateObject();
		cJSON_AddItemToObject(filter, "Rules", rules);
		char* pStr = cJSON_PrintUnformatted(root);
		std::string reqParam(pStr);
		//LOG_ALWAYS("reqParam = %s\n", reqParam.c_str());
		cJSON_free(pStr);
		cJSON_Delete(root);
		return SendDynamicCfg(0, reqParam, 0);//Ĭ���Ƕ�̬����
	}
}

bool Device::SyncDevClock()
{
    char bufTime[50];
    int nLen = 0;
#ifdef _WIN32
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
	PacketDataResponse response;
	request.nRequest = PACKET_REQUEST_SET_SYNC_DEV_CLOCK;
	request.nTID = 0;
	request.vecData.assign(&bufTime[0], &bufTime[0] + nLen);
	if (RequestDevData(request, response))
	{
		if (response.nResult)
		{
			std::string strResult(response.vecData.begin(), response.vecData.end());
			return true;
		}
		return response.nResult > 0;
	}
}

ZCAN_RET_STATUS Device::UDS_Request(const ZCAN_UDS_REQUEST* req, ZCAN_UDS_RESPONSE* resp, BYTE* dataBuf, UINT dataBufSize)
{
	if (!req || (req->req_id > 65535)) {
        return STATUS_ERR;
    }

    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "ReqID", req->req_id);
    cJSON_AddNumberToObject(root, "Chn", req->channel);
    cJSON_AddNumberToObject(root, "FrameType", req->frame_type);
    cJSON_AddNumberToObject(root, "SrcAddr", req->src_addr);
    cJSON_AddNumberToObject(root, "DstAddr", req->dst_addr);
    cJSON_AddNumberToObject(root, "SuppressResp", req->suppress_response);
    cJSON_AddNumberToObject(root, "SID", req->sid);
    cJSON_AddNumberToObject(root, "DateLen", req->data_len);

    cJSON* sessionParamNode = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "SessionParam", sessionParamNode);
    cJSON_AddNumberToObject(sessionParamNode, "Timeout", req->session_param.timeout);
    cJSON_AddNumberToObject(sessionParamNode, "EnhancedTimeout", req->session_param.enhanced_timeout);
    cJSON_AddNumberToObject(sessionParamNode, "ChkAnyNegResp", req->session_param.check_any_negative_response);
    cJSON_AddNumberToObject(sessionParamNode, "WaitIfSuppressResp", req->session_param.wait_if_suppress_response);

    cJSON* transParamNode = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "ISO15765Param", transParamNode);
    cJSON_AddNumberToObject(transParamNode, "Version", req->trans_param.version);
    cJSON_AddNumberToObject(transParamNode, "MaxDataLen", req->trans_param.max_data_len);
    cJSON_AddNumberToObject(transParamNode, "LocalStMin", req->trans_param.local_st_min);
    cJSON_AddNumberToObject(transParamNode, "BlockSize", req->trans_param.block_size);
    cJSON_AddNumberToObject(transParamNode, "FillByte", req->trans_param.fill_byte);
    cJSON_AddNumberToObject(transParamNode, "Ext", req->trans_param.ext_frame);
    cJSON_AddNumberToObject(transParamNode, "IsModifyEcuStMin", req->trans_param.is_modify_ecu_st_min);
    cJSON_AddNumberToObject(transParamNode, "RemoteStMin", req->trans_param.remote_st_min);
    cJSON_AddNumberToObject(transParamNode, "FCTimeout", req->trans_param.fc_timeout);
    cJSON_AddNumberToObject(transParamNode, "FillMode", req->trans_param.fill_mode);

    char* pStr = cJSON_PrintUnformatted(root);
    std::string reqParam(pStr);
    cJSON_free(pStr);
    cJSON_Delete(root);

    std::string respParam;
    std::vector<BYTE> respData;

    LOG_INFO("uds request: udsId:%u, len:%u, data:%02x %02x ...", req->req_id, req->data_len + 1, req->sid, req->data_len > 0 ? req->data[0] : 0);

    if (!UDSRequest(req->req_id, reqParam, req->data, req->data_len, respParam, respData)) {
        LOG_ERR("uds request failed, udsId: %u, %s", req->req_id, reqParam.c_str());
        return STATUS_ERR;
    }

    // ���������Ӧ
    if (!resp) {
        return STATUS_OK;
    }

    cJSON* respRoot = cJSON_Parse(respParam.c_str());
    if (!respRoot) {
        LOG_ERR("response json parse error\n");
        return STATUS_ERR;
    }
    
    ZCAN_RET_STATUS ret = STATUS_OK;
    memset(resp, 0, sizeof(resp));
    resp->status = (UINT)UtilGetcJSONNumber(cJSON_GetObjectItemCaseSensitive(respRoot, "Status"));
    resp->type = (UINT)UtilGetcJSONNumber(cJSON_GetObjectItemCaseSensitive(respRoot, "RespType"));

    if (resp->status == ZCAN_UDS_ERROR_OK) {
        if (resp->type == ZCAN_UDS_RT_POSITIVE) {
            resp->positive.sid = (UINT)UtilGetcJSONNumber(cJSON_GetObjectItemCaseSensitive(respRoot, "SID"));
            resp->positive.data_len = (UINT)UtilGetcJSONNumber(cJSON_GetObjectItemCaseSensitive(respRoot, "DateLen"));
            if (respData.size() > 0) {
                if (dataBuf) {
                    if (dataBufSize < respData.size()) {
                        ret = STATUS_BUFFER_TOO_SMALL;
                    }
                    else {
                        memcpy(dataBuf, &respData[0], respData.size());
                    }
                }
            }
        }
        else {
            resp->negative.neg_code = (UINT)UtilGetcJSONNumber(cJSON_GetObjectItemCaseSensitive(respRoot, "NegCode"));
            resp->negative.sid = (UINT)UtilGetcJSONNumber(cJSON_GetObjectItemCaseSensitive(respRoot, "SID"));
            resp->negative.error_code = (UINT)UtilGetcJSONNumber(cJSON_GetObjectItemCaseSensitive(respRoot, "NegErr"));
        }
    }

    cJSON_Delete(respRoot);

    return ret;
}

ZCAN_RET_STATUS Device::UDS_Control(const ZCAN_UDS_CTRL_REQ *ctrl, ZCAN_UDS_CTRL_RESP* resp)
{
    
    if (!ctrl) {
        return STATUS_ERR;
    }

    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "ReqID", ctrl->reqID);
    cJSON_AddNumberToObject(root, "CtrlType", ctrl->cmd);

    char* pStr = cJSON_PrintUnformatted(root);
    std::string reqParam(pStr);
    cJSON_free(pStr);
    cJSON_Delete(root);

    LOG_INFO("uds control: %s", reqParam.c_str());

    std::string respParam;
    if (!UDSControl(reqParam, respParam)) {
        return STATUS_ERR;
    }

    // ���������Ӧ
    if (!resp) {
        return STATUS_OK;
    }

    cJSON* respRoot = cJSON_Parse(respParam.c_str());
    if (!respRoot) {
        LOG_ERR("response json parse error\n");
        return STATUS_ERR;
    }
    
    memset(resp, 0, sizeof(resp));
    resp->result = (UINT)UtilGetcJSONNumber(cJSON_GetObjectItemCaseSensitive(respRoot, "Status"));
    cJSON_Delete(respRoot);

    return STATUS_OK;
}

ZCAN_RET_STATUS Device::UDS_RequestEX(const ZCANUdsRequestDataObj* requestData, ZCAN_UDS_RESPONSE* resp, BYTE* dataBuf, UINT dataBufSize)
{
	ZCAN_RET_STATUS ret = STATUS_ERR;
	do
	{
		if (requestData == NULL || requestData->dataType > 2)
		{
			break;
		}
		switch (requestData->dataType)
		{
		case DEF_CAN_UDS_DATA:
			ret = UDS_Request(requestData->data.zcanCANFDUdsData.req, resp, dataBuf, dataBufSize);
			break;
		case DEF_LIN_UDS_DATA:
			//ret = UDS_RequestLIN(requestData->data.zcanLINUdsData.req, resp, dataBuf, dataBufSize);
			break;
		default:
			break;
		}
	} while (0);
	return ret;
}

ZCAN_RET_STATUS Device::UDS_ControlEX(ZCAN_UDS_DATA_DEF dataType, const ZCAN_UDS_CTRL_REQ *ctrl, ZCAN_UDS_CTRL_RESP* resp)
{
	ZCAN_RET_STATUS ret = STATUS_ERR;
	do
	{
		if (ctrl == NULL || dataType > 2)
		{
			break;
		}
		switch (dataType)
		{
		case DEF_CAN_UDS_DATA:
			ret = UDS_Control(ctrl, resp);
			break;
		case DEF_LIN_UDS_DATA:
			//ret = UDS_RequestLIN(requestData->data.zcanLINUdsData.req, resp, dataBuf, dataBufSize);
			break;
		default:
			break;
		}
	} while (0);
	return ret;
}

bool Device::IsSupportRequestResponseEx()
{
    // ��ȡͨѶЭ��汾��
    if (!m_bDevInfoUpdated) {
        GetDeviceInfo(m_stDevInfo);
    }

    // ��V1.19�汾��֧����չ������Ӧ���
    if (m_bDevInfoUpdated) {
        uint32_t ver = (m_protocolVersion[0] << 8) | m_protocolVersion[1];
        if (ver >= ((1 << 8) | 19)) {
            return true;
        }
    }

    return false;
}