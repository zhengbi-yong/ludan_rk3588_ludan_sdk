#include <stdio.h>
#include <string.h>
#include "CANFDNET.h"
#include "common.h"
#include "zlgcan.h"
#include <thread>

#define LOG_INFO printf("[INF] ");printf
#define LOG_ERR printf("[ERR] ");printf

int test(bool bTcp, bool bServer)
{
    LOG_INFO("Start test ... \n");

    UINT devtype = bTcp ? ZCAN_CANFDNET_200U_TCP : ZCAN_CANFDNET_200U_UDP;

    DEVICE_HANDLE hDev = ZCAN_OpenDevice(devtype, 0, 0);
    if (hDev == INVALID_DEVICE_HANDLE) {
        LOG_ERR("Open device failed! \n");
        return 0;
    }

    CHANNEL_HANDLE hChannel = ZCAN_InitCAN(hDev, 0, nullptr);
    if (hChannel == INVALID_CHANNEL_HANDLE) {
        LOG_ERR("Init CAN failed! \n");
        return 0;
    }
    
    // merge channel recv 
    UINT val = 1;
    ZCAN_SetReference(devtype, 0, 0, SETREF_SET_DATA_RECV_MERGE, &val);

    if (bTcp) {
        if (!bServer) {
            LOG_INFO("as tcp client \n");
            val = 0;
            ZCAN_SetReference(devtype, 0, 0, CMD_TCP_TYPE, &val);
            ZCAN_SetReference(devtype, 0, 0, CMD_DESIP, (void*)"127.0.0.1");
            val = 4001;
            ZCAN_SetReference(devtype, 0, 0, CMD_DESPORT, &val);
        }
        else {
            LOG_INFO("as tcp server \n");
            val = 1;
            ZCAN_SetReference(devtype, 0, 0, CMD_TCP_TYPE, &val);
            val = 4001;
            ZCAN_SetReference(devtype, 0, 0, CMD_SRCPORT, &val);
        }
    }
    else {
        LOG_INFO("as udp \n");
        val = 0;
        ZCAN_SetReference(devtype, 0, 0, CMD_TCP_TYPE, &val);
        ZCAN_SetReference(devtype, 0, 0, CMD_DESIP, (void*)"127.0.0.1");
        val = 4001;
        ZCAN_SetReference(devtype, 0, 0, CMD_DESPORT, &val);
        val = 4002;
        ZCAN_SetReference(devtype, 0, 0, CMD_SRCPORT, &val);
    }

    if (ZCAN_StartCAN(hChannel) != STATUS_OK) {
        LOG_ERR("Start CAN error! \n");
        return 0;
    }

    ZCANDataObj dataBuf[100];
    memset(dataBuf, 0, sizeof(dataBuf));

    while (1) {
        UINT recv = ZCAN_ReceiveData(hChannel, dataBuf, sizeof(dataBuf)/sizeof(dataBuf[0]));
        if (recv == 0) {
            cc_sleep(1);
        }
        else {
            for (UINT i=0; i<recv; i++) {
                ZCANDataObj &data = dataBuf[i];
                if (data.dataType != ZCAN_DT_ZCAN_CAN_CANFD_DATA) {
                    LOG_INFO("recv: chn=%d, type=%d \n", data.chnl, data.dataType);
                }
                else {
                    ZCANCANFDData &can = data.data.zcanCANFDData;
                    LOG_INFO("recv: chn=%d, time=%llu, %s, id=0x%X, len=%d, data=%s \n", data.chnl, can.timeStamp, 
                        can.flag.unionVal.frameType ? "CANFD" : "CAN", can.frame.can_id, can.frame.len, 
                        GetHexString(can.frame.data, can.frame.len).c_str());
                }
            }

            // send back
            UINT sent = ZCAN_TransmitData(hChannel, dataBuf, recv);
            if (sent != recv) {
                LOG_ERR("Transmit failed, %d / %d \n", sent, recv);
            }
        }

    }

    ZCAN_CloseDevice(hDev);
    return 0;
}

static int gUdsReqId = 0;

void uds_request(DEVICE_HANDLE hDev) 
{
    ZCAN_UDS_REQUEST req;
    memset(&req, 0, sizeof(req));
    req.req_id = ++gUdsReqId;                            
    req.channel = 0;   
    req.frame_type = ZCAN_UDS_FRAME_CAN;                   
    req.src_addr = 0x700;                         
    req.dst_addr = 0x748;                         
    req.suppress_response = 0;                
    req.sid = 0x19;                              
    req.session_param.timeout = 3000;                      
    req.session_param.enhanced_timeout = 5000;             
    req.session_param.check_any_negative_response = 0;
    req.session_param.wait_if_suppress_response = 0;  
    req.trans_param.version = ZCAN_UDS_TRANS_VER_0;        
    req.trans_param.max_data_len = 64;                 
    req.trans_param.local_st_min = 0;                 
    req.trans_param.block_size = 0;                   
    req.trans_param.fill_byte = 0x00;                    
    req.trans_param.ext_frame = 0;                   
    req.trans_param.is_modify_ecu_st_min = 0;         
    req.trans_param.remote_st_min = 0;                
    req.trans_param.fc_timeout = 1000;      
    BYTE reqData[] = {0x0A};             
    req.data = reqData;                            
    req.data_len = sizeof(reqData);   

    ZCAN_UDS_RESPONSE resp;
    memset(&resp, 0, sizeof(resp));
    BYTE respDataBuf[1024];
    memset(respDataBuf, 0, sizeof(respDataBuf));
    ZCAN_RET_STATUS ret = ZCAN_UDS_Request(hDev, &req, &resp, respDataBuf, sizeof(respDataBuf));
    if (ret != STATUS_OK) {
        LOG_ERR("ZCAN_UDS_Request error! ret=%d \n", ret);
    }
    else {
        if (ZCAN_UDS_ERROR_OK == resp.status) {
            if (ZCAN_UDS_RT_POSITIVE == resp.type) {
                LOG_INFO("positive response sid=%02x len=%d data=%s \n", resp.positive.sid, resp.positive.data_len, GetHexString(respDataBuf, resp.positive.data_len).c_str());
            }
            else if (ZCAN_UDS_RT_NEGATIVE == resp.type) {
                LOG_INFO("negative response %02x %02x %02x \n", resp.negative.neg_code, resp.negative.sid, resp.negative.error_code);
            }
        }
        else {
            LOG_INFO("ZCAN_UDS_Request response error, status=%d \n", resp.status);
        }
    }
}

int test_uds() {
    LOG_INFO("Start test uds ... \n");

    UINT devtype = ZCAN_CANFDNET_400U_TCP;

    DEVICE_HANDLE hDev = ZCAN_OpenDevice(devtype, 0, 0);
    if (hDev == INVALID_DEVICE_HANDLE) {
        LOG_ERR("Open device failed! \n");
        return 0;
    }

    CHANNEL_HANDLE hChannel = ZCAN_InitCAN(hDev, 0, nullptr);
    if (hChannel == INVALID_CHANNEL_HANDLE) {
        LOG_ERR("Init CAN failed! \n");
        return 0;
    }
    
    // merge channel recv 
    UINT val = 1;
    ZCAN_SetReference(devtype, 0, 0, SETREF_SET_DATA_RECV_MERGE, &val);

    const char* ip = "192.168.0.178";
    UINT port = 8000;
    LOG_INFO("connect to %s:%d ...\n", ip, port);
    val = 0;
    ZCAN_SetReference(devtype, 0, 0, CMD_TCP_TYPE, &val);
    ZCAN_SetReference(devtype, 0, 0, CMD_DESIP, (void*)ip);
    val = port;
    ZCAN_SetReference(devtype, 0, 0, CMD_DESPORT, &val);
        
    if (ZCAN_StartCAN(hChannel) != STATUS_OK) {
        LOG_ERR("Start CAN error! \n");
        return 0;
    }

    LOG_INFO("connected.\n");

    bool exit = false;
    std::thread thd = std::thread([hChannel, &exit](){
        ZCANDataObj dataBuf[100];
        memset(dataBuf, 0, sizeof(dataBuf));

        while (!exit) {
            UINT recv = ZCAN_ReceiveData(hChannel, dataBuf, sizeof(dataBuf)/sizeof(dataBuf[0]), 0);
            if (recv == 0) {
                cc_sleep(1);
            }
            else {
                for (UINT i=0; i<recv; i++) {
                    ZCANDataObj &data = dataBuf[i];
                    if (data.dataType != ZCAN_DT_ZCAN_CAN_CANFD_DATA) {
                        // LOG_INFO("recv: chn=%d, type=%d \n", data.chnl, data.dataType);
                    }
                    else {
                        ZCANCANFDData &can = data.data.zcanCANFDData;
                        LOG_INFO("test recv: chn=%d, time=%llu, %s, id=0x%X, len=%d, data=%s \n", data.chnl, can.timeStamp, 
                            can.flag.unionVal.frameType ? "CANFD" : "CAN", can.frame.can_id, can.frame.len, 
                            GetHexString(can.frame.data, can.frame.len).c_str());
                    }
                }
            }
        }
        LOG_INFO("recv thread exit");
    });

#define RUN_STOP_UDS 0
#if RUN_STOP_UDS
    std::thread stopUdsThd = std::thread([hDev](){
        cc_sleep(1000);

        ZCAN_UDS_CTRL_REQ req;
        memset(&req, 0, sizeof(req));
        req.reqID = gUdsReqId;
        req.cmd = ZCAN_UDS_CTRL_STOP_REQ;
        ZCAN_UDS_CTRL_RESP resp;
        memset(&resp, 0, sizeof(resp));
        ZCAN_RET_STATUS ret = ZCAN_UDS_Control(hDev, &req, &resp);
        if (ret != STATUS_OK) {
            LOG_ERR("uds stop failed");
        }
    });
#endif

    int testCount = 1;
    while (testCount--) {
        uds_request(hDev);
    }

    exit = true;
    thd.join();

#if RUN_STOP_UDS
    stopUdsThd.join();
#endif

    ZCAN_CloseDevice(hDev); 
    return 0;
}

int main()
{
    // test(true, false);

    test_uds();

    LOG_INFO("Exit. \n");
    return 0;
}


