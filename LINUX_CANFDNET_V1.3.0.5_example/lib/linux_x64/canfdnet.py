from ctypes import *
import threading
import time
lib = cdll.LoadLibrary("./libCANFDNET.so")
import platform

ZCAN_DEVICE_TYPE  = c_uint32
ZCAN_DEVICE_INDEX = c_uint32
ZCAN_Reserved     = c_uint32
ZCAN_CHANNEL      = c_uint32
LEN               = c_uint32
ZCAN_CANFDNET_400U_TCP =ZCAN_DEVICE_TYPE(52)
ZCAN_CANFDNET_200U_TCP =ZCAN_DEVICE_TYPE(48)
ZCAN_CANFDDTU_CASCADE_TCP = ZCAN_DEVICE_TYPE(74)
USBCAN2       =   ZCAN_DEVICE_TYPE(4)
DEVICE_INDEX  =   ZCAN_DEVICE_INDEX(0)
Reserved      =   ZCAN_Reserved(0)
CHANNEL       =   ZCAN_CHANNEL(0)

def input_thread():
   input()

class ZCAN_CAN_BOARD_INFO(Structure):
    _fields_ = [("hw_Version",c_ushort),
                ("fw_Version",c_ushort),
                ("dr_Version",c_ushort),
                ("in_Version",c_ushort),
                ("irq_Num",c_ushort),
                ("can_Num",c_ubyte),
                ("str_Serial_Num",c_ubyte*20),
                ("str_hw_Type",c_ubyte*40),
                ("Reserved",c_ubyte*4)]

    def __str__(self):
        return "Hardware Version:%s\nFirmware Version:%s\nDriver Version:%s\nInterface:%s\nInterrupt Number:%s\nCAN_number:%d"%(\
                self.hw_Version,  self.fw_Version,  self.dr_Version,  self.in_Version,  self.irq_Num,  self.can_Num)

    def serial(self):
        serial=''
        for c in self.str_Serial_Num:
            if c>0:
                serial +=chr(c)
            else:
                break
        return serial   
        
    def hw_Type(self):
        hw_Type=''
        for c in self.str_hw_Type:
            if c>0:
                hw_Type +=chr(c)
            else:
                break
        return hw_Type   


#对于CANFDNET，此结构体不生效
class ZCAN_CAN_INIT_CONFIG(Structure):
    _fields_ = [("AccCode",c_int),
                ("AccMask",c_int),
                ("Reserved",c_int),
                ("Filter",c_ubyte),
                ("Timing0",c_ubyte),
                ("Timing1",c_ubyte),
                ("Mode",c_ubyte)]

class ZCAN_CAN_OBJ(Structure):
    _fields_ = [("ID",c_uint32),
                ("TimeStamp",c_uint32),
                ("TimeFlag",c_uint8),
                ("SendType",c_byte),
                ("RemoteFlag",c_byte),
                ("ExternFlag",c_byte),
                ("DataLen",c_byte),
                ("Data",c_ubyte*8),
                ("Reserved",c_ubyte*3)]



class ZCAN_CAN_FRAME(Structure):
    _fields_ = [("can_id",  c_uint, 29),
                ("err",     c_uint, 1),
                ("rtr",     c_uint, 1),
                ("eff",     c_uint, 1), 
                ("can_dlc", c_ubyte),
                ("__pad",   c_ubyte),
                ("__res0",  c_ubyte),
                ("__res1",  c_ubyte),
                ("data",    c_ubyte * 8)]

class ZCAN_CANFD_FRAME(Structure):
    _fields_ = [("can_id", c_uint, 29), 
                ("err",    c_uint, 1),
                ("rtr",    c_uint, 1),
                ("eff",    c_uint, 1), 
                ("len",    c_ubyte),
                ("brs",    c_ubyte, 1),
                ("esi",    c_ubyte, 1),
                ("__pad",  c_ubyte, 6),
                ("__res0", c_ubyte),
                ("__res1", c_ubyte),
                ("data",   c_ubyte * 64)]

class ZCAN_Transmit_Data(Structure):
    _fields_ = [("frame", ZCAN_CAN_FRAME), ("transmit_type", c_uint)]


class ZCAN_TransmitFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("transmit_type", c_uint)]

class ZCAN_Receive_Data(Structure):
    _fields_  = [("frame", ZCAN_CAN_FRAME), ("timestamp", c_ulonglong)]

class ZCAN_ReceiveFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("timestamp", c_ulonglong)]


class ZCANdataFlag(Structure):
    _pack_  =  1
    _fields_= [("frameType",c_uint,2),
               ("txDelay",c_uint,2),
               ("transmitType",c_uint,4),
               ("txEchoRequest",c_uint,1),
               ("txEchoed",c_uint,1),
               ("reserved",c_uint,22),
               ]


#表示 CAN/CANFD 帧结构,目前仅作为 ZCANDataObj 结构的成员使用
class ZCANFDData(Structure):            
    _pack_  =  1
    _fields_= [("timestamp",c_uint64),
               ("flag",ZCANdataFlag),
               ("extraData",c_ubyte*4),
               ("frame",ZCAN_CANFD_FRAME),]



#合并发送/合并接收的帧结构体   #旧写法已舍弃
# class ZCANDataObj(Structure):
#     _pack_  =  1   
#     _fields_= [("dataType",c_ubyte),
#                ("chnl",c_ubyte),
#                ("flag",c_ushort),
#                ("extraData",c_ubyte*4),
#                ("zcanfddata",ZCANFDData),##88个字节
#                ("reserved",c_ubyte*4),
#                ]

class ZCANErrorData(Structure):
    _fields_ = [("timestamp",  c_uint64),
                ("errType", c_uint8),
                ("errSubType", c_uint8),
                ("nodeState", c_uint8),
                ("rxErrCount", c_uint8),
                ("txErrCount", c_uint8),
                ("errData", c_uint8),
                ("reserved", c_uint8 * 2)]

class _ZCANData(Union):
    _pack_ = 1
    _fields_ = [("zcanfddata",  ZCANFDData), 
                 ("zcanErrData", ZCANErrorData),
                 ("raw",c_uint8*92)
                 ]

class ZCANDataObj(Structure):
    _pack_ = 1
    _fields_ = [("dataType",  c_uint8),
                ("chnl",      c_uint8),
                ("flag",      c_ushort),
                ("exterData", c_uint8 * 4),
                ("data",      _ZCANData)]

class chn_handle(Structure):
    _pack_  =  1   
    _fields_= [("hchnl",c_uint64*100),
               ]

class Value_(Structure):
    _fields_=[("val",c_uint32)
              ]

#############动态配置滤波###############

class unionCANFDFilterRulePresent(Structure):
    _fields_=[("bChnl",c_uint,1),       #通道条件 是否存在
             ("bFD",c_uint,1),          #CANFD标识 是否存在
             ("bEXT",c_uint,1),         #标准帧/扩展帧标识 是否存在
             ("bRTR",c_uint,1),         #数据帧/远程帧标识 是否存在
             ("bLen",c_uint,1),         #长度  是否存在
             ("bID",c_uint,1),          #起始ID/结束ID 是否存在
             ("bTime",c_uint,1),        #起始时间/结束时间 是否存在
             ("bFilterMask",c_uint,1),  #报文数据过滤/屏蔽 是否存在
             ("bErr",c_uint,1),         #错误报文 CAN/CANFD标志 是否存在
             ("nReserved",c_uint,23),
             ]


class CANFD_FILTER_RULE(Structure):
    _fields_=[("presentFlag",unionCANFDFilterRulePresent),  #滤波条件配置
             ("nErr",c_int32),                              #是否错误报文，此条件一定存在，表示此条过滤是正常帧还是错误帧,0:不过滤错误帧 1:过滤错误帧
             ("nChnl",c_int32),                             #通道
             ("nFD",c_int32),                               #CANFD标识，0：CAN; 1:CANFD
             ("nExt",c_int32),                              #扩展帧标识, 0:标准帧 1:扩展帧
             ("nRtr",c_int32),                              #远程帧标识, 0:数据帧 1:远程帧
             ("nLen",c_int32),                              #报文长度，0-64
             ("nBeginID",c_int32),                          #起始ID
             ("nEndID",c_int32),                            #结束ID，起始ID值必须<=结束ID，与起始ID成对存在
             ("nBeginTime",c_int32),                        #过滤起始时间，单位s，取值0-(24*60*60-1)(忽略)
             ("nEndTime",c_int32),                          #过滤结束时间，单位s，取值0-(24*60*60-1)，与起始时间成对存在(忽略)
             ("nFilterDataLen",c_int32),                    #数据段长度过滤
             ("nMaskDataLen",c_int32),
             ("nFilterData",c_byte*64),
             ("nMaskData",c_byte*64),
             ]


class CANFD_FILTER_CFG(Structure):
    _fields_=[("bEnable",c_int32),                 #0-禁用滤波表，1-使能滤波表
              ("enBlackWhiteList",c_uint32),       #黑白名单，0-黑名单，1-白名单
              ("vecFilters",CANFD_FILTER_RULE*16)]



class FILTER_CFG(Union):
    _fields_=[("filtercfg",CANFD_FILTER_CFG),
              ("reserved",c_byte*10240)]




class ZCAN_DYNAMIC_CONFIG(Structure):
    _fields_=[("dynamicConfigDataType",c_uint32),   #1-滤波，只有1有效
              ("isPersist",c_uint32),               #  // 是否是持久配置（即设备掉电保存配置）、TRUE-持久配置 FALSE-动态配置
              ("data",FILTER_CFG)]





def GetDeviceInf(Device_handle_):
    try:
        info = ZCAN_CAN_BOARD_INFO()
        ret  = lib.ZCAN_GetDeviceInf(Device_handle_, byref(info))
        return info if ret==1 else None
    except:
        print("Exception on GetDeviceInf")
        raise

#canfdnet作为服务器，电脑作为客户端
def tcp_start(DEVCIE_TYPE,Device_handle_,chn_num):
    Hchnl=chn_handle()
    init_config  = ZCAN_CAN_INIT_CONFIG() 
    
    memset(byref(init_config),0,sizeof(init_config))
    

    for i in range(chn_num):
        Hchnl.hchnl[i]=lib.ZCAN_InitCAN(Device_handle_,i,0)
        if Hchnl.hchnl[i] ==0:
            print("InitCAN fail!")
        else:
            print("InitCAN success!")

    
    
            
    
    #设置合并接收
    SETREF_SET_DATA_RECV_MERGE = 43
    value_v = Value_()
    Value="1"
    ret=lib.ZCAN_SetReference(DEVCIE_TYPE,0,0,SETREF_SET_DATA_RECV_MERGE,c_char_p(Value.encode("utf-8")))
    print("ret is %d"%ret)
    
    #设置电脑的TCP模式为客户端模式
    CMD_TCP_TYPE =4
    Value="0" 
    lib.ZCAN_SetReference(DEVCIE_TYPE,0,0,CMD_TCP_TYPE,c_char_p(Value.encode("utf-8")))
    
    #设置目标IP
    CMD_DESIP =0
    Value ="192.168.0.178"
    lib.ZCAN_SetReference(DEVCIE_TYPE,0,0,CMD_DESIP,c_char_p(Value.encode("utf-8")))
    
    #设置目标端口
    CMD_DESPORT =1
    value_v =c_uint32(8000)
    lib.ZCAN_SetReference(DEVCIE_TYPE,0,0,CMD_DESPORT,byref(value_v))
    

    for i in range(chn_num):
        ret=lib.ZCAN_StartCAN(Hchnl.hchnl[i])
        if ret ==0:
            print("StartCAN fail!")
        else:
            print("StartCAN success!")

    
    return Hchnl




if __name__=="__main__":

    DEVCIE_TYPE =ZCAN_CANFDDTU_CASCADE_TCP
    Device_handle_=lib.ZCAN_OpenDevice(DEVCIE_TYPE,0,0)
    if Device_handle_ ==0:
        print("Opendevice fail!")
    else:
        print("Opendevice success!")  
    chn_num = 2 
    Hchnl = tcp_start(DEVCIE_TYPE,Device_handle_,chn_num)

    thread=threading.Thread(target=input_thread)
    thread.start()

    # DataObj = (ZCANDataObj * 64)()
    # memset(DataObj,0,sizeof(DataObj))
    # for i in range(64):
        # DataObj[i].dataType =1   #can/canfd frame
        # DataObj[i].chnl=0        #can_channel
        # DataObj[i].data.zcanfddata.flag.frameType=0  #0-can,1-canfd
        # DataObj[i].data.zcanfddata.flag.txDelay=0    #不添加延迟
        # DataObj[i].data.zcanfddata.flag.transmitType=0  #发送方式，0-正常发送，2-自发自收
        # DataObj[i].data.zcanfddata.flag.txEchoRequest=1  #发送回显请求，0-不回显，1-回显
        # DataObj[i].data.zcanfddata.frame.can_id= i
        # DataObj[i].data.zcanfddata.frame.len =i
        # DataObj[i].data.zcanfddata.frame.brs =0   #不加速
        # for j in range(DataObj[i].data.zcanfddata.frame.len):
            # DataObj[i].data.zcanfddata.frame.data[j]=j
    # ret =lib.ZCAN_TransmitData(Device_handle_,DataObj,64)
    # print("Tranmit Num: %d." % ret)
    
    transmit_num = 8
    msgs = (ZCAN_Transmit_Data * transmit_num)()
    for i in range(transmit_num):
        msgs[i].transmit_type = 0 #0-正常发送，2-自发自收
        msgs[i].frame.eff     = 0 #0-标准帧，1-扩展帧
        msgs[i].frame.rtr     = 0 #0-数据帧，1-远程帧
        msgs[i].frame.can_id  = i
        msgs[i].frame.can_dlc = i
        #msgs[i].frame.__pad =0x20  #回显标志位
        for j in range(msgs[i].frame.can_dlc):
            msgs[i].frame.data[j] = j
    ret = lib.ZCAN_Transmit(Hchnl.hchnl[0], msgs, transmit_num)
    print("Tranmit Num: %d." % ret)
    
    #Send CANFD Messages
    transmit_canfd_num = 64
    canfd_msgs = (ZCAN_TransmitFD_Data * transmit_canfd_num)()
    for i in range(transmit_canfd_num):
        canfd_msgs[i].transmit_type = 0 #0-正常发送，2-自发自收
        canfd_msgs[i].frame.eff     = 0 #0-标准帧，1-扩展帧
        canfd_msgs[i].frame.rtr     = 0 #0-数据帧，1-远程帧
        canfd_msgs[i].frame.brs     = 1 #BRS 加速标志位：0不加速，1加速
        canfd_msgs[i].frame.can_id  = i
        canfd_msgs[i].frame.len     = i
        #canfd_msgs[i].frame.__pad  =0x08 #回显标志位
        for j in range(canfd_msgs[i].frame.len):
            canfd_msgs[i].frame.data[j] = j
    ret = lib.ZCAN_TransmitFD(Hchnl.hchnl[0], canfd_msgs, transmit_canfd_num)
    

    CAN_merge_type =2
    #Receive Messages
    rcv_msg=(ZCANDataObj*1000)()
    memset(rcv_msg,0,sizeof(rcv_msg))
    
    while True:
        rcv_num = lib.ZCAN_ReceiveData(Device_handle_,rcv_msg, 100,0)
        if rcv_num:
            for i in range(rcv_num):
                if rcv_msg[i].dataType ==1:#CAN/CANFD数据
                    print("[%d]:timestamps:%d, chn: %d ,%s， type:%s, id:%s, dlc:%d, eff:%d, rtr:%d, data:%s \n" %(i, rcv_msg[i].data.zcanfddata.timestamp,
                        rcv_msg[i].chnl,("Tx" if  rcv_msg[i].data.zcanfddata.flag.txEchoed  else "Rx"),
                        ("canfd" if rcv_msg[i].data.zcanfddata.flag.frameType else "can"), 
                        hex(rcv_msg[i].data.zcanfddata.frame.can_id), rcv_msg[i].data.zcanfddata.frame.len, 
                        rcv_msg[i].data.zcanfddata.frame.eff, rcv_msg[i].data.zcanfddata.frame.rtr,
                        ''.join(hex(rcv_msg[i].data.zcanfddata.frame.data[j])+ ' 'for j in range(rcv_msg[i].data.zcanfddata.frame.len))))
                if rcv_msg[i].dataType ==2:#错误数据
                    print("[%d]:timestamps:%d, chn: %d， errType:%d, errSubType:%s, nodeState:%d, rxErrCount:%d, txErrCount:%d, errData:%d \n" %(i, rcv_msg[i].data.zcanErrData.timestamp,
                        rcv_msg[i].chnl, rcv_msg[i].data.zcanErrData.errType, rcv_msg[i].data.zcanErrData.errSubType, 
                        rcv_msg[i].data.zcanErrData.nodeState, rcv_msg[i].data.zcanErrData.rxErrCount,
                        rcv_msg[i].data.zcanErrData.txErrCount, rcv_msg[i].data.zcanErrData.errData))

        
        if thread.is_alive() == False:
            break


    ret=lib.ZCAN_CloseDevice(Device_handle_)
    if ret ==0:
        print("Closedevice Failed!")
    else:
        print("Closedevice success!")
    del lib
    print('done')
