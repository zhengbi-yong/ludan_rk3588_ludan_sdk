
// net-tcpDlg.cpp : 实现文件
//

#include "stdafx.h"
#include <process.h>
#include "net-tcp.h"
#include "net-tcpDlg.h"
#include "utility.h"

using namespace std;

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


typedef struct _DeviceInfo
{
    UINT device_type;  //设备类型
    UINT channel_count;//设备的通道个数
    UINT device_tcp;   //设备是否tcp设备
    CString dev_name;
}DeviceInfo;
static const DeviceInfo kDeviceType[] = {
    { ZCAN_CANFDNET_TCP,        2, 1, _T("CANFDNET_200U_TCP") },
    { ZCAN_CANFDNET_UDP,        2, 0, _T("CANFDNET_200U_UDP") },
    { ZCAN_CANFDWIFI_TCP,       1, 1, _T("CANFDWIFI_100U_TCP") },
    { ZCAN_CANFDWIFI_UDP,       1, 0, _T("CANFDWIFI_100U_UDP") },
    { ZCAN_CANFDNET_400U_TCP,   4, 1, _T("CANFDNET_400U_TCP") },
    { ZCAN_CANFDNET_400U_UDP,   4, 0, _T("CANFDNET_400U_UDP") },
    { ZCAN_CANFDNET_100U_TCP,   1, 1, _T("CANFDNET_100_TCP") },
    { ZCAN_CANFDNET_100U_UDP,   1, 0, _T("CANFDNET_100_UDP") },
    { ZCAN_CANFDNET_800U_TCP,   8, 1, _T("CANFDNET_800U_TCP") },
    { ZCAN_CANFDNET_800U_UDP,   8, 0, _T("CANFDNET_800U_UDP") },
};


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// 对话框数据
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()

// Cusbcanfdx00udemoDlg 对话框

CnettcpDlg::CnettcpDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CnettcpDlg::IDD, pParent)
	, device_type_index_(0)
	, device_index_(0)
    , device_tcp_(1)
	, work_mode_index_(0)
	, frame_type_index_(0)
	, format_index_(0)
	, send_type_index_(0)
	, id_(_T("00000001"))
	, datas_(_T("00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63"))
	, local_port_(4001)
	, ip_(_T("172.16.9.230"))
	, work_port_(8000)
	, channel_index_(0)
    , m_nDevTimeStamp(0)
    , send_count_once_(1)
    , send_count_repeat_(1)
    , protocol_index_(1)
    , canfd_exp_index_(0)
    , sendtime_(1000)
    , lin_mode_master_(false)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	device_opened_ = FALSE;
	start_ = FALSE;
    recv_thread_running_ = FALSE;
}

void CnettcpDlg::DoDataExchange(CDataExchange* pDX)
{
    CDialog::DoDataExchange(pDX);
    DDX_CBIndex(pDX, IDC_COMBO_DEVICE, device_type_index_);
    DDX_CBIndex(pDX, IDC_COMBO_DEVICE_INDEX, device_index_);
    DDX_CBIndex(pDX, IDC_COMBO_MODE, work_mode_index_);
    DDX_CBIndex(pDX, IDC_COMBO_FRAME_TYPE, frame_type_index_);
    DDX_CBIndex(pDX, IDC_COMBO_FORMAT, format_index_);
    DDX_CBIndex(pDX, IDC_COMBO_SEND_TYPE, send_type_index_);
    DDX_Control(pDX, IDC_LIST1, data_recv_list_);
    DDX_Text(pDX, IDC_EDIT_FILTER_START3, datas_);
    DDX_Control(pDX, IDC_COMBO_DEVICE, ctrl_device_type_);
    DDX_Control(pDX, IDC_COMBO_DEVICE_INDEX, ctrl_device_index_);
    DDX_Control(pDX, IDC_BUTTON_OPEN, ctrl_open_device_);
    DDX_Control(pDX, IDC_BUTTON_INITCAN, ctrl_int_can_);
    DDX_Control(pDX, IDC_BUTTON_STARTCAN, ctrl_start_can_);
    DDX_Control(pDX, IDC_BUTTON_CLOSE, ctrl_close_device_);
    DDX_Text(pDX, IDC_EDIT_LOCAL_PORT, local_port_);
    DDX_Text(pDX, IDC_EDIT_IP, ip_);
    DDX_Text(pDX, IDC_EDIT_WORK_PORT, work_port_);
    DDX_CBIndex(pDX, IDC_COMBO_CHANNEL_INDEX, channel_index_);
    DDX_Text(pDX, IDC_EDIT_ID, id_);
    DDV_MaxChars(pDX, id_, 8);
    DDX_Control(pDX, IDC_CHECK_TX_ECHO, m_checkTxEcho);
    DDX_Text(pDX, IDC_EDIT_DEV_TIMESTAMP, m_nDevTimeStamp);
    DDX_Control(pDX, IDC_CHECK_BUS_USAGE, m_bthShowBusUsage);
    DDX_Text(pDX, IDC_EDIT_ID2, send_count_once_);
    DDV_MinMaxInt(pDX, send_count_once_, 1, 999);
    DDX_Text(pDX, IDC_EDIT_ID3, send_count_repeat_);
    DDV_MinMaxInt(pDX, send_count_repeat_, 1, 999);
    DDX_Control(pDX, IDC_CHECK_SHOW_DATA, m_btnShowData);
    DDX_CBIndex(pDX, IDC_COMBO_CAN_CANFD, protocol_index_);
    DDX_CBIndex(pDX, IDC_COMBO_CAN_CANFD2, canfd_exp_index_);
    DDX_Text(pDX, IDC_EDIT_TIME, sendtime_);
    DDX_Control(pDX, IDC_CHECK_DELAY_MODE, m_checkDelaySend);
    DDX_Control(pDX, IDC_CHECK_TIME_UNIT, m_checkTimeUnit);
    DDX_Control(pDX, IDC_STATIC_TIME_UNIT, m_staticTimeUnit);
    DDX_Control(pDX, IDC_CHECK_LIN_MASTER, m_checkLINMaster);
    DDX_Control(pDX, IDC_CHECK_ENHANCE_CHKSUM, m_checkLINEnhanceChksum);
    DDX_Control(pDX, IDC_CHECK_VAR_DLC, m_checkLINVarDLC);
    DDX_Control(pDX, IDC_COMBO_LIN_BAUD, m_comboLINBaud);
    DDX_Control(pDX, IDC_COMBO_LIN_VERSION, m_comboLINVersion);
    DDX_Control(pDX, IDC_BUTTON_INIT_LIN, ctrl_int_lin_);
    DDX_Control(pDX, IDC_BUTTON_START_LIN, ctrl_start_lin_);
    DDX_Control(pDX, IDC_BUTTON_RESET_LIN, ctrl_reset_lin_);
    DDX_Control(pDX, IDC_CHECK_RECV_DATA, m_btnRecvData);
    DDX_Control(pDX, IDC_CHECK_RECV_MERGE, m_checkRecvMerge);
}

BEGIN_MESSAGE_MAP(CnettcpDlg, CDialog)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDC_BUTTON_OPEN, &CnettcpDlg::OnBnClickedButtonOpen)
	ON_BN_CLICKED(IDC_BUTTON_INITCAN, &CnettcpDlg::OnBnClickedButtonInitcan)
	ON_BN_CLICKED(IDC_BUTTON_STARTCAN, &CnettcpDlg::OnBnClickedButtonStartcan)
	ON_BN_CLICKED(IDC_BUTTON_RESET, &CnettcpDlg::OnBnClickedButtonReset)
	ON_BN_CLICKED(IDC_BUTTON_CLOSE, &CnettcpDlg::OnBnClickedButtonClose)
	ON_BN_CLICKED(IDC_BUTTON_SEND, &CnettcpDlg::OnBnClickedButtonSend)
	ON_WM_CLOSE()
	ON_BN_CLICKED(IDC_BUTTON_CLEAR, &CnettcpDlg::OnBnClickedButtonClear)
	ON_CBN_SELCHANGE(IDC_COMBO_MODE, &CnettcpDlg::OnCbnSelchangeComboMode)
	ON_CBN_SELCHANGE(IDC_COMBO_DEVICE, &CnettcpDlg::OnCbnSelchangeComboDevice)
    ON_BN_CLICKED(IDC_BUTTON_SET_DEV_TIMESTAMP, &CnettcpDlg::OnBnClickedButtonSetDevTimestamp)
    ON_BN_CLICKED(IDC_CHECK_TX_ECHO, &CnettcpDlg::OnBnClickedCheckTxEcho)
    ON_BN_CLICKED(IDC_BUTTON_DEV_INFO, &CnettcpDlg::OnBnClickedButtonDevInfo)
    ON_BN_CLICKED(IDC_BUTTON_GET_DELAY_SEND_AVAILABLE_COUNT, &CnettcpDlg::OnBnClickedButtonGetDelaySendAvailableCount)
    ON_BN_CLICKED(IDC_BUTTON_GET_AUTO_SEND_INFO, &CnettcpDlg::OnBnClickedButtonGetAutoSendInfo)
    ON_BN_CLICKED(IDC_CHECK_DELAY_MODE, &CnettcpDlg::OnBnClickedCheckDelayMode)
    ON_BN_CLICKED(IDC_BUTTON_CANCEL_DELAY_SEND, &CnettcpDlg::OnBnClickedButtonCancelDelaySend)
    ON_BN_CLICKED(IDC_BUTTON_SET_AUTO_SEND, &CnettcpDlg::OnBnClickedButtonSetAutoSend)
    ON_BN_CLICKED(IDC_BUTTON_CANCEL_AUTO_SEND, &CnettcpDlg::OnBnClickedButtonCancelAutoSend)
    ON_BN_CLICKED(IDC_BUTTON_STATE_SYS_INFO, &CnettcpDlg::OnBnClickedButtonStateSysInfo)
    ON_BN_CLICKED(IDC_BUTTON_STATE_CAN_INFO, &CnettcpDlg::OnBnClickedButtonStateCanInfo)
    ON_BN_CLICKED(IDC_BUTTON_STATE_RECORDER, &CnettcpDlg::OnBnClickedButtonStateRecorder)
    ON_BN_CLICKED(IDC_BUTTON_STATE_NET, &CnettcpDlg::OnBnClickedButtonStateNet)
    ON_BN_CLICKED(IDC_CHECK_TIME_UNIT, &CnettcpDlg::OnBnClickedCheckTimeUnit)
    ON_BN_CLICKED(IDC_BUTTON_CANCEL_SINGLE, &CnettcpDlg::OnBnClickedButtonCancelSingle)
    ON_BN_CLICKED(IDC_CHECK_LIN_MASTER, &CnettcpDlg::OnBnClickedCheckLinMaster)
    ON_CBN_SELCHANGE(IDC_COMBO_LIN_VERSION, &CnettcpDlg::OnCbnSelchangeComboLinVersion)
    ON_BN_CLICKED(IDC_BUTTON_INIT_LIN, &CnettcpDlg::OnBnClickedButtonInitLin)
    ON_BN_CLICKED(IDC_BUTTON_START_LIN, &CnettcpDlg::OnBnClickedButtonStartLin)
    ON_BN_CLICKED(IDC_BUTTON_RESET_LIN, &CnettcpDlg::OnBnClickedButtonResetLin)
    ON_BN_CLICKED(IDC_BUTTON_SEND_LIN, &CnettcpDlg::OnBnClickedButtonSendLin)
    ON_BN_CLICKED(IDC_BUTTON_CLEAR_SLAVE_ID_RESPONSE, &CnettcpDlg::OnBnClickedButtonClearSlaveIdResponse)
    ON_BN_CLICKED(IDC_BUTTON_GET_LIN_FIFO_INFO, &CnettcpDlg::OnBnClickedButtonGetLinFifoInfo)
    ON_BN_CLICKED(IDC_CHECK_RECV_MERGE, &CnettcpDlg::OnBnClickedCheckRecvMerge)
    ON_BN_CLICKED(IDC_BUTTON_SEND_MERGE, &CnettcpDlg::OnBnClickedButtonSendMerge)
    ON_BN_CLICKED(IDC_CHECK_RECV_DATA, &CnettcpDlg::OnBnClickedCheckRecvData)
END_MESSAGE_MAP()


// Cusbcanfdx00udemoDlg 消息处理程序

BOOL CnettcpDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码
    int nCount = _countof(kDeviceType);
    for (int i = 0; i < nCount; i++)
    {
        ctrl_device_type_.SetItemData(ctrl_device_type_.AddString(kDeviceType[i].dev_name), i);
    }
    ctrl_device_type_.SetCurSel(4);

    UINT linBaud[] = { 1000, 1200, 2400, 4800, 9600, 10417, 19200, 20000 };
    CString csText;
    nCount = _countof(linBaud);
    for (int i = 0; i < nCount; ++i)
    {
        csText.Format(_T("%dbps"), linBaud[i]);
        m_comboLINBaud.SetItemData(m_comboLINBaud.AddString(csText), linBaud[i]);
    }
    m_comboLINBaud.SetCurSel(nCount - 1);

    //V2.x
    m_comboLINVersion.SetCurSel(1);
    OnCbnSelchangeComboLinVersion();

    OnBnClickedCheckLinMaster();


	OnCbnSelchangeComboMode();
	OnCbnSelchangeComboDevice();

    //m_btnShowData.SetCheck(BST_CHECKED);
    m_btnRecvData.SetCheck(BST_CHECKED);

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CnettcpDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CnettcpDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CnettcpDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void CnettcpDlg::OnBnClickedButtonOpen()
{
	UpdateData(TRUE);
    device_handle_ = ZCAN_OpenDevice(kDeviceType[device_type_index_].device_type, device_index_, local_port_);
	if (INVALID_DEVICE_HANDLE == device_handle_)
	{
		AddData(_T("打开设备失败!"));
		return;
	}
	device_opened_ = TRUE;
	EnableCtrl(TRUE);
}

void CnettcpDlg::OnBnClickedButtonInitcan()
{
	if (!device_opened_)
	{
		AddData(_T("设备还没打开!"));
		return;
	}
	UpdateData(TRUE);
	property_ = GetIProperty(device_handle_);
	//ASSERT(property_ != NULL);
	
    if (device_tcp_)
    {
        if (!SetWorkMode())
        {
            AddData(_T("设置工作模式失败!"));
            return;
        }

        if (0 == work_mode_index_)//client
        {
            if (!SetClientParam())
            {
                AddData(_T("设置客户端参数失败!"));
                return;
            }
        }
        else if (!SetServerParam())
        {
            AddData(_T("设置服务端参数失败!"));
            return;
        }
    }
    else
    {
        if (!SetClientParam())
        {
            AddData(_T("设置UDP参数失败!"));
            return;
        }
    }
	
	ZCAN_CHANNEL_INIT_CONFIG config;
	memset(&config, 0, sizeof(config));
	config.can_type = TYPE_CAN;
	channel_handle_ = ZCAN_InitCAN(device_handle_, channel_index_, &config);
	if (INVALID_CHANNEL_HANDLE == channel_handle_)
	{
		AddData(_T("初始化CAN失败!"));
		return;
	}

    //设置TxEcho
    UINT nTxEcho = m_checkTxEcho.GetCheck() == BST_CHECKED;
    ZCAN_SetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, SETREF_SET_TX_ECHO_ENABLE, &nTxEcho);

	ctrl_int_can_.EnableWindow(FALSE);
}

void CnettcpDlg::OnBnClickedButtonStartcan()
{
    //启动前先设置发送模式
    OnBnClickedCheckDelayMode();

	if (ZCAN_StartCAN(channel_handle_) != STATUS_OK)
	{
		AddData(_T("启动CAN失败!"));
		return;
	}
	ctrl_start_can_.EnableWindow(FALSE);
    if (!recv_thread_running_)
    {
        start_ = TRUE;
        _beginthreadex(NULL, 0, OnDataRecv, this, 0, NULL);
    }
}

void CnettcpDlg::OnBnClickedButtonReset()
{
	if (ZCAN_ResetCAN(channel_handle_) != STATUS_OK)
	{
		AddData(_T("复位失败!"));
		return;
	}
	ctrl_start_can_.EnableWindow(TRUE);
	start_ = FALSE;
}

void CnettcpDlg::OnBnClickedButtonClose()
{
	// TODO: Add your control notification handler code here
	ZCAN_CloseDevice(device_handle_);
	start_ = FALSE;
	EnableCtrl(FALSE);
    ctrl_start_can_.EnableWindow(TRUE);
    ctrl_int_can_.EnableWindow(TRUE);
    ctrl_start_lin_.EnableWindow(TRUE);
    ctrl_int_lin_.EnableWindow(TRUE);
	device_opened_ = FALSE;
}

void CnettcpDlg::OnBnClickedButtonSend()
{
	UpdateData(TRUE);
	if (datas_.IsEmpty())
	{
		AddData(_T("数据为空"));
		return;
	}
	UINT id = _tcstoul(id_, 0, 16);
	string data = Utility::W2AEx(datas_);
    BOOL bDelaySend = m_checkDelaySend.GetCheck() == BST_CHECKED;
    BOOL bCheckTimeUnit = m_checkTimeUnit.GetCheck() == BST_CHECKED;
    if (0 == protocol_index_)//can
    {
        //can
        ZCAN_Transmit_Data can_data;
        memset(&can_data, 0, sizeof(can_data));
        can_data.frame.can_id = MAKE_CAN_ID(id, frame_type_index_, format_index_, 0);
        can_data.frame.can_dlc = Utility::split(can_data.frame.data, CAN_MAX_DLEN, data, ' ', 16);
        can_data.transmit_type = send_type_index_;
        can_data.frame.__res0 = LOBYTE(sendtime_);
        can_data.frame.__res1 = HIBYTE(sendtime_);
        if (bDelaySend)
        {
            can_data.frame.__pad |= TX_DELAY_SEND_FLAG;
        }
        if (bCheckTimeUnit)
        {
            can_data.frame.__pad |= TX_DELAY_SEND_TIME_UNIT_FLAG;
        }

        UINT count_sent_total = 0;
        ZCAN_Transmit_Data* pData = nullptr;
        if (send_count_once_ >= 1)
        {
            pData = new ZCAN_Transmit_Data[send_count_once_];
            for (int i = 0; i < send_count_once_; i++)
            {
                pData[i] = can_data;
            }
        }

        for (int j = 0; j < send_count_repeat_; j++)
        {
            UINT count_sent = ZCAN_Transmit(channel_handle_, pData, send_count_once_);
            count_sent_total += count_sent;
            if (count_sent != send_count_once_)
            {
                AddData(_T("发送CAN数据失败!"));
                break;
            }
        }

        if (count_sent_total == send_count_once_ * send_count_repeat_)
        {
            CString csText;
            csText.Format(_T("发送CAN数据成功: %d * %d = %d"), send_count_once_, send_count_repeat_, count_sent_total);
            AddData(csText);
        }
        else
        {
            CString csText;
            csText.Format(_T("发送CAN数据失败，已发送: %d"), count_sent_total);
            AddData(csText);
        }

        if (pData)
        {
            delete[] pData;
            pData = nullptr;
        }
    }
    else
    {
        //canfd
        ZCAN_TransmitFD_Data canfd_data;
        memset(&canfd_data, 0, sizeof(canfd_data));
        canfd_data.frame.can_id = MAKE_CAN_ID(id, frame_type_index_, 0, 0);
        canfd_data.frame.len = Utility::split(canfd_data.frame.data, CANFD_MAX_DLEN, data, ' ', 16);
        canfd_data.transmit_type = send_type_index_;
        canfd_data.frame.__res0 = LOBYTE(sendtime_);
        canfd_data.frame.__res1 = HIBYTE(sendtime_);
        if (bDelaySend)
        {
            canfd_data.frame.flags |= TX_DELAY_SEND_FLAG;
        }
        if (bCheckTimeUnit)
        {
            canfd_data.frame.flags |= TX_DELAY_SEND_TIME_UNIT_FLAG;
        }
        if (canfd_exp_index_)
        {
            canfd_data.frame.flags |= CANFD_BRS;
        }

        UINT count_sent_total = 0;
        ZCAN_TransmitFD_Data* pData = nullptr;
        if (send_count_once_ >= 1)
        {
            pData = new ZCAN_TransmitFD_Data[send_count_once_];
            for (int i = 0; i < send_count_once_; i++)
            {
                pData[i] = canfd_data;
            }
        }

        for (int j = 0; j < send_count_repeat_; j++)
        {
            UINT count_sent = ZCAN_TransmitFD(channel_handle_, pData, send_count_once_);
            count_sent_total += count_sent;
            if (count_sent != send_count_once_)
            {
                AddData(_T("发送CANFD数据失败!"));
                break;
            }
        }

        if (count_sent_total == send_count_once_ * send_count_repeat_)
        {
            CString csText;
            csText.Format(_T("发送CANFD数据成功: %d * %d = %d"), send_count_once_, send_count_repeat_, count_sent_total);
            AddData(csText);
        }
        else
        {
            CString csText;
            csText.Format(_T("发送CANFD数据失败，已发送: %d"), count_sent_total);
            AddData(csText);
        }

        if (pData)
        {
            delete[] pData;
            pData = nullptr;
        }
    }

}

void CnettcpDlg::EnableCtrl( BOOL opened )
{
	ctrl_device_type_.EnableWindow(!opened);
	ctrl_device_index_.EnableWindow(!opened);
	ctrl_open_device_.EnableWindow(!opened);
}

UINT WINAPI CnettcpDlg::OnDataRecv( LPVOID data )
{
	CnettcpDlg* this_ = static_cast<CnettcpDlg*>(data);
	if (this_)
	{
		this_->OnRecv();
	}
	return 0;
}

void CnettcpDlg::OnRecv()
{
    const int nTxEchoTimeStampCount = 10;
	ZCAN_Receive_Data can_data[100];
    ZCAN_ReceiveFD_Data canfd_data[100];
    ZCAN_LIN_MSG        lin_data[100];
    ZCANDataObj         zcan_data[100];
	UINT len;
    BusUsage stBusUsage;
    TxTimeStamp stTxTimeStamp;
    stTxTimeStamp.nWaitTime = 0;
    stTxTimeStamp.nBufferTimeStampCount = nTxEchoTimeStampCount;
    stTxTimeStamp.pTxTimeStampBuffer = new UINT64[nTxEchoTimeStampCount];

    ZCAN_GPS_DATA stGPSData;
    stGPSData.pData = new ZCAN_GPS_FRAME[nTxEchoTimeStampCount];
    stGPSData.nFrameCount = nTxEchoTimeStampCount;
    stGPSData.nWaitTime = 0;

	while(start_)
	{
        if (!m_bRecvData)
        {
            Sleep(10);
            continue;
        }
        bool bSleep = true;

        if (!m_bRecvMerge)
        {
            if (len = ZCAN_GetReceiveNum(channel_handle_, TYPE_CAN))
            {
                len = ZCAN_Receive(channel_handle_, can_data, 100, 50);
                if (m_btnShowData.GetCheck() == BST_CHECKED)
                {
                    AddData(can_data, len);
                }
                bSleep = false;
            }

            if (len = ZCAN_GetReceiveNum(channel_handle_, TYPE_CANFD))
            {
                len = ZCAN_ReceiveFD(channel_handle_, canfd_data, 100, 50);
                if (m_btnShowData.GetCheck() == BST_CHECKED)
                {
                    AddData(canfd_data, len);
                }
                bSleep = false;
            }

            if (len = ZCAN_GetLINReceiveNum(lin_channel_handle_))
            {
                len = ZCAN_ReceiveLIN(lin_channel_handle_, lin_data, 100, 50);
                if (m_btnShowData.GetCheck() == BST_CHECKED)
                {
                    AddData(lin_data, len);
                }
                bSleep = false;
            }
        }
        else
        {
            if (len = ZCAN_GetReceiveNum(channel_handle_, TYPE_ALL_DATA))
            {
                len = ZCAN_ReceiveData(device_handle_, zcan_data, 100, 0);
                if (m_btnShowData.GetCheck() == BST_CHECKED)
                {
                    AddData(zcan_data, len);
                }
                bSleep = false;
            }
        }

#if 0

        if (ZCAN_GetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, GETREF_GET_BUS_USAGE, &stBusUsage) == STATUS_OK)
        {
            CString csBusUsage;
            csBusUsage.Format(_T("BusLoad: %.2f%% Count:%d Time:%I64u-%I64u"), stBusUsage.nBusUsage / 100.0f, stBusUsage.nFrameCount, stBusUsage.nTimeStampBegin, stBusUsage.nTimeStampEnd);
            if (m_bthShowBusUsage.GetCheck() == BST_CHECKED)
            {
                AddData(csBusUsage);
            }
            bSleep = false;
        }


        stTxTimeStamp.nBufferTimeStampCount = nTxEchoTimeStampCount;
        if (ZCAN_GetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, GETREF_GET_TX_TIMESTAMP, &stTxTimeStamp) == STATUS_OK)
        {
            CString csText, csTemp;
            if (stTxTimeStamp.nBufferTimeStampCount > 0)
            {
                csText.Format(_T("TxEchoCount: %d"), stTxTimeStamp.nBufferTimeStampCount);
                AddData(csText);
                csText.Empty();
                for (size_t i = 1; i <= stTxTimeStamp.nBufferTimeStampCount; i++)
                {
                    csTemp.Format(_T("%I64u "), stTxTimeStamp.pTxTimeStampBuffer[i-1]);
                    if (i % 5)
                    {
                        csText.Append(csTemp);
                    }
                    else
                    {
                        csText.Append(csTemp);
                        AddData(csText);
                        csText.Empty();
                    }

                    if (i == stTxTimeStamp.nBufferTimeStampCount && !csText.IsEmpty())
                    {
                        AddData(csText);
                        csText.Empty();
                    }
                }
            }
            bSleep = false;
        }

        UINT nGPSCount = 0;
        if (ZCAN_GetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, GETREF_GET_DEV_GPS_COUNT, &nGPSCount) == STATUS_OK)
        {
            if (nGPSCount > 0)
            {
                if (ZCAN_GetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, GETREF_GET_DEV_GPS_DATA, &stGPSData) == STATUS_OK)
                {
                    CString csText;
                    csText.Format(_T("GPS query:%d, recv:%d"), nGPSCount, stGPSData.nRet);
                    AddData(csText);

                    for (UINT i = 0; i < stGPSData.nRet; i++)
                    {
                        AddData(stGPSData.pData, 1);
                    }
                }
            }
        }


#endif

        if (bSleep)
        {
            Sleep(10);
        }
    }

    delete[] stTxTimeStamp.pTxTimeStampBuffer;
    stTxTimeStamp.pTxTimeStampBuffer = nullptr;
}

void CnettcpDlg::AddData( const ZCAN_Receive_Data* data, UINT len )
{
	char item[1000];
	for (UINT i = 0; i < len; ++i)
	{
		const ZCAN_Receive_Data& can = data[i];
		const canid_t& id = can.frame.can_id;
		sprintf_s(item, "接收到CAN ID:%08X Time:%I64u %s %s 长度:%d 数据:", GET_ID(id), can.timestamp, IS_EFF(id)?"扩展帧":"标准帧"
			, IS_RTR(id)?"远程帧":"数据帧", can.frame.can_dlc);
		if (!IS_RTR(id))
		{
			for (UINT i = 0; i < can.frame.can_dlc; ++i)
			{
				size_t item_len = strlen(item);
                sprintf_s(&item[item_len], 1000 - item_len, "%02X ", can.frame.data[i]);
			}
		}
		AddData(CString(item));
	}
}

void CnettcpDlg::AddData(const ZCAN_ReceiveFD_Data* data, UINT len)
{
    char item[1000];
    for (UINT i = 0; i < len; ++i)
    {
        const ZCAN_ReceiveFD_Data& canfd = data[i];
        const canid_t& id = canfd.frame.can_id;
        sprintf_s(item, "接收到CANFD ID:%08X  Time:%I64u %s 长度:%d 数据:", GET_ID(id), canfd.timestamp, IS_EFF(id) ? "扩展帧" : "标准帧", canfd.frame.len);
        for (UINT i = 0; i < canfd.frame.len; ++i)
        {
            size_t item_len = strlen(item);
            sprintf_s(&item[item_len], 1000 - item_len, "%02X ", canfd.frame.data[i]);
        }
        AddData(CString(item));
    }
}

void CnettcpDlg::AddData(const ZCAN_GPS_FRAME* data, UINT len)
{
    CString csText;
    for (UINT i = 0; i < len; ++i)
    {
        csText.Format(_T("GPS Frame: lat:%.6f long:%.6f spd:%.6fkm/h "), data[i].latitude, data[i].longitude, data[i].speed);
        AddData(csText);
    }

}

void CnettcpDlg::AddData( const CString& data )
{
	data_recv_list_.AddString(data);
	data_recv_list_.SetCurSel(data_recv_list_.GetCount() - 1);
}

void CnettcpDlg::AddData(const ZCAN_LIN_MSG* data, UINT len)
{
    char item[1000];
    for (UINT i = 0; i < len; ++i)
    {
        const ZCAN_LIN_MSG& lin = data[i];
        sprintf_s(item, "LIN ID:%02X  Time:%d 长度:%d 数据:", lin.ID, lin.TimeStamp, lin.DataLen);
        for (UINT i = 0; i < lin.DataLen; ++i)
        {
            size_t item_len = strlen(item);
            sprintf_s(&item[item_len], 1000 - item_len, "%02X ", lin.Data[i]);
        }
        AddData(CString(item));
    }
}

void CnettcpDlg::AddData(const ZCANDataObj* data, UINT len)
{
    char item[1000] = { 0 };
    CString csText;
    for (UINT i = 0; i < len; ++i)
    {
        const ZCANDataObj& zcandata = data[i];
        csText.Format(_T("接收到ZCANDataObj Chnl:%d dataType:%d"), zcandata.chnl, zcandata.dataType);
        AddData(csText);
        if (zcandata.dataType == ZCAN_DT_ZCAN_CAN_CANFD_DATA)
        {
            const ZCANCANFDData& canfd = zcandata.data.zcanCANFDData;
            const canid_t& id = canfd.frame.can_id;
            sprintf_s(item, "Time:%I64u %s ID:%08X TxEcho:%d %s 长度:%d 数据:", 
                canfd.timeStamp, canfd.flag.unionVal.frameType ? "CANFD" : "CAN", GET_ID(id), canfd.flag.unionVal.txEchoed, IS_EFF(id) ? "扩展帧" : "标准帧", canfd.frame.len);
            for (UINT i = 0; i < canfd.frame.len; ++i)
            {
                size_t item_len = strlen(item);
                sprintf_s(&item[item_len], 1000 - item_len, "%02X ", canfd.frame.data[i]);
            }
        }
        else if (zcandata.dataType == ZCAN_DT_ZCAN_ERROR_DATA)
        {
            const ZCANErrorData err = zcandata.data.zcanErrData;
            sprintf_s(item, "ErrType:%d ErrSubType:%d nodeState:%d TxErrCnt:%d RxErrCnt:%d ErrData:%02X", err.errType, err.errSubType, err.nodeState, err.txErrCount, err.rxErrCount, err.errData);
        }
        else if (zcandata.dataType == ZCAN_DT_ZCAN_LIN_DATA)
        {
            const ZCANLINData& lin = zcandata.data.zcanLINData;
            sprintf_s(item, "LIN ID:%02X  Time:%d 长度:%d 数据:", lin.PID.rawVal, lin.timeStamp, lin.dataLen);
            for (UINT i = 0; i < lin.dataLen; ++i)
            {
                size_t item_len = strlen(item);
                sprintf_s(&item[item_len], 1000 - item_len, "%02X ", lin.data[i]);
            }
        }
        else if (zcandata.dataType == ZCAN_DT_ZCAN_GPS_DATA)
        {
            const ZCANGPSData& gps = zcandata.data.zcanGPSData;
            sprintf_s(item, "GPS Time:%04d-%02d-%02d %02d:%02d:%02d.%03d Lat:%.6f Long:%.6f", gps.time.year, gps.time.mon, gps.time.day, gps.time.hour, gps.time.min, gps.time.sec, gps.time.milsec,
                gps.latitude, gps.longitude);
        }
        else
        {
            sprintf_s(item, "Invalid data");
        }

        AddData(CString(item));
    }

}

void CnettcpDlg::OnClose()
{
	// TODO: Add your message handler code here and/or call default
	OnBnClickedButtonClose();
	CDialog::OnClose();
}
void CnettcpDlg::OnBnClickedButtonClear()
{
	data_recv_list_.ResetContent();
}
void CnettcpDlg::OnCbnSelchangeComboMode()
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);
    if (device_tcp_)
    {
        BOOL is_client = 0 == work_mode_index_;
        GetDlgItem(IDC_STATIC_LOCAL_PORT)->ShowWindow(!is_client);
        GetDlgItem(IDC_EDIT_LOCAL_PORT)->ShowWindow(!is_client);
        GetDlgItem(IDC_STATIC_IP)->ShowWindow(is_client);
        GetDlgItem(IDC_EDIT_IP)->ShowWindow(is_client);
        GetDlgItem(IDC_STATIC_WORK_PORT)->ShowWindow(is_client);
        GetDlgItem(IDC_EDIT_WORK_PORT)->ShowWindow(is_client);
    }
    else
    {
        int nCmdShow = SW_SHOWNORMAL;
        GetDlgItem(IDC_STATIC_LOCAL_PORT)->ShowWindow(nCmdShow);
        GetDlgItem(IDC_EDIT_LOCAL_PORT)->ShowWindow(nCmdShow);
        GetDlgItem(IDC_STATIC_IP)->ShowWindow(nCmdShow);
        GetDlgItem(IDC_EDIT_IP)->ShowWindow(nCmdShow);
        GetDlgItem(IDC_STATIC_WORK_PORT)->ShowWindow(nCmdShow);
        GetDlgItem(IDC_EDIT_WORK_PORT)->ShowWindow(nCmdShow);
    }
}

void CnettcpDlg::OnCbnSelchangeComboDevice()
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);
	InitCombobox(IDC_COMBO_CHANNEL_INDEX, 0, kDeviceType[device_type_index_].channel_count, 0);
    device_tcp_ = kDeviceType[device_type_index_].device_tcp;
    OnCbnSelchangeComboMode();
    int nCmdShow = device_tcp_ ? SW_SHOWNORMAL : SW_HIDE;
    GetDlgItem(IDC_STATIC_WORK_MODE)->ShowWindow(nCmdShow);
    GetDlgItem(IDC_COMBO_MODE)->ShowWindow(nCmdShow);
}

void CnettcpDlg::InitCombobox( int ctrl_id, int start, int end, int current )
{
	CComboBox* ctrl = static_cast<CComboBox*>(GetDlgItem(ctrl_id));
	ASSERT(ctrl != NULL);
	ctrl->ResetContent();
	CString temp;
	for (int i = start; i < end; ++i)
	{
		temp.Format(_T("%d"), i);
		ctrl->AddString(temp);
	}
	ctrl->SetCurSel(current);
}

BOOL CnettcpDlg::SetClientParam()
{
#if 0
	char path[50] = {0};
	char value[100] = {0};
	sprintf_s(path, "%d/ip", channel_index_);
	sprintf_s(value, "%s", Utility::W2AEx(ip_).data());
	if (property_->SetValue(path, value) != 1)
	{
		return FALSE;
	}
	sprintf_s(path, "%d/work_port", channel_index_);
	sprintf_s(value, "%d", work_port_);
	return 1 == property_->SetValue(path, value);
#else 
    char value[100] = {0};
    sprintf_s(value, "%s", Utility::W2AEx(ip_).data());
    ZCAN_SetReference(kDeviceType[device_type_index_].device_type, device_index_, 0, CMD_DESIP, value);
    ZCAN_SetReference(kDeviceType[device_type_index_].device_type, device_index_, 0, CMD_DESPORT, &work_port_);
    return TRUE;
#endif
}

BOOL CnettcpDlg::SetServerParam()
{
#if 0
	char path[50] = {0};
	char value[100] = {0};
	sprintf_s(path, "%d/local_port", channel_index_);
	sprintf_s(value, "%d", local_port_);
	return 1 == property_->SetValue(path, value);
#else 
    ZCAN_SetReference(kDeviceType[device_type_index_].device_type, device_index_, 0, CMD_SRCPORT, &local_port_);
    return TRUE;
#endif

}

BOOL CnettcpDlg::SetWorkMode()
{
#if 0
    char path[50] = {0};
    char value[100] = {0};
    sprintf_s(path, "%d/work_mode", channel_index_);
    sprintf_s(value, "%d", work_mode_index_);
    return 1 == property_->SetValue(path, value);
#else
    ZCAN_SetReference(kDeviceType[device_type_index_].device_type, device_index_, 0, CMD_TCP_TYPE, &work_mode_index_);
    return TRUE;
#endif

}


void CnettcpDlg::OnBnClickedButtonSetDevTimestamp()
{
    // TODO:  在此添加控件通知处理程序代码
    //设置TimeStamp
    UpdateData(TRUE);
    ZCAN_SetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, SETREF_SET_DEV_TIMESTAMP, &m_nDevTimeStamp);
}


void CnettcpDlg::OnBnClickedCheckTxEcho()
{
    // TODO:  在此添加控件通知处理程序代码
    //设置TxEcho
    UINT nTxEcho = m_checkTxEcho.GetCheck() == BST_CHECKED;
    ZCAN_SetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, SETREF_SET_TX_ECHO_ENABLE, &nTxEcho);
}

CString CnettcpDlg::FormatVersion(USHORT uVersion)
{
    // 按BCD来显示
    //     如
    //     0x0100   1.00
    //     0x0010   0.10
    //     0x0001   0.01
    //     0x0120   1.20
    CString csRet;
    BYTE    ByteData[4] = { 0 };
    for (int i = 0; i<4; ++i)
    {
        ByteData[i] = (uVersion >> (i * 4)) & 0x0F;
    }
    if (ByteData[3])
    {
        csRet.Format(_T("%1d%1d.%1d%1d"), ByteData[3], ByteData[2], ByteData[1], ByteData[0]);
    }
    else
    {
        csRet.Format(_T("%1d.%1d%1d"), ByteData[2], ByteData[1], ByteData[0]);
    }
    return csRet;
}

void CnettcpDlg::ShowDevInfo(ZCAN_DEVICE_INFO& devInfo)
{
    CString csText;
    csText.Format(_T("Hardware:%s"), FormatVersion(devInfo.hw_Version));
    AddData(csText);
    csText.Format(_T("Firmware:%s"), FormatVersion(devInfo.fw_Version));
    AddData(csText);
    CStringA csModule, csSN;
    csModule.Format("Model:%s", devInfo.str_hw_Type);
    csText = csModule;
    AddData(csText);
    csSN.Format("SN:%s", devInfo.str_Serial_Num);
    csText = csSN;
    AddData(csText);
}


void CnettcpDlg::OnBnClickedButtonDevInfo()
{
    // TODO:  在此添加控件通知处理程序代码
    ZCAN_DEVICE_INFO devInfo;
    if (ZCAN_GetDeviceInf(device_handle_, &devInfo))
    {
        ShowDevInfo(devInfo);
    }
    else
    {
        AddData(_T("获取失败!"));
    }
}


void CnettcpDlg::OnBnClickedButtonGetDelaySendAvailableCount()
{
    // TODO:  在此添加控件通知处理程序代码
    UINT nCount = 0;
    if (ZCAN_GetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, GETREF_GET_DELAY_SEND_AVALIABLE_COUNT, &nCount))
    {
        CString csText;
        csText.Format(_T("队列剩余空间:%d"), nCount);
        AddData(csText);
    }
    else
    {
        AddData(_T("获取失败!"));
    }
}


void CnettcpDlg::OnBnClickedButtonGetAutoSendInfo()
{
    // TODO:  在此添加控件通知处理程序代码
    UINT nCount = 0;
    if (ZCAN_GetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, GETREF_GET_DEV_CAN_AUTO_SEND_COUNT, &nCount))
    {
        CString csText;
        csText.Format(_T("CAN定时发送数量:%d"), nCount);
        AddData(csText);
        if (nCount > 0)
        {
            ZCAN_AUTO_TRANSMIT_OBJ* pData = new ZCAN_AUTO_TRANSMIT_OBJ[nCount];
            if (ZCAN_GetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, GETREF_GET_DEV_CAN_AUTO_SEND_DATA, pData))
            {
                for (UINT i = 0; i < nCount; i++)
                {
                    char item[1000];
                    const ZCAN_AUTO_TRANSMIT_OBJ& canAuto = pData[i];
                    const ZCAN_Transmit_Data& can = canAuto.obj;
                    const canid_t& id =  can.frame.can_id;
                    sprintf_s(item, "CAN 索引:%d 间隔:%dms ID:%08X %s %s 长度:%d 数据:", canAuto.index, canAuto.interval, GET_ID(id), IS_EFF(id) ? "扩展帧" : "标准帧"
                        , IS_RTR(id) ? "远程帧" : "数据帧", can.frame.can_dlc);
                    if (!IS_RTR(id))
                    {
                        for (UINT i = 0; i < can.frame.can_dlc; ++i)
                        {
                            size_t item_len = strlen(item);
                            sprintf_s(&item[item_len], 1000 - item_len, "%02X ", can.frame.data[i]);
                        }
                    }

                    AddData(CString(item));
                }
            }
            delete[] pData;
        }
    }
    else
    {
        AddData(_T("CAN定时发送获取失败!"));
    }

    if (ZCAN_GetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, GETREF_GET_DEV_CANFD_AUTO_SEND_COUNT, &nCount))
    {
        CString csText;
        csText.Format(_T("CANFD定时发送数量:%d"), nCount);
        AddData(csText);

        if (nCount > 0)
        {
            ZCANFD_AUTO_TRANSMIT_OBJ* pData = new ZCANFD_AUTO_TRANSMIT_OBJ[nCount];
            if (ZCAN_GetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, GETREF_GET_DEV_CANFD_AUTO_SEND_DATA, pData))
            {
                for (UINT i = 0; i < nCount; i++)
                {
                    char item[1000];
                    const ZCANFD_AUTO_TRANSMIT_OBJ& canAuto = pData[i];
                    const ZCAN_TransmitFD_Data& can = canAuto.obj;
                    const canid_t& id = can.frame.can_id;
                    sprintf_s(item, "CANFD 索引:%d 间隔:%dms ID:%08X %s %s 长度:%d 数据:", canAuto.index, canAuto.interval, GET_ID(id), IS_EFF(id) ? "扩展帧" : "标准帧"
                        , CANFD_BRS &can.frame.flags ? "加速" : "不加速", can.frame.len);
                    if (can.frame.len > 0)
                    {
                        for (UINT i = 0; i < can.frame.len; ++i)
                        {
                            size_t item_len = strlen(item);
                            sprintf_s(&item[item_len], 1000 - item_len, "%02X ", can.frame.data[i]);
                        }
                    }

                    AddData(CString(item));

                }
            }
            delete[] pData;
        }
    }
    else
    {
        AddData(_T("CANFD定时发送获取失败!"));
    }
}



void CnettcpDlg::OnBnClickedCheckDelayMode()
{
    // TODO:  在此添加控件通知处理程序代码
    BOOL bDelaySend = m_checkDelaySend.GetCheck() == BST_CHECKED;

}


void CnettcpDlg::OnBnClickedButtonCancelDelaySend()
{
    // TODO:  在此添加控件通知处理程序代码
    ZCAN_SetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, SETREF_CLEAR_DELAY_SEND_QUEUE, NULL);
}


void CnettcpDlg::OnBnClickedButtonSetAutoSend()
{
    // TODO:  在此添加控件通知处理程序代码
    ZCAN_AUTO_TRANSMIT_OBJ      autoCAN;
    ZCANFD_AUTO_TRANSMIT_OBJ    autoCANFD;
    for (int i = 0; i < 16; i++)
    {
        autoCAN.enable = 1;
        autoCAN.index = i;
        autoCAN.interval = (i + 1) * 1000 + 55000;
        memset(&autoCAN.obj, 0, sizeof(autoCAN.obj));
        autoCAN.obj.frame.can_id = MAKE_CAN_ID(i, i%2, i%2, 0);
        autoCAN.obj.frame.can_dlc = i % 9;

        UINT nRet = ZCAN_SetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, SETREF_ADD_TIMER_SEND_CAN, &autoCAN);
        if (!nRet)
        {
            CString csText;
            csText.Format(_T("CAN定时发送设置失败, Index:%d"), i);
            AddData(csText);
            break;
        }
    }
#if 1
    for (int i = 16; i < 20; i++)
    {
        autoCANFD.enable = 1;
        autoCANFD.index = i;
        autoCANFD.interval = (i + 1) * 1000 + 55000;
        memset(&autoCANFD.obj, 0, sizeof(autoCANFD.obj));
        autoCANFD.obj.frame.can_id = MAKE_CAN_ID(i, i % 2, i % 2, 0);
        autoCANFD.obj.frame.len = i;
        autoCANFD.obj.frame.flags =  i%2;

        UINT nRet = ZCAN_SetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, SETREF_ADD_TIMER_SEND_CANFD, &autoCANFD);
        if (!nRet)
        {
            CString csText;
            csText.Format(_T("CANFD定时发送设置失败, Index:%d"), i);
            AddData(csText);
            break;
        }
    }
#endif

    UINT nRet = ZCAN_SetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, SETREF_APPLY_TIMER_SEND, NULL);
    if (!nRet)
    {
        CString csText;
        csText.Format(_T("定时发送启动失败"));
        AddData(csText);
    }
}


void CnettcpDlg::OnBnClickedButtonCancelAutoSend()
{
    // TODO:  在此添加控件通知处理程序代码
    UINT nRet = ZCAN_SetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, SETREF_CLEAR_TIMER_SEND, NULL);
    if (!nRet)
    {
        CString csText;
        csText.Format(_T("定时发送清除失败"));
        AddData(csText);
    }
}


void CnettcpDlg::OnBnClickedButtonStateSysInfo()
{
    // TODO:  在此添加控件通知处理程序代码
    int nLen = 1024;
    ZCAN_RAW_DATA rawData;
    rawData.pData = new char[nLen];
    rawData.nDataLen = nLen;
    rawData.nResultLen = 0;
    rawData.nWaitTime = 0;
    UINT nRet = ZCAN_GetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, GETREF_GET_DEV_STATE_SYS_INFO, &rawData);
    if (nRet)
    {
        CString csText;
        csText.Format(_T("获取系统状态成功， resultLen:%d"), rawData.nResultLen);
        AddData(csText);
        CStringA csTextA;
        csTextA = (char*)rawData.pData;
        csText = csTextA;
        AddData(csText);
    }
    else
    {
        AddData(_T("获取系统状态失败!"));
    }

    delete[] rawData.pData;
}


void CnettcpDlg::OnBnClickedButtonStateCanInfo()
{
    // TODO:  在此添加控件通知处理程序代码
    int nLen = 1024;
    ZCAN_RAW_DATA rawData;
    rawData.pData = new char[nLen];
    rawData.nDataLen = nLen;
    rawData.nResultLen = 0;
    rawData.nWaitTime = 0;
    UINT nRet = ZCAN_GetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, GETREF_GET_DEV_STATE_CAN_INFO, &rawData);
    if (nRet)
    {
        CString csText;
        csText.Format(_T("获取通道状态成功， resultLen:%d"), rawData.nResultLen);
        AddData(csText);
        CStringA csTextA;
        csTextA = (char*)rawData.pData;
        csText = csTextA;
        AddData(csText);
    }
    else
    {
        AddData(_T("获取通道状态失败!"));
    }

    delete[] rawData.pData;
}


void CnettcpDlg::OnBnClickedButtonStateRecorder()
{
    // TODO:  在此添加控件通知处理程序代码
    int nLen = 1024;
    ZCAN_RAW_DATA rawData;
    rawData.pData = new char[nLen];
    rawData.nDataLen = nLen;
    rawData.nResultLen = 0;
    rawData.nWaitTime = 0;
    UINT nRet = ZCAN_GetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, GETREF_GET_DEV_STATE_RECORDER_INFO, &rawData);
    if (nRet)
    {
        CString csText;
        csText.Format(_T("获取记录状态成功， resultLen:%d"), rawData.nResultLen);
        AddData(csText);
        CStringA csTextA;
        csTextA = (char*)rawData.pData;
        csText = csTextA;
        AddData(csText);
    }
    else
    {
        AddData(_T("获取记录状态失败!"));
    }

    delete[] rawData.pData;
}


void CnettcpDlg::OnBnClickedButtonStateNet()
{
    // TODO:  在此添加控件通知处理程序代码
    int nLen = 1024;
    ZCAN_RAW_DATA rawData;
    rawData.pData = new char[nLen];
    rawData.nDataLen = nLen;
    rawData.nResultLen = 0;
    rawData.nWaitTime = 0;
    UINT nRet = ZCAN_GetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, GETREF_GET_DEV_STATE_NET_INFO, &rawData);
    if (nRet)
    {
        CString csText;
        csText.Format(_T("获取网络状态成功， resultLen:%d"), rawData.nResultLen);
        AddData(csText);
        CStringA csTextA;
        csTextA = (char*)rawData.pData;
        csText = csTextA;
        AddData(csText);
    }
    else
    {
        AddData(_T("获取网络状态失败!"));
    }

    delete[] rawData.pData;
}

void CnettcpDlg::OnBnClickedCheckTimeUnit()
{
    // TODO:  在此添加控件通知处理程序代码
    BOOL bCheck = m_checkTimeUnit.GetCheck() == BST_CHECKED;
    m_staticTimeUnit.SetWindowText(bCheck ? _T("* 0.1ms") : _T("ms"));
}


void CnettcpDlg::OnBnClickedButtonCancelSingle()
{
    // TODO:  在此添加控件通知处理程序代码
    ZCAN_AUTO_TRANSMIT_OBJ      autoCAN;
    
    autoCAN.enable = 0;
    autoCAN.index = 0;
    autoCAN.interval = 1000;
    memset(&autoCAN.obj, 0, sizeof(autoCAN.obj));
    autoCAN.obj.frame.can_id = 1;
    autoCAN.obj.frame.can_dlc = 8;

    UINT nRet = ZCAN_SetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, SETREF_ADD_TIMER_SEND_CAN, &autoCAN);
    if (!nRet)
    {
        CString csText;
        csText.Format(_T("CAN定时发送设置失败, Index:%d"), 0);
        AddData(csText);
    }
}


void CnettcpDlg::OnBnClickedCheckLinMaster()
{
    // TODO:  在此添加控件通知处理程序代码
    lin_mode_master_ = m_checkLINMaster.GetCheck() == BST_CHECKED;
    m_checkLINMaster.SetWindowText(lin_mode_master_ ? _T("Master") : _T("Slave"));
}


void CnettcpDlg::OnCbnSelchangeComboLinVersion()
{
    // TODO:  在此添加控件通知处理程序代码
    int nVersion = m_comboLINVersion.GetCurSel();
    m_checkLINEnhanceChksum.SetCheck(nVersion !=0 ? BST_CHECKED : BST_UNCHECKED);
    m_checkLINVarDLC.SetCheck(nVersion == 0 ? BST_CHECKED : BST_UNCHECKED);
}


void CnettcpDlg::OnBnClickedButtonInitLin()
{
    // TODO:  在此添加控件通知处理程序代码
    if (!device_opened_)
    {
        AddData(_T("设备还没打开!"));
        return;
    }
    UpdateData(TRUE);
    property_ = GetIProperty(device_handle_);
    //ASSERT(property_ != NULL);

    if (device_tcp_)
    {
        if (!SetWorkMode())
        {
            AddData(_T("设置工作模式失败!"));
            return;
        }

        if (0 == work_mode_index_)//client
        {
            if (!SetClientParam())
            {
                AddData(_T("设置客户端参数失败!"));
                return;
            }
        }
        else if (!SetServerParam())
        {
            AddData(_T("设置服务端参数失败!"));
            return;
        }
    }
    else
    {
        if (!SetClientParam())
        {
            AddData(_T("设置UDP参数失败!"));
            return;
        }
    }

    ZCAN_LIN_INIT_CONFIG config;
    memset(&config, 0, sizeof(config));
    lin_mode_master_ = m_checkLINMaster.GetCheck() == BST_CHECKED;
    config.linMode = (lin_mode_master_ ? LIN_MODE_MASTER : LIN_MODE_SLAVE);
    config.linBaud = m_comboLINBaud.GetItemData(m_comboLINBaud.GetCurSel());
    if (m_checkLINEnhanceChksum.GetCheck() == BST_CHECKED)
    {
        config.linFlag |= LIN_FLAG_CHK_ENHANCE;
    }
    if (m_checkLINVarDLC.GetCheck() == BST_CHECKED)
    {
        config.linFlag |= LIN_FLAG_VAR_DLC;
    }

    lin_channel_handle_ = ZCAN_InitLIN(device_handle_, channel_index_, &config);
    if (INVALID_CHANNEL_HANDLE == lin_channel_handle_)
    {
        AddData(_T("初始化LIN失败!"));
        return;
    }
    CString csText;
    csText.Format(_T("LIN IsMaster:%d Baud:%dbps EnhanceChk：%d, VarDLC:%d"), config.linMode == LIN_MODE_MASTER, config.linBaud,
        !!(config.linFlag & LIN_FLAG_CHK_ENHANCE), !!(config.linFlag & LIN_FLAG_VAR_DLC) );
    AddData(csText);

    ctrl_int_lin_.EnableWindow(FALSE);
}


void CnettcpDlg::OnBnClickedButtonStartLin()
{
    // TODO:  在此添加控件通知处理程序代码
    if (ZCAN_StartLIN(lin_channel_handle_) != STATUS_OK)
    {
        AddData(_T("启动LIN失败!"));
        return;
    }
    else
    {
        AddData(_T("启动LIN成功!"));
    }

    ctrl_start_lin_.EnableWindow(FALSE);
    if (!recv_thread_running_)
    {
        start_ = TRUE;
        _beginthreadex(NULL, 0, OnDataRecv, this, 0, NULL);
    }
}


void CnettcpDlg::OnBnClickedButtonResetLin()
{
    // TODO:  在此添加控件通知处理程序代码
    if (ZCAN_ResetLIN(lin_channel_handle_) != STATUS_OK)
    {
        AddData(_T("复位LIN失败!"));
        return;
    }
    else
    {
        AddData(_T("复位LIN成功!"));
    }
    ctrl_int_lin_.EnableWindow(TRUE);
    ctrl_start_lin_.EnableWindow(TRUE);
    start_ = FALSE;
}


void CnettcpDlg::OnBnClickedButtonSendLin()
{
    // TODO:  在此添加控件通知处理程序代码
    UpdateData(TRUE);
    UINT id = _tcstoul(id_, 0, 16);
    string data = Utility::W2AEx(datas_);
    ZCAN_LIN_MSG lin_data;
    memset(&lin_data, 0, sizeof(lin_data));
    lin_data.ID = id & 0xFF;
    lin_data.DataLen = Utility::split(lin_data.Data, 8, data, ' ', 16);

    UINT count_sent_total = 0;
    ZCAN_LIN_MSG* pData = nullptr;

    if (lin_mode_master_)
    {
        if (send_count_once_ >= 1)
        {
            pData = new ZCAN_LIN_MSG[send_count_once_];
            for (int i = 0; i < send_count_once_; i++)
            {
                pData[i] = lin_data;
            }
        }

        for (int j = 0; j < send_count_repeat_; j++)
        {
            UINT count_sent = ZCAN_TransmitLIN(lin_channel_handle_, pData, send_count_once_);
            count_sent_total += count_sent;
            if (count_sent != send_count_once_)
            {
                AddData(_T("LIN Master发送数据失败!"));
                break;
            }
        }

        if (count_sent_total == send_count_once_ * send_count_repeat_)
        {
            CString csText;
            csText.Format(_T("LIN Master发送数据成功: %d * %d = %d"), send_count_once_, send_count_repeat_, count_sent_total);
            AddData(csText);
        }
        else
        {
            CString csText;
            csText.Format(_T("LIN Master发送数据失败，已发送: %d"), count_sent_total);
            AddData(csText);
        }
    }
    else
    {
        //slave
        bool bSuccess = ZCAN_SetLINSlaveMsg(lin_channel_handle_, &lin_data, 1) == 1;
        CString csText;
        csText.Format(_T("LIN Slave 设置响应数据(ID:0x%02x Len:%d) %s!"), lin_data.ID, lin_data.DataLen, bSuccess ? _T("成功") : _T("失败"));
        AddData(csText);
    }

    if (pData)
    {
        delete[] pData;
        pData = nullptr;
    }
}


void CnettcpDlg::OnBnClickedButtonClearSlaveIdResponse()
{
    // TODO:  在此添加控件通知处理程序代码
    UpdateData(TRUE);
    UINT id = _tcstoul(id_, 0, 16);
    BYTE bID = id & 0xFF;
    bool bResult = (ZCAN_ClearLINSlaveMsg(lin_channel_handle_, &bID, 1) == 1);

    CString csText;
    csText.Format(_T("LIN Slave 清除ID:0x%02x 响应数据 %s!"), bID, bResult ? _T("成功") : _T("失败"));
    AddData(csText);
}


void CnettcpDlg::OnBnClickedButtonGetLinFifoInfo()
{
    // TODO:  在此添加控件通知处理程序代码
    UINT nCount = 0, nAvailable = 0;
    if (ZCAN_GetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, GETREF_GET_LIN_TX_FIFO_TOTAL, &nCount) &&
        ZCAN_GetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, GETREF_GET_LIN_TX_FIFO_AVAILABLE, &nAvailable))
    {
        CString csText;
        csText.Format(_T("LIN FIFO 大小:%d 可用:%d"), nCount, nAvailable);
        AddData(csText);
    }
    else
    {
        AddData(_T("获取LIN FIFO失败!"));
    }
}


void CnettcpDlg::OnBnClickedCheckRecvMerge()
{
    // TODO:  在此添加控件通知处理程序代码
    //设置合并接收
    UINT nRecvMerge = m_checkRecvMerge.GetCheck() == BST_CHECKED;
    ZCAN_SetReference(kDeviceType[device_type_index_].device_type, device_index_, channel_index_, SETREF_SET_DATA_RECV_MERGE, &nRecvMerge);
    m_bRecvMerge = nRecvMerge;
}


void CnettcpDlg::OnBnClickedButtonSendMerge()
{
    // TODO:  在此添加控件通知处理程序代码
    UpdateData(TRUE);
    if (datas_.IsEmpty())
    {
        AddData(_T("数据为空"));
        return;
    }
    ZCANDataObj dataObj;
    dataObj.chnl = channel_index_;
    dataObj.dataType = ZCAN_DT_ZCAN_CAN_CANFD_DATA;
    UINT id = _tcstoul(id_, 0, 16);
    string data = Utility::W2AEx(datas_);
    BOOL bDelaySend = m_checkDelaySend.GetCheck() == BST_CHECKED;
    BOOL bCheckTimeUnit = m_checkTimeUnit.GetCheck() == BST_CHECKED;


    ZCANCANFDData& can_data = dataObj.data.zcanCANFDData;
    memset(&can_data, 0, sizeof(can_data));
    can_data.frame.can_id = MAKE_CAN_ID(id, frame_type_index_, format_index_, 0);
    can_data.frame.len = Utility::split(can_data.frame.data, CAN_MAX_DLEN, data, ' ', 16);
    can_data.flag.unionVal.transmitType = send_type_index_;
    can_data.timeStamp = sendtime_;
    can_data.flag.unionVal.txDelay = bDelaySend ? (bCheckTimeUnit ? ZCAN_TX_DELAY_UNIT_100US : ZCAN_TX_DELAY_UNIT_MS) : (ZCAN_TX_DELAY_NO_DELAY);
    can_data.flag.unionVal.frameType = protocol_index_ != 0;
    can_data.flag.unionVal.txEchoRequest = 1;

    {
        UINT count_sent_total = 0;
        ZCANDataObj* pData = nullptr;
        if (send_count_once_ >= 1)
        {
            pData = new ZCANDataObj[send_count_once_];
            for (int i = 0; i < send_count_once_; i++)
            {
                pData[i] = dataObj;
            }
        }

        for (int j = 0; j < send_count_repeat_; j++)
        {
            UINT count_sent = ZCAN_TransmitData(device_handle_, pData, send_count_once_);
            count_sent_total += count_sent;
            if (count_sent != send_count_once_)
            {
                AddData(_T("发送数据失败!"));
                break;
            }
        }

        if (count_sent_total == send_count_once_ * send_count_repeat_)
        {
            CString csText;
            csText.Format(_T("发送数据成功: %d * %d = %d"), send_count_once_, send_count_repeat_, count_sent_total);
            AddData(csText);
        }
        else
        {
            CString csText;
            csText.Format(_T("发送数据失败，已发送: %d"), count_sent_total);
            AddData(csText);
        }

        if (pData)
        {
            delete[] pData;
            pData = nullptr;
        }
    }
}


void CnettcpDlg::OnBnClickedCheckRecvData()
{
    // TODO:  在此添加控件通知处理程序代码
    m_bRecvData = m_btnRecvData.GetCheck() == BST_CHECKED;
}
