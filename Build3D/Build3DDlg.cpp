
// Build3DDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "Build3D.h"
#include "Build3DDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

DWORD WINAPI ThreadReal(LPVOID pthread)
{
	CBuild3DDlg * m_pDlg = (CBuild3DDlg*)pthread;
	CString butCStr;
	do{
		m_pDlg->GetDlgItemTextW(IDC_BTN_REAL, butCStr);
		m_pDlg->halconToolL.ShowImage(m_pDlg->hcam_L.GetHImage());
		m_pDlg->halconToolR.ShowImage(m_pDlg->hcam_R.GetHImage());
	}while(L"实时：开" == butCStr);
	return 0;
}

// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
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

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CBuild3DDlg 对话框




CBuild3DDlg::CBuild3DDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CBuild3DDlg::IDD, pParent)
	, m_comNum(1)
	, m_textAB(_T(""))
	, m_textB(_T(""))
	, m_textA(_T(""))
	, m_textO(_T(""))
	, m_sendCStr(_T(""))
	, m_savePic(FALSE)
	, m_picNum(1)
	, m_MSG(_T(""))
	, m_build3D(FALSE)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CBuild3DDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT1, m_comNum);
	DDV_MinMaxInt(pDX, m_comNum, 1, 10);
	DDX_Text(pDX, IDC_TEXT_AB, m_textAB);
	DDX_Text(pDX, IDC_TEXT_B, m_textB);
	DDX_Text(pDX, IDC_TEXT_G, m_textA);
	DDX_Text(pDX, IDC_TEXT_R, m_textO);
	DDX_Text(pDX, IDC_EDIT_SEND, m_sendCStr);
	DDX_Check(pDX, IDC_SAVE_PIC, m_savePic);
	DDX_Text(pDX, IDC_PICNUM, m_picNum);
	DDX_Text(pDX, IDC_MSG, m_MSG);
	DDX_Check(pDX, IDC_BUILD3D, m_build3D);
}

BEGIN_MESSAGE_MAP(CBuild3DDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BTN_COMM, &CBuild3DDlg::OnBnClickedBtnComm)
	ON_BN_CLICKED(IDC_BTN_R, &CBuild3DDlg::OnBnClickedBtnR)
	ON_BN_CLICKED(IDC_BTN_G, &CBuild3DDlg::OnBnClickedBtnG)
	ON_BN_CLICKED(IDC_BTN_B, &CBuild3DDlg::OnBnClickedBtnB)
	ON_BN_CLICKED(IDC_BTN_CAM, &CBuild3DDlg::OnBnClickedBtnCam)
	ON_BN_CLICKED(IDC_BTN_GET, &CBuild3DDlg::OnBnClickedBtnGet)
	ON_BN_CLICKED(IDC_BTN_WCS, &CBuild3DDlg::OnBnClickedBtnWcs)
	ON_BN_CLICKED(IDC_BTN_SEND, &CBuild3DDlg::OnBnClickedBtnSend)
	ON_BN_CLICKED(IDC_BTN_REAL, &CBuild3DDlg::OnBnClickedBtnReal)
END_MESSAGE_MAP()


// CBuild3DDlg 消息处理程序

BOOL CBuild3DDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

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
	halconToolResult.SetWindow(GetDlgItem(IDC_ImageRectifiedL));
	halconToolL.SetWindow(GetDlgItem(IDC_ImageL));
	halconToolR.SetWindow(GetDlgItem(IDC_ImageR));
	InitDatas();
	GetDlgItem(IDC_BTN_R)->EnableWindow(FALSE);
	GetDlgItem(IDC_BTN_G)->EnableWindow(FALSE);
	GetDlgItem(IDC_BTN_B)->EnableWindow(FALSE);

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CBuild3DDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CBuild3DDlg::OnPaint()
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
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CBuild3DDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void CBuild3DDlg::OnBnClickedBtnComm()
{
	//串口开关
	CString butCStr;
	GetDlgItemTextW(IDC_BTN_COMM, butCStr);
	if(L"关" == butCStr)
	{
		UpdateData(true);
		CString comCStr;
		comCStr.Format(L"COM%d", m_comNum);
		if ( ! com.Open(comCStr, 19200, 0, 0, 0) )
		{
			USES_CONVERSION; 
			CString str;
			str.Format(L"Error: Can't open communication device!\n%s", A2W(com.GetErrorMessage()));
			MessageBox(str);
			return;
		}
		SetDlgItemText(IDCANCEL,L"开");
	}
	else
	{
		com.Close();
		SetDlgItemText(IDCANCEL,L"关");
	}
	UpdateDatas();
}

void CBuild3DDlg::OnCancel()
{	
	CString butCStr;
	//关闭串口
	GetDlgItemTextW(IDCANCEL, butCStr);
	if(L"开" == butCStr)
	{
		OnBnClickedBtnComm();
	}
	//关闭程序
	CDialogEx::OnCancel();
}

void CBuild3DDlg::OnBnClickedBtnR()
{
	try{
		ReadImage(&ho_ReferenceImage, "../Image/Reference.png"); 
		InitDatas();
		SelectPoint(IDC_BTN_R, L"红：开", L"红：关", 0);
		HTuple  hv_Pose;
		halconAlgorithm.SetWCS(ho_ReferenceImage, hv_Pose, point0ABs[0]);
		HalconPublic::disp_3d_coord_system(halconToolResult.hv_WindowHandle, halconAlgorithm.hv_RectCamParL, hv_Pose, 0.02);
	}catch(...)
	{
		MessageBox(L"标定失败！");
	}
}

void CBuild3DDlg::OnBnClickedBtnG()
{
	try{
		SelectPoint(IDC_BTN_G, L"绿：开", L"绿：关", 1);
	}catch(...)
	{
		MessageBox(L"选A点失败！");
	}
}

void CBuild3DDlg::OnBnClickedBtnB()
{
	try{
		SelectPoint(IDC_BTN_B, L"蓝：开", L"蓝：关", 2);
	}catch(...)
	{
		MessageBox(L"选B点失败！");
	}
}

void CBuild3DDlg::SelectPoint(int IDC, CString btnOpenText, CString btnCloseText, int index)
{
	SetDlgItemText(IDC, btnOpenText);
	GetDlgItem(IDC_BTN_R)->EnableWindow(FALSE);
	GetDlgItem(IDC_BTN_G)->EnableWindow(FALSE);
	GetDlgItem(IDC_BTN_B)->EnableWindow(FALSE);

	//左图选点
	vector<double> point3D;
	SetColor(halconToolResult.hv_WindowHandle, "red");
	halconAlgorithm.GetPoint3D(halconToolResult.hv_WindowHandle, point3Ds, point3D);	

	SetDlgItemText(IDC, btnCloseText);

	point0ABs[index] = point3D;
	HalconPublic::DispPoints(halconToolResult.hv_WindowHandle, point0ABs);
	GetDlgItem(IDC_BTN_R)->EnableWindow(TRUE);
	GetDlgItem(IDC_BTN_G)->EnableWindow(TRUE);
	GetDlgItem(IDC_BTN_B)->EnableWindow(TRUE);
	UpdateDatas();
}

void CBuild3DDlg::UpdateDatas()
{
	int conversion = 1000;//单位m -> mm
	//m_textO.Format(L"O(%.0f, %.0f)[mm]：\n相机坐标(%.2f, %.2f, %.2f)\n标定坐标(%.2f, %.2f, %.2f)", point0ABs[0][3], point0ABs[0][4], 
	//	point0ABs[0][0]*conversion, point0ABs[0][1]*conversion, point0ABs[0][2]*conversion,
	//	point0ABs[0][5]*conversion, point0ABs[0][6]*conversion, point0ABs[0][7]*conversion);
	//m_textA.Format(L"A(%.0f, %.0f)[mm]：\n相机坐标(%.2f, %.2f, %.2f)\n标定坐标(%.2f, %.2f, %.2f)", point0ABs[1][3], point0ABs[1][4], 
	//	point0ABs[1][0]*conversion, point0ABs[1][1]*conversion, point0ABs[1][2]*conversion,
	//	point0ABs[1][5]*conversion, point0ABs[1][6]*conversion, point0ABs[1][7]*conversion);
	//m_textB.Format(L"B(%.0f, %.0f)[mm]：\n相机坐标(%.2f, %.2f, %.2f)\n标定坐标(%.2f, %.2f, %.2f)", point0ABs[2][3], point0ABs[2][4], 
	//	point0ABs[2][0]*conversion, point0ABs[2][1]*conversion, point0ABs[2][2]*conversion,
	//	point0ABs[2][5]*conversion, point0ABs[2][6]*conversion, point0ABs[2][7]*conversion);
	if(-1!=point0ABs[0][5] && -1!=point0ABs[0][6] && -1!=point0ABs[0][7])
		m_textO.Format(L"原点\nO(%.2f, %.2f, %.2f)", 
			point0ABs[0][5]*conversion, point0ABs[0][6]*conversion, point0ABs[0][7]*conversion);
	else
		m_textO.Format(L"原点\n");
	if(-1!=point0ABs[1][5] && -1!=point0ABs[1][6] && -1!=point0ABs[1][7])
		m_textA.Format(L"A点\nA(%.2f, %.2f, %.2f)",
			point0ABs[1][5]*conversion, point0ABs[1][6]*conversion, point0ABs[1][7]*conversion);
	else
		m_textA.Format(L"A点\n");
	if(-1!=point0ABs[2][5] && -1!=point0ABs[2][6] && -1!=point0ABs[2][7])
		m_textB.Format(L"B点\nB(%.2f, %.2f, %.2f)",
			point0ABs[2][5]*conversion, point0ABs[2][6]*conversion, point0ABs[2][7]*conversion);
	else
		m_textB.Format(L"B点\n");
	double dis = sqrt((point0ABs[1][0]-point0ABs[2][0])*(point0ABs[1][0]-point0ABs[2][0])
		+ (point0ABs[1][1]-point0ABs[2][1])*(point0ABs[1][1]-point0ABs[2][1]) 
		+ (point0ABs[1][2]-point0ABs[2][2])*(point0ABs[1][2]-point0ABs[2][2]));
	m_textAB.Format(L"AB距离[mm]：%.4f", dis*conversion);
	m_sendCStr.Format(L"A(%.2f, %.2f, %.2f);B(%.2f, %.2f, %.2f)", 
		point0ABs[1][5]*conversion, point0ABs[1][6]*conversion, point0ABs[1][7]*conversion,
		point0ABs[2][5]*conversion, point0ABs[2][6]*conversion, point0ABs[2][7]*conversion);

	
	GetDlgItem(IDC_BTN_REAL)->EnableWindow(TRUE);

	CString butCStr;
	//判断实时状态
	GetDlgItemTextW(IDC_BTN_REAL, butCStr);
	if(L"实时：关" == butCStr)
	{
		GetDlgItem(IDC_BTN_WCS)->EnableWindow(TRUE);	
		GetDlgItem(IDC_BTN_GET)->EnableWindow(TRUE);	
		GetDlgItem(IDC_BTN_CAM)->EnableWindow(TRUE);		
	}
	else
	{
		GetDlgItem(IDC_BTN_WCS)->EnableWindow(FALSE);	
		GetDlgItem(IDC_BTN_GET)->EnableWindow(FALSE);	
		GetDlgItem(IDC_BTN_CAM)->EnableWindow(FALSE);	
	}

	//判断相机开关状态
	GetDlgItemTextW(IDC_BTN_CAM, butCStr);
	if(L"关" == butCStr)
	{
		GetDlgItem(IDC_BTN_WCS)->EnableWindow(FALSE);	
		GetDlgItem(IDC_BTN_REAL)->EnableWindow(FALSE);		
	}
	else
	{	
		GetDlgItem(IDC_BTN_WCS)->EnableWindow(TRUE);	
		GetDlgItem(IDC_BTN_REAL)->EnableWindow(TRUE);	
	}

	//判断串口开关状态
	GetDlgItemTextW(IDC_BTN_COMM, butCStr);
	if(L"关" == butCStr)
	{
		GetDlgItem(IDC_BTN_SEND)->EnableWindow(FALSE);	
	}
	else
	{
		GetDlgItem(IDC_BTN_SEND)->EnableWindow(TRUE);		
	}
	//三维重构成功判断
	GetDlgItemTextW(IDC_MSG, butCStr);
	if(butCStr.Find(L"成功") > 0)
	{
		GetDlgItem(IDC_BTN_R)->EnableWindow(TRUE);
		GetDlgItem(IDC_BTN_G)->EnableWindow(TRUE);
		GetDlgItem(IDC_BTN_B)->EnableWindow(TRUE);
	}
	UpdateData(false);	
}

void CBuild3DDlg::InitDatas()
{
	halconToolResult.ShowImage(ho_ImageRectifiedL);
	point0ABs.clear();
	vector<double> point3D;
	point3D.push_back(-1);
	point3D.push_back(-1);
	point3D.push_back(-1);
	point3D.push_back(-1);
	point3D.push_back(-1);
	point3D.push_back(-1);
	point3D.push_back(-1);
	point3D.push_back(-1);
	point0ABs.push_back(point3D);
	point0ABs.push_back(point3D);
	point0ABs.push_back(point3D);

	UpdateDatas();
}

void CBuild3DDlg::Build3D(HObject image_L, HObject image_R)
{
	halconAlgorithm.Build3D(image_L, image_R, point3Ds, ho_ImageRectifiedL, ho_ImageRectifiedR);
	halconToolResult.ShowImage(ho_ImageRectifiedL);
}

void CBuild3DDlg::OnBnClickedBtnCam()
{
	CString butCStr;
	GetDlgItemTextW(IDC_BTN_CAM, butCStr);
	if(L"关" == butCStr)
	{
		char* idL = "000A4701120B1E35";
		
		if(!hcam_L.OpenCamera(idL))
		{
			MessageBox(L"左相机打开失败！");
		}
		char* idR = "000A4701120B3687";
		if(!hcam_R.OpenCamera(idR))
		{
			MessageBox(L"右相机打开失败！");
		}

		if(hcam_L.isOpen && hcam_R.isOpen)
		{
			SetDlgItemText(IDC_BTN_CAM, L"开");				
		}
	}
	else
	{
		hcam_L.CloseCamera();
		hcam_R.CloseCamera();
		SetDlgItemText(IDC_BTN_CAM, L"关");			
	}
	UpdateDatas();
}

void CBuild3DDlg::OnBnClickedBtnGet()
{
	UpdateData(TRUE);
	CString butCStr;
	GetDlgItemTextW(IDC_BTN_CAM, butCStr);
	if(L"关" == butCStr)
	{
		//跑图
		ReadImage(&image_L, "../标定文件/cal_image/Left/1.png"); 
		ReadImage(&image_R, "../标定文件/cal_image/Right/1.png");
	}
	else
	{		
		//采图
		image_L = hcam_L.GetHImage();
		image_R = hcam_R.GetHImage();
		if(m_savePic)
		{
			USES_CONVERSION; 
			butCStr.Format(L"../Image/Left/%d", m_picNum);
			WriteImage(image_L, "bmp", 0,  W2A(butCStr));
			butCStr.Format(L"../Image/Right/%d", m_picNum);
			WriteImage(image_R, "bmp", 0,  W2A(butCStr));
			m_picNum++;
			butCStr.Format(L"%d", m_picNum);
			SetDlgItemText(IDC_PICNUM, butCStr);
		}
	}
	if(m_build3D)
	{
		try
		{
			long start = GetTickCount();
			m_MSG.Format(L"三维重构中..");
			UpdateData(FALSE);
			Build3D(image_L, image_R);
			long end = GetTickCount();
			m_MSG.Format(L"三维重构成功！耗时：%d ms", end-start);
		}catch(...)
		{
			m_MSG.Format(L"三维重构失败！");
		}		
		UpdateData(FALSE);
	}
	halconToolL.ShowImage(image_L);
	halconToolR.ShowImage(image_R);
	UpdateDatas();
}

void CBuild3DDlg::OnBnClickedBtnWcs()
{
	try{
		WriteImage(image_L, "bmp", 0, "../Image/Reference");
	}catch(...)
	{
		MessageBox(L"保存标定图像失败！");
	}
}

void CBuild3DDlg::OnBnClickedBtnSend()
{
	UpdateData(TRUE);
	//发送给串口
	CString butCStr;
	GetDlgItemTextW(IDC_BTN_COMM, butCStr);
	if(L"开" == butCStr)
	{
		USES_CONVERSION; 
		char* str = W2A(m_sendCStr);
		int write_result = com.WriteData(str, strlen(str));

		// -1 ? then we got an error and print it
		if ( write_result < 0 )
			MessageBox(A2W(com.GetErrorMessage()));

		//接收串口回馈信息
		int DATA_LEN = 10;
		char buf[255];
		int read_result = com.ReadData(buf, DATA_LEN);

		// -1 ? then we got an error and print it
		if ( (read_result < 0)  )
			MessageBox(A2W(com.GetErrorMessage()));

		// set end of received data
		buf[DATA_LEN] = '\0';

		CString dataRecv =  A2W(buf);
	}
	else
	{
		MessageBox(L"串口未打开！");
	}
}

void CBuild3DDlg::OnBnClickedBtnReal()
{
	CString butCStr;
	GetDlgItemTextW(IDC_BTN_REAL, butCStr);
	if(L"实时：关" == butCStr)
	{
		CreateThread(NULL,0,ThreadReal,this,0,NULL);
		SetDlgItemText(IDC_BTN_REAL, L"实时：开");
	}
	else
	{
		SetDlgItemText(IDC_BTN_REAL, L"实时：关");
	}
	UpdateDatas();
}
