
// Build3DDlg.cpp : ʵ���ļ�
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
	}while(L"ʵʱ����" == butCStr);
	return 0;
}

// ����Ӧ�ó��򡰹��ڡ��˵���� CAboutDlg �Ի���

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// �Ի�������
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

// ʵ��
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


// CBuild3DDlg �Ի���




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


// CBuild3DDlg ��Ϣ�������

BOOL CBuild3DDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// ��������...���˵�����ӵ�ϵͳ�˵��С�

	// IDM_ABOUTBOX ������ϵͳ���Χ�ڡ�
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

	// ���ô˶Ի����ͼ�ꡣ��Ӧ�ó��������ڲ��ǶԻ���ʱ����ܽ��Զ�
	//  ִ�д˲���
	SetIcon(m_hIcon, TRUE);			// ���ô�ͼ��
	SetIcon(m_hIcon, FALSE);		// ����Сͼ��

	// TODO: �ڴ���Ӷ���ĳ�ʼ������
	halconToolResult.SetWindow(GetDlgItem(IDC_ImageRectifiedL));
	halconToolL.SetWindow(GetDlgItem(IDC_ImageL));
	halconToolR.SetWindow(GetDlgItem(IDC_ImageR));
	InitDatas();
	GetDlgItem(IDC_BTN_R)->EnableWindow(FALSE);
	GetDlgItem(IDC_BTN_G)->EnableWindow(FALSE);
	GetDlgItem(IDC_BTN_B)->EnableWindow(FALSE);

	return TRUE;  // ���ǽ��������õ��ؼ������򷵻� TRUE
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

// �����Ի��������С����ť������Ҫ����Ĵ���
//  �����Ƹ�ͼ�ꡣ����ʹ���ĵ�/��ͼģ�͵� MFC Ӧ�ó���
//  �⽫�ɿ���Զ���ɡ�

void CBuild3DDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // ���ڻ��Ƶ��豸������

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// ʹͼ���ڹ����������о���
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// ����ͼ��
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//���û��϶���С������ʱϵͳ���ô˺���ȡ�ù��
//��ʾ��
HCURSOR CBuild3DDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void CBuild3DDlg::OnBnClickedBtnComm()
{
	//���ڿ���
	CString butCStr;
	GetDlgItemTextW(IDC_BTN_COMM, butCStr);
	if(L"��" == butCStr)
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
		SetDlgItemText(IDCANCEL,L"��");
	}
	else
	{
		com.Close();
		SetDlgItemText(IDCANCEL,L"��");
	}
	UpdateDatas();
}

void CBuild3DDlg::OnCancel()
{	
	CString butCStr;
	//�رմ���
	GetDlgItemTextW(IDCANCEL, butCStr);
	if(L"��" == butCStr)
	{
		OnBnClickedBtnComm();
	}
	//�رճ���
	CDialogEx::OnCancel();
}

void CBuild3DDlg::OnBnClickedBtnR()
{
	try{
		ReadImage(&ho_ReferenceImage, "../Image/Reference.png"); 
		InitDatas();
		SelectPoint(IDC_BTN_R, L"�죺��", L"�죺��", 0);
		HTuple  hv_Pose;
		halconAlgorithm.SetWCS(ho_ReferenceImage, hv_Pose, point0ABs[0]);
		HalconPublic::disp_3d_coord_system(halconToolResult.hv_WindowHandle, halconAlgorithm.hv_RectCamParL, hv_Pose, 0.02);
	}catch(...)
	{
		MessageBox(L"�궨ʧ�ܣ�");
	}
}

void CBuild3DDlg::OnBnClickedBtnG()
{
	try{
		SelectPoint(IDC_BTN_G, L"�̣���", L"�̣���", 1);
	}catch(...)
	{
		MessageBox(L"ѡA��ʧ�ܣ�");
	}
}

void CBuild3DDlg::OnBnClickedBtnB()
{
	try{
		SelectPoint(IDC_BTN_B, L"������", L"������", 2);
	}catch(...)
	{
		MessageBox(L"ѡB��ʧ�ܣ�");
	}
}

void CBuild3DDlg::SelectPoint(int IDC, CString btnOpenText, CString btnCloseText, int index)
{
	SetDlgItemText(IDC, btnOpenText);
	GetDlgItem(IDC_BTN_R)->EnableWindow(FALSE);
	GetDlgItem(IDC_BTN_G)->EnableWindow(FALSE);
	GetDlgItem(IDC_BTN_B)->EnableWindow(FALSE);

	//��ͼѡ��
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
	int conversion = 1000;//��λm -> mm
	//m_textO.Format(L"O(%.0f, %.0f)[mm]��\n�������(%.2f, %.2f, %.2f)\n�궨����(%.2f, %.2f, %.2f)", point0ABs[0][3], point0ABs[0][4], 
	//	point0ABs[0][0]*conversion, point0ABs[0][1]*conversion, point0ABs[0][2]*conversion,
	//	point0ABs[0][5]*conversion, point0ABs[0][6]*conversion, point0ABs[0][7]*conversion);
	//m_textA.Format(L"A(%.0f, %.0f)[mm]��\n�������(%.2f, %.2f, %.2f)\n�궨����(%.2f, %.2f, %.2f)", point0ABs[1][3], point0ABs[1][4], 
	//	point0ABs[1][0]*conversion, point0ABs[1][1]*conversion, point0ABs[1][2]*conversion,
	//	point0ABs[1][5]*conversion, point0ABs[1][6]*conversion, point0ABs[1][7]*conversion);
	//m_textB.Format(L"B(%.0f, %.0f)[mm]��\n�������(%.2f, %.2f, %.2f)\n�궨����(%.2f, %.2f, %.2f)", point0ABs[2][3], point0ABs[2][4], 
	//	point0ABs[2][0]*conversion, point0ABs[2][1]*conversion, point0ABs[2][2]*conversion,
	//	point0ABs[2][5]*conversion, point0ABs[2][6]*conversion, point0ABs[2][7]*conversion);
	if(-1!=point0ABs[0][5] && -1!=point0ABs[0][6] && -1!=point0ABs[0][7])
		m_textO.Format(L"ԭ��\nO(%.2f, %.2f, %.2f)", 
			point0ABs[0][5]*conversion, point0ABs[0][6]*conversion, point0ABs[0][7]*conversion);
	else
		m_textO.Format(L"ԭ��\n");
	if(-1!=point0ABs[1][5] && -1!=point0ABs[1][6] && -1!=point0ABs[1][7])
		m_textA.Format(L"A��\nA(%.2f, %.2f, %.2f)",
			point0ABs[1][5]*conversion, point0ABs[1][6]*conversion, point0ABs[1][7]*conversion);
	else
		m_textA.Format(L"A��\n");
	if(-1!=point0ABs[2][5] && -1!=point0ABs[2][6] && -1!=point0ABs[2][7])
		m_textB.Format(L"B��\nB(%.2f, %.2f, %.2f)",
			point0ABs[2][5]*conversion, point0ABs[2][6]*conversion, point0ABs[2][7]*conversion);
	else
		m_textB.Format(L"B��\n");
	double dis = sqrt((point0ABs[1][0]-point0ABs[2][0])*(point0ABs[1][0]-point0ABs[2][0])
		+ (point0ABs[1][1]-point0ABs[2][1])*(point0ABs[1][1]-point0ABs[2][1]) 
		+ (point0ABs[1][2]-point0ABs[2][2])*(point0ABs[1][2]-point0ABs[2][2]));
	m_textAB.Format(L"AB����[mm]��%.4f", dis*conversion);
	m_sendCStr.Format(L"A(%.2f, %.2f, %.2f);B(%.2f, %.2f, %.2f)", 
		point0ABs[1][5]*conversion, point0ABs[1][6]*conversion, point0ABs[1][7]*conversion,
		point0ABs[2][5]*conversion, point0ABs[2][6]*conversion, point0ABs[2][7]*conversion);

	
	GetDlgItem(IDC_BTN_REAL)->EnableWindow(TRUE);

	CString butCStr;
	//�ж�ʵʱ״̬
	GetDlgItemTextW(IDC_BTN_REAL, butCStr);
	if(L"ʵʱ����" == butCStr)
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

	//�ж��������״̬
	GetDlgItemTextW(IDC_BTN_CAM, butCStr);
	if(L"��" == butCStr)
	{
		GetDlgItem(IDC_BTN_WCS)->EnableWindow(FALSE);	
		GetDlgItem(IDC_BTN_REAL)->EnableWindow(FALSE);		
	}
	else
	{	
		GetDlgItem(IDC_BTN_WCS)->EnableWindow(TRUE);	
		GetDlgItem(IDC_BTN_REAL)->EnableWindow(TRUE);	
	}

	//�жϴ��ڿ���״̬
	GetDlgItemTextW(IDC_BTN_COMM, butCStr);
	if(L"��" == butCStr)
	{
		GetDlgItem(IDC_BTN_SEND)->EnableWindow(FALSE);	
	}
	else
	{
		GetDlgItem(IDC_BTN_SEND)->EnableWindow(TRUE);		
	}
	//��ά�ع��ɹ��ж�
	GetDlgItemTextW(IDC_MSG, butCStr);
	if(butCStr.Find(L"�ɹ�") > 0)
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
	if(L"��" == butCStr)
	{
		char* idL = "000A4701120B1E35";
		
		if(!hcam_L.OpenCamera(idL))
		{
			MessageBox(L"�������ʧ�ܣ�");
		}
		char* idR = "000A4701120B3687";
		if(!hcam_R.OpenCamera(idR))
		{
			MessageBox(L"�������ʧ�ܣ�");
		}

		if(hcam_L.isOpen && hcam_R.isOpen)
		{
			SetDlgItemText(IDC_BTN_CAM, L"��");				
		}
	}
	else
	{
		hcam_L.CloseCamera();
		hcam_R.CloseCamera();
		SetDlgItemText(IDC_BTN_CAM, L"��");			
	}
	UpdateDatas();
}

void CBuild3DDlg::OnBnClickedBtnGet()
{
	UpdateData(TRUE);
	CString butCStr;
	GetDlgItemTextW(IDC_BTN_CAM, butCStr);
	if(L"��" == butCStr)
	{
		//��ͼ
		ReadImage(&image_L, "../�궨�ļ�/cal_image/Left/1.png"); 
		ReadImage(&image_R, "../�궨�ļ�/cal_image/Right/1.png");
	}
	else
	{		
		//��ͼ
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
			m_MSG.Format(L"��ά�ع���..");
			UpdateData(FALSE);
			Build3D(image_L, image_R);
			long end = GetTickCount();
			m_MSG.Format(L"��ά�ع��ɹ�����ʱ��%d ms", end-start);
		}catch(...)
		{
			m_MSG.Format(L"��ά�ع�ʧ�ܣ�");
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
		MessageBox(L"����궨ͼ��ʧ�ܣ�");
	}
}

void CBuild3DDlg::OnBnClickedBtnSend()
{
	UpdateData(TRUE);
	//���͸�����
	CString butCStr;
	GetDlgItemTextW(IDC_BTN_COMM, butCStr);
	if(L"��" == butCStr)
	{
		USES_CONVERSION; 
		char* str = W2A(m_sendCStr);
		int write_result = com.WriteData(str, strlen(str));

		// -1 ? then we got an error and print it
		if ( write_result < 0 )
			MessageBox(A2W(com.GetErrorMessage()));

		//���մ��ڻ�����Ϣ
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
		MessageBox(L"����δ�򿪣�");
	}
}

void CBuild3DDlg::OnBnClickedBtnReal()
{
	CString butCStr;
	GetDlgItemTextW(IDC_BTN_REAL, butCStr);
	if(L"ʵʱ����" == butCStr)
	{
		CreateThread(NULL,0,ThreadReal,this,0,NULL);
		SetDlgItemText(IDC_BTN_REAL, L"ʵʱ����");
	}
	else
	{
		SetDlgItemText(IDC_BTN_REAL, L"ʵʱ����");
	}
	UpdateDatas();
}
