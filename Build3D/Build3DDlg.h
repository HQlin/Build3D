
// Build3DDlg.h : 头文件
//

#pragma once
#include "HalocnTool.h"
#include "comaccess.h"
#include "HalconAlgorithm.h"
#include "HalconPublic.h"
#include "HalconCamera.h"

// CBuild3DDlg 对话框
class CBuild3DDlg : public CDialogEx
{
// 构造
public:
	CBuild3DDlg(CWnd* pParent = NULL);	// 标准构造函数

// 对话框数据
	enum { IDD = IDD_BUILD3D_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持
// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

public:
	HObject ho_ImageRectifiedL, ho_ImageRectifiedR;		//左右校正后图像
	HalconTool halconToolResult;						//结果工具对象
	HalconAlgorithm halconAlgorithm;					//算法对象
	vector<HObject> point3Ds;							//左图某点（三维信息，二维信息）
	vector<vector<double>> point0ABs;					//左图三点（三维信息，二维信息）
	int m_comNum;										//串口号
	ComAccess com;										//串口对象
	void SelectPoint(int IDC, CString btnOpenText, CString btnCloseText, int index);			//选择图像点	
	void UpdateDatas();				//更新界面数据
	void InitDatas();				//初始化界面数据
	void Build3D(HObject image_L, HObject image_R);					//双目三维重构
	CHalconCamera hcam_L;			//左相机
	CHalconCamera hcam_R;			//右相机
	HObject image_L,image_R;		//左原图，右原图
	HObject ho_ReferenceImage;		//标定图像
	HalconTool halconToolL,halconToolR;	//实时工具对象

public:
	afx_msg void OnBnClickedBtnComm();
	virtual void OnCancel();
	afx_msg void OnBnClickedBtnR();
	// AB距离
	CString m_textAB;
	// B点
	CString m_textB;
	// A点
	CString m_textA;
	// 原点
	CString m_textO;
	afx_msg void OnBnClickedBtnG();
	afx_msg void OnBnClickedBtnB();
	afx_msg void OnBnClickedBtnCam();
	afx_msg void OnBnClickedBtnGet();
	afx_msg void OnBnClickedBtnWcs();
	// 发送信息
	CString m_sendCStr;
	afx_msg void OnBnClickedBtnSend();
	afx_msg void OnBnClickedBtnReal();
	// 是否保存采图
	BOOL m_savePic;
	// 保存图像ID
	int m_picNum;
	// 显示信息
	CString m_MSG;
	// 采图是否三维重构
	BOOL m_build3D;
};
