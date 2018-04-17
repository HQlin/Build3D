
// Build3DDlg.h : ͷ�ļ�
//

#pragma once
#include "HalocnTool.h"
#include "comaccess.h"
#include "HalconAlgorithm.h"
#include "HalconPublic.h"
#include "HalconCamera.h"

// CBuild3DDlg �Ի���
class CBuild3DDlg : public CDialogEx
{
// ����
public:
	CBuild3DDlg(CWnd* pParent = NULL);	// ��׼���캯��

// �Ի�������
	enum { IDD = IDD_BUILD3D_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV ֧��
// ʵ��
protected:
	HICON m_hIcon;

	// ���ɵ���Ϣӳ�亯��
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

public:
	HObject ho_ImageRectifiedL, ho_ImageRectifiedR;		//����У����ͼ��
	HalconTool halconToolResult;						//������߶���
	HalconAlgorithm halconAlgorithm;					//�㷨����
	vector<HObject> point3Ds;							//��ͼĳ�㣨��ά��Ϣ����ά��Ϣ��
	vector<vector<double>> point0ABs;					//��ͼ���㣨��ά��Ϣ����ά��Ϣ��
	int m_comNum;										//���ں�
	ComAccess com;										//���ڶ���
	void SelectPoint(int IDC, CString btnOpenText, CString btnCloseText, int index);			//ѡ��ͼ���	
	void UpdateDatas();				//���½�������
	void InitDatas();				//��ʼ����������
	void Build3D(HObject image_L, HObject image_R);					//˫Ŀ��ά�ع�
	CHalconCamera hcam_L;			//�����
	CHalconCamera hcam_R;			//�����
	HObject image_L,image_R;		//��ԭͼ����ԭͼ
	HObject ho_ReferenceImage;		//�궨ͼ��
	HalconTool halconToolL,halconToolR;	//ʵʱ���߶���

public:
	afx_msg void OnBnClickedBtnComm();
	virtual void OnCancel();
	afx_msg void OnBnClickedBtnR();
	// AB����
	CString m_textAB;
	// B��
	CString m_textB;
	// A��
	CString m_textA;
	// ԭ��
	CString m_textO;
	afx_msg void OnBnClickedBtnG();
	afx_msg void OnBnClickedBtnB();
	afx_msg void OnBnClickedBtnCam();
	afx_msg void OnBnClickedBtnGet();
	afx_msg void OnBnClickedBtnWcs();
	// ������Ϣ
	CString m_sendCStr;
	afx_msg void OnBnClickedBtnSend();
	afx_msg void OnBnClickedBtnReal();
	// �Ƿ񱣴��ͼ
	BOOL m_savePic;
	// ����ͼ��ID
	int m_picNum;
	// ��ʾ��Ϣ
	CString m_MSG;
	// ��ͼ�Ƿ���ά�ع�
	BOOL m_build3D;
};
