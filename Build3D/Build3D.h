
// Build3D.h : PROJECT_NAME Ӧ�ó������ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"		// ������


// CBuild3DApp:
// �йش����ʵ�֣������ Build3D.cpp
//

class CBuild3DApp : public CWinApp
{
public:
	CBuild3DApp();

// ��д
public:
	virtual BOOL InitInstance();

// ʵ��

	DECLARE_MESSAGE_MAP()
};

extern CBuild3DApp theApp;