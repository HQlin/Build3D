#include "StdAfx.h"
#include "HalocnTool.h"

HalconTool::HalconTool(void)
{
}

HalconTool::~HalconTool(void)
{
}

void HalconTool::ShowImage(const HObject& ho_Image){ 
	try{
		try{
			//图片长宽
			HTuple width, height;
			GetImageSize(ho_Image, &height, &width); 
			SetPart(hv_WindowHandle, 0, 0, width, height);
		}catch(...)
		{
		}
		DispObj(ho_Image, hv_WindowHandle);	
	}catch(...)
	{
	}
}

void HalconTool::SetWindow(CWnd * pCWnd){ 
	//定义显示的起点和长宽高      
	HTuple HWindowRow,HWindowColumn,HWindowWidth,HWindowHeight;     
	//定义窗口ID     
	HTuple HWindowID; 

	CRect rect;  
	pCWnd->GetClientRect(&rect);
	HWindowRow = rect.top;
	HWindowColumn = rect.left;
	HWindowWidth = rect.Width();
	HWindowHeight = rect.Height();
	HWindowID = (Hlong)pCWnd->m_hWnd;
	SetWindowAttr("background_color","black");
	SetCheck("~father");	
	OpenWindow(HWindowRow,HWindowColumn,HWindowWidth,HWindowHeight,HWindowID,"visible","",&hv_WindowHandle);
	SetCheck("father");
	SetDraw(hv_WindowHandle,"margin");
}

void HalconTool::SetWindow(CWnd * pCWnd0, CWnd * pCWnd1)
{	
	CRect rect0;  	
	//定义显示的起点和长宽高      
	HTuple HWindowRow0,HWindowColumn0,HWindowWidth0,HWindowHeight0;     
	//定义窗口ID     
	HTuple HWindowID0; 
	pCWnd0->GetClientRect(&rect0);
	HWindowRow0 = rect0.top;
	HWindowColumn0 = rect0.left;
	HWindowWidth0 = rect0.Width();
	HWindowHeight0 = rect0.Height();
	HWindowID0 = (Hlong)pCWnd0->m_hWnd;

	CRect rect1; 
	HTuple HWindowRow1,HWindowColumn1,HWindowWidth1,HWindowHeight1;         
	HTuple HWindowID1;
	pCWnd1->GetClientRect(&rect1);
	HWindowRow1 = rect1.top;
	HWindowColumn1 = rect1.left;
	HWindowWidth1 = rect1.Width();
	HWindowHeight1 = rect1.Height();
	HWindowID1 = (Hlong)pCWnd1->m_hWnd;

	SetWindowAttr("background_color","white");
	SetCheck("~father");	

	OpenWindow(HWindowRow0,HWindowColumn0,HWindowWidth0,HWindowHeight0,HWindowID0,"visible","",&WindowHandle0);
	OpenWindow(HWindowRow1,HWindowColumn1,HWindowWidth1,HWindowHeight1,HWindowID1,"visible","",&WindowHandle1);

	SetCheck("father");

	SetDraw(WindowHandle0,"margin");
	SetDraw(WindowHandle1,"margin");
}

void HalconTool::ShowLinearImage(const HObject& ho_Image){ 
	try{
		//图片长宽
		HTuple width, height;
		GetImageSize(ho_Image, &height, &width); 

		//分图显示
		HObject ho_ImageRotate;
		RotateImage(ho_Image, &ho_ImageRotate,90, "nearest_neighbor");
		GetImageSize(ho_ImageRotate, &height, &width); 
		SetPart(WindowHandle0,0, 0, width, height/2);
		SetPart(WindowHandle1,0, height/2, width, height);
        DispObj(ho_ImageRotate, WindowHandle0);
        DispObj(ho_ImageRotate, WindowHandle1);
	}catch( HException& except){
		CString ProcName(except.ProcName().Text());
		CString ErrorText(except.ErrorText().Text());
		TRACE(L"lin: " + ProcName + L", " + ErrorText);		
	}
}
