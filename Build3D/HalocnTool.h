#pragma once
#include "HalconCpp.h"
#include <vector>

using namespace HalconCpp;
using namespace std;

class HalconTool
{
public:	
	HTuple hv_WindowHandle;
	HalconTool(void);
	~HalconTool(void);

	/************************************************
	*   名称：CloseCamera
	*   功能：将拍到的图像或者HObject类型的图案，显示在绑定的窗口上
	*	参数：
	ho_Image：显示的图像

	*	返回值：空
	* 	修改日期：2017年7月5日
	************************************************/
	void ShowImage(const HObject& ho_Image);

	/************************************************
	*   名称：SetWindow
	*   功能：设置窗口句柄为显示图像准备
	*	参数：
	pCWnd：窗口句柄数组

	*	返回值：空
	* 	修改日期：2017年7月5日
	************************************************/
	void SetWindow(CWnd * pCWnd);

	HTuple WindowHandle0, WindowHandle1;
	void SetWindow(CWnd * pCWnd0, CWnd * pCWnd1);
	void ShowLinearImage(const HObject& ho_Image);
};

