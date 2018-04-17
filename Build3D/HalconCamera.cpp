#include "StdAfx.h"
#include "HalconCamera.h"


CHalconCamera::CHalconCamera(void)
{
	isOpen = false;
}


CHalconCamera::~CHalconCamera(void)
{
	CloseCamera();
}

bool CHalconCamera::OpenCamera(char* id)
{
	try
	{
		OpenFramegrabber("1394IIDC", 1, 1, 0, 0, 0, 0, "progressive", -1, "default", 
			-1, "false", "default", id, 0, -1, &hv_AcqHandle);
		GrabImage(&ho_Image, hv_AcqHandle);
	}catch( HException& except){
		CString ProcName(except.ProcName().Text());
		CString ErrorText(except.ErrorText().Text());
		CString currentexpection = ProcName + _T("\n") + ErrorText;
		AfxMessageBox(currentexpection);
		return false;
	}
	isOpen = true;
	return true;
}

bool CHalconCamera::CloseCamera()
{
	if(isOpen)
		CloseFramegrabber(hv_AcqHandle);
	isOpen = false;
	return true;
}

HObject CHalconCamera::GetHImage()
{
	try{
		GrabImage(&ho_Image, hv_AcqHandle);
	}
	catch( HException& except){ 
		CString ProcName(except.ProcName().Text());
		CString ErrorText(except.ErrorText().Text());
		CString currentexpection = ProcName + _T("\n") + ErrorText;
		AfxMessageBox(currentexpection);
	}
	return ho_Image;
}
