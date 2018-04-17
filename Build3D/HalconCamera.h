/*
	//Halcon²âÊÔ
	CHalconCamera hcam;
	hcam.OpenCamera();
	hcam.SetWindow(GetDlgItem(IDC_PIC));
	hcam.SetWindow(GetDlgItem(IDC_PIC0), GetDlgItem(IDC_PIC1));
	HObject ho_Image = hcam.GetHImage();
	hcam.ShowImage(ho_Image);
*/
#pragma once
#include "HalconCpp.h"

using namespace HalconCpp;
class CHalconCamera
{
public:
	CHalconCamera(void);
	~CHalconCamera(void);

	HObject GetHImage();
	bool OpenCamera(char* id);
	bool CloseCamera();

	// Local iconic variables 
	HObject  ho_Image;
	// Local control variables 
	HTuple  hv_AcqHandle;
	//Ïà»ú×´Ì¬
	bool isOpen;
};

