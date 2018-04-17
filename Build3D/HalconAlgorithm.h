#pragma once
#include "HalconCpp.h"
#include <vector>

using namespace HalconCpp;
using namespace std;

class HalconAlgorithm
{
public:
	HalconAlgorithm(void);
	~HalconAlgorithm(void);

	//˫Ŀ��ά�ع�
	bool Build3D(HObject ho_ImageL, HObject ho_ImageR, vector<HObject> &point3Ds, HObject &ho_ImageRectifiedL, HObject &ho_ImageRectifiedR);
	//��ȡͼ����ĳ����������
	bool GetPoint3D(HTuple &hv_WindowHandleL, vector<HObject> &point3Ds, vector<double> &point3D);
	//ȷ���궨����ϵ
	bool SetWCS(HObject ho_ImageL, HTuple &hv_Pose, vector<double> point3Ds);

	// Local control variables 
	HTuple  hv_CamParamL, hv_CamParamR, hv_cLPcR;
	HObject	ho_MapL, ho_MapR;
	HTuple  hv_RectCamParL, hv_RectCamParR, hv_CamPoseRectL, hv_CamPoseRectR, hv_RectLPosRectR;
	HObject ho_Disparity, ho_Score;
	HObject ho_X, ho_Y, ho_Z;

	HObject  ho_ReferenceImageRectified,ho_Caltab;
	HTuple   hv_RCoord,hv_CCoord,hv_ReferencePose;
	HTuple   hv_HomMat3d_WCS_to_RectCCS, hv_HomMat3d_RectCCS_to_WCS;
	HTuple	 hv_X_WCS, hv_Y_WCS, hv_Z_WCS;

	bool isSetWCS;//�궨����ϵԭ���Ƿ�ȷ��
};

