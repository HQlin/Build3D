#include "StdAfx.h"
#include "HalconAlgorithm.h"

HalconAlgorithm::HalconAlgorithm(void)
{
	isSetWCS = false;
}


HalconAlgorithm::~HalconAlgorithm(void)
{
}

bool HalconAlgorithm::Build3D(HObject ho_ImageL, HObject ho_ImageR, vector<HObject> &point3Ds, HObject &ho_ImageRectifiedL, HObject &ho_ImageRectifiedR)
{
	ReadCamPar("../Config/cam_left-125.dat", &hv_CamParamL);
	ReadCamPar("../Config/cam_right-125.dat", &hv_CamParamR);
	ReadPose("../Config/pos_right2left.dat", &hv_cLPcR);

	//根据参数相机校正
	GenBinocularRectificationMap(&ho_MapL, &ho_MapR, hv_CamParamL, hv_CamParamR, hv_cLPcR, 
      1, "geometric", "bilinear", &hv_RectCamParL, &hv_RectCamParR, 
      &hv_CamPoseRectL, &hv_CamPoseRectR, &hv_RectLPosRectR);

	//左右图像校正
	MapImage(ho_ImageL, ho_MapL, &ho_ImageRectifiedL);
	MapImage(ho_ImageR, ho_MapR, &ho_ImageRectifiedR);
	BinocularDisparityMg(ho_ImageRectifiedL, ho_ImageRectifiedR, &ho_Disparity, &ho_Score, 
        1, 30, 5, 0, "true", "default_parameters", "fast_accurate");

	//使用3D object方式，重建表面。并计算世界坐标（m）
    DisparityImageToXyz(ho_Disparity, &ho_X, &ho_Y, &ho_Z, hv_RectCamParL, hv_RectCamParR, 
        hv_RectLPosRectR);

	point3Ds.clear();
	point3Ds.push_back(ho_X);
	point3Ds.push_back(ho_Y);
	point3Ds.push_back(ho_Z);

	HObject ho_Rectangle;
	ShapeTrans(ho_ImageRectifiedL, &ho_Rectangle, "inner_rectangle1");
    ReduceDomain(ho_ImageRectifiedL, ho_Rectangle, &ho_ImageRectifiedL);
	ShapeTrans(ho_ImageRectifiedR, &ho_Rectangle, "inner_rectangle1");
    ReduceDomain(ho_ImageRectifiedR, ho_Rectangle, &ho_ImageRectifiedR);

	return true;
}

bool HalconAlgorithm::GetPoint3D(HTuple &hv_WindowHandleL, vector<HObject> &point3Ds, vector<double> &point3D)
{
	HTuple  hv_Row, hv_Column;
	HTuple  hv_XR, hv_YR, hv_ZR;
	DrawPoint(hv_WindowHandleL, &hv_Row, &hv_Column);
	
	GetGrayval(point3Ds[0], hv_Row, hv_Column, &hv_XR);
    GetGrayval(point3Ds[1], hv_Row, hv_Column, &hv_YR);
    GetGrayval(point3Ds[2], hv_Row, hv_Column, &hv_ZR);

	point3D.clear();
	point3D.push_back(hv_XR);
	point3D.push_back(hv_YR);
	point3D.push_back(hv_ZR);

	point3D.push_back(hv_Row);
	point3D.push_back(hv_Column);

	if(isSetWCS)
	{
		HomMat3dInvert(hv_HomMat3d_WCS_to_RectCCS, &hv_HomMat3d_RectCCS_to_WCS);
		AffineTransPoint3d(hv_HomMat3d_RectCCS_to_WCS, hv_XR, hv_YR, hv_ZR, &hv_X_WCS, 
			&hv_Y_WCS, &hv_Z_WCS);
		point3D.push_back(hv_X_WCS);
		point3D.push_back(hv_Y_WCS);
		point3D.push_back(hv_Z_WCS);
	}
	else
	{
		point3D.push_back(0);
		point3D.push_back(0);
		point3D.push_back(0);
	}
	return true;
}

bool HalconAlgorithm::SetWCS(HObject ho_ReferenceImage, HTuple &hv_Pose, vector<double> point3Ds)
{
	MapImage(ho_ReferenceImage, ho_MapL, &ho_ReferenceImageRectified);
    FindCaltab(ho_ReferenceImageRectified, &ho_Caltab, "../Config/caltab_30mm.descr", 3, 112, 5);
    FindMarksAndPose(ho_ReferenceImageRectified, ho_Caltab, "../Config/caltab_30mm.descr", 
        hv_RectCamParL, 128, 10, 18, 0.9, 15.0, 100.0, &hv_RCoord, &hv_CCoord, &hv_ReferencePose);
    //标定坐标系
    PoseToHomMat3d(hv_ReferencePose, &hv_HomMat3d_WCS_to_RectCCS);
    //Invert the transformation matrix
    HomMat3dInvert(hv_HomMat3d_WCS_to_RectCCS, &hv_HomMat3d_RectCCS_to_WCS);
    HomMat3dInvert(hv_HomMat3d_RectCCS_to_WCS, &hv_HomMat3d_WCS_to_RectCCS);
    HomMat3dToPose(hv_HomMat3d_WCS_to_RectCCS, &hv_Pose);
	
	if(-1 != point3Ds[0] && -1 != point3Ds[0] && -1 != point3Ds[0])
	{
		//标定坐标系平移旋转
		AffineTransPoint3d(hv_HomMat3d_RectCCS_to_WCS, point3Ds[0], point3Ds[1], point3Ds[2], &hv_X_WCS, &hv_Y_WCS, &hv_Z_WCS);
		PoseToHomMat3d(hv_Pose, &hv_HomMat3d_WCS_to_RectCCS);
		//Optionally: Adapt the transformation matrix
		HomMat3dTranslateLocal(hv_HomMat3d_WCS_to_RectCCS, hv_X_WCS, hv_Y_WCS, hv_Z_WCS, &hv_HomMat3d_WCS_to_RectCCS);
		//hom_mat3d_rotate_local (HomMat3d_WCS_to_RectCCS, rad(180), 'y', HomMat3d_WCS_to_RectCCS)
		//Invert the transformation matrix
		HomMat3dToPose(hv_HomMat3d_WCS_to_RectCCS, &hv_Pose);
		isSetWCS = true;
	}
	return true;
}



