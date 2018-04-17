#pragma once
#include "HalconCpp.h"
#include <vector>

using namespace HalconCpp;
using namespace std;

using namespace HalconCpp;

class HalconPublic
{
public:
	HalconPublic(void);
	~HalconPublic(void);

	//标出图像上的AB点
	static bool DispPoints(HTuple &hv_WindowHandleL, vector<vector<double>> &point3Ds);

	// Chapter: XLD / Creation
	// Short Description: Creates an arrow shaped XLD contour.
	static void gen_arrow_contour_xld (HObject *ho_Arrow, HTuple hv_Row1, HTuple hv_Column1, 
		HTuple hv_Row2, HTuple hv_Column2, HTuple hv_HeadLength, HTuple hv_HeadWidth);

	// Chapter: Graphics / Output
	// Short Description: Display the axes of a 3d coordinate system
	static void disp_3d_coord_system (HTuple hv_WindowHandle, HTuple hv_CamParam, HTuple hv_Pose, 
		HTuple hv_CoordAxesLength);
};

