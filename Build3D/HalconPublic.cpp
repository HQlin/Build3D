#include "StdAfx.h"
#include "HalconPublic.h"


HalconPublic::HalconPublic(void)
{
}


HalconPublic::~HalconPublic(void)
{
}

bool HalconPublic::DispPoints(HTuple &hv_WindowHandleL, vector<vector<double>> &point3Ds)
{
	SetColor(hv_WindowHandleL, "red");
	//DispCross(hv_WindowHandleL, point3Ds[0][3], point3Ds[0][4], 50, 90);
	SetColor(hv_WindowHandleL, "green");
	if(point3Ds[1][3]>0 && point3Ds[1][4]>0)
		DispCross(hv_WindowHandleL, point3Ds[1][3], point3Ds[1][4], 50, 90);
	SetColor(hv_WindowHandleL, "blue");
	if(point3Ds[2][3]>0 && point3Ds[2][4]>0)
		DispCross(hv_WindowHandleL, point3Ds[2][3], point3Ds[2][4], 50, 90);
	return true;
}


// Chapter: XLD / Creation
// Short Description: Creates an arrow shaped XLD contour.
void HalconPublic::gen_arrow_contour_xld (HObject *ho_Arrow, HTuple hv_Row1, HTuple hv_Column1, 
    HTuple hv_Row2, HTuple hv_Column2, HTuple hv_HeadLength, HTuple hv_HeadWidth)
{

  // Local iconic variables 
  HObject  ho_TempArrow;


  // Local control variables 
  HTuple  hv_Length, hv_ZeroLengthIndices, hv_DR;
  HTuple  hv_DC, hv_HalfHeadWidth, hv_RowP1, hv_ColP1, hv_RowP2;
  HTuple  hv_ColP2, hv_Index;

  //This procedure generates arrow shaped XLD contours,
  //pointing from (Row1, Column1) to (Row2, Column2).
  //If starting and end point are identical, a contour consisting
  //of a single point is returned.
  //
  //input parameteres:
  //Row1, Column1: Coordinates of the arrows' starting points
  //Row2, Column2: Coordinates of the arrows' end points
  //HeadLength, HeadWidth: Size of the arrow heads in pixels
  //
  //output parameter:
  //Arrow: The resulting XLD contour
  //
  //The input tuples Row1, Column1, Row2, and Column2 have to be of
  //the same length.
  //HeadLength and HeadWidth either have to be of the same length as
  //Row1, Column1, Row2, and Column2 or have to be a single element.
  //If one of the above restrictions is violated, an error will occur.
  //
  //
  //Init
  GenEmptyObj(&(*ho_Arrow));
  //
  //Calculate the arrow length
  DistancePp(hv_Row1, hv_Column1, hv_Row2, hv_Column2, &hv_Length);
  //
  //Mark arrows with identical start and end point
  //(set Length to -1 to avoid division-by-zero exception)
  hv_ZeroLengthIndices = hv_Length.TupleFind(0);
  if (0 != (hv_ZeroLengthIndices!=-1))
  {
    hv_Length[hv_ZeroLengthIndices] = -1;
  }
  //
  //Calculate auxiliary variables.
  hv_DR = (1.0*(hv_Row2-hv_Row1))/hv_Length;
  hv_DC = (1.0*(hv_Column2-hv_Column1))/hv_Length;
  hv_HalfHeadWidth = hv_HeadWidth/2.0;
  //
  //Calculate end points of the arrow head.
  hv_RowP1 = (hv_Row1+((hv_Length-hv_HeadLength)*hv_DR))+(hv_HalfHeadWidth*hv_DC);
  hv_ColP1 = (hv_Column1+((hv_Length-hv_HeadLength)*hv_DC))-(hv_HalfHeadWidth*hv_DR);
  hv_RowP2 = (hv_Row1+((hv_Length-hv_HeadLength)*hv_DR))-(hv_HalfHeadWidth*hv_DC);
  hv_ColP2 = (hv_Column1+((hv_Length-hv_HeadLength)*hv_DC))+(hv_HalfHeadWidth*hv_DR);
  //
  //Finally create output XLD contour for each input point pair
  HTuple end_val45 = (hv_Length.TupleLength())-1;
  HTuple step_val45 = 1;
  for (hv_Index=0; hv_Index.Continue(end_val45, step_val45); hv_Index += step_val45)
  {
    if (0 != (HTuple(hv_Length[hv_Index])==-1))
    {
      //Create_ single points for arrows with identical start and end point
      GenContourPolygonXld(&ho_TempArrow, HTuple(hv_Row1[hv_Index]), HTuple(hv_Column1[hv_Index]));
    }
    else
    {
      //Create arrow contour
      GenContourPolygonXld(&ho_TempArrow, ((((HTuple(hv_Row1[hv_Index]).TupleConcat(HTuple(hv_Row2[hv_Index]))).TupleConcat(HTuple(hv_RowP1[hv_Index]))).TupleConcat(HTuple(hv_Row2[hv_Index]))).TupleConcat(HTuple(hv_RowP2[hv_Index]))).TupleConcat(HTuple(hv_Row2[hv_Index])), 
          ((((HTuple(hv_Column1[hv_Index]).TupleConcat(HTuple(hv_Column2[hv_Index]))).TupleConcat(HTuple(hv_ColP1[hv_Index]))).TupleConcat(HTuple(hv_Column2[hv_Index]))).TupleConcat(HTuple(hv_ColP2[hv_Index]))).TupleConcat(HTuple(hv_Column2[hv_Index])));
    }
    ConcatObj((*ho_Arrow), ho_TempArrow, &(*ho_Arrow));
  }
  return;
}

// Chapter: Graphics / Output
// Short Description: Display the axes of a 3d coordinate system
void HalconPublic::disp_3d_coord_system (HTuple hv_WindowHandle, HTuple hv_CamParam, HTuple hv_Pose, 
    HTuple hv_CoordAxesLength)
{
	SetColor(hv_WindowHandle, (HTuple("red").Append("green").Append("blue")));
  // Local iconic variables 
  HObject  ho_Arrows;


  // Local control variables 
  HTuple  hv_TransWorld2Cam, hv_OrigCamX, hv_OrigCamY;
  HTuple  hv_OrigCamZ, hv_Row0, hv_Column0, hv_X, hv_Y, hv_Z;
  HTuple  hv_RowAxX, hv_ColumnAxX, hv_RowAxY, hv_ColumnAxY;
  HTuple  hv_RowAxZ, hv_ColumnAxZ, hv_Distance, hv_HeadLength;
  HTuple  hv_Red, hv_Green, hv_Blue;

  //This procedure displays a 3D coordinate system.
  //It needs the procedure gen_arrow_contour_xld.
  //
  //Input parameters:
  //WindowHandle: The window where the coordinate system shall be displayed
  //CamParam: The camera paramters
  //Pose: The pose to be displayed
  //CoordAxesLength: The length of the coordinate axes in world coordinates
  //
  //Check, if Pose is a correct pose tuple.
  if (0 != ((hv_Pose.TupleLength())!=7))
  {
    return;
  }
  //
  //Poses with Z position zero cannot be projected
  //(that would lead to a division by zero error).
  if (0 != (HTuple(hv_Pose[2])==0.0))
  {
    return;
  }
  //Convert to pose to a transformation matrix
  PoseToHomMat3d(hv_Pose, &hv_TransWorld2Cam);
  //Project the world origin into the image
  AffineTransPoint3d(hv_TransWorld2Cam, 0, 0, 0, &hv_OrigCamX, &hv_OrigCamY, &hv_OrigCamZ);
  Project3dPoint(hv_OrigCamX, hv_OrigCamY, hv_OrigCamZ, hv_CamParam, &hv_Row0, &hv_Column0);
  //Project the coordinate axes into the image
  AffineTransPoint3d(hv_TransWorld2Cam, hv_CoordAxesLength, 0, 0, &hv_X, &hv_Y, &hv_Z);
  Project3dPoint(hv_X, hv_Y, hv_Z, hv_CamParam, &hv_RowAxX, &hv_ColumnAxX);
  AffineTransPoint3d(hv_TransWorld2Cam, 0, hv_CoordAxesLength, 0, &hv_X, &hv_Y, &hv_Z);
  Project3dPoint(hv_X, hv_Y, hv_Z, hv_CamParam, &hv_RowAxY, &hv_ColumnAxY);
  AffineTransPoint3d(hv_TransWorld2Cam, 0, 0, hv_CoordAxesLength, &hv_X, &hv_Y, &hv_Z);
  Project3dPoint(hv_X, hv_Y, hv_Z, hv_CamParam, &hv_RowAxZ, &hv_ColumnAxZ);
  //
  //Generate an XLD contour for each axis
  DistancePp((hv_Row0.TupleConcat(hv_Row0)).TupleConcat(hv_Row0), (hv_Column0.TupleConcat(hv_Column0)).TupleConcat(hv_Column0), 
      (hv_RowAxX.TupleConcat(hv_RowAxY)).TupleConcat(hv_RowAxZ), (hv_ColumnAxX.TupleConcat(hv_ColumnAxY)).TupleConcat(hv_ColumnAxZ), 
      &hv_Distance);
  hv_HeadLength = ((((hv_Distance.TupleMax())/12.0).TupleConcat(5.0)).TupleMax()).TupleInt();
  gen_arrow_contour_xld(&ho_Arrows, (hv_Row0.TupleConcat(hv_Row0)).TupleConcat(hv_Row0), 
      (hv_Column0.TupleConcat(hv_Column0)).TupleConcat(hv_Column0), (hv_RowAxX.TupleConcat(hv_RowAxY)).TupleConcat(hv_RowAxZ), 
      (hv_ColumnAxX.TupleConcat(hv_ColumnAxY)).TupleConcat(hv_ColumnAxZ), hv_HeadLength, 
      hv_HeadLength);
  //
  //Display coordinate system
  DispXld(ho_Arrows, hv_WindowHandle);
  //
  GetRgb(hv_WindowHandle, &hv_Red, &hv_Green, &hv_Blue);
  SetRgb(hv_WindowHandle, HTuple(hv_Red[0]), HTuple(hv_Green[0]), HTuple(hv_Blue[0]));
  SetTposition(hv_WindowHandle, hv_RowAxX+3, hv_ColumnAxX+3);
  WriteString(hv_WindowHandle, "X");
  SetRgb(hv_WindowHandle, HTuple(hv_Red[1%(hv_Red.TupleLength())]), HTuple(hv_Green[1%(hv_Green.TupleLength())]), 
      HTuple(hv_Blue[1%(hv_Blue.TupleLength())]));
  SetTposition(hv_WindowHandle, hv_RowAxY+3, hv_ColumnAxY+3);
  WriteString(hv_WindowHandle, "Y");
  SetRgb(hv_WindowHandle, HTuple(hv_Red[2%(hv_Red.TupleLength())]), HTuple(hv_Green[2%(hv_Green.TupleLength())]), 
      HTuple(hv_Blue[2%(hv_Blue.TupleLength())]));
  SetTposition(hv_WindowHandle, hv_RowAxZ+3, hv_ColumnAxZ+3);
  WriteString(hv_WindowHandle, "Z");
  SetRgb(hv_WindowHandle, hv_Red, hv_Green, hv_Blue);
  return;
}