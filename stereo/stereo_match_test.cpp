// stereo.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include <iostream> 
#include <stdio.h>

using namespace std;
using namespace cv;

static void print_help()
{
    printf("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
    printf("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh|var] [--blocksize=<block_size>]\n"
           "[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i <intrinsic_filename>] [-e <extrinsic_filename>]\n"
           "[--no-display] [-o <disparity_image>] [-p <point_cloud_file>]\n");
}

static void saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}

int alpha_slider = 90;
int alpha_slider_max = 100;
int gamma_slider = 0;
int gamma_slider_max = 100;
Point origin;					//鼠标按下的起始点
Vec3f depth3D;					//鼠标按下的起始点处理后的三维坐标
Rect selection;					//定义矩形选框
bool selectObject = false;		//是否选择对象
Mat xyz;						//三维坐标

////oencv官方实例默认，立体匹配参数
int blockSize = 2;			//SAD窗口大小 
int uniquenessRatio = 15;	//使用匹配功能模式 
int numDisparities = 4;		//在该数值确定的视差范围内进行搜索,视差窗口,即最大视差值与最小视差值之差

const char* algorithm_opt = "--algorithm=";
const char* maxdisp_opt = "--max-disparity=";
const char* blocksize_opt = "--blocksize=";
const char* nodisplay_opt = "--no-display";
const char* scale_opt = "--scale=";

const char* img1_filename = 0;
const char* img2_filename = 0;
const char* intrinsic_filename = 0;
const char* extrinsic_filename = 0;
const char* disparity_filename = 0;
const char* point_cloud_filename = 0;

enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3 };
int alg = STEREO_SGBM;
int SADWindowSize = 0, numberOfDisparities = 0;
bool no_display = false;
float scale = 1.f;

StereoBM bm;
StereoSGBM sgbm;
StereoVar var;

Mat img1;//左图
Mat img2;//右图
Rect roi1, roi2;//左右匹配区域
Mat Q;//重投影矩阵Q
Mat disp, disp8;//深度图
Mat addImg;//深度图与原图合成图


//直线与平面的交点
//p[]:XYZ for point P(intersection)
//P1[]:XYZ for P1
//P2[]:XYZ for P2
//coffi[]:a,b,c,d for the plane
bool intersectionLinePlane(float P[],float P1[],float P2[],float coeffi[])
{
    float P1P2[3]={P2[0]-P1[0],P2[1]-P1[1],P2[2]-P1[2]};
    int i=0;
    float num,den,n;
    num = coeffi[0]*P1[0]+coeffi[1]*P1[1]+coeffi[2]*P1[2] + coeffi[3];
    den = coeffi[0]*P1P2[0]+coeffi[1]*P1P2[1]+coeffi[2]*P1P2[2];
    if(fabs(den)<1e-5)
    {
        //parallel
        return false;
    }
    n=num/den;
    for(i=0;i<3;i++)
        P[i]=P1[i]+n*P1P2[i];

    return true;
}

//Ax+By+Cz=D,三维离散点拟合平面
void cvFitPlane(const CvMat* points, float* plane){  
    // Estimate geometric centroid.  
    int nrows = points->rows;  
    int ncols = points->cols;  
    int type = points->type;  
    CvMat* centroid = cvCreateMat(1, ncols, type);  
    cvSet(centroid, cvScalar(0));  
    for (int c = 0; c<ncols; c++){  
        for (int r = 0; r < nrows; r++)  
        {  
            centroid->data.fl[c] += points->data.fl[ncols*r + c];  
        }  
        centroid->data.fl[c] /= nrows;  
    }  
    // Subtract geometric centroid from each point.  
    CvMat* points2 = cvCreateMat(nrows, ncols, type);  
    for (int r = 0; r<nrows; r++)  
        for (int c = 0; c<ncols; c++)  
            points2->data.fl[ncols*r + c] = points->data.fl[ncols*r + c] - centroid->data.fl[c];  
    // Evaluate SVD of covariance matrix.  
    CvMat* A = cvCreateMat(ncols, ncols, type);  
    CvMat* W = cvCreateMat(ncols, ncols, type);  
    CvMat* V = cvCreateMat(ncols, ncols, type);  
    cvGEMM(points2, points, 1, NULL, 0, A, CV_GEMM_A_T);  
    cvSVD(A, W, NULL, V, CV_SVD_V_T);  
    // Assign plane coefficients by singular vector corresponding to smallest singular value.  
    plane[ncols] = 0;  
    for (int c = 0; c<ncols; c++){  
        plane[c] = V->data.fl[ncols*(ncols - 1) + c];  
        plane[ncols] += plane[c] * centroid->data.fl[c];  
    }  
    // Release allocated resources.  
    cvReleaseMat(&centroid);  
    cvReleaseMat(&points2);  
    cvReleaseMat(&A);  
    cvReleaseMat(&W);  
    cvReleaseMat(&V);  
}  

//以点击坐标为中心，获取一定大小正方形
Rect getPoitCube(Point origin, int size)
{
	int xStart = origin.x - size;
	int yStart = origin.y - size;
	int xEnd = origin.x + size;
	int yEnd = origin.y + size;
	//边界处理
	if(xStart<0)
		xStart = 0;
	if(yStart<0)
		yStart = 0;
	if(xEnd>addImg.cols-size)
		xEnd = addImg.cols;
	if(yEnd>addImg.rows-size)
		yEnd = addImg.rows;
	//绘画计算区域
	Mat addImgRGB;
	cvtColor(addImg, addImgRGB, CV_GRAY2BGR);
	Rect roi(xStart,yStart,xEnd-xStart,yEnd-yStart);
	rectangle(addImgRGB, roi, Scalar(0, 0, 255), 3, 2);                      //画上一个矩形  
	imshow("addWeighted", addImgRGB);
	return roi;
}

//以点击坐标为中心，获取一定大小正方形所有有效的三维坐标，求平均Z
Vec3f getPoint3DAve(Point origin, int size)
{
	const double max_z = 1.0e4;
	//以点击坐标为中心，获取一定大小正方形所有有效的三维坐标
	Rect roi = getPoitCube(origin, size);
	double Xsum = 0;
	double Ysum = 0;
	double Zsum = 0;
	int count = 0;
	for(int x = roi.x;x<roi.x+roi.width;x++)
	{
		for(int y = roi.y;y<roi.y+roi.height;y++)
		{
			Vec3f point = xyz.at<Vec3f>(Point(x, y));
			//舍去无效三维坐标
			if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) 
				continue;
			Xsum = Xsum + point[0];
			Ysum = Ysum + point[1];
			Zsum = Zsum + point[2];
			count++;
		}
	}
	Vec3f depth3D(Xsum/count,Ysum/count,Zsum/count);
	return depth3D;
}

//以点击坐标为中心，获取一定大小正方形所有有效的三维坐标，拟合平面获取交点作为最终该点的三维坐标
Vec3f getPoint3D(Point origin, int size)
{
	const double max_z = 1.0e4;
	//以点击坐标为中心，获取一定大小正方形所有有效的三维坐标
	Rect roi = getPoitCube(origin, size);
	vector<double> Xs;
	vector<double> Ys;
	vector<double> Zs;
	for(int x = roi.x;x<roi.x+roi.width;x++)
	{
		for(int y = roi.y;y<roi.x+roi.height;y++)
		{
			Vec3f point = xyz.at<Vec3f>(Point(x, y));
			//舍去无效三维坐标
			if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
			Xs.push_back(point[0]);
			Ys.push_back(point[1]);
			Zs.push_back(point[2]);
		}
	}
	CvMat* points_mat = cvCreateMat(Xs.size(), 3, CV_32FC1);//定义用来存储需要拟合点的矩阵   
	for (int i=0;i < Xs.size(); ++i)  
	{  
		points_mat->data.fl[i*3+0] = Xs[i];//矩阵的值进行初始化   X的坐标值  
		points_mat->data.fl[i * 3 + 1] = Ys[i];//  Y的坐标值  
		points_mat->data.fl[i * 3 + 2] = Zs[i];//  Z的坐标值
	}  
	float plane12[4] = { 0 };//定义用来储存平面参数的数组   
	//Ax+By+Cz=D,三维离散点拟合平面
	cvFitPlane(points_mat, plane12);//调用方程  

	//直线与平面的交点
	float P[3] = {0,0,0};
	float P1[3] = {0,0,0};
	Vec3f p2_stemp = xyz.at<Vec3f>(origin);
	float P2[3] = {p2_stemp[0],p2_stemp[1],p2_stemp[2]};
	bool hasPoint = intersectionLinePlane(P, P1, P2, plane12);
	Vec3f depth3D(P[0],P[1],P[2]);
	return depth3D;
}

//灰度图转伪彩色图的代码，主要功能是使灰度图中
//亮度越高的像素点，在伪彩色图中对应的点越趋向于 红色；亮度越低，则对应的伪彩色越趋向于 蓝色；总体上按照灰度值高低，由红渐变至蓝，中间色为绿色。
void F_Gray2Color(Mat &gray_mat, Mat &color_mat)
{		
	color_mat = Mat::zeros(gray_mat.rows,gray_mat.cols,CV_8UC3);  
	int stype = gray_mat.type(), dtype = color_mat.type();

	// 判断输入的灰度图和输出的伪彩色图是否大小相同、格式是否符合要求
	if (stype == CV_8UC1 && dtype == CV_8UC3)
	{
		int tmp=0;
		for (int y=0;y<gray_mat.rows;y++)//转为伪彩色图像的具体算法
		{
			for (int x=0;x<gray_mat.cols;x++)
			{
				tmp = gray_mat.at<unsigned char>(y,x);
				color_mat.at<Vec3b>(y,x)[0] = abs(255-tmp); //blue
				color_mat.at<Vec3b>(y,x)[1] = abs(127-tmp); //green
				color_mat.at<Vec3b>(y,x)[2] = abs( 0-tmp);	//red
			}
		}
	}
}

/*****深度图与原图合成*****/
void add_weighted(int,void*)
{
	double alpha = (double) alpha_slider/alpha_slider_max ;  
	addWeighted( img1, alpha, disp8, 1-alpha, gamma_slider, addImg);  
	imshow("addWeighted", addImg);
}

/*****立体匹配*****/
void stereo_match(int,void*)
{
	//oencv官方实例默认
 //	  bm.state->roi1 = roi1;
 //   bm.state->roi2 = roi2;
 //   bm.state->preFilterCap = 31;
 //   bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
 //   bm.state->minDisparity = 0;
 //   bm.state->numberOfDisparities = numberOfDisparities;
 //   bm.state->textureThreshold = 10;
 //   bm.state->uniquenessRatio = 15;
 //   bm.state->speckleWindowSize = 100;
 //   bm.state->speckleRange = 32;
 //   bm.state->disp12MaxDiff = 1;

	numberOfDisparities = numDisparities*16+16;
	bm.state->roi1 = roi1;
    bm.state->roi2 = roi2;
    bm.state->preFilterCap = 31;
    bm.state->SADWindowSize = 2*blockSize+5;							//SAD窗口大小 
    bm.state->minDisparity = 0;											//确定匹配搜索从哪里开始  默认值是0
    bm.state->numberOfDisparities = numberOfDisparities;				//在该数值确定的视差范围内进行搜索,视差窗口,即最大视差值与最小视差值之差, 大小必须是16的整数倍 
    bm.state->textureThreshold = 10;									//保证有足够的纹理以克服噪声
    bm.state->uniquenessRatio = uniquenessRatio;						//使用匹配功能模式  
    bm.state->speckleWindowSize = 100;									//检查视差连通区域变化度的窗口大小, 值为0时取消 speckle 检查
    bm.state->speckleRange = 32;										//视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零  
    bm.state->disp12MaxDiff = 1;										//左视差图（直接计算得出）和右视差图（通过cvValidateDisparity计算得出）之间的最大容许差异，默认为-1  

    sgbm.preFilterCap = 63;
    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

    int cn = img1.channels();

    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = 0;
    sgbm.numberOfDisparities = numberOfDisparities;
    sgbm.uniquenessRatio = 10;
    sgbm.speckleWindowSize = bm.state->speckleWindowSize;
    sgbm.speckleRange = bm.state->speckleRange;
    sgbm.disp12MaxDiff = 1;
    sgbm.fullDP = alg == STEREO_HH;

    var.levels = 3;                                 // ignored with USE_AUTO_PARAMS
    var.pyrScale = 0.5;                             // ignored with USE_AUTO_PARAMS
    var.nIt = 25;
    var.minDisp = -numberOfDisparities;
    var.maxDisp = 0;
    var.poly_n = 3;
    var.poly_sigma = 0.0;
    var.fi = 15.0f;
    var.lambda = 0.03f;
    var.penalization = var.PENALIZATION_TICHONOV;   // ignored with USE_AUTO_PARAMS
    var.cycle = var.CYCLE_V;                        // ignored with USE_AUTO_PARAMS
    var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING ;

    int64 t = getTickCount();
    if( alg == STEREO_BM )
        bm(img1, img2, disp);
    else if( alg == STEREO_VAR ) {
        var(img1, img2, disp);
    }
    else if( alg == STEREO_SGBM || alg == STEREO_HH )
        sgbm(img1, img2, disp);
    t = getTickCount() - t;
    printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

    if( alg != STEREO_VAR )
        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
    else
        disp.convertTo(disp8, CV_8U);
    if( !no_display )
    {
        namedWindow("left", 1);
        imshow("left", img1);
        namedWindow("right", 1);
        imshow("right", img2);
        namedWindow("disparity", 0);
		Mat vdispRGB;
		//灰度图转伪彩色图
		F_Gray2Color(disp8, vdispRGB);
        imshow("disparity", vdispRGB);
        printf("press any key to continue...");		
        fflush(stdout);
        printf("\n");
    }

    if(disparity_filename)
        imwrite(disparity_filename, disp8);

    if(point_cloud_filename)
    {
        printf("storing the point cloud...");
        fflush(stdout);
        reprojectImageTo3D(disp, xyz, Q, true);
		//在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
		xyz = xyz * 16;
        saveXYZ(point_cloud_filename, xyz);
        printf("\n");
    }
}

/*****描述：鼠标操作回调*****/
static void onMouse(int event, int x, int y, int, void*)
{
    if (selectObject)
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);
    }
    switch (event)
    {
    case EVENT_LBUTTONDOWN:   //鼠标左按钮按下的事件
        origin = Point(x, y);
        selection = Rect(x, y, 0, 0);
        selectObject = true;
		depth3D = getPoint3DAve(origin, 10);
        cout << origin <<"in world coordinate is: " << depth3D << endl;
		cout << "dis: " << sqrt(depth3D[0]*depth3D[0]+depth3D[1]*depth3D[1]+depth3D[2]*depth3D[2]) << endl;
		cout << "dis: " << sqrt(depth3D[0]*depth3D[0]+depth3D[1]*depth3D[1]+depth3D[2]*depth3D[2]) << endl;
        break;
    case EVENT_LBUTTONUP:    //鼠标左按钮释放的事件
        selectObject = false;
        if (selection.width > 0 && selection.height > 0)
        break;
    }
}

int main(int argc, char** argv)
{
	argc = 12;
	argv[0] = "demoDCamera";//用项目名称，我的是demoDCamera
	argv[1] = "left01.jpg";
	argv[2] = "right01.jpg";
	argv[3] = "--algorithm=bm";

	argv[4] = "-i";
	argv[5] = "intrinsics.yml";
	argv[6] = "-e";
	argv[7] = "extrinsics.yml";

	argv[8] = "-o";
	argv[9] = "disparity_filename.bmp";
	argv[10] = "-p";
	argv[11] = "point_cloud_filename.txt";

    if(argc < 3)
    {
        print_help();
        return 0;
    }

    for( int i = 1; i < argc; i++ )
    {
        if( argv[i][0] != '-' )
        {
            if( !img1_filename )
                img1_filename = argv[i];
            else
                img2_filename = argv[i];
        }
        else if( strncmp(argv[i], algorithm_opt, strlen(algorithm_opt)) == 0 )
        {
            char* _alg = argv[i] + strlen(algorithm_opt);
            alg = strcmp(_alg, "bm") == 0 ? STEREO_BM :
                  strcmp(_alg, "sgbm") == 0 ? STEREO_SGBM :
                  strcmp(_alg, "hh") == 0 ? STEREO_HH :
                  strcmp(_alg, "var") == 0 ? STEREO_VAR : -1;
            if( alg < 0 )
            {
                printf("Command-line parameter error: Unknown stereo algorithm\n\n");
                print_help();
                return -1;
            }
        }
        else if( strncmp(argv[i], maxdisp_opt, strlen(maxdisp_opt)) == 0 )
        {
            if( sscanf( argv[i] + strlen(maxdisp_opt), "%d", &numberOfDisparities ) != 1 ||
                numberOfDisparities < 1 || numberOfDisparities % 16 != 0 )
            {
                printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
                print_help();
                return -1;
            }
        }
        else if( strncmp(argv[i], blocksize_opt, strlen(blocksize_opt)) == 0 )
        {
            if( sscanf( argv[i] + strlen(blocksize_opt), "%d", &SADWindowSize ) != 1 ||
                SADWindowSize < 1 || SADWindowSize % 2 != 1 )
            {
                printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
                return -1;
            }
        }
        else if( strncmp(argv[i], scale_opt, strlen(scale_opt)) == 0 )
        {
            if( sscanf( argv[i] + strlen(scale_opt), "%f", &scale ) != 1 || scale < 0 )
            {
                printf("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
                return -1;
            }
        }
        else if( strcmp(argv[i], nodisplay_opt) == 0 )
            no_display = true;
        else if( strcmp(argv[i], "-i" ) == 0 )
            intrinsic_filename = argv[++i];
        else if( strcmp(argv[i], "-e" ) == 0 )
            extrinsic_filename = argv[++i];
        else if( strcmp(argv[i], "-o" ) == 0 )
            disparity_filename = argv[++i];
        else if( strcmp(argv[i], "-p" ) == 0 )
            point_cloud_filename = argv[++i];
        else
        {
            printf("Command-line parameter error: unknown option %s\n", argv[i]);
            return -1;
        }
    }

    if( !img1_filename || !img2_filename )
    {
        printf("Command-line parameter error: both left and right images must be specified\n");
        return -1;
    }

    if( (intrinsic_filename != 0) ^ (extrinsic_filename != 0) )
    {
        printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
        return -1;
    }

    if( extrinsic_filename == 0 && point_cloud_filename )
    {
        printf("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
        return -1;
    }

    int color_mode = alg == STEREO_BM ? 0 : -1;
    img1 = imread(img1_filename, color_mode);
    img2 = imread(img2_filename, color_mode);

    if( scale != 1.f )
    {
        Mat temp1, temp2;
        int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
        resize(img1, temp1, Size(), scale, scale, method);
        img1 = temp1;
        resize(img2, temp2, Size(), scale, scale, method);
        img2 = temp2;
    }

    Size img_size = img1.size();

    if( intrinsic_filename )
    {
        // reading intrinsic parameters
        FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", intrinsic_filename);
            return -1;
        }

        Mat M1, D1, M2, D2;
        fs["M1"] >> M1;
        fs["D1"] >> D1;
        fs["M2"] >> M2;
        fs["D2"] >> D2;

        M1 *= scale;
        M2 *= scale;

        fs.open(extrinsic_filename, CV_STORAGE_READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", extrinsic_filename);
            return -1;
        }

        Mat R, T, R1, P1, R2, P2;
        fs["R"] >> R;
        fs["T"] >> T;
		//是对标定过的摄像机进行校正
        stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

        Mat map11, map12, map21, map22;
        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

        Mat img1r, img2r;
        remap(img1, img1r, map11, map12, INTER_LINEAR);
        remap(img2, img2r, map21, map22, INTER_LINEAR);

        img1 = img1r;
        img2 = img2r;
    }

	/*
    把校正结果显示出来
    */
    Mat rgbRectifyImageL, rgbRectifyImageR;
    cvtColor(img1, rgbRectifyImageL, CV_GRAY2BGR);  //伪彩色图
    cvtColor(img2, rgbRectifyImageR, CV_GRAY2BGR);

	//显示在同一张图上
    Mat canvas;
    double sf;
    int w, h;
    sf = 600. / max(img_size.width, img_size.height);
    w = cvRound(img_size.width * sf);
    h = cvRound(img_size.height * sf);
    canvas.create(h, w * 2, CV_8UC3);   //注意通道

    //左图像画到画布上
    Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                                //得到画布的一部分  
    resize(rgbRectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);     //把图像缩放到跟canvasPart一样大小  
    Rect vroiL(cvRound(roi1.x*sf), cvRound(roi1.y*sf),                //获得被截取的区域    
        cvRound(roi1.width*sf), cvRound(roi1.height*sf));
    rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);                      //画上一个矩形  
	Mat imageROI_L;
	imageROI_L = canvasPart(vroiL);
	//imwrite("ROI_L", imageROI_L);
	imshow("ROI_L", imageROI_L);
    cout << "Painted ImageL" << endl;

    //右图像画到画布上
    canvasPart = canvas(Rect(w, 0, w, h));                                      //获得画布的另一部分  
    resize(rgbRectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
    Rect vroiR(cvRound(roi2.x * sf), cvRound(roi2.y*sf),
        cvRound(roi2.width * sf), cvRound(roi2.height * sf));
    rectangle(canvasPart, vroiR, Scalar(0, 0, 255), 3, 8);
	// 定义一个Mat类型并给定其设定的区域
	Mat imageROI_R;
	imageROI_R = canvasPart(vroiR);
	//imwrite("ROI_R", imageROI_R);
	imshow("ROI_R", imageROI_R);
	cout << "Painted ImageR" << endl;

    //画上对应的线条
    for (int i = 0; i < canvas.rows; i += 16)
        line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
    imshow("rectified", canvas);

    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
	
    //立体匹配
    namedWindow("disparity", CV_WINDOW_AUTOSIZE);
    // 创建SAD窗口 Trackbar
    createTrackbar("BlockSize:\n", "disparity",&blockSize, 8, stereo_match);
    // 创建视差唯一性百分比窗口 Trackbar
    createTrackbar("UniquenessRatio:\n", "disparity", &uniquenessRatio, 50, stereo_match);
    // 创建视差窗口 Trackbar
    createTrackbar("NumDisparities:\n", "disparity", &numDisparities, 16, stereo_match);
    //鼠标响应函数setMouseCallback(窗口名称, 鼠标回调函数, 传给回调函数的参数，一般取0)
    setMouseCallback("disparity", onMouse, 0);
    stereo_match(0,0);

	// 创建左图合成窗口 Trackbar
	add_weighted(0,0);
	createTrackbar("Alpha :\n", "addWeighted", &alpha_slider, alpha_slider_max, add_weighted ); 
	createTrackbar("Aamma :\n", "addWeighted", &gamma_slider, gamma_slider_max, add_weighted ); 
	setMouseCallback("addWeighted", onMouse, 0);

    waitKey(0);

    return 0;
}


