#include "F:\VS2010版RobotController\MoRoController\PrimeSenseDet\PrimeSenseDet.h"
#include "VectorAndMatrix.h"
#include <malloc.h>
#include "F:\VS2010版RobotController\MoRoController\Sensor.h"
#include "F:\VS2010版RobotController\MoRoController\RobotControl\Kine.h"
#include "F:\VS2010版RobotController\MoRoController\RobotControl\Setup.h"
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <vector>


#define DETECT_POLE_NUM_MAX 5//HU
//////////////////////////////////////
//全局变量
////////////////////////////////////

//OpenNI
using namespace std;

// 杆件提取识别和检测
//float PCentralLine[DETECT_POLE_NUM_MAX][2][3];//HU
//float PonCentline[DETECT_POLE_NUM_MAX][3];//HU
int DrawN[DETECT_POLE_NUM_MAX];  //每个杆件的轮廓点数HU
int g_PolesNum;
int RSJ[DETECT_POLE_NUM_MAX];//HU

//类的静态变量
LocalEnvList* CPrimeSenseDet::m_pLocalEnvList;
MtxKine CPrimeSenseDet::PUntifyMtx;
MtxKine CPrimeSenseDet::G2_PUntifyMtx;
//bool CPrimeSenseDet::OpenGL_Mode = TRUE; //Opengl显示模式：1、自主抓夹模式，2、杆件重构模式//GU160517，原位TRUE
bool CPrimeSenseDet::OpenGL_Mode = FALSE;//Opengl显示模式：1、自主抓夹模式，2、杆件重构模式//GU160517，原位TRUE,此处修改是为了解决在未开启工具对话前，打开仿真界面时出现的中断现象
//执行自主抓夹的线程函数
int PoleStyle[10],PoleNum;

float PCentralLineTemp[DETECT_POLE_NUM_MAX][2][3];//GU  第二项0表示方向，1表示中心点
int PoleLenghtTemp[DETECT_POLE_NUM_MAX];                               //GU
int PoleStyleTemp[10];    //杆件类型

bool DeteSymbol;
int InPlane;

bool bool_Tool1;
bool bool_Tool2;

CPrimeSenseDet::CPrimeSenseDet(void)
{
	//初始化杆件链表
	//m_pLocalEnvList = createInfoList();
	CamineCheck = FALSE; 
	////////////////////////////////////////////////////
	CSensorPos[0]=SensorX;
	CSensorPos[1]=SensorY;
	CSensorPos[2]=SensorZ;
	CSensorPos[3]=-SensorAlpha;
	CSensorPos[4]=SensorBeta;
	CSensorPos[5]=SensorGamma;

	FrameTrans[0]=0;
	FrameTrans[1]=0;
	FrameTrans[2]=0;
	FrameTrans[3]=180;
	FrameTrans[4]=0;
	FrameTrans[5]=0;

	//CSensorPos[0]=-120;
	//CSensorPos[1]=-50;
	//CSensorPos[2]=260;
	//CSensorPos[3]=90;
	//CSensorPos[4]=180;
	//CSensorPos[5]=0;

	PUntifyMtx.R11=1; PUntifyMtx.R12=0; PUntifyMtx.R13=0;
	PUntifyMtx.R21=0; PUntifyMtx.R22=1; PUntifyMtx.R23=0;
	PUntifyMtx.R31=0; PUntifyMtx.R32=0; PUntifyMtx.R33=1;
	PUntifyMtx.X=0;   PUntifyMtx.Y=0;   PUntifyMtx.Z=0;
	Trans_PosToMtx(CSensorPos, &PUntifyMtx, 0);//该处实现相机坐标系在夹持器坐标系下的矩阵表示
	////////////////////////////////////////////////
	////OpenNI Initalize
	//Status rc=STATUS_OK;

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//int devicenumber;
	//char deviceUri[2][256] ={0};
	//char deviceUri1[256] = "\\\\?\\usb#vid_1d27&pid_0601&mi_00#6&31251095&1&0000#{c3b5f022-5a42-1980-1909-ea72095601b1}";//ASUS Xtion
	//char deviceUri2[256] = "\\\\?\\usb#vid_1d27&pid_0609&mi_00#6&574dccc&1&0000#{c3b5f022-5a42-1980-1909-ea72095601b1}";//carmine1.09

	//char compareUri1, compareUri2;

	//rc = OpenNI::initialize();
	//// 获取设备信息  
	//Array<DeviceInfo> aDeviceList;  
	//OpenNI::enumerateDevices( &aDeviceList );

	//devicenumber = aDeviceList.getSize();

	//for (int i=0; i<devicenumber; i++)
	//{
	//	const DeviceInfo& rDevInfo = aDeviceList[i];
	//	strcpy(deviceUri[i],rDevInfo.getUri());
	//}

	//strcmp(deviceUri[0],deviceUri1);
	//strcmp(deviceUri[0],deviceUri2);
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	//

	//if (rc != STATUS_OK)
	//{
	//	AfxMessageBox("Camine初始化失败，请排除错误后重试");
	//	//return TRUE;
	//}
	//else{
	//	rc = device.open(deviceUri2);
	//	if (rc != STATUS_OK)
	//	{
	//		AfxMessageBox("Camine打开失败，请排除错误后重试");
	//		//return TRUE;
	//	}
	//	else if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
	//	{
	//		rc = depth.create(device, SENSOR_DEPTH);
	//		VideoMode videomode = depth.getVideoMode();
	//		// 		Xresolution = videomode.getResolutionX();
	//		// 		Yresolution = videomode.getResolutionY();
	//		videomode.setResolution(640,480);
	//		if( depth.setVideoMode( videomode ) != STATUS_OK )
	//		{  
	//			AfxMessageBox("分辨率设置有问题");
	//			//return TRUE;
	//		}
	//		if (rc != STATUS_OK)
	//		{
	//			AfxMessageBox("Camine创建图像失败，请排除错误后重试");
	//			//return TRUE;
	//		}
	//		rc = depth.start();
	//		if (rc != STATUS_OK)
	//		{
	//			AfxMessageBox("检测失败，请排除错误后重试");
	//			//return TRUE;
	//		}
	//		int changedStreamDummy;
	//		VideoStream* pStream = &depth;
	//		rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
	//		if (rc != STATUS_OK)
	//		{
	//			AfxMessageBox("检测失败，请排除错误后重试");
	//			//return TRUE;
	//		}
	//		else
	//			CamineCheck = TRUE;
	//	}

	//}
	//WriteDepthImageFile();
	//DepthImageCapture();
}

//打开指定的设备
bool CPrimeSenseDet::OpenTheDevice(bool b_Tool1, bool b_Tool2)
{
	depth.stop();
	depth.destroy();
	device.close();
	OpenNI::shutdown();

	InPlane = 0;

	bool_Tool1 = b_Tool1;
	bool_Tool2 = b_Tool2;

	//两个设备的Uri
	char deviceUri1[256] = "\\\\?\\usb#vid_1d27&pid_0601&mi_00#6&31251095&1&0000#{c3b5f022-5a42-1980-1909-ea72095601b1}";//ASUS Xtion
	char deviceUri2[256] = "\\\\?\\usb#vid_1d27&pid_0609&mi_00#6&574dccc&1&0000#{c3b5f022-5a42-1980-1909-ea72095601b1}";//carmine1.09

	Status rc=STATUS_OK;
	rc = OpenNI::initialize();

	// 获取设备信息  
	Array<DeviceInfo> aDeviceList;  
	OpenNI::enumerateDevices( &aDeviceList );

	int devicenumber = aDeviceList.getSize();

	//if (rc != STATUS_OK)
	//{
	//	AfxMessageBox("Camine初始化失败，请排除错误后重试");
	//	return FALSE;
	//}

	//if (rc != STATUS_OK || devicenumber!=2)
	//{
	//	AfxMessageBox("Camine初始化失败，请排除错误后重试");
	//	//return TRUE;
	//}
	if (rc != STATUS_OK)
	{
		AfxMessageBox("Camine初始化失败，请排除错误后重试");
		//return TRUE;
	}
	else{
		    rc = device.open(ANY_DEVICE);
			//if ((b_Tool1) && (!b_Tool2))
			//{
			//	rc = device.open(deviceUri2);
			//}
			//else if ((!b_Tool1) && (b_Tool2))
			//{
			//	rc = device.open(deviceUri2);
			//}
			//else
			//{
			//	AfxMessageBox("请检查两个夹持器的夹持状态，保持一开一合");
			//}

			if (rc != STATUS_OK)
			{
				AfxMessageBox("Camine打开失败，请排除错误后重试");
				return FALSE;
			}
			else if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
			{
				
				rc = depth.create(device, SENSOR_DEPTH);
				VideoMode videomode = depth.getVideoMode();
				// 		Xresolution = videomode.getResolutionX();
				// 		Yresolution = videomode.getResolutionY();
				videomode.setResolution(640,480);
				if (depth.setVideoMode(videomode) != STATUS_OK)
				{  
					AfxMessageBox("分辨率设置有问题");
					return FALSE;
				}
				if (rc != STATUS_OK)
				{
					AfxMessageBox("Camine创建图像失败，请排除错误后重试");
					return FALSE;
				}
				rc = depth.start();
				if (rc != STATUS_OK)
				{
					AfxMessageBox("检测失败，请排除错误后重试");
					return FALSE;
				}
				int changedStreamDummy;
				VideoStream* pStream = &depth;
				rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
				if (rc != STATUS_OK)
				{
					AfxMessageBox("检测失败，请排除错误后重试");
					return FALSE;
				}
				else
				{
					CamineCheck = TRUE;
					return TRUE;
				}
			}
	    }
}

CPrimeSenseDet::~CPrimeSenseDet(void)
{
	//	pppTerminate();   // 结束调用, 在退出对话框时执行	
	depth.stop();
	depth.destroy();
	device.close();
	OpenNI::shutdown();

	//DestoryList(m_pLocalEnvList);
}

//关闭指定的设备
void CPrimeSenseDet::CloseTheDevice()
{
	depth.stop();
	depth.destroy();
	device.close();
	OpenNI::shutdown();
}

///////////////////////////////////
//链表操作函数
//////////////////////////////////
LocalEnvList * CPrimeSenseDet::createInfoList()
{
	LocalEnvList * newList = NULL;
	newList = new LocalEnvList;
	if(newList == NULL)
	{	
		return NULL;
	}
	newList->begin = new PoleNode;	
	if(newList->begin == NULL)
	{
		delete newList;
		return NULL;
	}
	newList->begin->num = 0;
	newList->begin->pnext = NULL;
	newList->end = newList->begin;
	return newList;
}

bool CPrimeSenseDet::PushList(LocalEnvList* list, PoleNode *Item)
{
	if (list == NULL)
	{
		return false;
	}
	if (Item == NULL)
	{
		return false;
	}
	PoleNode* AddNode = new PoleNode;
	if (AddNode == NULL)
	{
		delete AddNode;
		return false;
	}
	//赋值
	AddNode->num = Item->num;
	for (int i=0; i<3; ++i)
	{
		AddNode->PoleFunc[0][i] = Item->PoleFunc[0][i];
		AddNode->PoleFunc[1][i] = Item->PoleFunc[1][i];
		AddNode->PoleEnd[0][i] = Item->PoleEnd[0][i];
		AddNode->PoleEnd[1][i] = Item->PoleEnd[1][i];
	}
	AddNode->PoleLenght[0] = Item->PoleLenght[0];
	AddNode->PoleLenght[1] = Item->PoleLenght[1];

	list->end->pnext = AddNode;
	list->end = AddNode;
	list->end->pnext = NULL;
	return true;
}

bool CPrimeSenseDet::DestoryList(LocalEnvList* list)
{
	if (list == NULL||list->begin==NULL)
	{
		return true;
	}
	PoleNode *temp;
	while(list->begin != NULL)
	{
		temp = list->begin->pnext;
		delete list->begin;
		list->begin = temp;
	}
	return true;
}

//获取并记录图像数据
bool CPrimeSenseDet::WriteDepthImageFile()
{
	VideoFrameRef frame;  //frame data
	fstream ImageFile;
	Status rc=STATUS_OK;
	rc = depth.readFrame(&frame);   
	if (rc != STATUS_OK)
	{
		return false;
	}
	if (frame.getVideoMode().getPixelFormat() != 100 && frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_100_UM)
	{
		return false;
	}
	DepthPixel* pDepth = (DepthPixel*)frame.getData();
	float depthTempData[3] = {0};
	int i,j,k,q;  //local variable
	unsigned short PicHeight,PicWidth; //height and width of detph image
	////////////////////////////////////////
	PicHeight = frame.getHeight();
	PicWidth = frame.getWidth();
	//float PicPixelP[3];
	ImageFile.open("image.txt",ios::app);
	if(!ImageFile)
	{
		return false;
	}
	char tempArray[100];
	for (i=0;i<PicHeight;i++)
	{
		//扫描下一行的初始化 
		for (j=0;j<PicWidth;j++)
		{
			//float iiid = pDepth[i*PicWidth+j];
			//////////////////////////////////////////////////////////////
			//平面化超过距离范围的物体，类似于在杆件后面设置了一块幕布
			if(pDepth[i*PicWidth+j]>DetectionRangeMax)
			{
				pDepth[i*PicWidth+j] = DetectionRangeMax;
			}
			//近距离以及物体阴影部分处理
			if(pDepth[i*PicWidth+j]<DetectionRangeMin)
			{
				pDepth[i*PicWidth+j] = DetectionRangeMax;
			}

			CoordinateConverter::convertDepthToWorld(depth,j,i,pDepth[i*PicWidth+j],
				                                     &depthTempData[0],&depthTempData[1],&depthTempData[2]);//将一个点从深度坐标系中变换到世界坐标系

			
        //    ImageFile.open("..\PrimeSenseDet\image.dat",ios::app)
			memset(tempArray , '\0' , 100);
			sprintf(tempArray , "%d %d %.2f %.2f %.2f;",\
				i , j,depthTempData[0],depthTempData[1],depthTempData[2]);
			ImageFile<<tempArray;
	    //    fwrite(depthTempData,sizeof(float)*3,3,ImageFile);//将数据写入文件fp
		}
	}
	ImageFile.close();
	//fclose(ImageFile);
}

//比较重复杆件
bool CPrimeSenseDet::CheckRepeatPole(float *CentralP, float* mark)
{
	//比较重复杆件
		//对比所有已经获取的杆件，两点好处：1，针对此帧图像中的杆件判断重复，2，正对不同帧图像杆件判断重复
	PoleNode* PoleNode_S = m_pLocalEnvList->begin->pnext;
	PointCoordinateC(&PUntifyMtx,CentralP,CentralP);
	float EndV1[3],EndV2[3];
	int i=0;
	while(PoleNode_S!=NULL)
	{
//		float a=PtoLineDis(CentralP,PoleNode_S->PoleFunc[0],PoleNode_S->PoleFunc[1]);
		if (PtoLineDis(CentralP,PoleNode_S->PoleFunc[0],PoleNode_S->PoleFunc[1])<30)
		{
			*mark = 0;
			for (i=0; i<3; ++i)
			{
				EndV1[i] = PoleNode_S->PoleEnd[0][i]-CentralP[i];
				EndV2[i] = PoleNode_S->PoleEnd[1][i]-CentralP[i];
			}
			//判断是否需要延长杆件
			if (VectorAngle(EndV1,EndV2)<1.57)
			{
				if (VectorLenght(EndV1)>VectorLenght(EndV2))
				{
					for (i=0; i<3; ++i)
					{
						PoleNode_S->PoleEnd[1][i]=CentralP[i];
					}
				}
				else
				{
					for (i=0; i<3; ++i)
					{
						PoleNode_S->PoleEnd[0][i]=CentralP[i];
					}
				}
			}
			
			return false;  //标志位
			break;
		}
		PoleNode_S=PoleNode_S->pnext;
	}
	return true;
	/////////////////////
}

bool CPrimeSenseDet::CoordinateConv()//求解相机坐标系在基坐标系下的表示，矩阵表示形式
{
	/////////////////////////////////////////////////
	//Coordinate Convertion
	//针对当前获取图像所在的时刻，读取机器人关节信息
	double dPos[6],CRobotPos[6];//CSensorPos[6];
	double G2_CRobotPos[6];

	//Robot::I_Monit.Get_JointPos(dPos);
	//Robot::I_Task.m_iKine_CR_G1.FKine(dPos, CRobotPos);//CRobotPos为正解位姿, (x,y,z,w,p,r)

	if ((bool_Tool1) && (!bool_Tool2))
	{
		Robot::I_Monit.Get_JointPos(dPos);
		Robot::I_Task.m_iKine_CR_G1.FKine(dPos, CRobotPos);//CRobotPos为正解位姿, (x,y,z,w,p,r)
	}
	else if ((!bool_Tool1) && (bool_Tool2))
	{
		Robot::I_Monit.Get_JointPos(dPos);
		Robot::I_Task.m_iKine_CR_G2.FKine(dPos, CRobotPos);//CRobotPos为正解位姿, (x,y,z,w,p,r)
// 		Robot_IncreTransTool(FrameTrans,CRobotPos,G2_CRobotPos);
 		Robot_IncreTransTool(CRobotPos,CSensorPos,G2_CRobotPos);
		Robot_IncreTransTool(FrameTrans,G2_CRobotPos,G2_CRobotPos);
 		Trans_PosToMtx(G2_CRobotPos,&G2_PUntifyMtx,0);
	}

	Robot_IncreTransTool(CRobotPos,CSensorPos,CRobotPos);//求解相机坐标系在基坐标系下的表示，结果表示形式为(x,y,z,w,p,r)
	Trans_PosToMtx(CRobotPos, &PUntifyMtx, 0);//相机坐标系在基坐标系下的表示，矩阵表示形式
	return true;
}

/******************************************************************************
* 函数：CPrimeSenseDet::LocalEnvListProc(void)
* 功能：获取简化杆件信息后对长度进行计算
*
*
* 返回：
******************************************************************************/
bool CPrimeSenseDet::LocalEnvListProc(void)
{
	PoleNode* tempNode = m_pLocalEnvList->begin->pnext;
	if (tempNode==NULL)
	{
		return false;
	}
	while(tempNode!=NULL)
	{
		tempNode->PoleLenght[0]=sqrt(SQUARE(P2Distance(tempNode->PoleFunc[1],tempNode->PoleEnd[0]))-900);
		tempNode->PoleLenght[1]=sqrt(SQUARE(P2Distance(tempNode->PoleFunc[1],tempNode->PoleEnd[1]))-900);
		tempNode = tempNode->pnext;
	}
	return true;
}

/******************************************************************************
* 函数：GetDepthImageData(VideoFrameRef* frame, DepthPixel* pDepth)
* 功能：获取一帧深度图像数据，并且对其预处理
*
*
* 返回：
******************************************************************************/
Status CPrimeSenseDet::GetDepthImageData(static float frame3D[][640][3])
{
	if (CamineCheck==FALSE)
	{
		AfxMessageBox("传感器打开失败，请检查！");
		DeteSymbol = false;
		return STATUS_ERROR;
	}
	VideoFrameRef frame;
	Status rc=STATUS_OK;
	rc = depth.readFrame(&frame);
	if (rc != STATUS_OK)
	{
		return rc;
	}

	if (frame.getVideoMode().getPixelFormat() != 100 && frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_100_UM)
	{
		return rc;
	}
	DepthPixel *pDepth = (DepthPixel*)frame.getData();	
	/////////////////////////////////////////
	//预处理
	int i,j,k,q;
	int PicHeight,PicWidth;
	PicHeight = frame.getHeight();
	PicWidth = frame.getWidth();

	for (i=0;i<PicHeight;i++)
	{
		//扫描下一行的初始化 
		for (j=0;j<PicWidth;j++)
		{
			//平面化超过距离范围的物体，类似于在杆件后面设置了一块幕布
			if(pDepth[i*PicWidth+j]>DetectionRangeMax)
			{
				pDepth[i*PicWidth+j] = DetectionRangeMax;
			}
			//近距离以及物体阴影部分处理
			if(pDepth[i*PicWidth+j]<DetectionRangeMin)
			{
				pDepth[i*PicWidth+j] = DetectionRangeMax;
			}

			CoordinateConverter::convertDepthToWorld(depth,j,i,
				pDepth[i*PicWidth+j],&frame3D[i][j][0],&frame3D[i][j][1],&frame3D[i][j][2]);
			//深度坐标系->传感器坐标系
		}
	}
	return rc;
}

/******************************************************************************
* 函数：CPrimeSenseDet::ReadDepthImageFile(static float frame3D[][640][3])
* 功能：读取一帧深度图像文件的数据并转化成点云
*
*
* 返回：成功 ture ,失败 false
******************************************************************************/
bool CPrimeSenseDet::ReadDepthImageFile(static float frame3D[][640][3])
{
	ifstream ImageFile("image.txt");
	//	ImageFile("image.txt",ios::app);
	if (!ImageFile)
	{
		return false;
	}
	char PointDAT[100],tempNum[5][10];
	char *pPoint;
	int r=0,c=0,count=0,countIn=0;
	while(!ImageFile.eof())
	{
		ImageFile.getline(PointDAT,100,';');
		pPoint = PointDAT;
		count=0;
		countIn=0;
		while(*pPoint!='\0')
		{	
			if (*pPoint!=' ')
			{
				tempNum[count][countIn++] = *pPoint;
			}
			else
			{
				tempNum[count][countIn] = '\0';
				count++;
				countIn=0;
			}
			pPoint++;
		}
		r=atoi(tempNum[0]);
		c=atoi(tempNum[1]);
		frame3D[r][c][0] = atof(tempNum[2]);
		frame3D[r][c][1] = atof(tempNum[3]);
		frame3D[r][c][2] = atof(tempNum[4]);
	}
	ImageFile.close();
	return true;
}
/******************************************************************************
* 函数：CPrimeSenseDet::PolesDet(PoleInfo *poleINFO)
* 功能：杆件提取和检测算法
*
*
* 返回：提取杆件根数
******************************************************************************/
int CPrimeSenseDet::PolesDet(void)
{
	//初始化
	int i=0,j=0,k=0,q=0;   //循环变量
	OpenGL_Mode=FALSE;
	int PoleN = 0;
	g_PolesNum = 0;
	
	for (i=0;i<5;i++)
	{
		RSJ[i] = 0;
	}

	//获取深度图像数据
	static float Frame3D[480][640][3];  //将整个深度图像转化成3D真实空间点云。

	if (STATUS_ERROR == GetDepthImageData(Frame3D))
	{
		AfxMessageBox("Carmine 1.09 检测有误");

		DeteSymbol = false;
		return -1;
	}

	//double devicecount;
	//string deviceuri;
	//Array<DeviceInfo> aDeviceList;
	//const DeviceInfo& rDevInfo = aDeviceList[i];
	//OpenNI::enumerateDevices(&aDeviceList);
	//devicecount = aDeviceList.getSize();
	//cout<< aDeviceList.getSize() <<endl;
	//const char *uri = rDevInfo.getUri();
	
	

	//坐标系转化
	CoordinateConv();//求解相机坐标系在基坐标系下的表示，矩阵表示形式

	//判断从40行到100行中的杆件数目，并记录数目最多行的杆件边界点

	float CliffP[30][5];//单行出现的若干个悬崖点（两个点中距离近的点）以及该点在深度图像中的二维坐标//30个悬崖点是否足够？？？？
	int CliffPMark[30];//悬崖点正负标志,1表示正（前一点远），0表示负（后一点远）
	int CliffPn = 0;  //每行悬崖点个数
	float PolesStartBoundaryP[DetPoleNum][5]; //检测到杆件数目最多时的杆件第一个边界点（正悬崖点）以及该点在深度图像中的二维坐标
	float PolesEndBoundaryP[DetPoleNum][5]; //检测到杆件数目最多时的杆件第二个边界点（正悬崖点）以及该点在深度图像中的二维坐标
	float PolesStartBoundaryPTemp[DetPoleNum][5];
	float PolesEndBoundaryPTemp[DetPoleNum][5];
	int PoleNTemp = 0;
	for (i=LStart;i<LStart+LStep;i++)  //第10（5）到40（35）行中，求取杆件数目最多的行，杆件的个数（PoleN）和边界点PolesStartBoundaryP[i][k]、PolesEndBoundaryP[i][k]
	{
		//单行悬崖点检测
		for (j=0;j<640;j++)
		{
			if ((Frame3D[i][j][2]-Frame3D[i][j+1][2])>CliffDis)
			{
				for (k=0;k<3;k++)
				{
					CliffP[CliffPn][k] = Frame3D[i][j+1][k];  //记录两点中较近的点
					CliffPMark[CliffPn] = 1;
				}
				CliffP[CliffPn][3] = i;
				CliffP[CliffPn][4] = j+1;
				CliffPn++;
			}
			else if ((Frame3D[i][j][2]-Frame3D[i][j+1][2])<-CliffDis)
			{
				for (k=0;k<3;k++)
				{
					CliffP[CliffPn][k] = Frame3D[i][j][k]; //记录两点中较近的点
					CliffPMark[CliffPn] = 0;
				}
				CliffP[CliffPn][3] = i;
				CliffP[CliffPn][4] = j;
				CliffPn++;
			}
		}
		///////////////////////////////////
		//每检查完一行悬崖点后，对检测到的悬崖点重新整合
		for (j=0;j<CliffPn-1;j++)
		{
			if (CliffPMark[j]==1 && CliffPMark[j+1]==0)//悬崖点正负标志,1表示正（该点相对于前一点较近），0表示负（该点相对于后一点较近）
				                                       //该条件表示为某一物体的左右两个悬崖点
			{
				//检查是否满足可夹持条件
				if (P2Distance(CliffP[j],CliffP[j+1])<80*1.414 && P2Distance(CliffP[j],CliffP[j+1])>55)
				{
					for (k=0;k<5;k++)
					{
						PolesStartBoundaryPTemp[PoleNTemp][k] = CliffP[j][k];//对检测到的悬崖点中符合夹持杆件要求的重新整合
						PolesEndBoundaryPTemp[PoleNTemp][k] = CliffP[j+1][k];
					}
					PoleNTemp++;
				}
			}
		}
		CliffPn = 0;//悬崖点数目清零
		//////////////////////////////////////
		//求取杆件数最多时的杆件个数和标记整合（记录三维坐标和二维像素坐标）
		if (PoleNTemp>PoleN)
		{
			PoleN = PoleNTemp;
			for (i=0;i<PoleN;i++)
			{
				for (k=0;k<5;k++)
				{
					PolesStartBoundaryP[i][k] = PolesStartBoundaryPTemp[i][k];
					PolesEndBoundaryP[i][k] = PolesEndBoundaryPTemp[i][k];
				}
			}
		}
		PoleNTemp = 0;//临时杆件数目清零
	}
	if (PoleN > DETECT_POLE_NUM_MAX)//HU  为什么最多只有5根杆？？？

	{
		printf("More than max num\n");
		return -1;
	}

	//杆件信息结构体
	PoleInfo poleINFO[DETECT_POLE_NUM_MAX];
	(void)memset(poleINFO, 0, sizeof(poleINFO));//HU
	//////////////////////////////////////////////////////////////////////////
	//求出每一根杆件的外轮廓
	static float OutlineP[DetPoleNum][480][200][3] = {0};  //深度图像中同一行（DepthPixelY相等的）扫描杆件轮廓点
	                                                       //深度图像中某根杆的某行自边缘起某列点的三维坐标
	float P_BoundaryP[DetPoleNum][480][2][5] = {0};  //杆件轮廓边界点
	int PoleJudge[40][2];//存储拟杆件边界点（悬崖点）二维像素坐标中的j以及悬崖点的正负
	int PoleJudgeN =0;//拟杆件边界点（悬崖点）个数
	int PoleRowN[DetPoleNum] = {0};//属于某根杆的点云行数
	for (int Pole_i=0;Pole_i<PoleN;Pole_i++)
	{
		for (k=0;k<5;k++)//重新整合杆件数目最多行的悬崖点信息
		{
			P_BoundaryP[Pole_i][0][0][k] = PolesStartBoundaryP[Pole_i][k];
			P_BoundaryP[Pole_i][0][1][k] = PolesEndBoundaryP[Pole_i][k];
		}
		for (i=PolesStartBoundaryP[Pole_i][3];i<480;i++)  //从第Pole_i根杆件的第PolesStartBoundaryP[Pole_i][3]行像素开始扫描
		{
			//将每一行的探寻范围控制在0~640内
			if (P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][4]-50<0)
			{
				P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][4] = 50;
			}
			if (P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][4]+50>640)
			{
				P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][4] = 590;
			}
			////////////////////////////////////////////////////////
			//（以每行中符合夹持要求的杆件的左右两个悬崖点为起始，）
			//检测该杆件在该行中的悬崖点
			for (j=P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][4]-50;j<P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][4]+50;j++)
			{
				if ((Frame3D[i][j][2]-Frame3D[i][j+1][2])>CliffDis)
				{
					PoleJudge[PoleJudgeN][0] = j+1;//下一行中的拟杆件边界点（左边界）的二维像素坐标中的j
					PoleJudge[PoleJudgeN][1] = 1;//1表示正（该点相对于前一点较近），0表示负（该点相对于后一点较近）
					PoleJudgeN++;
				}
				else if ((Frame3D[i][j][2]-Frame3D[i][j+1][2])<-CliffDis)
				{
					PoleJudge[PoleJudgeN][0] = j;//下一行中的拟杆件边界点（右边界）的二维像素坐标中的j
					PoleJudge[PoleJudgeN][1] = 0;//1表示正（该点相对于前一点较近），0表示负（该点相对于后一点较近）
					PoleJudgeN++;
				}
			}
			//////////////////////////////////////////////////////
			if (PoleJudgeN>1)
			{
				int mar=0;//杆件数目
				for (j=0;j<PoleJudgeN-1;j++)
				{
					//对检测到的拟杆件边界点中符合夹持杆件要求的重新整合，将同一根杆件的左右两个拟边界点放在一个数组中统一表示并且
					//存储深度图像中同一行扫描杆件轮廓点
					if (PoleJudge[j][1]==1 && PoleJudge[j+1][1]==0
						&&P2Distance(Frame3D[i][PoleJudge[j][0]],Frame3D[i][PoleJudge[j+1][0]])<80*1.414 
						&& P2Distance(Frame3D[i][PoleJudge[j][0]],Frame3D[i][PoleJudge[j+1][0]])>45)
					{
						mar++;
						//存储该行拟杆件边界点（悬崖点）的三维坐标和二维像素坐标
						for (k=0;k<3;k++)
						{
							P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][0][k] = Frame3D[i][PoleJudge[j][0]][k];
							P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][1][k] = Frame3D[i][PoleJudge[j+1][0]][k];
						}
						P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][0][3] = i;
						P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][1][3] = i;
						P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][0][4] = PoleJudge[j][0];
						P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][1][4] = PoleJudge[j+1][0];
						///////////////////////////////////////////////////////////////
						int tempp=0;//属于该杆件的该行轮廓点数量
						float DisMax = Frame3D[i][PoleJudge[j][0]][2];
						if (Frame3D[i][PoleJudge[j+1][0]][2]>Frame3D[i][PoleJudge[j][0]][2])//确定两个拟杆件边界点中深度值较大者
						{
							DisMax = Frame3D[i][PoleJudge[j+1][0]][2];
						}
						for (q=PoleJudge[j][0];q<PoleJudge[j+1][0];q++)//存储深度图像中同一行扫描杆件轮廓点
						{
							if (Frame3D[i][q][2]<DisMax && Frame3D[i][q][2]>(DisMax-80*1.414))
							{
								for (k=0;k<3;k++)
								{
									OutlineP[Pole_i][PoleRowN[Pole_i]][tempp][k] = Frame3D[i][q][k]; 	
								}
								tempp++;
							}
						}
						if (tempp<((PoleJudge[j+1][0]-PoleJudge[j][0])*0.8))//
						{
							PoleRowN[Pole_i]--;
						}
					}
				}
				if (mar==0)
				{
					PoleRowN[Pole_i]--;
				}
			} 
			else
			{
				PoleRowN[Pole_i]--;
			}
			PoleJudgeN = 0;
			PoleRowN[Pole_i]++;
		}
	}

// 	float TempCloudP[3];
// 	for (i=0;i<PoleN;i++)
// 	{
// 		for (j=0;j<PoleRowN[i];j++)
// 		{
// 			for (int qq=0;qq<200;qq++)
// 			{
// 				if (OutlineP[i][j][qq][2]==0)
// 				{
// 					break;
// 				}
// 				PointCoordinateC(&PUntifyMtx,OutlineP[i][j][qq],TempCloudP);
// 				pointcloud[i][DrawN[i]][0] = TempCloudP[0]/1000.0;
// 				pointcloud[i][DrawN[i]][1] = TempCloudP[1]/1000.0;
// 				pointcloud[i][DrawN[i]][2] = TempCloudP[2]/1000.0;
// 				// 				for (int aase=0;aase<3;aase++)
// 				// 				{
// 				// 					pointcloud[i][DrawN[i]][aase]=OutlineP[i][j][qq][aase]/1000.0;
// 				// 				}
// 				DrawN[i]++;
// 			}
// 		}
// 	}

	////////////////////////////////////////////////////////////////////
	//寻找杆件的最近线（脊）
	////////////////////////////////////////////////////////////////////
	float NearestLineP[DetPoleNum][480][3]={0}; //每一行中最近的点（脊点）
	int NearestLinePN[5][480];   //每一行中最近的点在该行中的位置
	//	float* nearpp = &nearp[0][0];
	for (i=0;i<PoleN;i++)
	{
		for (j=0;j<PoleRowN[i];j++)
		{
			// 	*(nearpp++) = P_BoundaryP[i][j][0]/200.0;
			// 	*(nearpp++) = P_BoundaryP[i][j][1]/200.0;
			// 	*(nearpp++) = (P_BoundaryP[i][j][2]-5)/200.0;

			NearestLineP[i][j][2] = 1600;//故意设大些，以保证取得每行中的最近点
			for(int rowPN=0;rowPN<300;rowPN++)
			{
				if (OutlineP[i][j][rowPN][2]==0)
				{
					break;
				}
				//rowPN表示属于杆件的点云个数，这里是要遍历属于杆的所有表面点，找出最近点
				if (NearestLineP[i][j][2]>OutlineP[i][j][rowPN][2])
				{
					NearestLineP[i][j][0]=OutlineP[i][j][rowPN][0];
					NearestLineP[i][j][1]=OutlineP[i][j][rowPN][1];
					NearestLineP[i][j][2]=OutlineP[i][j][rowPN][2];
					NearestLinePN[i][j] = rowPN;
				}
			}
			//NearestLineP[i][j][0] = neartempp[0];
			//*(nearpp++) = neartempp[0]/200.0;
			//NearestLineP[i][j][1] = neartempp[1];
			//*(nearpp++) = neartempp[1]/200.0;
			//NearestLineP[i][j][2] = neartempp[2];
			//*(nearpp++) = (neartempp[2]/*-5*/+30)/200.0;
		}
	}
	///////////////////////////////////////////////////////////
	//判断脊线是否落在平面内？？？？？？？？？？？？？？？？
	///////////////////////////////////////////////////////////
	for (i=0;i<PoleN;i++)
	{
		j=0;
		//计算中间像素行的点到脊点的距离小于15的点的数量
		while(P2Distance(OutlineP[i][PoleRowN[i]/2][j],NearestLineP[i][PoleRowN[i]/2])<15)
		{
			j++;
		}
		int k=j;
		if (k>198)
		{
			k=198;
		}
		//计算中间像素行的点到脊点的距离大于15的点的数量
		while(P2Distance(OutlineP[i][PoleRowN[i]/2][j],NearestLineP[i][PoleRowN[i]/2])>15)
		{
			j++;
		}
		if (j>198)
		{
			j=198;
		}
		// 		int addde= NearestLineP[i][PoleRowN[i]/2][2]-OutlineP[i][PoleRowN[i]/2][j][2];
		// 		int addd = NearestLineP[i][PoleRowN[i]/2][2]-OutlineP[i][PoleRowN[i]/2][k][2];
		if ((abs(NearestLineP[i][PoleRowN[i]/2][2]-OutlineP[i][PoleRowN[i]/2][j][2])<1.5)
			||(abs(NearestLineP[i][PoleRowN[i]/2][2]-OutlineP[i][PoleRowN[i]/2][k][2])<1.5))
		{
			InPlane++;
			if (InPlane>20)
			{
				AfxMessageBox("平面内");
				InPlane = 0;
				DeteSymbol = false;
			}
			//return -1;//20170215注释掉
			
		}
	}

	////////////////////////////////////////////////////////////////
	//记录扫描到的杆件的上下限
	// 	for (i=0;i<PoleN;i++)
	// 	{
	// 		*(PolesLimit++) = NearestLineP[i][0][1];
	// 		*(PolesLimit++) = NearestLineP[i][PoleRowN[i]-3][1];
	// 	}
	////////////////////////////////////////////////////////////////
	//拟合深度最近点直线
	///////////////////////////////////////////////////////////////
	float NearestLine[DetPoleNum][2][3];
	for (i=0;i<PoleN;i++)
	{
		//NearestLine[i][0]为脊线方向，NearestLine[i][1]为脊线中点
		//NearestLine[i][x],x=0为方向，x=1为位置，其中位置为剔除错误点后，各个坐标的平均值
		BoundaryLineFit(PoleRowN[i],NearestLineP[i][0],NearestLine[i][0],NearestLine[i][1]);
		// 		PCentralLine[i][0][0] = (NearestLine[i][1][0]+190*NearestLine[i][0][0])/200.0;
		// 		PCentralLine[i][0][1] = (NearestLine[i][1][1]+190*NearestLine[i][0][1])/200.0;
		// 		PCentralLine[i][0][2] = (NearestLine[i][1][2]+190*NearestLine[i][0][2]-5)/200.0;
		// 
		// 		PCentralLine[i][1][0] = (NearestLine[i][1][0]-190*NearestLine[i][0][0])/200.0;
		// 		PCentralLine[i][1][1] = (NearestLine[i][1][1]-190*NearestLine[i][0][1])/200.0;
		// 		PCentralLine[i][1][2] = (NearestLine[i][1][2]-190*NearestLine[i][0][2]-5)/200.0;
	}
	//曲直度（脊的拟合误差）
	for (i=0;i<PoleN;i++)
	{
		poleINFO[i].BendDeg = 0;//“脊”的拟合误差P24
		for (j=0;j<PoleRowN[i];j++)
		{
			poleINFO[i].BendDeg += PtoLineDis(NearestLineP[i][j],NearestLine[i][0],NearestLine[i][1]);// 点到直线的距离函数
		}
		poleINFO[i].BendDeg = poleINFO[i].BendDeg/PoleRowN[i];
	}
	//各个杆件长度
	for (i=0;i<PoleN;i++)
	{
		poleINFO[i].PoleLenght = P2Distance(NearestLineP[i][0],NearestLineP[i][PoleRowN[i]]);
		PoleLenghtTemp[i]=poleINFO[i].PoleLenght;
	}
	//////////////////////////////////////////////////////////////////////////
	//找出边缘距离背脊线所在平面f(x,y)=0（脊平面）最近的点，比较两边边缘最近点距离大小
	//////////////////////////////////////////////////////////////////////////
	//	float BackPlane[DetPoleNum][2][3];  //脊平面方程第一个表示平面方向向量，第二个表示面上一点
	float EdgeToBackLineDis[DetPoleNum][2]={{1000,1000},{1000,1000},{1000,1000},{1000,1000},{1000,1000}};//,{1000,1000},{1000,1000},{1000,1000},{1000,1000},{1000,1000}};  
	                                       //表示五根杆件的左右两个边缘距离脊平面
	float EBLDMax[10];  //两侧拟边界点到该行脊点最小距离中那边更大（两边最小距离中最大的一个）
	// 	for (i=0;i<PoleN;i++)
	// 	{
	// 		BackPlane[i][0][0] = 1.0;
	// 		BackPlane[i][0][1] = -NearestLine[i][0][0]/NearestLine[i][0][1];
	// 		BackPlane[i][0][2] = 0.0;
	// 		for (j=0;j<3;j++)
	// 		{
	// 			BackPlane[i][1][j] = NearestLine[i][1][j];
	// 		}
	// 	}
	///////////////////////////////////////////////////////////////////////////
	//提取夹板之间的判断区域
	///////////////////////////////////////////////////////////////////////////
	int checkhigh[5]={0};//杆长在70mm之内的点云行数
	for (i=0;i<PoleN;i++)
	{
		//杆长在70之内的点云行数
		while (P2Distance(OutlineP[i][(PoleRowN[i]-checkhigh[i])/2][10],OutlineP[i][(PoleRowN[i]+checkhigh[i])/2][10])<70)//？？？
		{
			checkhigh[i] = checkhigh[i]+2;
		}
	}
	float tempdis;
	for (i=0;i<PoleN;i++)
	{
		//在70mm杆长的范围内搜寻拟边界点到该行脊点的最近距离
		for (j=(PoleRowN[i]-checkhigh[i])/2;j<(PoleRowN[i]+checkhigh[i])/2;j++)//j=0;j<Xcounts;j++)
		{
			//计算起始拟边界点到该行的脊点的距离
			tempdis = P2Distance(P_BoundaryP[i][j][0],NearestLineP[i][j]);
			//			tempdis = PtoLineDis(P_BoundaryP[i][j][0],NearestLine[i][0],NearestLine[i][1]);
			// 			tempdis = PtoPlaneDis(TRUE,BackPlane[i][0],BackPlane[i][1],P_BoundaryP[i][j][0]);
			//记录最近距离
			if (EdgeToBackLineDis[i][0]>tempdis)
			{
				EdgeToBackLineDis[i][0] = tempdis;
			}
			//计算终止拟边界点到该行的脊点的距离
			tempdis = P2Distance(P_BoundaryP[i][j][1],NearestLineP[i][j]);
			//			tempdis = PtoLineDis(P_BoundaryP[i][j][1],NearestLine[i][0],NearestLine[i][1]);
			// 			tempdis = PtoPlaneDis(TRUE,BackPlane[i][0],BackPlane[i][1],P_BoundaryP[i][j][1]);
			//记录最近距离
			if (EdgeToBackLineDis[i][1]>tempdis)
			{
				EdgeToBackLineDis[i][1] = tempdis;
			}
		}
	}
	/////////////////////////////////////////////////////////////////////////
	//使得脊平面方向向量朝向距离大的一边
	/////////////////////////////////////////////////////////////////////////
	int CheckStartFlag[5]; //若为0表示左边大，为1表示右边大
	for (i=0;i<PoleN;i++)
	{
		EBLDMax[i] = EdgeToBackLineDis[i][0];
		if (EdgeToBackLineDis[i][0]>EdgeToBackLineDis[i][1])
		{
			CheckStartFlag[i] = 0;
			// 			for(j=0;j<2;j++)
			// 			{
			// 				BackPlane[i][0][j] = - BackPlane[i][0][j];
			// 			}
		}
		else
		{
			CheckStartFlag[i] = 1;
			EBLDMax[i] = EdgeToBackLineDis[i][1];
		}
		/*		VectorUnitization(BackPlane[i][0]);*/
	}
	///////////////////////////////////////////////////////////////////////
	//寻找判断方杆圆杆的区域,求解该区域的左右两边的夹板面
	//////////////////////////////////////////////////////////////////////
	// 	float SplintPlaneP[DetPoleNum][2][3];  //判断方杆圆杆的区域的左右两边的夹板面上的点，离脊平面由近及远
	// 	float SplintDis[DetPoleNum];
	// 	for (i=0;i<PoleN;i++)
	// 	{
	// 		if (EBLDMax[i]>45)
	// 		{
	// 			SplintDis[i] = 45;
	// 		}
	// 		else
	// 			SplintDis[i] = EBPDMax[i];
	// 		for (j=0;j<3;j++)
	// 		{
	// 			SplintPlaneP[i][0][j] = BackPlane[i][1][j] + (/*EBPDMax[i]/2-*/10)*BackPlane[i][0][j];
	// 			SplintPlaneP[i][1][j] = BackPlane[i][1][j] + (SplintDis[i]-1)*BackPlane[i][0][j];
	// 		}
	// 	}
	///////////////////////////////////////////////////////////////////////////
	//提取夹板之间的判断区域
	///////////////////////////////////////////////////////////////////////////
	// 	int checkhigh[5]={0};
	// 	for (i=0;i<PoleN;i++)
	// 	{
	// 		while (P2Distance(OutlineP[i][(Xcounts-checkhigh[i])/2][10],OutlineP[i][(Xcounts+checkhigh[i])/2][10])<100)
	// 		{
	// 			checkhigh[i] = checkhigh[i]+2;
	// 		}
	// 	}
	int CheckAreaP[DetPoleNum][200][3] = {0};  
	for (i=0;i<PoleN;i++)
	{
		int k=0;
		if (CheckStartFlag[i]==0)  //左边大
		{
			for (j=(PoleRowN[i]-checkhigh[i])/2;j<(PoleRowN[i]+checkhigh[i])/2;j++)
			{
				CheckAreaP[i][k][0] = 0;//杆件某一行（k）点云中在脊线左侧或者右侧大于最小距离（EBLDMax[i]-2）的点的个数
				while (P2Distance(OutlineP[i][j][CheckAreaP[i][k][0]],NearestLineP[i][j])>(EBLDMax[i]-2))
				{
					CheckAreaP[i][k][0] ++;
				}
				CheckAreaP[i][k][1] = 0;//杆件某一行（k）点云中在脊线左侧或者右侧大于距离（11）的点的个数
				while (P2Distance(OutlineP[i][j][CheckAreaP[i][k][1]],NearestLineP[i][j])>11)
				{
					CheckAreaP[i][k][1] ++;
				}
				CheckAreaP[i][k][2] = j;//杆件某一行的行数（）
				k++;
			}
		}
		else if (CheckStartFlag[i]==1) //右边大
		{
			for (j=(PoleRowN[i]-checkhigh[i])/2;j<(PoleRowN[i]+checkhigh[i])/2;j++)
			{
				CheckAreaP[i][k][0] = NearestLinePN[i][j];
				while (P2Distance(OutlineP[i][j][CheckAreaP[i][k][0]],NearestLineP[i][j])<(EBLDMax[i]-2))
				{
					CheckAreaP[i][k][0] ++;
				}
				CheckAreaP[i][k][1] = NearestLinePN[i][j];
				while (P2Distance(OutlineP[i][j][CheckAreaP[i][k][1]],NearestLineP[i][j])<11)
				{
					CheckAreaP[i][k][1] ++;
				}
				CheckAreaP[i][k][2] = j;
				k++;
			}
		}
		// 		for (j=(PoleRowN[i]-checkhigh[i])/2;j<(PoleRowN[i]+checkhigh[i])/2;j++)
		// 		{
		// 			while(PtoPlaneDis(TRUE,BackPlane[i][0],SplintPlaneP[i][0],OutlineP[i][j][CheckAreaP[i][k][0]])>2)
		// 			{
		// 				CheckAreaP[i][k][0] ++;
		// 			}
		// 			CheckAreaP[i][k][0] = CheckStartN[i][j];
		// 			while (P2Distance(OutlineP[i][j][CheckAreaP[i][k][0]],NearestLineP[i][j])>(EBLDMax[i]-2))
		// 			{
		// 				CheckAreaP[i][k][0] ++;
		// 			}
		// 			CheckAreaP[i][k][1] = CheckStartN[i][j];
		// 			while (P2Distance(OutlineP[i][j][CheckAreaP[i][k][0]],NearestLineP[i][j]))
		// 			{
		// 				CheckAreaP[i][k][1] ++;
		// 			}
		// 			while(PtoPlaneDis(TRUE,BackPlane[i][0],SplintPlaneP[i][1],OutlineP[i][j][CheckAreaP[i][k][1]])>2)
		// 			{
		// 				CheckAreaP[i][k][1] ++;
		// 			}
	}
	////////////////////////////////////////////////////////////////////////////////
	//判断方杆圆杆,
	////////////////////////////////////////////////////////////////////////////////
	float CheckPoint[DetPoleNum][5000][3];//某根杆件某行中距离该行脊点在（(EBLDMax[i]-2)，11）范围内的点的坐标
	int CheckTriangleN[DetPoleNum][3];   //检测区域三个点的坐标位置
	for (i=0;i<PoleN;i++)
	{
		CheckTriangleN[i][0] = 0;
		int checkn=0;
		for (int k=0;k<checkhigh[i];k++)
		{		
			if (CheckAreaP[i][k][0]>CheckAreaP[i][k][1])
			{
				for (j=CheckAreaP[i][k][1];j<CheckAreaP[i][k][0];j++)
				{
					for (int z=0;z<3;z++)
					{
						CheckPoint[i][checkn][z] = OutlineP[i][CheckAreaP[i][k][2]][j][z];
					}
					checkn++;
				}
				if (k==0)
				{
					CheckTriangleN[i][1] = checkn-1;
				}
			}
			else
			{
				for (j=CheckAreaP[i][k][0];j<CheckAreaP[i][k][1];j++)
				{
					for (int z=0;z<3;z++)
					{
						CheckPoint[i][checkn][z] = OutlineP[i][CheckAreaP[i][k][2]][j][z];
					}
					checkn++;
				}
				if (k==0)
				{
					CheckTriangleN[i][1] = checkn-1;
				}
			}
		}
		CheckTriangleN[i][2] = checkn-1;
	}

	// 	for (i=0;i<PoleN;i++)
	// 	{
	// 		for (j=0;j<CheckTriangleN[i][2];j++)
	// 		{
	// 			*(nearpp++) = CheckPoint[i][j][0]/200.0;
	// 			*(nearpp++) = CheckPoint[i][j][1]/200.0;
	// 			*(nearpp++) = (CheckPoint[i][j][2]-5)/200.0;
	// 		}
	// 	}
	float CheckPlane[DetPoleNum][2][3]={0};  //检验平面方程
	float CheckLine[DetPoleNum][3]={0};
	for (i=0;i<PoleN;i++)
	{
		if (CheckPoleStyle(CheckTriangleN[i][2],CheckPoint[i][0])>0.26)
		{
			poleINFO[i].PoleStyle = 1;  //圆杆
			RSJ[i] = 1;
		}
		else
		{
			//poleINFO[i].PoleStyle = -1;  //方杆
			//RSJ[i] = -1;
			//PlaneFitting(CheckTriangleN[i][2],CheckPoint[i][0],CheckPlane[i][0]);
			//CheckLine[i][0] = CheckPlane[i][0][1]*NearestLine[i][0][2] - CheckPlane[i][0][2]*NearestLine[i][0][1];
			//CheckLine[i][1] = CheckPlane[i][0][2]*NearestLine[i][0][0] - CheckPlane[i][0][0]*NearestLine[i][0][2];
			//CheckLine[i][2] = CheckPlane[i][0][0]*NearestLine[i][0][1] - CheckPlane[i][0][1]*NearestLine[i][0][0];
			//VectorUnitization(CheckLine[i]);
		}
	}
	//计算杆件中心线方程和杆件尺寸
	for (i=0;i<PoleN;i++)
	{
		int openglk;
		float PolesCentre[3];
		float PolesTempCentre[3];	
		float tempD;
		float R[2];
		//圆杆
		if (poleINFO[i].PoleStyle==1)
		{
			///////////////////////////
			poleINFO[i].Cylinder_Dis = 1000;
			for (k=0;k<3;k++)
			{
				poleINFO[i].PoleFunc[0][k] = NearestLine[i][0][k];
				poleINFO[i].Pose[k] = 0;

				PCentralLineTemp[i][0][k] = NearestLine[i][0][k];//GU
			}
			///////////////////////////////寻找杆件轴线的最优化方法GU
			for (j=0;j<11;j++)
			{
				PolesTempCentre[0] = NearestLine[i][1][0]+(10-2*j);//范围：±10
				PolesTempCentre[1] = NearestLine[i][1][1];
				PolesTempCentre[2] = NearestLine[i][1][2]+10;//
				for (k=0;k<50/StepLen;k++)
				{
					PolesTempCentre[2] = PolesTempCentre[2]+StepLen;
					CheckPtoLine(CheckRowN*200,OutlineP[i][(PoleRowN[i]-CheckRowN)/2][0],NearestLine[i][0],PolesTempCentre,R);
					//////////////////////////////////
					//////////////////////////////////////////////////////////////////////////////////////
					tempD = R[1]-R[0];
					if (poleINFO[i].Cylinder_Dis>tempD)
					{
						poleINFO[i].Cylinder_Dis = tempD;
						for (k=0;k<3;k++)
						{
							PolesCentre[k] = PolesTempCentre[k];//该处求得杆件的中点位置P25
						}
					}
				}

			}
			for (k=0;k<3;k++)
			{
					poleINFO[i].PoleFunc[1][k] = PolesCentre[k];

				PCentralLineTemp[i][1][k] = PolesCentre[k];//GU
			}
			//杆件半径
			poleINFO[i].PoleDiameter = PtoLineDis(PolesCentre,NearestLine[i][0],NearestLine[i][1]);
// 			if (poleINFO[i].PoleDiameter<25 || poleINFO[i].PoleDiameter>=150)
// 			{
// 					for (int k=0; k<3; k++)
// 					{
// 						PCentralLineTemp[i][0][k]=0;
// 						PCentralLineTemp[i][1][k]=0;
// 					}
// 				PoleN = PoleN - 1;
// 			}
			////PolesDim[i] = PolesCentre[2]-NearestLine[i][1][2];
			//PCentralLine[i][0][0] = PolesCentre[0]+220*NearestLine[i][0][0];
			//PCentralLine[i][0][1] = PolesCentre[1]+220*NearestLine[i][0][1];
			//PCentralLine[i][0][2] = PolesCentre[2]+220*NearestLine[i][0][2];

			////for (int k = 0; k < 3; k++)//GU
			////{
			////	PCentralLineTemp[i][0][k] = PCentralLine[i][0][k];
			////}

			//PointCoordinateC(&PUntifyMtx,PCentralLine[i][0],PCentralLine[i][0]);
			//for (openglk=0;openglk<3;openglk++)
			//{
			//	PCentralLine[i][0][openglk] = PCentralLine[i][0][openglk]/1000.0;
			//}
			//PCentralLine[i][1][0] = PolesCentre[0]-220*NearestLine[i][0][0];
			//PCentralLine[i][1][1] = PolesCentre[1]-220*NearestLine[i][0][1];
			//PCentralLine[i][1][2] = PolesCentre[2]-220*NearestLine[i][0][2];

			////for (int k = 0; k < 3; k++)//GU
			////{
			////	PCentralLineTemp[i][1][k] = PCentralLine[i][1][k];
			////}

			//PointCoordinateC(&PUntifyMtx,PCentralLine[i][1],PCentralLine[i][1]);
			//for (openglk=0;openglk<3;openglk++)
			//{
			//	PCentralLine[i][1][openglk] = PCentralLine[i][1][openglk]/1000.0;
			//	PonCentline[i][openglk] = PolesCentre[openglk]/1000.0;
			//}
		}
		/////////////////////////////////////////
		//方杆
		//if (-1 == poleINFO[i].PoleStyle)
		//{
		//	////////////////////
		//	for (k=0;k<3;k++)
		//	{
		//		poleINFO[i].Pose[k] = NearestLine[i][1][k];
		//	}
		//	///////////////////////////////////////
		//	poleINFO[i].Cylinder_Dis = 1000;
		//	for (k=0;k<3;k++)
		//	{
		//		poleINFO[i].PoleFunc[0][k] = NearestLine[i][0][k];
		//		//	PolesTempCentre[k] = NearestLine[i][1][k];
		//	}
		//	//	PolesTempCentre[0] -= 50;
		//	///////////////////////////
		//	for (j=0;j<21;j++)
		//	{
		//		//	PolesTempCentre[0]++;
		//		//	PolesTempCentre[2] = NearestLine[i][1][2]+10;
		//		for (q=0;q<3;q++)
		//		{
		//			PolesTempCentre[q] = CheckPlane[i][1][q]+(20-2*j)*CheckLine[i][q]+10*CheckPlane[i][0][q];
		//		}
		//		for (k=0;k<60/StepLen;k++)
		//		{
		//			//	PolesTempCentre[2]++;
		//			for (q=0;q<3;q++)
		//			{
		//				PolesTempCentre[q] = PolesTempCentre[q]+StepLen*CheckPlane[i][0][q];
		//			}
		//			CheckPtoLine(CheckRowN*200,OutlineP[i][(PoleRowN[i]-CheckRowN)/2][0],NearestLine[i][0],PolesTempCentre,R);
		//			//////////////////////////////////
		//			//////////////////////////////////////////////////////////////////////////////////////
		//			tempD = R[1]-R[0];
		//			if (poleINFO[i].Cylinder_Dis>tempD)
		//			{
		//				poleINFO[i].Cylinder_Dis = tempD;
		//				for (q=0;q<3;q++)
		//				{
		//					PolesCentre[q] = PolesTempCentre[q];
		//				}
		//			}
		//		}
		//	}
		//	for (q=0;q<3;q++)
		//	{
		//		poleINFO[i].PoleFunc[1][q] = PolesCentre[q];
		//	}
		//	//杆件直径
		//	poleINFO[i].PoleDiameter = PtoLineDis(PolesCentre,NearestLine[i][0],NearestLine[i][1]);
		//	PCentralLine[i][0][0] = PolesCentre[0]+220*NearestLine[i][0][0];//？？？220
		//	PCentralLine[i][0][1] = PolesCentre[1]+220*NearestLine[i][0][1];
		//	PCentralLine[i][0][2] = PolesCentre[2]+220*NearestLine[i][0][2];

		//	for (int k = 0; k < 3; k++)//GU
		//	{
		//		PCentralLineTemp[i][0][k] = PCentralLine[i][0][k];
		//	}
		//	
		//	PointCoordinateC(&PUntifyMtx,PCentralLine[i][0],PCentralLine[i][0]);
		//	for (openglk=0;openglk<3;openglk++)
		//	{
		//		PCentralLine[i][0][openglk] = PCentralLine[i][0][openglk]/1000.0;
		//	}
		//	PCentralLine[i][1][0] = PolesCentre[0]-220*NearestLine[i][0][0];
		//	PCentralLine[i][1][1] = PolesCentre[1]-220*NearestLine[i][0][1];
		//	PCentralLine[i][1][2] = PolesCentre[2]-220*NearestLine[i][0][2];

		//	for (int k = 0; k < 3; k++)//GU
		//	{
		//		PCentralLineTemp[i][1][k] = PCentralLine[i][1][k];
		//	}

		//	PointCoordinateC(&PUntifyMtx,PCentralLine[i][1],PCentralLine[i][1]);
		//	for (openglk=0;openglk<3;openglk++)
		//	{
		//		PCentralLine[i][1][openglk] = PCentralLine[i][1][openglk]/1000.0;
		//		PonCentline[i][openglk] = PolesCentre[openglk]/1000.0;
		//	}
		//}
		//////////////////////////////////////////////////////////
		//二次判断方杆圆杆, 如果已经判断为圆杆的，那就为圆杆，如果判断为方杆的，但是Dmin<5,则化为圆杆


		//if (poleINFO[i].PoleStyle!=Round)//GU  避免测试过程中检测到方杆机器人仍会运动的情况
		//{
		//	for (k=0;k<3;k++)
		//	{
		//		PolesCentre[k] = 0;//该处求得杆件的中点位置P25
		//	}
		//	for (i=0;i<PoleN;i++)
		//	{
		//		for (k=0;k<3;k++)
		//		{
		//			NearestLine[i][0][k]=0;
		//		}
		//		
		//	}
		//}



		if (poleINFO[i].Cylinder_Dis>5 && poleINFO[i].PoleStyle==Square)
		{
			poleINFO[i].PoleStyle==Square;
		}
		else
			poleINFO[i].PoleStyle==Round;
	}


	for (i=0; i<=PoleN; i++)
	{
		PoleStyleTemp[i] = poleINFO[i].PoleStyle;
	}


	g_PolesNum = PoleN;
	//清零静态数组
	memset(Frame3D,0,480*640*3*sizeof(float));
	memset(OutlineP,0,DetPoleNum*480*200*3*sizeof(float));
	return PoleN;
}

/******************************************************************************
* 函数：CPrimeSenseDet::DetectPoleEnvInImage()
* 功能：获取一帧深度图像中的简化杆件信息
*
*
* 返回：成功 ture ,失败 false
******************************************************************************/
bool CPrimeSenseDet::DetectPoleEnvInImage()
{
	//初始化
	int i=0,j=0,k=0,q=0;  //循环变量
	OpenGL_Mode=TRUE;    
	unsigned char PoleN = 0;   //检测杆件数量
	g_PolesNum = 0;      //检测杆件数量
	for (i=0;i<5;i++)   //OpenGL控制变量
	{
		DrawN[i] = 0;
		RSJ[i] = 0;   
	}

	//针对当前获取图像所在的时刻，读取机器人关节信息
	CoordinateConv();

	//获取深度图像数据
	static float Frame3D[480][640][3];  //将整个深度图像转化成3D真实空间点云。
	if (STATUS_ERROR == GetDepthImageData(Frame3D))
	{
		AfxMessageBox("Carmine 1.09 检测有误");
		return false;
	}

	//文件数据读取
// 	if (!ReadDepthImageFile(Frame3D))
// 	{
// 		return false;
// 	}
	
	/////////////////////////////////////////////////////////////////////////////////////
	
	//横向扫描
	{
		float CliffP[30][4];//单行出现的若干个悬崖点（两个点中距离近的点）以及该点在深度图像中的二维坐标
		char CliffPMark[30];//悬崖点正负标志,1表示正（前一点远），0表示负（后一点远）
		unsigned short CliffPN = 0;  //每行悬崖点个数
		float PolesStartBoundaryP[DetPoleNum][4]; //检测到杆件数目最多时的杆件第一个边界点（正悬崖点）以及该点在深度图像中的二维坐标
		float PolesEndBoundaryP[DetPoleNum][4]; //检测到杆件数目最多时的杆件第二个边界点（正悬崖点）以及该点在深度图像中的二维坐标
		// 	float PolesStartBoundaryPTemp[DetPoleNum][5];
		// 	float PolesEndBoundaryPTemp[DetPoleNum][5];
		unsigned char PoleNTemp = 0;
		unsigned short ContiuneMissLineN;
		unsigned short LS;//将图像划分为10格，从每个开始寻找杆件
//		unsigned char PicPoleNum = ListPoleNum+1;  //此副图像里面最先一根杆件在链表中的序列
		//	float J_NewPolePoint[DetPoleNum][3];
		//	unsigned short RowOffset=0;
		for (char PicBlock=0; PicBlock<((480-LStart)/LStep); ++PicBlock)
		{
			//	float StartRidgePoint[DetPoleNum][3]={0};  //杆件头一行脊点
			LS=LStart+PicBlock*LStep;
			for (j=0;j<640;j++)
			{
				if ((Frame3D[LS][j][2]-Frame3D[LS][j+1][2])>CliffDis)
				{
					for (k=0;k<3;k++)
					{
						CliffP[CliffPN][k] = Frame3D[LS][j+1][k];  //记录两点中较近的点
						CliffPMark[CliffPN] = 1;
					}
					/*CliffP[CliffPN][3] = i;*/
					CliffP[CliffPN][3] = j+1;
					CliffPN++;
				}
				else if ((Frame3D[LS][j][2]-Frame3D[LS][j+1][2])<-CliffDis)
				{
					for (k=0;k<3;k++)
					{
						CliffP[CliffPN][k] = Frame3D[LS][j][k]; //记录两点中较近的点
						CliffPMark[CliffPN] = 0;
					}
					/*CliffP[CliffPn][3] = i;*/
					CliffP[CliffPN][3] = j;
					CliffPN++;
				}
			}

			///////////////////////////////////
			for (j=0;j<CliffPN-1;j++)
			{
				if (CliffPMark[j]==1 && CliffPMark[j+1]==0)
				{
					if (P2Distance(CliffP[j],CliffP[j+1])<60*1.414 && P2Distance(CliffP[j],CliffP[j+1])>50)
					{
						for (k=0;k<4;k++)
						{
							PolesStartBoundaryP[PoleNTemp][k] = CliffP[j][k];
							PolesEndBoundaryP[PoleNTemp][k] = CliffP[j+1][k];
						}
						PoleNTemp++;
					}
				}
			}
			CliffPN=0;

			float CentralP[3];
            //比较重复杆件
			if (m_pLocalEnvList->begin->pnext != NULL)
			{
				//对比所有已经获取的杆件，两点好处：1，针对此帧图像中的杆件判断重复，2，正对不同帧图像杆件判断重复
				for (i=0;i<PoleNTemp;i++)
				{
					for (int qq=0;qq<3;++qq)
					{
						CentralP[qq] = (PolesEndBoundaryP[i][qq]+PolesStartBoundaryP[i][qq])/2;
					}
					CheckRepeatPole(CentralP, &PolesStartBoundaryP[i][2]);
				}

			}
			//////////////////////////////////////////////////////////////////////////
			//求出每一根杆件的外轮廓
			//////////////////////////////////////////////////////////////////////////
			//static float OutlineP[DetPoleNum][480][200][3] = {0};  //深度图像中同一行（DepthPixelY相等的）扫描杆件轮廓点
			float P_BoundaryP[DetPoleNum][480][2][4] = {0};  //杆件轮廓边界点
			int PoleJudge[30][2]={0};
			int PoleJudgeN =0;
			int PoleRowN[DetPoleNum] = {0};
			for (int Pole_i=0; Pole_i<PoleNTemp; Pole_i++)
			{
				if (PolesStartBoundaryP[Pole_i][2]==0)
				{
					continue;
				}
				float RidgePoint[480][3]={0};
				for (k=0;k<4;k++)
				{
					P_BoundaryP[Pole_i][0][0][k] = PolesStartBoundaryP[Pole_i][k];
					P_BoundaryP[Pole_i][0][1][k] = PolesEndBoundaryP[Pole_i][k];
				}
				for (i=LS+1;i<480;i++)  //从第Pole_i根杆件的第LS行开始扫描
				{
					//将每一行的探寻范围控制在0~640内
					if (P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][3]-50<0)
					{
						P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][3] = 50;
					}
					if (P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][3]+50>640)
					{
						P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][3] = 590;
					}
					////////////////////////////////////////////////////////
					//杆件两边界点前后50pixels范围开始扫描
					for (j=P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][3]-50;j<P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][3]+50;j++)
					{
						if ((Frame3D[i][j][2]-Frame3D[i][j+1][2])>CliffDis)
						{
							PoleJudge[PoleJudgeN][0] = j+1;  //记录悬崖点在改行的位置和性质
							PoleJudge[PoleJudgeN][1] = 1;
							PoleJudgeN++;
						}
						else if ((Frame3D[i][j][2]-Frame3D[i][j+1][2])<-CliffDis)
						{
							PoleJudge[PoleJudgeN][0] = j;  //记录悬崖点在改行的位置和性质
							PoleJudge[PoleJudgeN][1] = 0;
							PoleJudgeN++;
						}
					}
					//////////////////////////////////////////////////////
					    bool isRidgePoint = false;
						for (j=0;j<PoleJudgeN-1;j++)
						{
							if (PoleJudge[j][1]==1 && PoleJudge[j+1][1]==0
								&&P2Distance(Frame3D[i][PoleJudge[j][0]],Frame3D[i][PoleJudge[j+1][0]])<80*1.414 
								&& P2Distance(Frame3D[i][PoleJudge[j][0]],Frame3D[i][PoleJudge[j+1][0]])>45)  //满足杆件尺寸条件
							{
								for (k=0;k<3;k++)
								{
									P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][0][k] = Frame3D[i][PoleJudge[j][0]][k];
									P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][1][k] = Frame3D[i][PoleJudge[j+1][0]][k];
								}
								// 					P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][0][3] = i;
								// 					P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][1][3] = i;
								P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][0][3] = PoleJudge[j][0];
								P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][1][3] = PoleJudge[j+1][0];
								///////////////////////////////////////////////////////////////
								//寻找两个杆件边界点之间的脊点
								RidgePoint[PoleRowN[Pole_i]][2]=2000;
								{
									int templ=0;
									for (int f=PoleJudge[j][0];f<=PoleJudge[j+1][0];f++)
									{
										if (Frame3D[i][f][2]<RidgePoint[PoleRowN[Pole_i]][2])
										{
											templ=f;
											//  RidgePoint[PoleRowN[Pole_i]][0]=Frame3D[i][f][0];
											// 	RidgePoint[PoleRowN[Pole_i]][1]=Frame3D[i][f][1];
											RidgePoint[PoleRowN[Pole_i]][2]=Frame3D[i][f][2];
										}
									}
									RidgePoint[PoleRowN[Pole_i]][0]=Frame3D[i][templ][0];
									RidgePoint[PoleRowN[Pole_i]][1]=Frame3D[i][templ][1];
								}
								//在每一次产生新的RidgePoint脊点的时候，将其一上一个脊点进行比较
								if ((PoleRowN[Pole_i]!=0)&&(P2Distance(RidgePoint[PoleRowN[Pole_i]-1],RidgePoint[PoleRowN[Pole_i]])>40))
								{
									i=481;
									PoleRowN[Pole_i]--;
									break;
								}
								isRidgePoint = true;
								break;   //找到符合要求的，
							}
						}
						if (!isRidgePoint)
						{
							PoleRowN[Pole_i]--;
						}
					PoleJudgeN = 0;
					PoleRowN[Pole_i]++;
				}
				//检验捕获的杆件曲直度
				float PoleLine[2][3];   //杆件直线方程的方向向量和点
				BoundaryLineFit(PoleRowN[Pole_i],RidgePoint[0],PoleLine[0],PoleLine[1]);
				//曲直度检验
				float PolesBend = 0, PolesLen=0;
				for (j=0;j<PoleRowN[Pole_i];j++)
				{
					PolesBend += PtoLineDis(RidgePoint[j],PoleLine[0],PoleLine[1]);
				}
				PolesBend= PolesBend/PoleRowN[Pole_i];
				PolesLen=P2Distance(RidgePoint[0],RidgePoint[PoleRowN[Pole_i]-1]);

				if (PolesBend<PoleBendMax && PolesLen>PoleLenMin)
				{
					// 				for (k=0;k<3;k++)
					// 				{
					// 					J_NewPolePoint[PicPoleNum][k]=RidgePoint[48-RowOffset][k];
					// 				}
					// 				PicPoleNum++;
					////////////////////////////////////////////////////////
					//将获取到的杆件轴线转化到基座坐标系下
					/////////////////////////////////////////////////
					float S_Z[3]={0,0,1};
					PoleLine[1][2]+=abs(30.0/(sin(VectorAngle(PoleLine[0],S_Z))));  //沿着Z方向向后推一个杆件半径
					//////////////////////////////////////////////////
					PointCoordinateC(&PUntifyMtx,PoleLine[1],PoleLine[1]);
					//向量坐标变换，需要将转换矩阵的距离值变成零
					VectorCoordinateC(&PUntifyMtx,PoleLine[0],PoleLine[0]);
					////////////////////////////////
					VectorUnitization(PoleLine[0]);
					PoleNode NewPole;  //提取和检测出的新杆件
					for (k=0;k<3;k++)
					{
						NewPole.PoleFunc[0][k]=PoleLine[0][k];
						NewPole.PoleFunc[1][k]=PoleLine[1][k]; 
						NewPole.PoleEnd[0][k]=RidgePoint[0][k];
						NewPole.PoleEnd[1][k]=RidgePoint[PoleRowN[Pole_i]-1][k];
					}
					PointCoordinateC(&PUntifyMtx,NewPole.PoleEnd[0],NewPole.PoleEnd[0]);
					PointCoordinateC(&PUntifyMtx,NewPole.PoleEnd[1],NewPole.PoleEnd[1]);
					NewPole.num = 1+m_pLocalEnvList->end->num;
// 					NewPole.PoleLenght[0]=sqrt(SQUARE(P2Distance(PoleLine[1],NewPole.PoleEnd[0]))-900);
// 					NewPole.PoleLenght[1]=sqrt(SQUARE(P2Distance(PoleLine[1],NewPole.PoleEnd[1]))-900);
					PushList(m_pLocalEnvList,&NewPole);
				}

			}
			PoleNTemp=0;
		}
	}
	/////////////////////////////////////////////////////////////////////////////////////
	//纵向扫描
	{
		float CliffP[30][4];//单行出现的若干个悬崖点（两个点中距离近的点）以及该点在深度图像中的二维坐标
		char CliffPMark[30];//悬崖点正负标志,1表示正（前一点远），0表示负（后一点远）
		unsigned short CliffPN = 0;  //每行悬崖点个数
		float PolesStartBoundaryP[DetPoleNum][4]; //检测到杆件数目最多时的杆件第一个边界点（正悬崖点）以及该点在深度图像中的二维坐标
		float PolesEndBoundaryP[DetPoleNum][4]; //检测到杆件数目最多时的杆件第二个边界点（正悬崖点）以及该点在深度图像中的二维坐标
		unsigned char PoleNTemp = 0;
		//将图像划分为10格，从每个开始寻找杆件
		unsigned short CS;
		for (char PicBlock=0; PicBlock<((640-CStart)/CStep); PicBlock++)
		{
			CS=CStart+PicBlock*CStep;
			//找到杆件抽取的突破口
			for (i=0;i<480;i++)
			{
				if ((Frame3D[i][CS][2]-Frame3D[i+1][CS][2])>CliffDis)
				{
					for (k=0;k<3;k++)
					{
						CliffP[CliffPN][k] = Frame3D[i+1][CS][k];  //记录两点中较近的点
						CliffPMark[CliffPN] = 1;
					}
					/*CliffP[CliffPN][3] = i;*/
					CliffP[CliffPN][3] = i+1;
					CliffPN++;
				}
				else if ((Frame3D[i][CS][2]-Frame3D[i+1][CS][2])<-CliffDis)
				{
					for (k=0;k<3;k++)
					{
						CliffP[CliffPN][k] = Frame3D[i][CS][k]; //记录两点中较近的点
						CliffPMark[CliffPN] = 0;
					}
					/*CliffP[CliffPn][3] = i;*/
					CliffP[CliffPN][3] = i;
					CliffPN++;
				}
			}

			///////////////////////////////////
			for (j=0;j<CliffPN-1;j++)
			{
				if (CliffPMark[j]==1 && CliffPMark[j+1]==0)
				{
					if (P2Distance(CliffP[j],CliffP[j+1])<60*1.414 && P2Distance(CliffP[j],CliffP[j+1])>50)
					{
						for (k=0;k<4;k++)
						{
							PolesStartBoundaryP[PoleNTemp][k] = CliffP[j][k];
							PolesEndBoundaryP[PoleNTemp][k] = CliffP[j+1][k];
						}
						PoleNTemp++;
					}
				}
			}
			CliffPN=0;
			///////////////////////////////////////////////
			float CentralP[3];
			//比较重复杆件
			if (m_pLocalEnvList->begin->pnext != NULL)
			{
				//对比所有已经获取的杆件，两点好处：1，针对此帧图像中的杆件判断重复，2，正对不同帧图像杆件判断重复
				for (i=0;i<PoleNTemp;i++)
				{
					for (int qq=0;qq<3;++qq)
					{
						CentralP[qq] = (PolesEndBoundaryP[i][qq]+PolesStartBoundaryP[i][qq])/2;
					}
					CheckRepeatPole(CentralP, &PolesStartBoundaryP[i][2]);
				}

			}
			//////////////////////////////////////////////////////////////////////////
			//求出每一根杆件的外轮廓
			//////////////////////////////////////////////////////////////////////////
			//static float OutlineP[DetPoleNum][480][200][3] = {0};  //深度图像中同一行（DepthPixelY相等的）扫描杆件轮廓点
			float P_BoundaryP[DetPoleNum][640][2][4] = {0};  //杆件轮廓边界点
			int PoleJudge[30][2]={0};
			int PoleJudgeN =0;
			int PoleRowN[DetPoleNum] = {0};
			for (int Pole_i=0; Pole_i<PoleNTemp; Pole_i++)
			{
				if (PolesStartBoundaryP[Pole_i][2]==0)
				{
					continue;
				}
				float RidgePoint[640][3]={0};
				for (k=0;k<4;k++)
				{
					P_BoundaryP[Pole_i][0][0][k] = PolesStartBoundaryP[Pole_i][k];
					P_BoundaryP[Pole_i][0][1][k] = PolesEndBoundaryP[Pole_i][k];
				}
				for (j=CS+1;j<640;j++)  //从第Pole_i根杆件的第LS行开始扫描
				{
					//将每一行的探寻范围控制在0~480内
					if (P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][3]-50<0)
					{
						P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][3] = 50;
					}
					if (P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][3]+50>480)
					{
						P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][3] = 430;
					}
					////////////////////////////////////////////////////////
					//杆件两边界点前后50pixels范围开始扫描
					for (i=P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][3]-50;i<P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][3]+50;i++)
					{
						if ((Frame3D[i][j][2]-Frame3D[i+1][j][2])>CliffDis)
						{
							PoleJudge[PoleJudgeN][0] = i+1;  //记录悬崖点在改行的位置和性质
							PoleJudge[PoleJudgeN][1] = 1;
							PoleJudgeN++;
						}
						else if ((Frame3D[i][j][2]-Frame3D[i+1][j][2])<-CliffDis)
						{
							PoleJudge[PoleJudgeN][0] = i;  //记录悬崖点在改行的位置和性质
							PoleJudge[PoleJudgeN][1] = 0;
							PoleJudgeN++;
						}
					}
					//////////////////////////////////////////////////////
						bool isRidgePoint = false;
						for (i=0;i<PoleJudgeN-1;i++)
						{
							if (PoleJudge[i][1]==1 && PoleJudge[i+1][1]==0
								&&P2Distance(Frame3D[PoleJudge[i][0]][j],Frame3D[PoleJudge[i+1][0]][j])<80*1.414 
								&& P2Distance(Frame3D[PoleJudge[i][0]][j],Frame3D[PoleJudge[i+1][0]][j])>45)  //满足杆件条件
							{
								for (k=0;k<3;k++)
								{
									P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][0][k] = Frame3D[PoleJudge[i][0]][j][k];
									P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][1][k] = Frame3D[PoleJudge[i+1][0]][j][k];
								}
								P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][0][3] = PoleJudge[i][0];
								P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][1][3] = PoleJudge[i+1][0];
								///////////////////////////////////////////////////////////////
								//寻找两个杆件边界点之间的脊点
								RidgePoint[PoleRowN[Pole_i]][2]=2000;
								{
									int templ=0;
									for (int f=PoleJudge[i][0];f<=PoleJudge[i+1][0];f++)
									{
										if (Frame3D[f][j][2]<RidgePoint[PoleRowN[Pole_i]][2])
										{
											templ=f;
											RidgePoint[PoleRowN[Pole_i]][2]=Frame3D[f][j][2];
										}
									}
									RidgePoint[PoleRowN[Pole_i]][0]=Frame3D[templ][j][0];
									RidgePoint[PoleRowN[Pole_i]][1]=Frame3D[templ][j][1];
								}
								//在每一次产生新的RidgePoint脊点的时候，将其一上一个脊点进行比较
								if ((PoleRowN[Pole_i]!=0)&&(P2Distance(RidgePoint[PoleRowN[Pole_i]-1],RidgePoint[PoleRowN[Pole_i]])>40))
								{
									j=641;
									PoleRowN[Pole_i]--;
									break;
								}
								isRidgePoint = true;
								break;
							}
						}
						if (!isRidgePoint)
						{
							PoleRowN[Pole_i]--;
						}
					PoleJudgeN = 0;
					PoleRowN[Pole_i]++;
				}
				//检验捕获的杆件曲直度
				float PoleLine[2][3];   //杆件直线方程的方向向量和点
				BoundaryLineFit(PoleRowN[Pole_i],RidgePoint[0],PoleLine[0],PoleLine[1]);
				//曲直度检验
				float PolesBend = 0, PolesLen=0;
				for (k=0;k<PoleRowN[Pole_i];k++)
				{
					PolesBend += PtoLineDis(RidgePoint[k],PoleLine[0],PoleLine[1]);
				}
				PolesBend= PolesBend/PoleRowN[Pole_i];
				PolesLen=P2Distance(RidgePoint[0],RidgePoint[PoleRowN[Pole_i]-1]);

				if (PolesBend<PoleBendMax && PolesLen>PoleLenMin)
				{
					// 				for (k=0;k<3;k++)
					// 				{
					// 					J_NewPolePoint[PicPoleNum][k]=RidgePoint[48-RowOffset][k];
					// 				}
					// 				PicPoleNum++;
					////////////////////////////////////////////////////////
					//将获取到的杆件轴线转化到基座坐标系下
					/////////////////////////////////////////////////
					float S_Z[3]={0,0,1};
					PoleLine[1][2]+=abs(30.0/(sin(VectorAngle(PoleLine[0],S_Z))));  //沿着Z方向向后推一个杆件半径
					//////////////////////////////////////////////////
					PointCoordinateC(&PUntifyMtx,PoleLine[1],PoleLine[1]);
					//向量坐标变换，需要将转换矩阵的距离值变成零
					VectorCoordinateC(&PUntifyMtx,PoleLine[0],PoleLine[0]);
					////////////////////////////////
					VectorUnitization(PoleLine[0]);
					PoleNode NewPole;
					for (k=0;k<3;k++)
					{
						NewPole.PoleFunc[0][k]=PoleLine[0][k];
						NewPole.PoleFunc[1][k]=PoleLine[1][k];
						NewPole.PoleEnd[0][k]=RidgePoint[0][k];
						NewPole.PoleEnd[1][k]=RidgePoint[PoleRowN[Pole_i]-1][k];
					}
					PointCoordinateC(&PUntifyMtx,NewPole.PoleEnd[0],NewPole.PoleEnd[0]);
					PointCoordinateC(&PUntifyMtx,NewPole.PoleEnd[1],NewPole.PoleEnd[1]);
					NewPole.num = 1+m_pLocalEnvList->end->num;
// 					NewPole.PoleLenght[0]=PolesLen/2;
// 					NewPole.PoleLenght[1]=PolesLen/2;
					PushList(m_pLocalEnvList,&NewPole);
				}

			}
			PoleNTemp=0;
		}
	}
	LocalEnvListProc();
	//清零静态数组
	memset(Frame3D,0,480*640*3*sizeof(float));
	return true;
}      

float CPrimeSenseDet::CheckPoleStyle(int CheckPN,float* CheckPointP)
{
	int i=0;
	int k=0;
	float PlaneCurvature = 0;  //函数返回值，杆件局部面相对于平面和圆弧面之间的曲率，
	float OutlineP[10000][3]={0};
	float Plane[2][3]={0};
	//取出轮廓点的值
	for (i=0;i<CheckPN;i++)
	{
		for (k=0;k<3;k++)
		{
			OutlineP[i][k] = *(CheckPointP+i*3+k);
		}
	}
	//确定算子平面的法向量与其上一点
	PlaneFitting(CheckPN,CheckPointP,Plane[0]);

	//各个点到平面距离的平均值
	for (i=0;i<CheckPN;i++)
	{	
		//加入平方比较好，因为对于方杆平面来说
		//一般多为0.几，平反后更小，对于圆杆面
		//则距离为1.几，平方变大，这样二者区分明显
		PlaneCurvature += pow(PtoPlaneDis(true,Plane[0],Plane[1],OutlineP[i]),2);
		//PtoPlaneDis(true,Plane[0],Plane[1],OutlineP[i]);
	}
	PlaneCurvature = abs(PlaneCurvature)/CheckPN;
	return PlaneCurvature;
}

void CPrimeSenseDet::CheckPtoLine(int CheckPN, float* CheckP, float* LineDir, float* LineP, float* R)
{
	int i=0;
	int k=0;
	float OutlineP[10000][3]={0};
	//取出轮廓点的值
	R[0]=2000;
	R[1]=0;
	int num=0;
	for (i=0;i<CheckPN;i++)
	{
		for (k=0;k<3;k++)
		{
			OutlineP[num][k] = *(CheckP+i*3+k);
		}
		if (OutlineP[num][2] == 0)  //排除杆件轮廓数组中后面的空白
		{
			num--;
		}
		num++;
	}
	float TempD;
	for (i=0;i<num-1;i++)
	{
		TempD = PtoLineDis(OutlineP[i],LineDir,LineP);
		if (R[0]>TempD)
		{
			R[0] = TempD;
		}
		if (R[1]<TempD)
		{
			R[1] = TempD;
		}
	}

}//
