#include "F:\VS2010��RobotController\MoRoController\PrimeSenseDet\PrimeSenseDet.h"
#include "VectorAndMatrix.h"
#include <malloc.h>
#include "F:\VS2010��RobotController\MoRoController\Sensor.h"
#include "F:\VS2010��RobotController\MoRoController\RobotControl\Kine.h"
#include "F:\VS2010��RobotController\MoRoController\RobotControl\Setup.h"
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <vector>


#define DETECT_POLE_NUM_MAX 5//HU
//////////////////////////////////////
//ȫ�ֱ���
////////////////////////////////////

//OpenNI
using namespace std;

// �˼���ȡʶ��ͼ��
//float PCentralLine[DETECT_POLE_NUM_MAX][2][3];//HU
//float PonCentline[DETECT_POLE_NUM_MAX][3];//HU
int DrawN[DETECT_POLE_NUM_MAX];  //ÿ���˼�����������HU
int g_PolesNum;
int RSJ[DETECT_POLE_NUM_MAX];//HU

//��ľ�̬����
LocalEnvList* CPrimeSenseDet::m_pLocalEnvList;
MtxKine CPrimeSenseDet::PUntifyMtx;
MtxKine CPrimeSenseDet::G2_PUntifyMtx;
//bool CPrimeSenseDet::OpenGL_Mode = TRUE; //Opengl��ʾģʽ��1������ץ��ģʽ��2���˼��ع�ģʽ//GU160517��ԭλTRUE
bool CPrimeSenseDet::OpenGL_Mode = FALSE;//Opengl��ʾģʽ��1������ץ��ģʽ��2���˼��ع�ģʽ//GU160517��ԭλTRUE,�˴��޸���Ϊ�˽����δ�������߶Ի�ǰ���򿪷������ʱ���ֵ��ж�����
//ִ������ץ�е��̺߳���
int PoleStyle[10],PoleNum;

float PCentralLineTemp[DETECT_POLE_NUM_MAX][2][3];//GU  �ڶ���0��ʾ����1��ʾ���ĵ�
int PoleLenghtTemp[DETECT_POLE_NUM_MAX];                               //GU
int PoleStyleTemp[10];    //�˼�����

bool DeteSymbol;
int InPlane;

bool bool_Tool1;
bool bool_Tool2;

CPrimeSenseDet::CPrimeSenseDet(void)
{
	//��ʼ���˼�����
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
	Trans_PosToMtx(CSensorPos, &PUntifyMtx, 0);//�ô�ʵ���������ϵ�ڼг�������ϵ�µľ����ʾ
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
	//// ��ȡ�豸��Ϣ  
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
	//	AfxMessageBox("Camine��ʼ��ʧ�ܣ����ų����������");
	//	//return TRUE;
	//}
	//else{
	//	rc = device.open(deviceUri2);
	//	if (rc != STATUS_OK)
	//	{
	//		AfxMessageBox("Camine��ʧ�ܣ����ų����������");
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
	//			AfxMessageBox("�ֱ�������������");
	//			//return TRUE;
	//		}
	//		if (rc != STATUS_OK)
	//		{
	//			AfxMessageBox("Camine����ͼ��ʧ�ܣ����ų����������");
	//			//return TRUE;
	//		}
	//		rc = depth.start();
	//		if (rc != STATUS_OK)
	//		{
	//			AfxMessageBox("���ʧ�ܣ����ų����������");
	//			//return TRUE;
	//		}
	//		int changedStreamDummy;
	//		VideoStream* pStream = &depth;
	//		rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
	//		if (rc != STATUS_OK)
	//		{
	//			AfxMessageBox("���ʧ�ܣ����ų����������");
	//			//return TRUE;
	//		}
	//		else
	//			CamineCheck = TRUE;
	//	}

	//}
	//WriteDepthImageFile();
	//DepthImageCapture();
}

//��ָ�����豸
bool CPrimeSenseDet::OpenTheDevice(bool b_Tool1, bool b_Tool2)
{
	depth.stop();
	depth.destroy();
	device.close();
	OpenNI::shutdown();

	InPlane = 0;

	bool_Tool1 = b_Tool1;
	bool_Tool2 = b_Tool2;

	//�����豸��Uri
	char deviceUri1[256] = "\\\\?\\usb#vid_1d27&pid_0601&mi_00#6&31251095&1&0000#{c3b5f022-5a42-1980-1909-ea72095601b1}";//ASUS Xtion
	char deviceUri2[256] = "\\\\?\\usb#vid_1d27&pid_0609&mi_00#6&574dccc&1&0000#{c3b5f022-5a42-1980-1909-ea72095601b1}";//carmine1.09

	Status rc=STATUS_OK;
	rc = OpenNI::initialize();

	// ��ȡ�豸��Ϣ  
	Array<DeviceInfo> aDeviceList;  
	OpenNI::enumerateDevices( &aDeviceList );

	int devicenumber = aDeviceList.getSize();

	//if (rc != STATUS_OK)
	//{
	//	AfxMessageBox("Camine��ʼ��ʧ�ܣ����ų����������");
	//	return FALSE;
	//}

	//if (rc != STATUS_OK || devicenumber!=2)
	//{
	//	AfxMessageBox("Camine��ʼ��ʧ�ܣ����ų����������");
	//	//return TRUE;
	//}
	if (rc != STATUS_OK)
	{
		AfxMessageBox("Camine��ʼ��ʧ�ܣ����ų����������");
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
			//	AfxMessageBox("���������г����ļг�״̬������һ��һ��");
			//}

			if (rc != STATUS_OK)
			{
				AfxMessageBox("Camine��ʧ�ܣ����ų����������");
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
					AfxMessageBox("�ֱ�������������");
					return FALSE;
				}
				if (rc != STATUS_OK)
				{
					AfxMessageBox("Camine����ͼ��ʧ�ܣ����ų����������");
					return FALSE;
				}
				rc = depth.start();
				if (rc != STATUS_OK)
				{
					AfxMessageBox("���ʧ�ܣ����ų����������");
					return FALSE;
				}
				int changedStreamDummy;
				VideoStream* pStream = &depth;
				rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
				if (rc != STATUS_OK)
				{
					AfxMessageBox("���ʧ�ܣ����ų����������");
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
	//	pppTerminate();   // ��������, ���˳��Ի���ʱִ��	
	depth.stop();
	depth.destroy();
	device.close();
	OpenNI::shutdown();

	//DestoryList(m_pLocalEnvList);
}

//�ر�ָ�����豸
void CPrimeSenseDet::CloseTheDevice()
{
	depth.stop();
	depth.destroy();
	device.close();
	OpenNI::shutdown();
}

///////////////////////////////////
//�����������
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
	//��ֵ
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

//��ȡ����¼ͼ������
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
		//ɨ����һ�еĳ�ʼ�� 
		for (j=0;j<PicWidth;j++)
		{
			//float iiid = pDepth[i*PicWidth+j];
			//////////////////////////////////////////////////////////////
			//ƽ�滯�������뷶Χ�����壬�������ڸ˼�����������һ��Ļ��
			if(pDepth[i*PicWidth+j]>DetectionRangeMax)
			{
				pDepth[i*PicWidth+j] = DetectionRangeMax;
			}
			//�������Լ�������Ӱ���ִ���
			if(pDepth[i*PicWidth+j]<DetectionRangeMin)
			{
				pDepth[i*PicWidth+j] = DetectionRangeMax;
			}

			CoordinateConverter::convertDepthToWorld(depth,j,i,pDepth[i*PicWidth+j],
				                                     &depthTempData[0],&depthTempData[1],&depthTempData[2]);//��һ������������ϵ�б任����������ϵ

			
        //    ImageFile.open("..\PrimeSenseDet\image.dat",ios::app)
			memset(tempArray , '\0' , 100);
			sprintf(tempArray , "%d %d %.2f %.2f %.2f;",\
				i , j,depthTempData[0],depthTempData[1],depthTempData[2]);
			ImageFile<<tempArray;
	    //    fwrite(depthTempData,sizeof(float)*3,3,ImageFile);//������д���ļ�fp
		}
	}
	ImageFile.close();
	//fclose(ImageFile);
}

//�Ƚ��ظ��˼�
bool CPrimeSenseDet::CheckRepeatPole(float *CentralP, float* mark)
{
	//�Ƚ��ظ��˼�
		//�Ա������Ѿ���ȡ�ĸ˼�������ô���1����Դ�֡ͼ���еĸ˼��ж��ظ���2�����Բ�ͬ֡ͼ��˼��ж��ظ�
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
			//�ж��Ƿ���Ҫ�ӳ��˼�
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
			
			return false;  //��־λ
			break;
		}
		PoleNode_S=PoleNode_S->pnext;
	}
	return true;
	/////////////////////
}

bool CPrimeSenseDet::CoordinateConv()//����������ϵ�ڻ�����ϵ�µı�ʾ�������ʾ��ʽ
{
	/////////////////////////////////////////////////
	//Coordinate Convertion
	//��Ե�ǰ��ȡͼ�����ڵ�ʱ�̣���ȡ�����˹ؽ���Ϣ
	double dPos[6],CRobotPos[6];//CSensorPos[6];
	double G2_CRobotPos[6];

	//Robot::I_Monit.Get_JointPos(dPos);
	//Robot::I_Task.m_iKine_CR_G1.FKine(dPos, CRobotPos);//CRobotPosΪ����λ��, (x,y,z,w,p,r)

	if ((bool_Tool1) && (!bool_Tool2))
	{
		Robot::I_Monit.Get_JointPos(dPos);
		Robot::I_Task.m_iKine_CR_G1.FKine(dPos, CRobotPos);//CRobotPosΪ����λ��, (x,y,z,w,p,r)
	}
	else if ((!bool_Tool1) && (bool_Tool2))
	{
		Robot::I_Monit.Get_JointPos(dPos);
		Robot::I_Task.m_iKine_CR_G2.FKine(dPos, CRobotPos);//CRobotPosΪ����λ��, (x,y,z,w,p,r)
// 		Robot_IncreTransTool(FrameTrans,CRobotPos,G2_CRobotPos);
 		Robot_IncreTransTool(CRobotPos,CSensorPos,G2_CRobotPos);
		Robot_IncreTransTool(FrameTrans,G2_CRobotPos,G2_CRobotPos);
 		Trans_PosToMtx(G2_CRobotPos,&G2_PUntifyMtx,0);
	}

	Robot_IncreTransTool(CRobotPos,CSensorPos,CRobotPos);//����������ϵ�ڻ�����ϵ�µı�ʾ�������ʾ��ʽΪ(x,y,z,w,p,r)
	Trans_PosToMtx(CRobotPos, &PUntifyMtx, 0);//�������ϵ�ڻ�����ϵ�µı�ʾ�������ʾ��ʽ
	return true;
}

/******************************************************************************
* ������CPrimeSenseDet::LocalEnvListProc(void)
* ���ܣ���ȡ�򻯸˼���Ϣ��Գ��Ƚ��м���
*
*
* ���أ�
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
* ������GetDepthImageData(VideoFrameRef* frame, DepthPixel* pDepth)
* ���ܣ���ȡһ֡���ͼ�����ݣ����Ҷ���Ԥ����
*
*
* ���أ�
******************************************************************************/
Status CPrimeSenseDet::GetDepthImageData(static float frame3D[][640][3])
{
	if (CamineCheck==FALSE)
	{
		AfxMessageBox("��������ʧ�ܣ����飡");
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
	//Ԥ����
	int i,j,k,q;
	int PicHeight,PicWidth;
	PicHeight = frame.getHeight();
	PicWidth = frame.getWidth();

	for (i=0;i<PicHeight;i++)
	{
		//ɨ����һ�еĳ�ʼ�� 
		for (j=0;j<PicWidth;j++)
		{
			//ƽ�滯�������뷶Χ�����壬�������ڸ˼�����������һ��Ļ��
			if(pDepth[i*PicWidth+j]>DetectionRangeMax)
			{
				pDepth[i*PicWidth+j] = DetectionRangeMax;
			}
			//�������Լ�������Ӱ���ִ���
			if(pDepth[i*PicWidth+j]<DetectionRangeMin)
			{
				pDepth[i*PicWidth+j] = DetectionRangeMax;
			}

			CoordinateConverter::convertDepthToWorld(depth,j,i,
				pDepth[i*PicWidth+j],&frame3D[i][j][0],&frame3D[i][j][1],&frame3D[i][j][2]);
			//�������ϵ->����������ϵ
		}
	}
	return rc;
}

/******************************************************************************
* ������CPrimeSenseDet::ReadDepthImageFile(static float frame3D[][640][3])
* ���ܣ���ȡһ֡���ͼ���ļ������ݲ�ת���ɵ���
*
*
* ���أ��ɹ� ture ,ʧ�� false
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
* ������CPrimeSenseDet::PolesDet(PoleInfo *poleINFO)
* ���ܣ��˼���ȡ�ͼ���㷨
*
*
* ���أ���ȡ�˼�����
******************************************************************************/
int CPrimeSenseDet::PolesDet(void)
{
	//��ʼ��
	int i=0,j=0,k=0,q=0;   //ѭ������
	OpenGL_Mode=FALSE;
	int PoleN = 0;
	g_PolesNum = 0;
	
	for (i=0;i<5;i++)
	{
		RSJ[i] = 0;
	}

	//��ȡ���ͼ������
	static float Frame3D[480][640][3];  //���������ͼ��ת����3D��ʵ�ռ���ơ�

	if (STATUS_ERROR == GetDepthImageData(Frame3D))
	{
		AfxMessageBox("Carmine 1.09 �������");

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
	
	

	//����ϵת��
	CoordinateConv();//����������ϵ�ڻ�����ϵ�µı�ʾ�������ʾ��ʽ

	//�жϴ�40�е�100���еĸ˼���Ŀ������¼��Ŀ����еĸ˼��߽��

	float CliffP[30][5];//���г��ֵ����ɸ����µ㣨�������о�����ĵ㣩�Լ��õ������ͼ���еĶ�ά����//30�����µ��Ƿ��㹻��������
	int CliffPMark[30];//���µ�������־,1��ʾ����ǰһ��Զ����0��ʾ������һ��Զ��
	int CliffPn = 0;  //ÿ�����µ����
	float PolesStartBoundaryP[DetPoleNum][5]; //��⵽�˼���Ŀ���ʱ�ĸ˼���һ���߽�㣨�����µ㣩�Լ��õ������ͼ���еĶ�ά����
	float PolesEndBoundaryP[DetPoleNum][5]; //��⵽�˼���Ŀ���ʱ�ĸ˼��ڶ����߽�㣨�����µ㣩�Լ��õ������ͼ���еĶ�ά����
	float PolesStartBoundaryPTemp[DetPoleNum][5];
	float PolesEndBoundaryPTemp[DetPoleNum][5];
	int PoleNTemp = 0;
	for (i=LStart;i<LStart+LStep;i++)  //��10��5����40��35�����У���ȡ�˼���Ŀ�����У��˼��ĸ�����PoleN���ͱ߽��PolesStartBoundaryP[i][k]��PolesEndBoundaryP[i][k]
	{
		//�������µ���
		for (j=0;j<640;j++)
		{
			if ((Frame3D[i][j][2]-Frame3D[i][j+1][2])>CliffDis)
			{
				for (k=0;k<3;k++)
				{
					CliffP[CliffPn][k] = Frame3D[i][j+1][k];  //��¼�����нϽ��ĵ�
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
					CliffP[CliffPn][k] = Frame3D[i][j][k]; //��¼�����нϽ��ĵ�
					CliffPMark[CliffPn] = 0;
				}
				CliffP[CliffPn][3] = i;
				CliffP[CliffPn][4] = j;
				CliffPn++;
			}
		}
		///////////////////////////////////
		//ÿ�����һ�����µ�󣬶Լ�⵽�����µ���������
		for (j=0;j<CliffPn-1;j++)
		{
			if (CliffPMark[j]==1 && CliffPMark[j+1]==0)//���µ�������־,1��ʾ�����õ������ǰһ��Ͻ�����0��ʾ�����õ�����ں�һ��Ͻ���
				                                       //��������ʾΪĳһ����������������µ�
			{
				//����Ƿ�����ɼг�����
				if (P2Distance(CliffP[j],CliffP[j+1])<80*1.414 && P2Distance(CliffP[j],CliffP[j+1])>55)
				{
					for (k=0;k<5;k++)
					{
						PolesStartBoundaryPTemp[PoleNTemp][k] = CliffP[j][k];//�Լ�⵽�����µ��з��ϼгָ˼�Ҫ�����������
						PolesEndBoundaryPTemp[PoleNTemp][k] = CliffP[j+1][k];
					}
					PoleNTemp++;
				}
			}
		}
		CliffPn = 0;//���µ���Ŀ����
		//////////////////////////////////////
		//��ȡ�˼������ʱ�ĸ˼������ͱ�����ϣ���¼��ά����Ͷ�ά�������꣩
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
		PoleNTemp = 0;//��ʱ�˼���Ŀ����
	}
	if (PoleN > DETECT_POLE_NUM_MAX)//HU  Ϊʲô���ֻ��5���ˣ�����

	{
		printf("More than max num\n");
		return -1;
	}

	//�˼���Ϣ�ṹ��
	PoleInfo poleINFO[DETECT_POLE_NUM_MAX];
	(void)memset(poleINFO, 0, sizeof(poleINFO));//HU
	//////////////////////////////////////////////////////////////////////////
	//���ÿһ���˼���������
	static float OutlineP[DetPoleNum][480][200][3] = {0};  //���ͼ����ͬһ�У�DepthPixelY��ȵģ�ɨ��˼�������
	                                                       //���ͼ����ĳ���˵�ĳ���Ա�Ե��ĳ�е����ά����
	float P_BoundaryP[DetPoleNum][480][2][5] = {0};  //�˼������߽��
	int PoleJudge[40][2];//�洢��˼��߽�㣨���µ㣩��ά���������е�j�Լ����µ������
	int PoleJudgeN =0;//��˼��߽�㣨���µ㣩����
	int PoleRowN[DetPoleNum] = {0};//����ĳ���˵ĵ�������
	for (int Pole_i=0;Pole_i<PoleN;Pole_i++)
	{
		for (k=0;k<5;k++)//�������ϸ˼���Ŀ����е����µ���Ϣ
		{
			P_BoundaryP[Pole_i][0][0][k] = PolesStartBoundaryP[Pole_i][k];
			P_BoundaryP[Pole_i][0][1][k] = PolesEndBoundaryP[Pole_i][k];
		}
		for (i=PolesStartBoundaryP[Pole_i][3];i<480;i++)  //�ӵ�Pole_i���˼��ĵ�PolesStartBoundaryP[Pole_i][3]�����ؿ�ʼɨ��
		{
			//��ÿһ�е�̽Ѱ��Χ������0~640��
			if (P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][4]-50<0)
			{
				P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][4] = 50;
			}
			if (P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][4]+50>640)
			{
				P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][4] = 590;
			}
			////////////////////////////////////////////////////////
			//����ÿ���з��ϼг�Ҫ��ĸ˼��������������µ�Ϊ��ʼ����
			//���ø˼��ڸ����е����µ�
			for (j=P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][4]-50;j<P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][4]+50;j++)
			{
				if ((Frame3D[i][j][2]-Frame3D[i][j+1][2])>CliffDis)
				{
					PoleJudge[PoleJudgeN][0] = j+1;//��һ���е���˼��߽�㣨��߽磩�Ķ�ά���������е�j
					PoleJudge[PoleJudgeN][1] = 1;//1��ʾ�����õ������ǰһ��Ͻ�����0��ʾ�����õ�����ں�һ��Ͻ���
					PoleJudgeN++;
				}
				else if ((Frame3D[i][j][2]-Frame3D[i][j+1][2])<-CliffDis)
				{
					PoleJudge[PoleJudgeN][0] = j;//��һ���е���˼��߽�㣨�ұ߽磩�Ķ�ά���������е�j
					PoleJudge[PoleJudgeN][1] = 0;//1��ʾ�����õ������ǰһ��Ͻ�����0��ʾ�����õ�����ں�һ��Ͻ���
					PoleJudgeN++;
				}
			}
			//////////////////////////////////////////////////////
			if (PoleJudgeN>1)
			{
				int mar=0;//�˼���Ŀ
				for (j=0;j<PoleJudgeN-1;j++)
				{
					//�Լ�⵽����˼��߽���з��ϼгָ˼�Ҫ����������ϣ���ͬһ���˼�������������߽�����һ��������ͳһ��ʾ����
					//�洢���ͼ����ͬһ��ɨ��˼�������
					if (PoleJudge[j][1]==1 && PoleJudge[j+1][1]==0
						&&P2Distance(Frame3D[i][PoleJudge[j][0]],Frame3D[i][PoleJudge[j+1][0]])<80*1.414 
						&& P2Distance(Frame3D[i][PoleJudge[j][0]],Frame3D[i][PoleJudge[j+1][0]])>45)
					{
						mar++;
						//�洢������˼��߽�㣨���µ㣩����ά����Ͷ�ά��������
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
						int tempp=0;//���ڸø˼��ĸ�������������
						float DisMax = Frame3D[i][PoleJudge[j][0]][2];
						if (Frame3D[i][PoleJudge[j+1][0]][2]>Frame3D[i][PoleJudge[j][0]][2])//ȷ��������˼��߽�������ֵ�ϴ���
						{
							DisMax = Frame3D[i][PoleJudge[j+1][0]][2];
						}
						for (q=PoleJudge[j][0];q<PoleJudge[j+1][0];q++)//�洢���ͼ����ͬһ��ɨ��˼�������
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
	//Ѱ�Ҹ˼�������ߣ�����
	////////////////////////////////////////////////////////////////////
	float NearestLineP[DetPoleNum][480][3]={0}; //ÿһ��������ĵ㣨���㣩
	int NearestLinePN[5][480];   //ÿһ��������ĵ��ڸ����е�λ��
	//	float* nearpp = &nearp[0][0];
	for (i=0;i<PoleN;i++)
	{
		for (j=0;j<PoleRowN[i];j++)
		{
			// 	*(nearpp++) = P_BoundaryP[i][j][0]/200.0;
			// 	*(nearpp++) = P_BoundaryP[i][j][1]/200.0;
			// 	*(nearpp++) = (P_BoundaryP[i][j][2]-5)/200.0;

			NearestLineP[i][j][2] = 1600;//�������Щ���Ա�֤ȡ��ÿ���е������
			for(int rowPN=0;rowPN<300;rowPN++)
			{
				if (OutlineP[i][j][rowPN][2]==0)
				{
					break;
				}
				//rowPN��ʾ���ڸ˼��ĵ��Ƹ�����������Ҫ�������ڸ˵����б���㣬�ҳ������
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
	//�жϼ����Ƿ�����ƽ���ڣ�������������������������������
	///////////////////////////////////////////////////////////
	for (i=0;i<PoleN;i++)
	{
		j=0;
		//�����м������еĵ㵽����ľ���С��15�ĵ������
		while(P2Distance(OutlineP[i][PoleRowN[i]/2][j],NearestLineP[i][PoleRowN[i]/2])<15)
		{
			j++;
		}
		int k=j;
		if (k>198)
		{
			k=198;
		}
		//�����м������еĵ㵽����ľ������15�ĵ������
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
				AfxMessageBox("ƽ����");
				InPlane = 0;
				DeteSymbol = false;
			}
			//return -1;//20170215ע�͵�
			
		}
	}

	////////////////////////////////////////////////////////////////
	//��¼ɨ�赽�ĸ˼���������
	// 	for (i=0;i<PoleN;i++)
	// 	{
	// 		*(PolesLimit++) = NearestLineP[i][0][1];
	// 		*(PolesLimit++) = NearestLineP[i][PoleRowN[i]-3][1];
	// 	}
	////////////////////////////////////////////////////////////////
	//�����������ֱ��
	///////////////////////////////////////////////////////////////
	float NearestLine[DetPoleNum][2][3];
	for (i=0;i<PoleN;i++)
	{
		//NearestLine[i][0]Ϊ���߷���NearestLine[i][1]Ϊ�����е�
		//NearestLine[i][x],x=0Ϊ����x=1Ϊλ�ã�����λ��Ϊ�޳������󣬸��������ƽ��ֵ
		BoundaryLineFit(PoleRowN[i],NearestLineP[i][0],NearestLine[i][0],NearestLine[i][1]);
		// 		PCentralLine[i][0][0] = (NearestLine[i][1][0]+190*NearestLine[i][0][0])/200.0;
		// 		PCentralLine[i][0][1] = (NearestLine[i][1][1]+190*NearestLine[i][0][1])/200.0;
		// 		PCentralLine[i][0][2] = (NearestLine[i][1][2]+190*NearestLine[i][0][2]-5)/200.0;
		// 
		// 		PCentralLine[i][1][0] = (NearestLine[i][1][0]-190*NearestLine[i][0][0])/200.0;
		// 		PCentralLine[i][1][1] = (NearestLine[i][1][1]-190*NearestLine[i][0][1])/200.0;
		// 		PCentralLine[i][1][2] = (NearestLine[i][1][2]-190*NearestLine[i][0][2]-5)/200.0;
	}
	//��ֱ�ȣ����������
	for (i=0;i<PoleN;i++)
	{
		poleINFO[i].BendDeg = 0;//��������������P24
		for (j=0;j<PoleRowN[i];j++)
		{
			poleINFO[i].BendDeg += PtoLineDis(NearestLineP[i][j],NearestLine[i][0],NearestLine[i][1]);// �㵽ֱ�ߵľ��뺯��
		}
		poleINFO[i].BendDeg = poleINFO[i].BendDeg/PoleRowN[i];
	}
	//�����˼�����
	for (i=0;i<PoleN;i++)
	{
		poleINFO[i].PoleLenght = P2Distance(NearestLineP[i][0],NearestLineP[i][PoleRowN[i]]);
		PoleLenghtTemp[i]=poleINFO[i].PoleLenght;
	}
	//////////////////////////////////////////////////////////////////////////
	//�ҳ���Ե���뱳��������ƽ��f(x,y)=0����ƽ�棩����ĵ㣬�Ƚ����߱�Ե���������С
	//////////////////////////////////////////////////////////////////////////
	//	float BackPlane[DetPoleNum][2][3];  //��ƽ�淽�̵�һ����ʾƽ�淽���������ڶ�����ʾ����һ��
	float EdgeToBackLineDis[DetPoleNum][2]={{1000,1000},{1000,1000},{1000,1000},{1000,1000},{1000,1000}};//,{1000,1000},{1000,1000},{1000,1000},{1000,1000},{1000,1000}};  
	                                       //��ʾ����˼�������������Ե���뼹ƽ��
	float EBLDMax[10];  //������߽�㵽���м�����С�������Ǳ߸���������С����������һ����
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
	//��ȡ�а�֮����ж�����
	///////////////////////////////////////////////////////////////////////////
	int checkhigh[5]={0};//�˳���70mm֮�ڵĵ�������
	for (i=0;i<PoleN;i++)
	{
		//�˳���70֮�ڵĵ�������
		while (P2Distance(OutlineP[i][(PoleRowN[i]-checkhigh[i])/2][10],OutlineP[i][(PoleRowN[i]+checkhigh[i])/2][10])<70)//������
		{
			checkhigh[i] = checkhigh[i]+2;
		}
	}
	float tempdis;
	for (i=0;i<PoleN;i++)
	{
		//��70mm�˳��ķ�Χ����Ѱ��߽�㵽���м�����������
		for (j=(PoleRowN[i]-checkhigh[i])/2;j<(PoleRowN[i]+checkhigh[i])/2;j++)//j=0;j<Xcounts;j++)
		{
			//������ʼ��߽�㵽���еļ���ľ���
			tempdis = P2Distance(P_BoundaryP[i][j][0],NearestLineP[i][j]);
			//			tempdis = PtoLineDis(P_BoundaryP[i][j][0],NearestLine[i][0],NearestLine[i][1]);
			// 			tempdis = PtoPlaneDis(TRUE,BackPlane[i][0],BackPlane[i][1],P_BoundaryP[i][j][0]);
			//��¼�������
			if (EdgeToBackLineDis[i][0]>tempdis)
			{
				EdgeToBackLineDis[i][0] = tempdis;
			}
			//������ֹ��߽�㵽���еļ���ľ���
			tempdis = P2Distance(P_BoundaryP[i][j][1],NearestLineP[i][j]);
			//			tempdis = PtoLineDis(P_BoundaryP[i][j][1],NearestLine[i][0],NearestLine[i][1]);
			// 			tempdis = PtoPlaneDis(TRUE,BackPlane[i][0],BackPlane[i][1],P_BoundaryP[i][j][1]);
			//��¼�������
			if (EdgeToBackLineDis[i][1]>tempdis)
			{
				EdgeToBackLineDis[i][1] = tempdis;
			}
		}
	}
	/////////////////////////////////////////////////////////////////////////
	//ʹ�ü�ƽ�淽���������������һ��
	/////////////////////////////////////////////////////////////////////////
	int CheckStartFlag[5]; //��Ϊ0��ʾ��ߴ�Ϊ1��ʾ�ұߴ�
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
	//Ѱ���жϷ���Բ�˵�����,����������������ߵļа���
	//////////////////////////////////////////////////////////////////////
	// 	float SplintPlaneP[DetPoleNum][2][3];  //�жϷ���Բ�˵�������������ߵļа����ϵĵ㣬�뼹ƽ���ɽ���Զ
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
	//��ȡ�а�֮����ж�����
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
		if (CheckStartFlag[i]==0)  //��ߴ�
		{
			for (j=(PoleRowN[i]-checkhigh[i])/2;j<(PoleRowN[i]+checkhigh[i])/2;j++)
			{
				CheckAreaP[i][k][0] = 0;//�˼�ĳһ�У�k���������ڼ����������Ҳ������С���루EBLDMax[i]-2���ĵ�ĸ���
				while (P2Distance(OutlineP[i][j][CheckAreaP[i][k][0]],NearestLineP[i][j])>(EBLDMax[i]-2))
				{
					CheckAreaP[i][k][0] ++;
				}
				CheckAreaP[i][k][1] = 0;//�˼�ĳһ�У�k���������ڼ����������Ҳ���ھ��루11���ĵ�ĸ���
				while (P2Distance(OutlineP[i][j][CheckAreaP[i][k][1]],NearestLineP[i][j])>11)
				{
					CheckAreaP[i][k][1] ++;
				}
				CheckAreaP[i][k][2] = j;//�˼�ĳһ�е���������
				k++;
			}
		}
		else if (CheckStartFlag[i]==1) //�ұߴ�
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
	//�жϷ���Բ��,
	////////////////////////////////////////////////////////////////////////////////
	float CheckPoint[DetPoleNum][5000][3];//ĳ���˼�ĳ���о�����м����ڣ�(EBLDMax[i]-2)��11����Χ�ڵĵ������
	int CheckTriangleN[DetPoleNum][3];   //������������������λ��
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
	float CheckPlane[DetPoleNum][2][3]={0};  //����ƽ�淽��
	float CheckLine[DetPoleNum][3]={0};
	for (i=0;i<PoleN;i++)
	{
		if (CheckPoleStyle(CheckTriangleN[i][2],CheckPoint[i][0])>0.26)
		{
			poleINFO[i].PoleStyle = 1;  //Բ��
			RSJ[i] = 1;
		}
		else
		{
			//poleINFO[i].PoleStyle = -1;  //����
			//RSJ[i] = -1;
			//PlaneFitting(CheckTriangleN[i][2],CheckPoint[i][0],CheckPlane[i][0]);
			//CheckLine[i][0] = CheckPlane[i][0][1]*NearestLine[i][0][2] - CheckPlane[i][0][2]*NearestLine[i][0][1];
			//CheckLine[i][1] = CheckPlane[i][0][2]*NearestLine[i][0][0] - CheckPlane[i][0][0]*NearestLine[i][0][2];
			//CheckLine[i][2] = CheckPlane[i][0][0]*NearestLine[i][0][1] - CheckPlane[i][0][1]*NearestLine[i][0][0];
			//VectorUnitization(CheckLine[i]);
		}
	}
	//����˼������߷��̺͸˼��ߴ�
	for (i=0;i<PoleN;i++)
	{
		int openglk;
		float PolesCentre[3];
		float PolesTempCentre[3];	
		float tempD;
		float R[2];
		//Բ��
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
			///////////////////////////////Ѱ�Ҹ˼����ߵ����Ż�����GU
			for (j=0;j<11;j++)
			{
				PolesTempCentre[0] = NearestLine[i][1][0]+(10-2*j);//��Χ����10
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
							PolesCentre[k] = PolesTempCentre[k];//�ô���ø˼����е�λ��P25
						}
					}
				}

			}
			for (k=0;k<3;k++)
			{
					poleINFO[i].PoleFunc[1][k] = PolesCentre[k];

				PCentralLineTemp[i][1][k] = PolesCentre[k];//GU
			}
			//�˼��뾶
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
		//����
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
		//	//�˼�ֱ��
		//	poleINFO[i].PoleDiameter = PtoLineDis(PolesCentre,NearestLine[i][0],NearestLine[i][1]);
		//	PCentralLine[i][0][0] = PolesCentre[0]+220*NearestLine[i][0][0];//������220
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
		//�����жϷ���Բ��, ����Ѿ��ж�ΪԲ�˵ģ��Ǿ�ΪԲ�ˣ�����ж�Ϊ���˵ģ�����Dmin<5,��ΪԲ��


		//if (poleINFO[i].PoleStyle!=Round)//GU  ������Թ����м�⵽���˻������Ի��˶������
		//{
		//	for (k=0;k<3;k++)
		//	{
		//		PolesCentre[k] = 0;//�ô���ø˼����е�λ��P25
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
	//���㾲̬����
	memset(Frame3D,0,480*640*3*sizeof(float));
	memset(OutlineP,0,DetPoleNum*480*200*3*sizeof(float));
	return PoleN;
}

/******************************************************************************
* ������CPrimeSenseDet::DetectPoleEnvInImage()
* ���ܣ���ȡһ֡���ͼ���еļ򻯸˼���Ϣ
*
*
* ���أ��ɹ� ture ,ʧ�� false
******************************************************************************/
bool CPrimeSenseDet::DetectPoleEnvInImage()
{
	//��ʼ��
	int i=0,j=0,k=0,q=0;  //ѭ������
	OpenGL_Mode=TRUE;    
	unsigned char PoleN = 0;   //���˼�����
	g_PolesNum = 0;      //���˼�����
	for (i=0;i<5;i++)   //OpenGL���Ʊ���
	{
		DrawN[i] = 0;
		RSJ[i] = 0;   
	}

	//��Ե�ǰ��ȡͼ�����ڵ�ʱ�̣���ȡ�����˹ؽ���Ϣ
	CoordinateConv();

	//��ȡ���ͼ������
	static float Frame3D[480][640][3];  //���������ͼ��ת����3D��ʵ�ռ���ơ�
	if (STATUS_ERROR == GetDepthImageData(Frame3D))
	{
		AfxMessageBox("Carmine 1.09 �������");
		return false;
	}

	//�ļ����ݶ�ȡ
// 	if (!ReadDepthImageFile(Frame3D))
// 	{
// 		return false;
// 	}
	
	/////////////////////////////////////////////////////////////////////////////////////
	
	//����ɨ��
	{
		float CliffP[30][4];//���г��ֵ����ɸ����µ㣨�������о�����ĵ㣩�Լ��õ������ͼ���еĶ�ά����
		char CliffPMark[30];//���µ�������־,1��ʾ����ǰһ��Զ����0��ʾ������һ��Զ��
		unsigned short CliffPN = 0;  //ÿ�����µ����
		float PolesStartBoundaryP[DetPoleNum][4]; //��⵽�˼���Ŀ���ʱ�ĸ˼���һ���߽�㣨�����µ㣩�Լ��õ������ͼ���еĶ�ά����
		float PolesEndBoundaryP[DetPoleNum][4]; //��⵽�˼���Ŀ���ʱ�ĸ˼��ڶ����߽�㣨�����µ㣩�Լ��õ������ͼ���еĶ�ά����
		// 	float PolesStartBoundaryPTemp[DetPoleNum][5];
		// 	float PolesEndBoundaryPTemp[DetPoleNum][5];
		unsigned char PoleNTemp = 0;
		unsigned short ContiuneMissLineN;
		unsigned short LS;//��ͼ�񻮷�Ϊ10�񣬴�ÿ����ʼѰ�Ҹ˼�
//		unsigned char PicPoleNum = ListPoleNum+1;  //�˸�ͼ����������һ���˼��������е�����
		//	float J_NewPolePoint[DetPoleNum][3];
		//	unsigned short RowOffset=0;
		for (char PicBlock=0; PicBlock<((480-LStart)/LStep); ++PicBlock)
		{
			//	float StartRidgePoint[DetPoleNum][3]={0};  //�˼�ͷһ�м���
			LS=LStart+PicBlock*LStep;
			for (j=0;j<640;j++)
			{
				if ((Frame3D[LS][j][2]-Frame3D[LS][j+1][2])>CliffDis)
				{
					for (k=0;k<3;k++)
					{
						CliffP[CliffPN][k] = Frame3D[LS][j+1][k];  //��¼�����нϽ��ĵ�
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
						CliffP[CliffPN][k] = Frame3D[LS][j][k]; //��¼�����нϽ��ĵ�
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
            //�Ƚ��ظ��˼�
			if (m_pLocalEnvList->begin->pnext != NULL)
			{
				//�Ա������Ѿ���ȡ�ĸ˼�������ô���1����Դ�֡ͼ���еĸ˼��ж��ظ���2�����Բ�ͬ֡ͼ��˼��ж��ظ�
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
			//���ÿһ���˼���������
			//////////////////////////////////////////////////////////////////////////
			//static float OutlineP[DetPoleNum][480][200][3] = {0};  //���ͼ����ͬһ�У�DepthPixelY��ȵģ�ɨ��˼�������
			float P_BoundaryP[DetPoleNum][480][2][4] = {0};  //�˼������߽��
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
				for (i=LS+1;i<480;i++)  //�ӵ�Pole_i���˼��ĵ�LS�п�ʼɨ��
				{
					//��ÿһ�е�̽Ѱ��Χ������0~640��
					if (P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][3]-50<0)
					{
						P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][3] = 50;
					}
					if (P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][3]+50>640)
					{
						P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][3] = 590;
					}
					////////////////////////////////////////////////////////
					//�˼����߽��ǰ��50pixels��Χ��ʼɨ��
					for (j=P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][3]-50;j<P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][3]+50;j++)
					{
						if ((Frame3D[i][j][2]-Frame3D[i][j+1][2])>CliffDis)
						{
							PoleJudge[PoleJudgeN][0] = j+1;  //��¼���µ��ڸ��е�λ�ú�����
							PoleJudge[PoleJudgeN][1] = 1;
							PoleJudgeN++;
						}
						else if ((Frame3D[i][j][2]-Frame3D[i][j+1][2])<-CliffDis)
						{
							PoleJudge[PoleJudgeN][0] = j;  //��¼���µ��ڸ��е�λ�ú�����
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
								&& P2Distance(Frame3D[i][PoleJudge[j][0]],Frame3D[i][PoleJudge[j+1][0]])>45)  //����˼��ߴ�����
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
								//Ѱ�������˼��߽��֮��ļ���
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
								//��ÿһ�β����µ�RidgePoint�����ʱ�򣬽���һ��һ��������бȽ�
								if ((PoleRowN[Pole_i]!=0)&&(P2Distance(RidgePoint[PoleRowN[Pole_i]-1],RidgePoint[PoleRowN[Pole_i]])>40))
								{
									i=481;
									PoleRowN[Pole_i]--;
									break;
								}
								isRidgePoint = true;
								break;   //�ҵ�����Ҫ��ģ�
							}
						}
						if (!isRidgePoint)
						{
							PoleRowN[Pole_i]--;
						}
					PoleJudgeN = 0;
					PoleRowN[Pole_i]++;
				}
				//���鲶��ĸ˼���ֱ��
				float PoleLine[2][3];   //�˼�ֱ�߷��̵ķ��������͵�
				BoundaryLineFit(PoleRowN[Pole_i],RidgePoint[0],PoleLine[0],PoleLine[1]);
				//��ֱ�ȼ���
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
					//����ȡ���ĸ˼�����ת������������ϵ��
					/////////////////////////////////////////////////
					float S_Z[3]={0,0,1};
					PoleLine[1][2]+=abs(30.0/(sin(VectorAngle(PoleLine[0],S_Z))));  //����Z���������һ���˼��뾶
					//////////////////////////////////////////////////
					PointCoordinateC(&PUntifyMtx,PoleLine[1],PoleLine[1]);
					//��������任����Ҫ��ת������ľ���ֵ�����
					VectorCoordinateC(&PUntifyMtx,PoleLine[0],PoleLine[0]);
					////////////////////////////////
					VectorUnitization(PoleLine[0]);
					PoleNode NewPole;  //��ȡ�ͼ������¸˼�
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
	//����ɨ��
	{
		float CliffP[30][4];//���г��ֵ����ɸ����µ㣨�������о�����ĵ㣩�Լ��õ������ͼ���еĶ�ά����
		char CliffPMark[30];//���µ�������־,1��ʾ����ǰһ��Զ����0��ʾ������һ��Զ��
		unsigned short CliffPN = 0;  //ÿ�����µ����
		float PolesStartBoundaryP[DetPoleNum][4]; //��⵽�˼���Ŀ���ʱ�ĸ˼���һ���߽�㣨�����µ㣩�Լ��õ������ͼ���еĶ�ά����
		float PolesEndBoundaryP[DetPoleNum][4]; //��⵽�˼���Ŀ���ʱ�ĸ˼��ڶ����߽�㣨�����µ㣩�Լ��õ������ͼ���еĶ�ά����
		unsigned char PoleNTemp = 0;
		//��ͼ�񻮷�Ϊ10�񣬴�ÿ����ʼѰ�Ҹ˼�
		unsigned short CS;
		for (char PicBlock=0; PicBlock<((640-CStart)/CStep); PicBlock++)
		{
			CS=CStart+PicBlock*CStep;
			//�ҵ��˼���ȡ��ͻ�ƿ�
			for (i=0;i<480;i++)
			{
				if ((Frame3D[i][CS][2]-Frame3D[i+1][CS][2])>CliffDis)
				{
					for (k=0;k<3;k++)
					{
						CliffP[CliffPN][k] = Frame3D[i+1][CS][k];  //��¼�����нϽ��ĵ�
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
						CliffP[CliffPN][k] = Frame3D[i][CS][k]; //��¼�����нϽ��ĵ�
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
			//�Ƚ��ظ��˼�
			if (m_pLocalEnvList->begin->pnext != NULL)
			{
				//�Ա������Ѿ���ȡ�ĸ˼�������ô���1����Դ�֡ͼ���еĸ˼��ж��ظ���2�����Բ�ͬ֡ͼ��˼��ж��ظ�
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
			//���ÿһ���˼���������
			//////////////////////////////////////////////////////////////////////////
			//static float OutlineP[DetPoleNum][480][200][3] = {0};  //���ͼ����ͬһ�У�DepthPixelY��ȵģ�ɨ��˼�������
			float P_BoundaryP[DetPoleNum][640][2][4] = {0};  //�˼������߽��
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
				for (j=CS+1;j<640;j++)  //�ӵ�Pole_i���˼��ĵ�LS�п�ʼɨ��
				{
					//��ÿһ�е�̽Ѱ��Χ������0~480��
					if (P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][3]-50<0)
					{
						P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][3] = 50;
					}
					if (P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][3]+50>480)
					{
						P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][3] = 430;
					}
					////////////////////////////////////////////////////////
					//�˼����߽��ǰ��50pixels��Χ��ʼɨ��
					for (i=P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][3]-50;i<P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][3]+50;i++)
					{
						if ((Frame3D[i][j][2]-Frame3D[i+1][j][2])>CliffDis)
						{
							PoleJudge[PoleJudgeN][0] = i+1;  //��¼���µ��ڸ��е�λ�ú�����
							PoleJudge[PoleJudgeN][1] = 1;
							PoleJudgeN++;
						}
						else if ((Frame3D[i][j][2]-Frame3D[i+1][j][2])<-CliffDis)
						{
							PoleJudge[PoleJudgeN][0] = i;  //��¼���µ��ڸ��е�λ�ú�����
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
								&& P2Distance(Frame3D[PoleJudge[i][0]][j],Frame3D[PoleJudge[i+1][0]][j])>45)  //����˼�����
							{
								for (k=0;k<3;k++)
								{
									P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][0][k] = Frame3D[PoleJudge[i][0]][j][k];
									P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][1][k] = Frame3D[PoleJudge[i+1][0]][j][k];
								}
								P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][0][3] = PoleJudge[i][0];
								P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][1][3] = PoleJudge[i+1][0];
								///////////////////////////////////////////////////////////////
								//Ѱ�������˼��߽��֮��ļ���
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
								//��ÿһ�β����µ�RidgePoint�����ʱ�򣬽���һ��һ��������бȽ�
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
				//���鲶��ĸ˼���ֱ��
				float PoleLine[2][3];   //�˼�ֱ�߷��̵ķ��������͵�
				BoundaryLineFit(PoleRowN[Pole_i],RidgePoint[0],PoleLine[0],PoleLine[1]);
				//��ֱ�ȼ���
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
					//����ȡ���ĸ˼�����ת������������ϵ��
					/////////////////////////////////////////////////
					float S_Z[3]={0,0,1};
					PoleLine[1][2]+=abs(30.0/(sin(VectorAngle(PoleLine[0],S_Z))));  //����Z���������һ���˼��뾶
					//////////////////////////////////////////////////
					PointCoordinateC(&PUntifyMtx,PoleLine[1],PoleLine[1]);
					//��������任����Ҫ��ת������ľ���ֵ�����
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
	//���㾲̬����
	memset(Frame3D,0,480*640*3*sizeof(float));
	return true;
}      

float CPrimeSenseDet::CheckPoleStyle(int CheckPN,float* CheckPointP)
{
	int i=0;
	int k=0;
	float PlaneCurvature = 0;  //��������ֵ���˼��ֲ��������ƽ���Բ����֮������ʣ�
	float OutlineP[10000][3]={0};
	float Plane[2][3]={0};
	//ȡ���������ֵ
	for (i=0;i<CheckPN;i++)
	{
		for (k=0;k<3;k++)
		{
			OutlineP[i][k] = *(CheckPointP+i*3+k);
		}
	}
	//ȷ������ƽ��ķ�����������һ��
	PlaneFitting(CheckPN,CheckPointP,Plane[0]);

	//�����㵽ƽ������ƽ��ֵ
	for (i=0;i<CheckPN;i++)
	{	
		//����ƽ���ȽϺã���Ϊ���ڷ���ƽ����˵
		//һ���Ϊ0.����ƽ�����С������Բ����
		//�����Ϊ1.����ƽ���������������������
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
	//ȡ���������ֵ
	R[0]=2000;
	R[1]=0;
	int num=0;
	for (i=0;i<CheckPN;i++)
	{
		for (k=0;k<3;k++)
		{
			OutlineP[num][k] = *(CheckP+i*3+k);
		}
		if (OutlineP[num][2] == 0)  //�ų��˼����������к���Ŀհ�
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
