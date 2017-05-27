// ToolDlg.cpp : implementation file
//

#include "../stdafx.h"
#include "../MoRoController.h"
//#include "../CameraDS.h"
#include "ToolDlg.h"
#include "../RobotControl/ServoControl.h"
#include <math.h>

//#include "../Camera/ppp.h"
#include "../camerads.h"
#include <D:\Study program\PCL\PCL 1.8.0\3rdParty\OpenNI2\Include\OpenNI.h>
#include "F:\VS2010��RobotController\MoRoController\Sensor.h"
#include "F:\VS2010��RobotController\MoRoController\PrimeSenseDet\VectorAndMatrix.h"
#include <iostream>
#include <fstream>
#include <Windows.h>



//#define _CRTDBG_MAP_ALLOC//g
//#include <stdlib.h>
//#include <crtdbg.h>
//
//#include <iostream>//g

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#include <math.h>

using namespace std;
int test_count = 0;
bool gb_endofstep2 = true;

double gd_URG_Len_y = 0;
bool gb_URG_Enable = true;

int cam_count;
CCameraDS camera;
BOOL isRunCam[2]; //����ͷ״̬:0--�رգ�1--�򿪣�2--����ʹ�ã�3--ͨѶ����
int RedetSucCount = 0;//���¼������һ�μ����ͬ�ĳɹ�������>=3ʱ��������ץ��

//void CToolDlg::CameraFunction();
//void CToolDlg::CameraFunction1();
bool CToolDlg::m_bSensor = false;
double CToolDlg::m_dwz = 0;
double CToolDlg::m_dwz_bzq = 0;
int CToolDlg::m_linecount = 0;
int CToolDlg::m_linecount1 = 0;
//void CToolDlg::MotionFunc();  // ��̬������ʼ��
bool CToolDlg::m_bRunFlag = false;// ��̬������ʼ��
int testnum = 0;//����㷨����

////////////////////////////////////////////////////////
// �˼���ȡʶ��ͼ��
// float pointcloud[5][72000][3];
//extern float PCentralLine[5][2][3];
// float PonCentline[5][3];
// int DrawN[5];  //ÿ���˼�����������
// int g_PolesNum;
// int RSJ[5];
//extern MtxKine PUntifyMtx;
//ִ������ץ�е��̺߳���
extern int PoleStyle[10],PoleNum;

extern float PCentralLineTemp[5][2][3];//GU
extern int PoleLenghtTemp[5];
extern int PoleStyleTemp[10];
double jpostarget[6]; //�洢�����˵���Ŀ��˼�λ��ʱ�Ĺؽڽ�GU
double  dposend[6],dposendtemp[6];//ԭ���ں���FunIntelligentGraspProc(LPVOID lpParameter)�ж���GU


float PoleCentralLine[10][2][3],PoleCentralLineCount[10][2][20][3],Dmin[10],PoleDim[10],PoleLimit[10],PoleLen[10],PoleBend[10],SConf[10][3];
bool bTool1;
bool bTool2;
int  sliderVel;
bool RealtimeCon;  //ʵʱ�������
extern bool DeteSymbol;//20�μ����ɱ�־
bool ChooseGraspPoint;   //ѡ��ץ�е�
bool ReDetection = false;//�ظ����
bool ReDetectionSaveData = true;//�ظ�������ݴ洢��־
bool m_bReachTarget;    //�������Ƿ񵽴�Ԥ�г�λ��

float PCentralLine[5][2][3];//GU--OPENGL
float PonCentline[5][3];//GU--OPENGL

double DetectJointPosition[6];

//////////////////////////////////////////////////////////

//-------Darwin XZG-----------
#define MINANGPILE 2//
#define MINXPILE 40


struct MyLine//ֱ�߽ṹ��
{
	int LineMUN;//ֱ�ߺţ��ڼ���ֱ�ߣ�

	int ApRank;//�Ƕȴ�������
	int XpRank;//�����������

};
MyLine MyLineStorage[5000];//�洢ֱ������
MyLine *MyLineIndex[5000];


struct MyLinePile//ֱ�ߴؽṹ��
{
	//int LineMUN[50];//ֱ�ߺŴ�Ŵ�
	MyLine *MyLine[1000];//����ָ��ֱ�����Ե�
	int LineCounter;//
	//-----------------------------------------
	//�Ƕȴش�-45�ȵ�45�ȣ�ÿMINANGPILEΪһ��
	//�����ΪYȡĳ�̶�ֵʱ��ÿMINXPILEΪһ��
	//-----------------------------------------
	int AnglePile;//�Ƕȴ�
	int XPile;//X������

	int APRank;//�ýǶȴ�������
	int XPRank;//�þ����������

};

MyLinePile MLPStorageA[1000];
MyLinePile MLPStorageX[1000];
MyLinePile *MLPStorageAIndex[1000]={NULL};
MyLinePile *MLPStorageXIndex[1000]={NULL};




int PileCounterMark = 0;
int linecounter=0;

/////////////////////////////////////////////////////////////////////////////
// CToolDlg dialog


CToolDlg::CToolDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CToolDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CToolDlg)
	// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
}


void CToolDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CToolDlg)
	DDX_Control(pDX, IDC_SLIDER_VEL, m_sliderVel);
	DDX_Control(pDX, IDC_MSCOMM_CAN, m_ctrlCanCom);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CToolDlg, CDialog)
	//{{AFX_MSG_MAP(CToolDlg)
	ON_BN_CLICKED(IDC_TOOL_FIRST, OnToolFirst)
	ON_BN_CLICKED(IDC_TOOL_SECOND, OnToolSecond)
	ON_BN_CLICKED(IDC_TOOL_STOP, OnToolStop)
	ON_WM_CLOSE()
	ON_WM_NCDESTROY()
	ON_BN_CLICKED(IDC_TOOL_GETDATA, OnToolGetdata)
	ON_BN_CLICKED(IDC_TOOL_MOVE, OnToolMove)
	ON_WM_HSCROLL()
	ON_BN_CLICKED(IDC_TOOL_MOVE2, OnToolMove2)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_TOOL_MOVE3, OnToolMove3)
	ON_BN_CLICKED(IDC_TOOL_MOVE4, OnToolMove4)
	ON_BN_CLICKED(IDC_BUTTON_XZG, OnButtonXzg)
	ON_MESSAGE(MY_MESSAGE_TOOLRUN, OnMyMessage)
	ON_BN_CLICKED(IDC_TOOL_TRACKING, OnToolTracking)
	ON_BN_CLICKED(IDC_CAMERA, OnCamera)
	ON_BN_CLICKED(IDC_BUTTON_CHANGE_SENSOR1, OnButtonChangeSensor)
	ON_BN_CLICKED(IDC_BUTTON_CHANGE_SENSOR2, OnButtonChangeSensor2)
	ON_BN_CLICKED(IDC_BUTTON_CAM_CLOSE, OnButtonCamClose)
	ON_BN_CLICKED(IDC_BUTTON_ROLL, OnButtonRoll)
	ON_BN_CLICKED(IDC_BUTTON_TURN, OnButtonTurn)
	ON_BN_CLICKED(IDC_BUTTON_INCHWORM, OnButtonInchworm)
	ON_BN_CLICKED(IDC_BUTTON_CHANGE_SENSOR, OnButtonChangeSensor)
	ON_BN_CLICKED(IDC_BUTTON_TURN2, OnButtonTurn2)
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDC_Bt_PrimeSenseCam, &CToolDlg::OnBnClickedBtPrimesensecam)
	ON_BN_CLICKED(IDC_BUTTON6, &CToolDlg::OnBnClickedButton6)
	ON_BN_CLICKED(IDC_BUTTON_ROBOTSTOP, &CToolDlg::OnBnClickedButtonRobotstop)
	ON_BN_CLICKED(IDC_BUTTON_ROBOTCONTINUE, &CToolDlg::OnBnClickedButtonRobotcontinue)
	ON_BN_CLICKED(IDC_BUTTON_OPENDEVICE, &CToolDlg::OnBnClickedButtonOpendevice)
	ON_BN_CLICKED(IDC_BUTTON_BROLL, &CToolDlg::OnBnClickedButtonBroll)
	ON_BN_CLICKED(IDC_BUTTON_BTURN, &CToolDlg::OnBnClickedButtonBturn)
	ON_BN_CLICKED(IDC_BUTTON_BINCHWORM, &CToolDlg::OnBnClickedButtonBinchworm)
	ON_BN_CLICKED(IDC_BUTTON_STOP, &CToolDlg::OnBnClickedButtonStop)
	ON_BN_CLICKED(IDC_BUTTON_CONTINUE, &CToolDlg::OnBnClickedButtonContinue)
	ON_BN_CLICKED(IDC_BUTTON_REINIT, &CToolDlg::OnBnClickedButtonReinit)
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CToolDlg message handlers
//using namespace openni;
//Device device;
//VideoStream depth;
BOOL CToolDlg::OnInitDialog() 
{
	CDialog::OnInitDialog();

	// TODO: Add extra initialization here
	////////////////////////////////////////////////////////
	m_pCprimeSenseDet = new CPrimeSenseDet;

	m_bTool1 = false; // ��צ1�򿪹رձ�־
	m_bTool2 = false; 

	ChooseGraspPoint = true;//GU

	//	m_bSensor = false;
	m_bRepeat = false;
	m_nTrack = 0;
	m_nUltraLen1 = 0;
	m_nUltraLen2 = 0;
	m_nTrend = 0;
	m_bInverseDir = false;
	m_nCount = 0;
	m_nCntOverCur = 0;

	m_deviceclosesymbol = false;
	m_MotionChoose = 0;
	m_bg1tog2 = TRUE;
	m_Stop = false;

	//m_dwz = 0;

	m_bWriteFile = false;

	int i;
	for(i=0; i<7; i++)
	{
		m_nCommData[i] = 0;
	}
	m_dCommAngle = 0;


	m_tool1Control.SubclassWindow(::GetDlgItem(m_hWnd, IDC_TOOL_FIRST));
	m_tool2Control.SubclassWindow(::GetDlgItem(m_hWnd, IDC_TOOL_SECOND));
	((CButton*)GetDlgItem(IDC_CHECK_ROLLDIR))->SetCheck(BST_UNCHECKED);


	switch (Robot::robotMode)
	{
	case ROBOT_MODE_CR_P: // ���� - ��ֻ��צ - �ŷ�����
		SetDlgItemText(IDC_TOLL_STATE, "����:   ");
		SetIDCState("Grip1", "��צ1�ɿ�", false);
		SetIDCState("Grip2", "��צ2�ɿ�", false);

		SetDlgItemText(IDC_COMM_POS1, "ERR");
		SetDlgItemText(IDC_INCREM_POS1, "ERR");

		SetDlgItemText(IDC_STATIC_VEL1, "USC");
		SetDlgItemText(IDC_STATIC_VEL2, "Len1");
		SetDlgItemText(IDC_STATIC_VEL3, "Len2");
		SetDlgItemText(IDC_STATIC_VEL4, "ת��");
		//SetDlgItemText(IDC_STATIC_VEL5, "����");
		SetDlgItemText(IDC_STATIC_VEL5, "Camera");

		SetDlgItemText(IDC_STATIC_POS1, "URG");
		SetDlgItemText(IDC_STATIC_POS2, "y");
		SetDlgItemText(IDC_STATIC_POS3, "z");
		SetDlgItemText(IDC_STATIC_POS4, "--");
		SetDlgItemText(IDC_STATIC_POS5, "--");
		SetDlgItemText(IDC_STATIC_POS7, "--");

		SetDlgItemText(IDC_STATIC_POS8, "--");

		SetDlgItemText(IDC_TOOL_MOVE, "Step1");
		SetDlgItemText(IDC_TOOL_MOVE2, "Step3-I");
		SetDlgItemText(IDC_TOOL_MOVE3, "Step2");
		SetDlgItemText(IDC_TOOL_MOVE4, "Step3-II");

		SetDlgItemText(IDC_RADIO1, "+");
		SetDlgItemText(IDC_RADIO2, "-");

		((CButton*)GetDlgItem(IDC_TOOL_TRACKING))->ShowWindow(SW_SHOW);
		((CButton*)GetDlgItem(IDC_INCREM_POS5))->ShowWindow(SW_HIDE);
		((CButton*)GetDlgItem(IDC_INCREM_POS6))->ShowWindow(SW_HIDE);

		((CButton*)GetDlgItem(IDC_TOOL_TRACKING))->ShowWindow(SW_SHOW);

		break;
	case ROBOT_MODE_IR_5: // ��е�� - һֻ��צ - �ŷ�����
	case ROBOT_MODE_IR_6:
		GetDlgItem(IDC_TOOL_SECOND)->ShowWindow(SW_HIDE);
		//GetDlgItem(IDC_TOOL_STOP)->ShowWindow(SW_HIDE);
		SetDlgItemText(IDC_TOLL_STATE, "��е��");
		SetIDCState("Grip1", "��צ�ɿ�", false);
		break;
	case ROBOT_MODE_CR_W: // ���� - �������� - ����
		//		GetDlgItem(IDC_TOOL_STOP)->ShowWindow(SW_HIDE);
		SetDlgItemText(IDC_TOLL_STATE, "����:   ");
		SetIDCState("Pump1", "����1�ɿ�", false);
		SetIDCState("Pump2", "����2�ɿ�", false);

		SetDlgItemText(IDC_STATIC_VEL1, "����1");
		SetDlgItemText(IDC_STATIC_VEL2, "����2");
		SetDlgItemText(IDC_STATIC_VEL3, "����3");
		SetDlgItemText(IDC_STATIC_VEL4, "���");
		SetDlgItemText(IDC_STATIC_VEL5, "��ѹ");

		SetDlgItemText(IDC_STATIC_POS1, "X");
		SetDlgItemText(IDC_STATIC_POS2, "Y");
		SetDlgItemText(IDC_STATIC_POS3, "Z");
		SetDlgItemText(IDC_STATIC_POS4, "A");
		SetDlgItemText(IDC_STATIC_POS5, "B");
		SetDlgItemText(IDC_STATIC_POS7, "C");

		SetDlgItemText(IDC_STATIC_POS8, "���ɽ�");

		SetDlgItemText(IDC_TOOL_MOVE, "ƽ���˶�");
		SetDlgItemText(IDC_TOOL_MOVE2, "�����˶�");
		SetDlgItemText(IDC_TOOL_MOVE3, "��ʼ��¼");
		SetDlgItemText(IDC_TOOL_MOVE4, "ֹͣ��¼");

		SetDlgItemText(IDC_RADIO1, "1");
		SetDlgItemText(IDC_RADIO2, "2");

		break;
	case ROBOT_MODE_BR: // ˫��
		GetDlgItem(IDC_TOOL_STOP)->ShowWindow(SW_HIDE);
		SetDlgItemText(IDC_TOLL_STATE, "˫�㲽��");
		SetIDCState("Grip1", "����1", false);
		SetIDCState("Grip2", "����2", false);
		break;

	case ROBOT_MODE_SM:
	case ROBOT_MODE_DM:
	case ROBOT_MODE_WR: // ��ʽ
		break;
	default:
		break;
	}

	m_sliderVel.SetRange(2, 100);
	m_sliderVel.SetTicFreq(1);
	m_sliderVel.SetPos(8);
	CString str = "8%";
	SetDlgItemText(IDC_STATIC_VEL, str);

	((CButton*)GetDlgItem(IDC_RADIO1))->SetCheck(TRUE);

	// �������б�
	if (ROBOT_MODE_CR_P == Robot::robotMode)
	{
		if (FALSE == m_urgControl.scip_uart_init(USB_URG_NUM_G1))
		{
			isUrg1 = 0;
			//AfxMessageBox("����1 URG��������ʼ��ʧ��");//GU
		}
		else
			isUrg1 = 1;
		if (FALSE == m_urgControl.scip_uart_init(USB_URG_NUM_G2))
		{
			isUrg2 = 0;
			//AfxMessageBox("����2 URG��������ʼ��ʧ��//GU
		}
		else
			isUrg2 = 1;

		//	if (Ok != m_uscControl.ultra_uart_init(USB_USC_NUM))
		//if (Ok != Ultra_Can_Open(USB_USC_NUM))      //GU
		//{
		//	AfxMessageBox("���봫������ʼ��ʧ��");
		//	isUltraCan = 0;
		//}
		//else
		//{
		//	isUltraCan = 1;
		//	//	UltraCanSendCommand(1, pabi1cs);
		//	//	UltraCanSendCommand(2, pabi2cs);
		//}

		////////////////////////////////////////////////////////////////////////
		// Camera��ʼ��
		//////////////////////////////////////////////////////////////////////////
		isRunCam[0] = FALSE;
		isRunCam[1] = FALSE;
		//������ȡ����ͷ��Ŀ
		//cam_count = CCameraDS::CameraCount();
		////��ȡ��������ͷ������
		//for(i=0; i < cam_count; i++)
		//{
		//	char camera_name[1024];  
		//	int retval = CCameraDS::CameraName(i, camera_name, sizeof(camera_name) );
		//}

		//if(cam_count==0)
		//	AfxMessageBox("û�м�⵽����ͷ�豸!");

		SetTimer(5, 500, NULL);	//��ʾ����������״̬
		m_exSensorSet = 0;

		//stability_test
		stability_test_total_count = 0;
		stability_test_success_count = 0;
		stability_test_fail_count = 0;
	}
	return TRUE;  // return TRUE unless you set the focus to a control
	// EXCEPTION: OCX Property Pages should return FALSE
}

void CToolDlg::OnTimer(UINT nIDEvent) 
{
	// TODO: Add your message handler code here and/or call default
	int i;

	// Timer1 - ������������ - 200ms
	if (1 == nIDEvent)
	{
		if (ROBOT_MODE_CR_P == Robot::robotMode)
		{
			ReadData_CP();
		}
		else if (ROBOT_MODE_CR_W == Robot::robotMode)
		{			
			ReadData_CW();			
		}
	}


	// Timer2 - ѭ���˶�
	if (2 == nIDEvent)
	{
		if (m_nCommData[3] == 0)  // ������Ч
		{
			// ֹͣʾ��
			Robot::I_Task.Stop_Teach();
			KillTimer(2);
			m_bRepeat = false;
			SetDlgItemText(IDC_TOOL_MOVE3, "Setp2");
		}
		/*
		else if (m_nTrend > 1)  // ƫ������
		{
		if (m_nCount-- <= 0)
		{
		m_nCount = 0;
		}

		if (!m_bInverseDir)
		{
		int ln_teachdir = (((CButton*)GetDlgItem(IDC_RADIO1))->GetCheck()) ? 1 : -1;

		// ����
		int ln_vel[6] = {0};
		int ln_dir[6] = {0};
		ln_vel[5] = (int)(m_sliderVel.GetPos());				
		ln_vel[1] = (int)((double)ln_vel[5]*MAX_TEACHVEL_CW*PI_RAD*SCIP_CENTER_LEN/MAX_TEACHVEL_CV);
		ln_dir[5] = -ln_teachdir;
		ln_dir[1] = ln_teachdir;
		if (RUNMODE_TEACH == Robot::runMode)   // ʾ��ģʽ
		{
		// ����ʾ������
		Robot::I_Task.Teach(COORDINATE_WORLD, ln_vel, ln_dir);
		}
		m_bInverseDir = true;
		m_nCount = 2;

		// ��¼��λ��
		Robot::I_Monit.Get_JointPos(m_dPos[0]);	
		}
		else if (m_nCount == 0)
		{
		// ֹͣʾ��
		Robot::I_Task.Stop_Teach();

		// ��¼��λ��
		Robot::I_Monit.Get_JointPos(m_dPos[1]);	

		for(i=0; i<JOINT_NUM; i++)
		{
		m_dPos[2][i] = 0.5*(m_dPos[0][i]+m_dPos[1][i]);
		}

		// �˶���ָ��λ��
		int vel = (int)(m_sliderVel.GetPos());
		CMotionData data;
		data.Line = 1;
		data.Mode = MODE_INTERP;
		data.Interp.Mode = INTERP_JOINT; //PTP
		data.Interp.IfLinkage = 0;
		for(i=0; i<JOINT_NUM; i++)
		data.Interp.JEnd[i] = m_dPos[2][i]; //deg
		data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
		data.Interp.Acc = MAX_AUTOACC_JOINT;
		data.Interp.Jerk = MAX_JERK_JOINT;

		Robot::I_Task.AddMotionData(data);	
		Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

		// ֹͣ
		KillTimer(2);
		m_bRepeat = false;
		SetDlgItemText(IDC_TOOL_MOVE3, "Setp2");

		gb_endofstep2 = true;

		}
		}
		*/
		else if (abs(m_nCommData[4]) > USC_MAX_LEN)  // ƫ������
		{
			if (m_nCount-- <= 0)
			{
				m_nCount = 0;
			}

			if (!m_bInverseDir)
			{
				int ln_teachdir = (((CButton*)GetDlgItem(IDC_RADIO1))->GetCheck()) ? 1 : -1;

				// ����
				int ln_vel[6] = {0};
				int ln_dir[6] = {0};
				ln_vel[5] = (int)(20);				
				ln_vel[1] = (int)((double)ln_vel[5]*MAX_TEACHVEL_CW*PI_RAD*SCIP_CENTER_LEN/MAX_TEACHVEL_CV);
				ln_dir[5] = -ln_teachdir;
				ln_dir[1] = ln_teachdir;
				if (RUNMODE_TEACH == Robot::runMode)   // ʾ��ģʽ
				{
					// ����ʾ������
					Robot::I_Task.Teach(COORDINATE_WORLD, ln_vel, ln_dir);
				}
				m_bInverseDir = true;
				m_nCount = 8;

				// ��¼��λ��
				Robot::I_Monit.Get_JointPos(m_dPos[0]);	
			}
			else if (m_nCount == 0)
			{
				// ֹͣʾ��
				Robot::I_Task.Stop_Teach();

				// ��¼��λ��
				Robot::I_Monit.Get_JointPos(m_dPos[1]);	

				for(i=0; i<JOINT_NUM; i++)
				{
					m_dPos[2][i] = 0.5*(m_dPos[0][i]+m_dPos[1][i]);
				}

				Sleep(500);

				// �˶���ָ��λ��
				int vel = (int)(m_sliderVel.GetPos());
				CMotionData data;
				data.Line = 1;
				data.Mode = MODE_INTERP;
				data.Interp.Mode = INTERP_JOINT; //PTP
				data.Interp.IfLinkage = 0;
				for(i=0; i<JOINT_NUM; i++)
					data.Interp.JEnd[i] = m_dPos[2][i]; //deg
				data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
				data.Interp.Acc = MAX_AUTOACC_JOINT;
				data.Interp.Jerk = MAX_JERK_JOINT;

				Robot::I_Task.AddMotionData(data);	
				Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

				// ֹͣ
				KillTimer(2);
				m_bRepeat = false;
				SetDlgItemText(IDC_TOOL_MOVE3, "Setp2");

				gb_endofstep2 = true;

			}
		}


		m_nTrend = 0;
	}


	// Timer3 - Tracking�˶�
	if (3 == nIDEvent)
	{
		int ln_vel[6] = {0};
		int ln_dir[6] = {0};

		if (m_nTrack == 1) // λ�ø���
		{
			// ��ȡ��λ��
			if (m_nCommData[0] = 1)
			{	
				double ld_temp = sqrt((double)(SQUARE(m_nCommData[1]) + SQUARE(m_nCommData[2])));
				double velpct1 = (ld_temp>10) ? abs(m_nCommData[1])/ld_temp : 0;
				double velpct2 = (ld_temp>10) ? abs(m_nCommData[2])/ld_temp : 0;			

				ln_vel[1] = (int)(m_sliderVel.GetPos()*velpct1);
				ln_vel[2] = (int)(m_sliderVel.GetPos()*velpct2);
				ln_dir[1] = (m_nCommData[1] > 0) ? 1 : -1;
				ln_dir[2] = (m_nCommData[2] > 0) ? 1 : -1;

				ln_vel[5] = (fabs(m_dwz)>5) ? 20 : 0;
				ln_dir[5] = (m_dwz > 0) ? 1 : -1;
			}
		}
		else if (m_nTrack == 2) // ��̬����
		{	
			ln_vel[5] = (fabs(m_dwz)>5) ? 20 : 0;
			ln_dir[5] = (m_dwz > 0) ? 1 : -1;
		}

		if (RUNMODE_TEACH == Robot::runMode)   // ʾ��ģʽ
		{
			// ����ʾ������
			Robot::I_Task.Teach(COORDINATE_WORLD, ln_vel, ln_dir);
		}
	}

	if (4 == nIDEvent)
	{
		// 		if (isRunCam[0] == TRUE && isRunCam[1] == FALSE)
		// 		{
		// 			//��ȡһ֡
		// 			IplImage *pFrame1 = camera.QueryFrame();
		// 			//��ʾ
		// 			cvShowImage("camera", pFrame1);
		// 		} 
		// 		else if (isRunCam[1] == TRUE && isRunCam[0] == FALSE)
		// 		{
		// 			//��ȡһ֡
		// 			IplImage *pFrame2 = camera.QueryFrame();
		// 			//��ʾ
		// 			cvShowImage("camera", pFrame2);
		// 		}
		CamProcess();
	}

	//��ʾ����������״̬
	if (5 == nIDEvent)	//��ʾ����������״̬
	{
		CString strSenState1;
		CString	strSenState2;
		//////////////////////////////////////////////////////////////////////////
		//URG
		switch (isUrg1)
		{
		case 0:
			strSenState1 = "Urg:No  ";
			break;
		case 1:
			strSenState1 = "Urg:Open  ";
			break;
		default:
			strSenState1 = "Urg:NUL  ";
			break;
		}
		switch (isUrg2)
		{
		case 0:
			strSenState2 = "Urg:No  ";
			break;
		case 1:
			strSenState2 = "Urg:Open  ";
			break;
		default:
			strSenState2 = "Urg:NUL  ";
			break;
		}

		//////////////////////////////////////////////////////////////////////////
		//Can2.0
		switch (isUltraCan)
		{
		case 0:
			strSenState1 += "Can:NO  ";
			strSenState2 += "Can:NO  ";
			break;
		case 1:
			strSenState1 += "Can:Open  ";
			strSenState2 += "Can:Open  ";
			break;
		case 2:
			strSenState1 += "Can:Runs  ";
			strSenState2 += "Can:Runs  ";
			break;
		case 3:
			strSenState1 += "Can:Err  ";
			strSenState2 += "Can:Err  ";
			break;
		default:
			strSenState1 += "Can:NUL  ";
			strSenState2 += "Can:NUL  ";
			break;
		}

		//////////////////////////////////////////////////////////////////////////
		//Camera
		if (isRunCam[0] == TRUE)
		{
			strSenState1 += "Cam:OK  ";
		}
		else if (isRunCam[0] == FALSE)
		{
			strSenState1 += "Cam:NO  ";
		}
		else
		{
			strSenState1 += "Cam:NUL  ";
		}

		if (isRunCam[1] == TRUE)
		{
			strSenState2 += "Cam:OK  ";
		}
		else if (isRunCam[1] == FALSE)
		{
			strSenState2 += "Cam:NO  ";
		}
		else
		{
			strSenState2 += "Cam:NUL  ";
		}

		SetDlgItemText(IDC_STATE_SENSOR1, strSenState1);
		SetDlgItemText(IDC_STATE_SENSOR2, strSenState2);

		if (m_bTool1 == true && m_bTool2 == false)	//����1�н�������2�ɿ����Ѽ���2�ϵĴ��������
		{		
			if (isRunCam[0]==TRUE && isRunCam[1]==FALSE)
			{
				CamDevOpen(2);
				m_exSensorSet = 2;
			}
		}			
		if (m_bTool2 == true && m_bTool1 == false)	//����2�н�������1�ɿ����Ѽ���1�ϵĴ��������
		{	
			if (isRunCam[1]==TRUE && isRunCam[0]==FALSE)
			{
				CamDevOpen(1);
				m_exSensorSet = 1;
			}
		}		
	}

	//��ʾ���ӵ���
	if (6 == nIDEvent)	//��ʾ���ӵ���
	{
		CString str;
		double curr[2];
		Robot::I_Servo.Get_GripCurrent(curr);
		//	Robot::I_Servo::m_giAmpGrip[0].GetCurrentActual(gcur1);
		//	::m_giAmpGrip[0].GetCurrentActual(gcur1); //ʵʱ��ص��ʵ�ʵ���
		GrpCur1 = curr[0];

		if (GrpCur1 >= 1.45 || GrpCur1 <= -1.45)
			m_nCntOverCur++;
		else
			m_nCntOverCur = 0;

		if (m_nCntOverCur > 5)
		{
			m_nCntOverCur = 0;
			KillTimer(6);
			StopTool();
			SetDlgItemText(IDC_STATE_G1_CUR, "0.000");
		}
		else
		{
			str.Format("%.3f", GrpCur1);
			SetDlgItemText(IDC_STATE_G1_CUR, str);
		}	
	}
	if (7==nIDEvent)
	{
		double dPos1[6];
		//���������Ƿ񵽴�Ԥ�гֵ�
		if (!m_bReachTarget)
		{
			Robot::I_Monit.Get_JointPos(dPos1);

			if ((abs(DetectJointPosition[0]-dPos1[0])<0.1)&&(abs(DetectJointPosition[1]-dPos1[1])<0.1)&&(abs(DetectJointPosition[2]-dPos1[2])<0.1)&&
				(abs(DetectJointPosition[3]-dPos1[3])<0.1)&&(abs(DetectJointPosition[4]-dPos1[4])<0.1))
			{
				ReDetection = true;
				m_bReachTarget = true;
			}
			else
			{
				Robot::I_Task.ClearData();

				CMotionData data;
				data.Line = 1;
				data.Mode = MODE_INTERP;
				data.Interp.Mode = INTERP_JOINT; //PTP
				data.Interp.IfLinkage = 0;
				for(int i=0; i<JOINT_NUM; i++)
					data.Interp.JEnd[i] = DetectJointPosition[i]; //deg
				data.Interp.Vel = sliderVel; //deg/s
				data.Interp.Acc = MAX_AUTOACC_JOINT;
				data.Interp.Jerk = MAX_JERK_JOINT;

				Robot::I_Task.AddMotionData(data);	
				Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

				m_bReachTarget = false;
			}
		}

		//�����ظ����
		if (ReDetection==true)
		{
			//�˶�
			Robot::I_Task.ClearData();

			int vel =sliderVel;
			CMotionData data;
			data.Line = 1;
			data.Mode = MODE_INTERP;
			data.Interp.Mode = INTERP_JOINT; //PTP
			data.Interp.IfLinkage = 0;//��������־
			for(int i=0; i<JOINT_NUM; i++)
			{
				data.Interp.JEnd[i] = dposendtemp[i]; //deg
			}
			data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 80; //deg/s
			data.Interp.Acc = MAX_AUTOACC_JOINT;
			data.Interp.Jerk = MAX_JERK_JOINT;
			// 
			Robot::I_Task.AddMotionData(data);	
			Robot::I_Task.Set_GivenPosRunMode(TRUE); // ����λ��ʾ��ģʽ
			RedetSucCount = 0;
			ReDetection = false;
			Sleep(500);
			KillTimer(7);
		} 
		//else
		//{
		//	ReDetection =  false;
		//}
	}

	if (8==nIDEvent)
	{
		int flag;
		double dPos[MAX_AXIS_NUM] = {0,0,0,0,0,0};
		double dgetpos[6];
		double judgepos[6];

		Robot::I_Monit.Get_JointPos(dgetpos);

		((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_CHECK_ROLLDIR))->EnableWindow(true);

		for (int i=0; i<6; i++)
		{
			judgepos[i] =  ((abs(tPos_temp[i]-dgetpos[i])>0.5) ? 1 : 0);
		}
		if (judgepos[0]+judgepos[1]+judgepos[2]+judgepos[3]+judgepos[4] == 0)
		{
			for (int i=0; i<JOINT_NUM; i++)
			{
				flag = Robot::I_Servo.Set_JointPosition(i, dPos[i]);
				if (Ok != flag)
					break;
			}
			if (m_MotionChoose==3 || m_MotionChoose==4)
			{
				ChangeDir();
			}


			if (Ok != flag)
				AfxMessageBox("����ʧ�ܣ������ŷ�ͨѶ");
		}
	}
	CDialog::OnTimer(nIDEvent);
}


int CToolDlg::Ultra_Can_Open(char Com)
{
	//��ر�����ʼ��
	ultra_dis1[0] = 0;
	ultra_dis2[0] = 0;
	ultra_dis1[1] = 0;
	ultra_dis2[1] = 0;
	//
	m_ctrlCanCom.SetSettings("1228800,n,8,1"); //������1228800����У�飬8������λ��1��ֹͣλ
	//�򿪴���
	m_ctrlCanCom.SetCommPort(Com);			//ѡ��COM����
	m_ctrlCanCom.SetInputMode(1);			//���뷽ʽΪ�����Ʒ�ʽ
	m_ctrlCanCom.SetInBufferSize(1024);	//�������뻺������С
	m_ctrlCanCom.SetOutBufferSize(512);	//���������������С
	if(!m_ctrlCanCom.GetPortOpen())
	{
		m_ctrlCanCom.SetPortOpen(TRUE);	//�򿪴���
		m_ctrlCanCom.SetRThreshold(1); 
		m_ctrlCanCom.SetInputLen(0);			//���õ�ǰ���������ݳ���Ϊ0
		m_ctrlCanCom.GetInput();				//��Ԥ���������������������
		return Ok;
	}
	else
	{
		return ERR_CAN20;
	}
}

void CToolDlg::OnClose() 
{
	// TODO: Add your message handler code here and/or call default
	CWnd* pWnd = AfxGetMainWnd();
	::SendMessage( pWnd->GetSafeHwnd(), WM_MAIN_DLG_MESSAGE, (WPARAM) MY_TOLLDLG_CLOSE, (LPARAM) 0);

	m_bSensor = false;

	delete(m_pCprimeSenseDet);  //�ͷ������m_pCprimeSenseDet�ĳ�Աָ����ָ�ڴ棬�ͷ�������ڴ�
	m_pCprimeSenseDet = NULL;
	CDialog::OnClose();
}


void CToolDlg::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar) 
{
	// TODO: Add your message handler code here and/or call default
	CDialog::OnHScroll(nSBCode, nPos, pScrollBar);

	CString str;
	str.Format("%d", m_sliderVel.GetPos());
	str += "%";
	SetDlgItemText(IDC_STATIC_VEL, str);
}


void CToolDlg::SetIDCState(CString IDC, CString msg, bool run)
{
	int R,B,G;

	if (false == run)
	{
		R = B = G = 175; //192
	}
	else
	{
		R = 0;
		G = 175;//175;
		B = 0; //45
	}

	if (("Pump1" == IDC) || ("Grip1" == IDC))
	{
		m_tool1Control.SetWindowText(msg);
#ifdef USING_CBUTTONST
		m_tool1Control.SetColor(CButtonST::BTNST_COLOR_BK_IN,RGB(R,G,B),true);
		m_tool1Control.SetColor(CButtonST::BTNST_COLOR_BK_FOCUS,RGB(R,G,B),true);
		m_tool1Control.SetColor(CButtonST::BTNST_COLOR_BK_OUT,RGB(R,G,B),false);
#endif
	}

	if (("Pump2" == IDC) || ("Grip2" == IDC))
	{	
		m_tool2Control.SetWindowText(msg);
#ifdef USING_CBUTTONST
		m_tool2Control.SetColor(CButtonST::BTNST_COLOR_BK_IN,RGB(R,G,B),true);
		m_tool2Control.SetColor(CButtonST::BTNST_COLOR_BK_FOCUS,RGB(R,G,B),true);
		m_tool2Control.SetColor(CButtonST::BTNST_COLOR_BK_OUT,RGB(R,G,B),false);
#endif
	}
}

//��צ1--�н�/�ɿ�����
void CToolDlg::ControlTool_First(bool bOpen)
{
	if ((ROBOT_MODE_CR_P == Robot::robotMode) || // ���� - ��ֻ��צ - �ŷ�����
		(ROBOT_MODE_IR_5 == Robot::robotMode) || // ��е�� - һֻ��צ - �ŷ�����
		(ROBOT_MODE_IR_6 == Robot::robotMode))
	{
		CString str ;

		int nGripCurr;

		// �н���צ1
		if (false == bOpen)
		{
			if (ROBOT_MODE_CR_P == Robot::robotMode)
				nGripCurr = -250;
			else
				nGripCurr = -150;

			// �н���צ
			if (Ok == Robot::I_Servo.Servo_Grip(GRIP_FIRST, nGripCurr))	//CAN6 ��Ӧ��צ2
			{
				str = (ROBOT_MODE_CR_P == Robot::robotMode) ? "��צ1�н�" : "��צ�н�";
				SetIDCState("Grip1", str, true);

				m_bTool1 = true;
				SetTimer(6, CURDETECTTIME, NULL);	//��ؼг���������ʱ��

				Robot::actualTool = GRIP_FIRST;
				//	m_exSensorSet = 2;
				Robot::I_Task.Set_GripNum(Robot::actualTool);
				Robot::I_Monit.Set_Grip(Robot::actualTool);
				if(ROBOT_MODE_CR_P == Robot::robotMode)
					SetDlgItemText(IDC_TOLL_STATE, "����: #1");
			}
			else
			{
				str = (ROBOT_MODE_CR_P == Robot::robotMode) ? "��צ1����" : "��צ����";
				SetIDCState("Grip1", str, true);
			}
		}
		// �ɿ���צ1
		else if (true == bOpen) 
		{
			if (ROBOT_MODE_CR_P == Robot::robotMode)
				nGripCurr = 250;
			else
				nGripCurr = 150;

			// �ɿ���צ
			if (Ok == Robot::I_Servo.Servo_Grip(GRIP_FIRST,nGripCurr))
			{
				// �ı䰴ť״̬
				str = (ROBOT_MODE_CR_P == Robot::robotMode) ? "��צ1�ɿ�" : "��צ�ɿ�";
				SetIDCState("Grip1", str, false);

				m_bTool1 = false;

				Robot::actualTool = GRIP_SECOND;
				Robot::I_Task.Set_GripNum(Robot::actualTool);
				Robot::I_Monit.Set_Grip(Robot::actualTool);
				if(ROBOT_MODE_CR_P == Robot::robotMode)
					SetDlgItemText(IDC_TOLL_STATE, "����: #2");
			}
			else
			{
				str = (ROBOT_MODE_CR_P == Robot::robotMode) ? "��צ1����" : "��צ����";
				SetIDCState("Grip1", str, true);
			}
		}	
	}
	else if (ROBOT_MODE_CR_W == Robot::robotMode)// ���� - �������� - ����
	{
		// �����̲���
		if (false == bOpen)
		{
			// �ر�Valve1
			Robot::I_Pump.Close_Valve(1);

			// ��Pump1
			Robot::I_Pump.Open_Pump(1);

			SetIDCState("Pump1", "����1����", true);

			m_bTool1 = true;

			Robot::actualTool = GRIP_FIRST;
			Robot::I_Task.Set_GripNum(Robot::actualTool);
			Robot::I_Monit.Set_Grip(Robot::actualTool);
			SetDlgItemText(IDC_TOLL_STATE, "����: #1");
		}
		// �ر����̲���
		else if (true == bOpen) 
		{
			// �ر�Pump1
			Robot::I_Pump.Close_Pump(1);

			// ��Valve1
			Robot::I_Pump.Open_Valve(1);

			// �ı䰴ť״̬
			SetIDCState("Pump1", "����1�ɿ�", false);

			m_bTool1 = false;

			Robot::actualTool = GRIP_SECOND;
			Robot::I_Task.Set_GripNum(Robot::actualTool);
			Robot::I_Monit.Set_Grip(Robot::actualTool);
			SetDlgItemText(IDC_TOLL_STATE, "����: #2");
		}
	}
	else  if (ROBOT_MODE_BR == Robot::robotMode)// ˫��
	{
		// �򿪲���
		if (false == bOpen)
		{
			m_bTool1 = true;

			SetIDCState("Grip1", "����1", true);

			Robot::actualTool = GRIP_FIRST;
			Robot::I_Task.Set_GripNum(Robot::actualTool);
			Robot::I_Monit.Set_Grip(Robot::actualTool);
		}
		// �رղ���
		else 
		{
			m_bTool1 = false;

			SetIDCState("Grip1", "����1", false);

			Robot::actualTool = GRIP_SECOND;
			Robot::I_Task.Set_GripNum(Robot::actualTool);
			Robot::I_Monit.Set_Grip(Robot::actualTool);
		}
	}
}

//��צ2--�н����ɿ���CAN6 ��Ӧ��צ2��
void CToolDlg::ControlTool_Second(bool bOpen)
{
	if (ROBOT_MODE_CR_P == Robot::robotMode) // ���� - ��ֻ��צ - �ŷ�����
	{
		// �н���צ2
		if (false == bOpen)
		{
			// �н���צ
			if (Ok == Robot::I_Servo.Servo_Grip(GRIP_SECOND, -250))
			{
				// �ı䰴ť״̬
				SetIDCState("Grip2", "��צ2�н�", true);

				m_bTool2 = true;

				Robot::actualTool = GRIP_SECOND;
				//	m_exSensorSet = 1;
				Robot::I_Task.Set_GripNum(Robot::actualTool);
				Robot::I_Monit.Set_Grip(Robot::actualTool);
				SetDlgItemText(IDC_TOLL_STATE, "����: #2");
			}
			else
			{
				SetIDCState("Grip2", "��צ2����", true);
			}
		}
		// �ɿ���צ2
		else if (true == bOpen) 
		{
			// �ɿ���צ
			if (Ok == Robot::I_Servo.Servo_Grip(GRIP_SECOND, 250))
			{
				// �ı䰴ť״̬
				SetIDCState("Grip2", "��צ2�ɿ�", false);

				m_bTool2 = false;

				Robot::actualTool = GRIP_FIRST;
				Robot::I_Task.Set_GripNum(Robot::actualTool);
				Robot::I_Monit.Set_Grip(Robot::actualTool);
				SetDlgItemText(IDC_TOLL_STATE, "����: #1");
			}
			else
			{
				SetIDCState("Grip2", "��צ2����", true);
			}
		}	
	}
	else if (ROBOT_MODE_CR_W == Robot::robotMode)// ���� - �������� - ����
	{
		// �����̲���
		if (false == bOpen)
		{
			// �ر�Valve2
			Robot::I_Pump.Close_Valve(2);

			// ��Pump2
			Robot::I_Pump.Open_Pump(2);

			// �ı䰴ť״̬
			SetIDCState("Pump2", "����2����", true);

			m_bTool2 = true;

			Robot::actualTool = GRIP_SECOND;
			Robot::I_Task.Set_GripNum(Robot::actualTool);
			Robot::I_Monit.Set_Grip(Robot::actualTool);
			SetDlgItemText(IDC_TOLL_STATE, "����: #2");
		}
		else if (true == bOpen) // �ر����̲���
		{
			// �ر�Pump2
			Robot::I_Pump.Close_Pump(2);

			// ��Valve2
			Robot::I_Pump.Open_Valve(2);

			// �ı䰴ť״̬
			SetIDCState("Pump2", "����2�ɿ�", false);

			m_bTool2 = false;

			Robot::actualTool = GRIP_FIRST;
			Robot::I_Task.Set_GripNum(Robot::actualTool);  // ���õ�ǰ����
			Robot::I_Monit.Set_Grip(Robot::actualTool);
			SetDlgItemText(IDC_TOLL_STATE, "����: #1");
		}
	}
	else  if (ROBOT_MODE_BR == Robot::robotMode)// ˫��
	{
		// �򿪲���
		if (false == bOpen)
		{
			m_bTool2 = true;

			SetIDCState("Grip2", "����2", true);

			Robot::actualTool = GRIP_SECOND;
			Robot::I_Task.Set_GripNum(Robot::actualTool);
			Robot::I_Monit.Set_Grip(Robot::actualTool);
		}
		// �رղ���
		else
		{
			m_bTool2 = false;

			SetIDCState("Grip2", "����2", false);

			Robot::actualTool = GRIP_FIRST;
			Robot::I_Task.Set_GripNum(Robot::actualTool);
			Robot::I_Monit.Set_Grip(Robot::actualTool);
		}
	}
}

void CToolDlg::StopTool()
{
	if (ROBOT_MODE_CR_P == Robot::robotMode) // ���� - ��ֻ��צ - �ŷ�����
	{
		if (Ok != Robot::I_Servo.Servo_Grip(GRIP_FIRST, 0))
			SetIDCState("Grip1", "��צ1����", true);
		if (Ok != Robot::I_Servo.Servo_Grip(GRIP_SECOND, 0))
			SetIDCState("Grip2", "��צ2����", true);
	}
	else if (ROBOT_MODE_CR_W == Robot::robotMode) // ���� - �������� - ����
	{
		if (Ok != Robot::I_Pump.Stop_Pump())
		{
			SetIDCState("Grip1", "���̴���", true);
		}

		m_bTool1 = false;
		m_bTool2 = false;
		//ֹͣ������ȡ
		SetDlgItemText(IDC_STATE_G1_CUR, "");
		KillTimer(6);

		// �ı䰴ť״̬
		SetIDCState("Pump1", "����1�ɿ�", false);
		SetIDCState("Pump2", "����2�ɿ�", false);
	}
	else if ((ROBOT_MODE_IR_5 == Robot::robotMode) || // ��е�� - һֻ��צ - �ŷ�����
		(ROBOT_MODE_IR_6 == Robot::robotMode))
	{
		if (Ok != Robot::I_Servo.Servo_Grip(GRIP_FIRST, 0))
			SetIDCState("Grip1", "��צ1����", true);
	}
}

//������������--���˹���
void CToolDlg::ReadData_CP()
{
	int y, z;
	CString str;
	int num = (((CButton*)GetDlgItem(IDC_RADIO1))->GetCheck()) ? 1 : 2;
	//		int data[7] = {0};

	// ��ȡ��λ��
	if (gb_URG_Enable)
	{

		if(TRUE == m_urgControl.GetPoleCoordinate(m_exSensorSet, &y, &z))
		{
			m_nCommData[0] = 1;
			m_nCommData[1] = y;
			m_nCommData[2] = z;
			gd_URG_Len_y = y;

			SetDlgItemText(IDC_INCREM_POS1, "OK");
			str.Format("%d", y);
			SetDlgItemText(IDC_INCREM_POS2, str);
			str.Format("%d", z);
			SetDlgItemText(IDC_INCREM_POS3, str);

		}
		else
		{
			m_nCommData[0] = 0;
			m_nCommData[1] = 0;
			m_nCommData[2] = 0;
			gd_URG_Len_y = 0;

			SetDlgItemText(IDC_INCREM_POS1, "ERR");
		}			
	}
	else
	{
		m_nCommData[0] = 0;
		m_nCommData[1] = 0;
		m_nCommData[2] = 0;
		gd_URG_Len_y = 0;

		SetDlgItemText(IDC_INCREM_POS1, "Disable");
	}

	// ��ȡλ�ô�������Ϣ
	//	if (Ok == m_uscControl.getUltrasonic(1,	&y, &z))
	if (TRUE == UltraCanSendCommand(m_exSensorSet, pabics))
	{
		int numDevtmp;
		if (0 == CANIDEX);
		else if (1 == CANIDEX)
		{
			switch (m_exSensorSet)
			{
			case 1:
				numDevtmp = 2;
				break;
			case 2:
				numDevtmp = 1;
				break;
			}
			m_exSensorSet = numDevtmp;
		}
		y = ultra_dis1[m_exSensorSet-1]; //��ȡ�����ж���ʵʱ���µ�����
		z = ultra_dis2[m_exSensorSet-1];
		//y = y-8;
		//z = z-14;
		m_nCommData[3] = 1;
		m_nCommData[4] = y;
		m_nCommData[5] = z;

		if (abs(y-z) < L_ULTRA)  // ������Ч
		{
			m_dCommAngle = asin((double)(y-z)/L_ULTRA) * PI_DEG;
			if (fabs(m_dCommAngle) > 20)
			{
				m_dCommAngle = 0;
			}

			if (abs(y-z) > 25)
			{
				m_nCommData[6] = 0;
			}
			else
			{
				m_nCommData[6] = (y+z)/2 - D_U_M;			
			}
		}
		else
		{
			m_dCommAngle = 0;
			m_nCommData[6] = 0;
		}

		if (isUltraCan == 2)
		{
			SetDlgItemText(IDC_COMM_POS1, "OK");
		} 
		else if (isUltraCan == 3)
		{
			SetDlgItemText(IDC_COMM_POS1, "Err");
		}	

		str.Format("%d", y);
		SetDlgItemText(IDC_COMM_POS4, str);	

		str.Format("%d", z);
		SetDlgItemText(IDC_COMM_POS3, str);	

		str.Format("%0.1f", m_dCommAngle);
		SetDlgItemText(IDC_COMM_DEG, str);

		//str.Format("%d", m_nCommData[6]);
		m_dwz = m_dwz_bzq;
		str.Format("%.1f", m_dwz);
		SetDlgItemText(IDC_COMM_PRESS, str);

		// ����仯�����ж�
		// ��צ1�̶�
		if (GRIP_FIRST == Robot::actualTool)
		{
			if (abs(y-z) > L_ULTRA)  // ������Ч
			{
				m_nTrend++;
			}
			else if (z>m_nUltraLen2)
			{
				m_nTrend++;
			}
			else if (z<m_nUltraLen2)
			{
				m_nTrend--;
			}
		}
		else if (GRIP_SECOND == Robot::actualTool)
		{
			if (abs(y-z) > L_ULTRA)  // ������Ч
			{
				m_nTrend++;
			}
			else if (y>m_nUltraLen1)
			{
				m_nTrend++;
			}
			else if (y<m_nUltraLen1)
			{
				m_nTrend--;
			}
		}

		m_nUltraLen1 = y;
		m_nUltraLen2 = z;
	}
	else
	{
		m_nCommData[3] = 0;
		m_nCommData[4] = 0;
		m_nCommData[5] = 0;

		SetDlgItemText(IDC_COMM_POS1, "ERR");
	}
	/*
	// ��¼��λ��
	for(i=0; i<JOINT_NUM; i++)
	{
	m_dPos[0][i] = m_dPos[1][i];
	m_dPos[1][i] = m_dPos[2][i];
	}
	Robot::I_Monit.Get_JointPos(m_dPos[2]);	
	*/

	str.Format("%d", test_count++);
	SetDlgItemText(IDC_INCREM_POS4, str);		
}

//������������--���ڹ���
void CToolDlg::ReadData_CW()
{
	CString str;
	int num = (((CButton*)GetDlgItem(IDC_RADIO1))->GetCheck()) ? 1 : 2;
	int data[7] = {0};
	int i;

	if (Ok != Robot::I_Pump.Get_PumpData(num, data))
	{
		str.Format("%d", 0);
		SetDlgItemText(IDC_COMM_POS1, str);	
		SetDlgItemText(IDC_COMM_POS4, str);	
		SetDlgItemText(IDC_COMM_POS3, str);	
		SetDlgItemText(IDC_COMM_DEG, str);	
		SetDlgItemText(IDC_COMM_PRESS, str);

		SetDlgItemText(IDC_STATIC_POS8, "Error");
	}
	else
	{

		data[0] += SENSOR1_INCREM;
		data[1] += SENSOR2_INCREM;
		data[2] += SENSOR3_INCREM;

		str.Format("%d", data[0]);
		SetDlgItemText(IDC_COMM_POS1, str);	

		str.Format("%d", data[1]);
		SetDlgItemText(IDC_COMM_POS4, str);	

		str.Format("%d", data[2]);
		SetDlgItemText(IDC_COMM_POS3, str);	

		str.Format("%d", data[5]);
		SetDlgItemText(IDC_COMM_PRESS, str);	

		str.Format("%d", data[6]);
		SetDlgItemText(IDC_COMM_DEG, str);	

		SetDlgItemText(IDC_STATIC_POS8, "���ɽ�");
	}

	for(i=0; i<7; i++)
		m_nCommData[i] = data[i];
	m_nCommData[1] = data[2];
	m_nCommData[2] = data[1];


	double increpos[6] = {0};

	if ((m_nCommData[0] < RT_LITTLE) &&
		(m_nCommData[1] < RT_LITTLE) &&
		(m_nCommData[2] < RT_LITTLE) )

	{
		//		return;
	}

	// ��ȡ����λ��
	Get_IncrePos(increpos);

	str.Format("%0.3f", increpos[0]);
	SetDlgItemText(IDC_INCREM_POS1, str);	

	str.Format("%0.3f", increpos[1]);
	SetDlgItemText(IDC_INCREM_POS2, str);	

	str.Format("%0.3f", increpos[2]);
	SetDlgItemText(IDC_INCREM_POS3, str);	

	str.Format("%0.3f", increpos[3]);
	SetDlgItemText(IDC_INCREM_POS4, str);	

	str.Format("%0.3f", increpos[4]);
	SetDlgItemText(IDC_INCREM_POS5, str);	

	str.Format("%0.3f", increpos[5]);
	SetDlgItemText(IDC_INCREM_POS6, str);


	// ��ȡ���ɽ�
	increpos[0] = atan2((float)(m_nCommData[1] - m_nCommData[2]), (float)SENSOR_LENGTH);
	increpos[0] *= PI_DEG;

	str.Format("%0.3f", increpos[0]);
	SetDlgItemText(IDC_INCREM_DEG1, str);	

}

void CToolDlg::OnToolFirst() 
{
	// TODO: Add your control notification handler code here
	ControlTool_First(m_bTool1);
	if (m_deviceclosesymbol)
	{
		m_pCprimeSenseDet->CloseTheDevice();
		m_deviceclosesymbol = false;
	}
}

void CToolDlg::OnToolSecond() 
{
	// TODO: Add your control notification handler code here
	ControlTool_Second(m_bTool2);
	if (m_deviceclosesymbol)
	{
		m_pCprimeSenseDet->CloseTheDevice();
		m_deviceclosesymbol = false;
	}
}

void CToolDlg::OnToolStop() 
{
	// TODO: Add your control notification handler code here
	StopTool();
}

//��ָ���豸
void CToolDlg::OnBnClickedButtonOpendevice()
{
	// TODO: Add your control notification handler code here
	m_deviceclosesymbol = m_pCprimeSenseDet->OpenTheDevice(m_bTool1,m_bTool2);

	if (m_deviceclosesymbol)
	{
		DeteSymbol = true;
	}	 
}


// ����Esc��Enter��
BOOL CToolDlg::PreTranslateMessage(MSG* pMsg) 
{
	// TODO: Add your specialized code here and/or call the base class

	// ����Esc��Enter��
	if(pMsg->message == WM_KEYDOWN) 
	{ 
		switch(pMsg->wParam) 
		{ 
		case VK_RETURN:  // ���λس�
		case VK_ESCAPE:  // ����Esc 
			return TRUE;
			break;
		}
	}

	return CDialog::PreTranslateMessage(pMsg);
}

void CToolDlg::OnNcDestroy() 
{
	CDialog::OnNcDestroy();

	// TODO: Add your message handler code here
	m_bSensor = false;
	delete this;
}

void CToolDlg::OnToolGetdata() 
{
	// TODO: Add your control notification handler code here

	if (ROBOT_MODE_CR_P == Robot::robotMode)
	{
		gb_endofstep2 = true;

		if (!(m_bTool1 == false && m_bTool2 == false))
		{
			if (false == m_bSensor)
			{
				if (isUltraCan == false)
				{
					if (Ok != Ultra_Can_Open(USB_USC_NUM))
					{
						isUltraCan = false;
						AfxMessageBox("Can2.0��ʧ��");
					}
					else
					{
						isUltraCan = true;
					}									
				}
				if (isUrg1 == 0)
				{
					if (FALSE == m_urgControl.scip_uart_init(USB_URG_NUM_G1))
					{
						isUrg1 = 0;
						AfxMessageBox("����1 URG��������ʼ��ʧ��");
					}
					else
						isUrg1 = 1;
				}
				if (isUrg2 == 0)
				{
					if (FALSE == m_urgControl.scip_uart_init(USB_URG_NUM_G2))
					{
						isUrg2 = 0;
						AfxMessageBox("����2 URG��������ʼ��ʧ��");
					}
					else
						isUrg2 = 1;
				}

				SetTimer(1, TIMER1_COUNT, NULL);
				m_bSensor = true;
				SetDlgItemText(IDC_TOOL_GETDATA, "ֹͣ������");

				gb_URG_Enable = true;
			}
			else
			{
				test_count = 0;

				if (isUltraCan > 0)
					m_ctrlCanCom.SetPortOpen(FALSE);
				isUltraCan = 0;
				if (isUrg1 > 0)
					m_urgControl.scip_uart_disconnect(USB_URG_NUM_G1);
				isUrg1 = 0;
				if (isUrg2 > 0)
					m_urgControl.scip_uart_disconnect(USB_URG_NUM_G2);
				isUrg2 = 0;


				KillTimer(1);
				m_bSensor = false;
				SetDlgItemText(IDC_TOOL_GETDATA, "������");
			}
		}
		else
		{
			AfxMessageBox("��ǰ�����г��������ɿ�״̬");
		}
	}
	else if (ROBOT_MODE_CR_W == Robot::robotMode) // ���ڲ���
	{
		if (false == m_bSensor)
		{
			SetTimer(1, 500, NULL);
			m_bSensor = true;
			SetDlgItemText(IDC_TOOL_GETDATA, "ֹͣ������");
		}
		else
		{			
			KillTimer(1);
			m_bSensor = false;
			SetDlgItemText(IDC_TOOL_GETDATA, "������");
		}
	}	

}

void CToolDlg::Get_IncrePos(double* pos)
{
	int i;
	double len[3];
	Vector vec(0,0,0); // ԭ��
	double trans[3];
	double angle[3];	

	int toolnum = (((CButton*)GetDlgItem(IDC_RADIO1))->GetCheck()) ? 1 : 2;

	for (i=0; i<6; i++)
	{
		pos[i] = 0;
	}

	if ( (m_nCommData[0] < RT_LITTLE) &&
		(m_nCommData[1] < RT_LITTLE) &&
		(m_nCommData[2] < RT_LITTLE) )

	{
		return;
	}

	len[0] = m_nCommData[0];
	len[1] = m_nCommData[1];
	len[2] = m_nCommData[2];

	// z��
	Robot::I_Pump.Get_Trans_Length(len, vec, toolnum, trans);
	Robot::I_Pump.Get_Rot_Angle(len, angle);

	pos[0] = trans[0];
	pos[1] = trans[1];
	pos[2] = trans[2];
	pos[3] = angle[0] * PI_DEG; // wz
	pos[4] = angle[1] * PI_DEG; // wy
	pos[5] = angle[2] * PI_DEG; // wx
}

void CToolDlg::OnToolMove() 
{
	// TODO: Add your control notification handler code here
	CString strtext;
	int i;
	double dPos1[MAX_AXIS_NUM];
	double dPos[MAX_AXIS_NUM];
	double CPos[6];
	double increpos[6] = {0};
	double currpos[6];

	Robot::I_Monit.Get_JointPos(dPos1);
	Robot::I_Monit.Get_CPos(currpos);


	// ��ȡ�����ƶ���Ϣ
	if (ROBOT_MODE_CR_P == Robot::robotMode)
	{
		// ������Ч
		if(1 == m_nCommData[0])
		{
			increpos[1] = m_nCommData[1]; // y
			increpos[2] = m_nCommData[2]; // z
		}
	}
	else if (ROBOT_MODE_CR_W == Robot::robotMode)
	{
		if ((m_nCommData[0] < RT_LITTLE) &&
			(m_nCommData[1] < RT_LITTLE) &&
			(m_nCommData[2] < RT_LITTLE) )

		{
			return;
		}	

		// ��ȡ����λ��
		Get_IncrePos(increpos);

		// ���������� �������㣬ֻ��һ��
		for (i=0; i<3; i++)
		{
			//			m_nCommData[i] = 0;
		}

		// ��������ϵ �� ����������ϵ�Ķ�Ӧ��ϵ
		increpos[4]  = increpos[5]; // wy �ظ�������ת
		increpos[5]  = 0;
		increpos[3]  = 0; // wz��������

		// ��С��һ���Ƕ�ʱ������������ (�ų�����Ӱ��)
		if (fabs(increpos[4]) < 0.6) 
		{
			increpos[4] = 0;

			// λ��С��һ������ʱ��ÿ��z�����߽�2mm
			if (increpos[2] < 2.5)
			{
				increpos[0] = 0;
				increpos[1] = 0;
				increpos[2] = 2;
			}
			else
			{
				increpos[2] -= 2;
			}	

			// te
			increpos[0] = 0;
			increpos[1] = 0;
			increpos[2] = 0;
		}
		else
		{
			// ����̬����ʱ���ȵ�����̬����������Ϊ��
			increpos[0] = 0;
			increpos[1] = 0;
			increpos[2] = 0;
		}
	}

	// ����Ŀ��λ��
	Robot_IncreTransTool(currpos, increpos, CPos);

	if((m_bTool1) && (!m_bTool2))
	{
		if(0 != Robot::I_Task.m_iKine_CR_G1.IKine(CPos, dPos1, dPos))
		{
			AfxMessageBox("����λ�λ���");
			return;
		}
	}
	else if ((!m_bTool1) && (m_bTool2))
	{
		if(0 != Robot::I_Task.m_iKine_CR_G2.IKine(CPos, dPos1, dPos))
		{
			AfxMessageBox("����λ�λ���");
			return;
		}
	}
	else
	{
		return;
	}

	// �˶�
	int vel = (int)(m_sliderVel.GetPos());
	CMotionData data;
	data.Line = 1;
	data.Mode = MODE_INTERP;
	data.Interp.Mode = INTERP_JOINT; //PTP
	data.Interp.IfLinkage = 0;
	for(i=0; i<JOINT_NUM; i++)
		data.Interp.JEnd[i] = dPos[i]; //deg
	data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
	data.Interp.Acc = MAX_AUTOACC_JOINT;
	data.Interp.Jerk = MAX_JERK_JOINT;

	Robot::I_Task.AddMotionData(data);	
	Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

}

/*
void CToolDlg::OnToolGetdata2() 
{
// TODO: Add your control notification handler code here


}

void CToolDlg::OnToolGetdata3() 
{
// TODO: Add your control notification handler code here
CString str;
double increpos[6] = {0};

// ��ȡ���ɽ�
if ( (m_nCommData[0] < RT_LITTLE) &&
(m_nCommData[1] < RT_LITTLE) &&
(m_nCommData[2] < RT_LITTLE) )

{
return;
}

increpos[0] = atan2(m_nCommData[1] - m_nCommData[2], SENSOR_LENGTH);
increpos[0] *= PI_DEG;

str.Format("%0.3f", increpos[0]);
SetDlgItemText(IDC_INCREM_POS1, str);	

str.Format("%0.3f", increpos[1]);
SetDlgItemText(IDC_INCREM_POS2, str);	

str.Format("%0.3f", increpos[2]);
SetDlgItemText(IDC_INCREM_POS3, str);	

str.Format("%0.3f", increpos[3]);
SetDlgItemText(IDC_INCREM_POS4, str);	

str.Format("%0.3f", increpos[4]);
SetDlgItemText(IDC_INCREM_POS5, str);	

str.Format("%0.3f", increpos[5]);
SetDlgItemText(IDC_INCREM_POS6, str);
}
*/

void CToolDlg::OnToolMove2() 
{
	// TODO: Add your control notification handler code here
	CString strtext;
	int i;

	double ld_currPos[MAX_AXIS_NUM];
	double dPos[MAX_AXIS_NUM];
	double increpos[6] = {0};
	double currpos[6];
	double CPos[6];

	// ��ǰ�ؽڽǡ�λ��
	Robot::I_Monit.Get_JointPos(ld_currPos);
	Robot::I_Monit.Get_CPos(currpos);

	// ��ȡĿ��λ��
	if (ROBOT_MODE_CR_P == Robot::robotMode)
	{
		double ld_angle = m_dCommAngle;
		int li_count = 0;

		ReadData_CP();

		while(fabs(ld_angle - m_dCommAngle) > 8)
		{
			Sleep(20);
			li_count++;
			if (li_count > 5)
			{
				AfxMessageBox("Err");
				return;
			}

			ld_angle = m_dCommAngle;
			ReadData_CP();
		}


		// ����̬wz
		increpos[0] = 0;
		increpos[1] = 0;
		increpos[2] = 0;
		increpos[3] = 0;
		increpos[4] = (fabs(m_dCommAngle) < 1.0) ? 0 : m_dCommAngle;
		increpos[5] = 0;

		// ����Ŀ��λ��
		Robot_IncreTransTool(currpos, increpos, CPos);

		if(Robot::actualTool == GRIP_FIRST)
		{
			if(0 != Robot::I_Task.m_iKine_CR_G1.IKine(CPos, ld_currPos, dPos))
			{
				AfxMessageBox("����λ�λ���");
				return;
			}
		}
		else if (Robot::actualTool == GRIP_SECOND)
		{
			if(0 != Robot::I_Task.m_iKine_CR_G2.IKine(CPos, ld_currPos, dPos))
			{
				AfxMessageBox("����λ�λ���");
				return;
			}
		}
		else
		{
			return;
		}

		// �˶�����
		int vel = (int)(m_sliderVel.GetPos());
		CMotionData data;
		data.Line = 1;
		data.Mode = MODE_INTERP;
		data.Interp.Mode = INTERP_JOINT; //PTP		data.Mode = MODE_INTERP;
		data.Interp.Mode = INTERP_JOINT; //PTP
		data.Interp.IfLinkage = 0;
		data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
		data.Interp.Acc = MAX_AUTOACC_JOINT;
		data.Interp.Jerk = MAX_JERK_JOINT;
		for(i=0; i<JOINT_NUM; i++)
			data.Interp.JEnd[i] = dPos[i]; //deg		
		Robot::I_Task.AddMotionData(data);
		Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ
	}
	else if (ROBOT_MODE_CR_W == Robot::robotMode)
	{	
		for(i=0; i<JOINT_NUM; i++)
			dPos[i] = ld_currPos[i];

		// ��ȡ���ɽ�
		if ( (m_nCommData[0] < RT_LITTLE) &&
			(m_nCommData[1] < RT_LITTLE) &&
			(m_nCommData[2] < RT_LITTLE) )

		{
			return;
		}

		increpos[0] = atan2((float)(m_nCommData[1] - m_nCommData[2]), (float)SENSOR_LENGTH);
		increpos[0] *= PI_DEG;


		// ���������� �������㣬ֻ��һ��
		for (i=0; i<3; i++)
		{
			m_nCommData[i] = 0;
		}

		// ��һ�����̶̹���ת��1��
		if((m_bTool1) && (!m_bTool2))
		{
			dPos[0] += increpos[0];
		}
		else if ((!m_bTool1) && (m_bTool2))
		{
			// �ڶ������̶̹���ת��2��
			dPos[4] += increpos[0];
		}
		else
		{
			return;
		}

		// �˶�����
		int vel = (int)(m_sliderVel.GetPos());
		CMotionData data;		
		data.Line = 1;
		data.Mode = MODE_INTERP;
		data.Interp.Mode = INTERP_JOINT; //PTP
		data.Interp.IfLinkage = 0;
		for(i=0; i<JOINT_NUM; i++)
			data.Interp.JEnd[i] = dPos[i]; //deg
		data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
		data.Interp.Acc = MAX_AUTOACC_JOINT;
		data.Interp.Jerk = MAX_JERK_JOINT;

		Robot::I_Task.AddMotionData(data);	
		Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ
	}			
}


void CToolDlg::OnToolMove3() 
{
	// TODO: Add your control notification handler code here
	if (ROBOT_MODE_CR_P == Robot::robotMode)
	{
		// ����̬wz
		//	CameraFunction();

		if (fabs(m_dwz)>45)
		{
			AfxMessageBox("Err");
			return;
		}

		if (fabs(m_dwz)<5)
		{
			return;
		}

		int i;
		double ld_currPos[MAX_AXIS_NUM];
		double dPos[MAX_AXIS_NUM];
		double increpos[6] = {0};
		double currpos[6];
		double CPos[6];

		// ��ǰ�ؽڽǡ�λ��
		Robot::I_Monit.Get_JointPos(ld_currPos);
		Robot::I_Monit.Get_CPos(currpos);


		increpos[0] =  SCIP_CENTER_LEN*(1-cos(m_dwz*PI_RAD));
		increpos[1] = -SCIP_CENTER_LEN*sin(m_dwz*PI_RAD);
		increpos[2] = 0;
		increpos[3] = 0;
		increpos[4] = 0;
		increpos[5] = 0;

		// ����Ŀ��λ��
		Robot_IncreTransTool(currpos, increpos, CPos);

		if(Robot::actualTool == GRIP_FIRST)
		{
			if(0 != Robot::I_Task.m_iKine_CR_G1.IKine(CPos, ld_currPos, dPos))
			{
				AfxMessageBox("����λ�λ���");
				return;
			}
			dPos[4] += (-m_dwz);
		}
		else if (Robot::actualTool == GRIP_SECOND)
		{
			if(0 != Robot::I_Task.m_iKine_CR_G2.IKine(CPos, ld_currPos, dPos))
			{
				AfxMessageBox("����λ�λ���");
				return;
			}
			dPos[0] += (-m_dwz);
		}
		else
		{
			return;
		}		

		// �˶�����
		int vel = (int)(m_sliderVel.GetPos());
		CMotionData data;
		data.Line = 2;
		data.Mode = MODE_INTERP;
		data.Interp.Mode = INTERP_JOINT; //PTP
		data.Interp.IfLinkage = 0;
		data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
		data.Interp.Acc = MAX_AUTOACC_JOINT;
		data.Interp.Jerk = MAX_JERK_JOINT;
		for(i=0; i<JOINT_NUM; i++)
			data.Interp.JEnd[i] = dPos[i]; //deg		
		Robot::I_Task.AddMotionData(data);
		Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

		/*
		int ln_teachdir = (((CButton*)GetDlgItem(IDC_RADIO1))->GetCheck()) ? 1 : -1;

		// ʾ��ģʽ���˶�
		if ((false == m_bRepeat) && (1 == m_nCommData[3]))
		{
		m_bRepeat = true;


		if (RUNMODE_TEACH == Robot::runMode)   // ʾ��ģʽ
		{
		// �����˶�
		int ln_vel[6] = {0};
		int ln_dir[6] = {0};
		ln_vel[5] = (int)(20);
		ln_vel[1] = (int)((double)ln_vel[5]*MAX_TEACHVEL_CW*PI_RAD*SCIP_CENTER_LEN/MAX_TEACHVEL_CV);					
		ln_dir[5] = ln_teachdir;
		ln_dir[1] = -ln_teachdir;

		// ����ʾ������
		Robot::I_Task.Teach(COORDINATE_WORLD, ln_vel, ln_dir);
		}

		// ��ʼ������, ������ʱ��
		m_nTrend = 0;
		m_bInverseDir = false;
		SetTimer(2, TIMER2_COUNT, NULL);

		SetDlgItemText(IDC_TOOL_MOVE3, "ֹͣ");
		}
		else
		{
		KillTimer(2);
		m_bRepeat = false;

		if (RUNMODE_TEACH == Robot::runMode)   // ʾ��ģʽ
		{
		// ����ʾ������
		Robot::I_Task.Stop_Teach();
		}

		SetDlgItemText(IDC_TOOL_MOVE3, "Setp2");
		}
		*/

	}
	else if (ROBOT_MODE_CR_W == Robot::robotMode)
	{
		m_File.Init_File(6);
		m_bWriteFile = true;
	}
}

void CToolDlg::OnToolMove4() 
{
	// TODO: Add your control notification handler code here
	CString strtext;
	int i;

	double ld_currPos[MAX_AXIS_NUM];
	double dPos[MAX_AXIS_NUM];
	double increpos[6] = {0};
	double currpos[6];
	double CPos[6];

	// ��ǰ�ؽڽǡ�λ��
	Robot::I_Monit.Get_JointPos(ld_currPos);
	Robot::I_Monit.Get_CPos(currpos);


	// ��ȡĿ��λ��
	if (ROBOT_MODE_CR_P == Robot::robotMode)
	{
		// ������, ��һ������̬wy, �ڶ�����λ��z
		//------------------------second step------------------------//
		increpos[0] = 0;
		increpos[1] = 0;
		increpos[2] = m_nCommData[6];
		increpos[3] = 0;
		increpos[4] = 0;
		increpos[5] = 0;

		// ����Ŀ��λ��
		Robot_IncreTransTool(currpos, increpos, CPos);

		if(Robot::actualTool == GRIP_FIRST)
		{
			if(0 != Robot::I_Task.m_iKine_CR_G1.IKine(CPos, ld_currPos, dPos))
			{
				AfxMessageBox("����λ�λ���");
				return;
			}
		}
		else if (Robot::actualTool == GRIP_SECOND)
		{
			if(0 != Robot::I_Task.m_iKine_CR_G2.IKine(CPos, ld_currPos, dPos))
			{
				AfxMessageBox("����λ�λ���");
				return;
			}
		}
		else
		{
			return;
		}

		// �˶�����
		int vel = (int)(m_sliderVel.GetPos());
		CMotionData data;
		data.Line = 2;
		data.Mode = MODE_INTERP;
		data.Interp.Mode = INTERP_JOINT; //PTP
		data.Interp.IfLinkage = 0;
		data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
		data.Interp.Acc = MAX_AUTOACC_JOINT;
		data.Interp.Jerk = MAX_JERK_JOINT;
		for(i=0; i<JOINT_NUM; i++)
			data.Interp.JEnd[i] = dPos[i]; //deg		
		Robot::I_Task.AddMotionData(data);
		Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

	}
	else if (ROBOT_MODE_CR_W == Robot::robotMode)
	{
		m_bWriteFile = false;		
		m_File.Close_File();
	}
}


void CToolDlg::MotionFunc()
{

	CWnd* pWnd = FindWindow(NULL, "����");
	if (!pWnd) 
	{
		AfxMessageBox("Find ToolDlg Err!");
		return;
	}


	//	for (int i=0; i<2; i++)
	while(fabs(gd_URG_Len_y) > 5)
	{
		// Step1
		::SendMessage(pWnd->GetSafeHwnd(), MY_MESSAGE_TOOLRUN, (WPARAM)1, (LPARAM)0);
		Sleep(500);
		while(1)
		{
			if (FLAG_RUNNING != Robot::I_Task.Get_RunState())
			{
				break;
			}
			Sleep(20);
		}
	}

	gb_URG_Enable = false;

	// Step2
	::SendMessage(pWnd->GetSafeHwnd(), MY_MESSAGE_TOOLRUN, (WPARAM)2, (LPARAM)0);
	while(1)
	{
		if (FLAG_RUNNING != Robot::I_Task.Get_RunState())
		{
			break;
		}
		Sleep(20);
	}
	/*
	gb_endofstep2 = false;
	::SendMessage(pWnd->GetSafeHwnd(), MY_MESSAGE_TOOLRUN, (WPARAM)2, (LPARAM)0);
	while(!gb_endofstep2)
	{
	Sleep(200);
	}
	Sleep(500);
	while(1)
	{
	if (FLAG_RUNNING != Robot::I_Task.Get_RunState())
	{
	break;
	}
	Sleep(20);
	}
	*/


	// Step3
	::SendMessage(pWnd->GetSafeHwnd(), MY_MESSAGE_TOOLRUN, (WPARAM)3, (LPARAM)0);
	Sleep(500);
	while(1)
	{
		if (FLAG_RUNNING != Robot::I_Task.Get_RunState())
		{
			break;
		}
		Sleep(20);
	}

	// Step4
	::SendMessage(pWnd->GetSafeHwnd(), MY_MESSAGE_TOOLRUN, (WPARAM)4, (LPARAM)0);
	Sleep(500);
	while(1)
	{
		if (FLAG_RUNNING != Robot::I_Task.Get_RunState())
		{
			break;
		}
		Sleep(20);
	}

	m_bRunFlag = false;
	gb_URG_Enable = true;

}


void CToolDlg::OnButtonXzg() 
{
	// TODO: Add your control notification handler code here
	HANDLE hThread;
	DWORD ThreadID;
	static int count = 0;

	if (ROBOT_MODE_CR_W == Robot::robotMode)
	{
		if(m_bWriteFile)
		{
			if ((abs(m_nCommData[0]) < RT_LITTLE) &&
				(abs(m_nCommData[1]) < RT_LITTLE) &&
				(abs(m_nCommData[2]) < RT_LITTLE) )
			{
			}
			else
			{
				//-------------------- д�ļ� ---------------------//
				double cpos[3];
				double inc[3];

				double dPos[6] = {0};
				double dVel[6] = {0};

				// ����1
				inc[0] = -0.5*SENSOR_RADIUS;// SENSOR_RADIUS;
				inc[1] =   0.5*sqrt((double)3.0)*SENSOR_RADIUS;//0;
				inc[2] =  m_nCommData[2] - SENSOR1_HIGHT;//m_nCommData[0] - SENSOR1_HIGHT;
				if (Ok == Robot::I_Monit.Get_CPos_Inc(inc, cpos))
				{
					dPos[0] = cpos[0];
					dPos[1] = cpos[1];
					dPos[2] = cpos[2];
				}

				// ����2
				inc[0] = SENSOR_RADIUS;// - 0.5*SENSOR_RADIUS;
				inc[1] = 0;// - 0.5*sqrt(3)*SENSOR_RADIUS;
				inc[2] = m_nCommData[0] - SENSOR1_HIGHT;// m_nCommData[1] - SENSOR1_HIGHT;
				if (Ok == Robot::I_Monit.Get_CPos_Inc(inc, cpos))
				{
					dPos[3] = cpos[0];
					dPos[4] = cpos[1];
					dPos[5] = cpos[2];
				}

				// ����3
				inc[0] = - 0.5*SENSOR_RADIUS;// -0.5*SENSOR_RADIUS;
				inc[1] = - 0.5*sqrt((double)3.0)*SENSOR_RADIUS;//  0.5*sqrt(3)*SENSOR_RADIUS;
				inc[2] = m_nCommData[1] - SENSOR1_HIGHT;// m_nCommData[2] - SENSOR1_HIGHT;
				if (Ok == Robot::I_Monit.Get_CPos_Inc(inc, cpos))
				{
					dVel[0] = cpos[0];
					dVel[1] = cpos[1];
					dVel[2] = cpos[2];
				}

				m_File.Write_File(dPos, count++);
				m_File.Write_Vel(dVel);
			}

		}
		else
		{
			count = 0;
		}
	}

	if (ROBOT_MODE_CR_P != Robot::robotMode)
	{
		return;
	}

	if (!m_bRunFlag)
	{
		// �����˶��߳�
		hThread = CreateThread(NULL,
			0,
			(LPTHREAD_START_ROUTINE)MotionFunc,
			NULL,
			0,
			&ThreadID
			);

		if (hThread)
		{
			m_bRunFlag = true;
		}
		else
		{
			//			return false;
		}
	}
}

LRESULT CToolDlg::OnMyMessage(WPARAM iParam1,LPARAM iParam2)
{
	if (1 == iParam1)
	{
		CToolDlg::OnToolMove();
	}
	else if (2 == iParam1)
	{
		CToolDlg::OnToolMove3();
	}
	else if (3 == iParam1)
	{
		CToolDlg::OnToolMove2();
	}
	else if (4 == iParam1)
	{
		CToolDlg::OnToolMove4();
	}
	return 0;  //VC2VS������return 0;
}

// ���ٸ�
void CToolDlg::OnToolTracking() 
{
	// TODO: Add your control notification handler code here

	// ���ٸ�
	//IDC_TOOL_TRACKING
	if (m_nTrack == 0)
	{
		SetTimer(3, TIMER3_COUNT, NULL);
		m_nTrack = 1; // λ�ø���

		SetDlgItemText(IDC_TOOL_TRACKING, "ֹͣ����");
	}
	else// if (m_nTrack == 2)
	{
		m_nTrack = 0;
		KillTimer(3);
		m_nTrack = false;
		SetDlgItemText(IDC_TOOL_TRACKING, "����λ��");

		Robot::I_Task.Stop_Teach();
	}

}

// void CToolDlg::CameraFunction1()
// {
// 
// 	while(m_bSensor)
// 	{
// 		CameraFunction();
// 		Sleep(300);
// 	}
// 
// }


//��֡��ȡ��Ƶ���ҵ���������ֱ�߼����ȡ����һ�飬����ʾ
void CToolDlg::Fun1Proc(void)
{	

	//����IplImageָ��
	IplImage* pFrame = NULL;
	IplImage* GrayImg = NULL;
	IplImage* SmooothImg = NULL;
	IplImage * pCannyImg=NULL;

	//CvMat* GrayMat = NULL;

	CvCapture* pCapture = NULL;

	//�߶β���
	CvSeq* lines = 0;
	CvMemStorage* storage = cvCreateMemStorage(0);
	double length=0.0;
	double maxlength=0.0;


	int nFrmNum = 0;
	int edgeThresh = 25;//10,15��25Ч������

	double tx=0.0;double tmax=0.0;
	double ty=0.0;double tmay=0.0;
	int nu=-1;

	//��������
	cvNamedWindow("����", 0);
	cvNamedWindow("����", 0);

	cvResizeWindow("����",360,240);
	cvResizeWindow("����",360,240);

	cvMoveWindow("����", 1000, 100);
	cvMoveWindow("����", 1000, 380);

	//������ͷ
	// 	if( !(pCapture = cvCaptureFromCAM(-1)))
	// 	{
	// 		fprintf(stderr, "Can not open camera.\n");
	// 	}
	// 
	if( !(cvCaptureFromCAM(0)))
	{
		if( !(pCapture = cvCaptureFromCAM(-1)))
		{
			AfxMessageBox("Can not open camera.");
		}		
	}

	//��֡��ȡ��Ƶ
	while(pFrame = cvQueryFrame( pCapture ))
	{
		nFrmNum++;

		//����ǵ�һ֡����Ҫ�����ڴ棬����ʼ��
		if(nFrmNum == 1)
		{
			GrayImg = cvCreateImage(cvSize(pFrame->width, pFrame->height),  IPL_DEPTH_8U,1);
			SmooothImg = cvCreateImage(cvSize(pFrame->width, pFrame->height),  IPL_DEPTH_8U,1);
			pCannyImg = cvCreateImage(cvSize(pFrame->width, pFrame->height),  IPL_DEPTH_8U,1);
		}

		/* Converts input array pixels from one color space to another */
		cvCvtColor(pFrame, GrayImg, CV_BGR2GRAY);
		//cvConvert(GrayImg, GrayMat);


		//�˲����룬��ƽ��ͼ��
		cvSmooth(GrayImg,SmooothImg,CV_GAUSSIAN,3,3,0,0);//3x3

		//canny  /* Runs canny edge detector */
		cvCanny(SmooothImg,pCannyImg,edgeThresh, edgeThresh*3, 3);
		//cvCanny(SmooothImg,pCannyImg,200, 190, 3);

		////---------hough
		lines = cvHoughLines2( pCannyImg, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 50, 50, 10 );
		//lines = cvHoughLines2( pCannyImg, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 80, 80, 30 );//Ч����

		int angletemp;
		int lineMUN[10000];
		int linenum = 0;
		//���ƽǶ���-45�ȵ�45��֮�䣬��ֱ�ߺŴ���lineMUN[]������
		for(int k=0; k<(lines->total); k++)
		{

			if (lines->total!=0  && lines!=NULL)
			{
				CvPoint* line = (CvPoint*)cvGetSeqElem(lines,k);//ȡ��
				tx=line[0].x-line[1].x;
				ty=line[0].y-line[1].y;

				if(ty<0.000001 && ty>=0) ty=0.000001;
				if(ty>-0.000001 && ty<0) ty=-0.000001;
				angletemp = atan(tx/ty)*180/3.1415926; //�Ƕ�ֵ��˼���б״̬��0���˼���ֱ����90���˼�ˮƽ����>0���˼�����б����<0���˼����㣩��
				if(angletemp>-25 && angletemp<25)
				{
					lineMUN[linenum] = k;
					linenum++;
				}
			}	
		}

		//---��������ֱ�߼����ȡ����һ��
		if(linenum>1)
		{
			int linemun_xmin = 0;
			int line_xmin = 10000;
			int line_xmin_temp = 0;
			int cx1, cx2;
			int linexmun[1000] = {0};//��x�����С����������ֱ�ߺ� 
			int linemun_xdmax = 0;
			int cx_dmax = 0;		
			//��x��С����ֱ�ߺţ������linexmun[]��
			int j;
			for(j=0; j<linenum; j++)
			{
				line_xmin = 10000;
				for(int k=0; k<linenum; k++)
				{				
					CvPoint* line1 = (CvPoint*)cvGetSeqElem(lines,lineMUN[k]);	
					tx=line1[0].x-line1[1].x;
					ty=line1[0].y-line1[1].y;
					if(ty<0.000001 && ty>=0) ty=0.000001;
					if(ty>-0.000001 && ty<0) ty=-0.000001;
					cx1 = 240*(tx/ty)+line1[0].x-(tx/ty)*line1[0].y;//ȡYΪ240ʱ��ֱ�߹�Y�ύ��X��ֵ
					if(cx1 < line_xmin && cx1 > line_xmin_temp)
					{
						linemun_xmin = lineMUN[k];
						line_xmin = cx1;
					}
				}
				line_xmin_temp = line_xmin;
				linexmun[j] = linemun_xmin;
			}

			linemun_xdmax = 0;
			cx_dmax = 0;
			//�ҵ�x�������һ��ֱ��
			for(j=0; j<(linenum-1); j++)
			{
				CvPoint* line1 = (CvPoint*)cvGetSeqElem(lines,linexmun[j]);
				CvPoint* line2 = (CvPoint*)cvGetSeqElem(lines,linexmun[j+1]);	
				tx=line1[0].x-line1[1].x;
				ty=line1[0].y-line1[1].y;
				if(ty<0.000001 && ty>=0) ty=0.000001;
				if(ty>-0.000001 && ty<0) ty=-0.000001;
				cx1 = (240*(tx/ty)+line1[0].x-(tx/ty)*line1[0].y);//ȡYΪ240ʱ��ֱ�߹�Y�ύ��X��ֵ
				tx=line2[0].x-line2[1].x;
				ty=line2[0].y-line2[1].y;
				if(ty<0.000001 && ty>=0) ty=0.000001;
				if(ty>-0.000001 && ty<0) ty=-0.000001;
				cx2 = (240*(tx/ty)+line2[0].x-(tx/ty)*line2[0].y);//ȡYΪ240ʱ��ֱ�߹�Y�ύ��X��ֵ

				if((cx2-cx1)>cx_dmax)
				{
					cx_dmax = (cx2-cx1);
					linemun_xdmax = j;
				}
			}

			//��ʾ���ҵ�ֱ��
			CvPoint* line = (CvPoint*)cvGetSeqElem(lines,linexmun[linemun_xdmax]);
			if (lines->total!=0  && line!=NULL)
			{ 
				cvLine( pFrame, line[0], line[1], CV_RGB(0,255,0), 3, CV_AA, 0 );
				tx=line[0].x-line[1].x;
				ty=line[0].y-line[1].y;
				if(ty<0.000001 && ty>=0) ty=0.000001;
				if(ty>-0.000001 && ty<0) ty=-0.000001;
				m_dwz_bzq=atan(tx/ty)*180/3.1415926;
			}

			line = (CvPoint*)cvGetSeqElem(lines,linexmun[linemun_xdmax+1]);
			if (lines->total!=0  && line!=NULL)
			{ 
				cvLine( pFrame, line[0], line[1], CV_RGB(0,255,0), 3, CV_AA, 0 );
				tx=line[0].x-line[1].x;
				ty=line[0].y-line[1].y;
				if(ty<0.000001 && ty>=0) ty=0.000001;
				if(ty>-0.000001 && ty<0) ty=-0.000001;
				m_dwz_bzq=atan(tx/ty)*180/3.1415926;
			}
		}




		//��ʾͼ��
		cvShowImage("����", pFrame);//IDD_DIALOG_TOOL
		//	cvShowImage(cvGetWindowName(cvGetWindowHandle("����")), pFrame);
		//cvShowImage("Gray", GrayImg);
		//cvShowImage("smooth", SmooothImg);
		cvShowImage("����", pCannyImg);

		//cout<<pa<<endl;

		//����а����¼���������ѭ��
		//�˵ȴ�ҲΪcvShowImage�����ṩʱ�������ʾ
		//�ȴ�ʱ����Ը���CPU�ٶȵ���
		if( cvWaitKey(2) >= 0 )
			break;

	}

	//cvReleaseCapture(&pCapture);

	// //���ٴ���
	cvDestroyWindow("����");
	//cvDestroyWindow("Gray");
	//cvDestroyWindow("smooth");
	cvDestroyWindow("����");
	//
	//
	// //�ͷ�ͼ��;���
	cvReleaseImage(&pFrame);
	//cvReleaseImage(&GrayImg);
	//cvReleaseImage(&SmooothImg);
	cvReleaseImage(&pCannyImg);

	//cvReleaseMat(&GrayMat);

	_endthreadex(0);
	//	return 0;
}

void CToolDlg::CamProcess(void)
{
	//����IplImageָ��
	IplImage* pFrame = NULL;
	IplImage* GrayImg = NULL;
	IplImage* SmooothImg = NULL;
	IplImage * pCannyImg=NULL;

	//CvMat* GrayMat = NULL;

	CvCapture* pCapture = NULL;

	//�߶β���
	CvSeq* lines = 0;
	CvMemStorage* storage = cvCreateMemStorage(0);
	double length=0.0;
	double maxlength=0.0;


	int nFrmNum = 0;
	int edgeThresh = 25;//10,15��25Ч������

	double tx=0.0;double tmax=0.0;
	double ty=0.0;double tmay=0.0;
	int nu=-1;

	//��ȡһ֡
	//	IplImage *pFrame1 = camera.QueryFrame();
	//	pFrame = cvQueryFrame( pCapture )
	pFrame = camera.QueryFrame();
	//��ʾ
	//	cvShowImage("camera", pFrame1);

	nFrmNum++;

	//����ǵ�һ֡����Ҫ�����ڴ棬����ʼ��
	if(nFrmNum == 1)
	{
		GrayImg = cvCreateImage(cvSize(pFrame->width, pFrame->height),  IPL_DEPTH_8U,1);
		SmooothImg = cvCreateImage(cvSize(pFrame->width, pFrame->height),  IPL_DEPTH_8U,1);
		pCannyImg = cvCreateImage(cvSize(pFrame->width, pFrame->height),  IPL_DEPTH_8U,1);	 
	}


	cvCvtColor(pFrame, GrayImg, CV_BGR2GRAY);
	//cvConvert(GrayImg, GrayMat);


	//�˲����룬��ƽ��ͼ��
	cvSmooth(GrayImg,SmooothImg,CV_GAUSSIAN,3,3,0,0);//3x3

	//canny  
	cvCanny(SmooothImg,pCannyImg,edgeThresh, edgeThresh*3, 3);
	//cvCanny(SmooothImg,pCannyImg,200, 190, 3);

	////---------hough
	lines = cvHoughLines2( pCannyImg, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 50, 50, 10 );
	//lines = cvHoughLines2( pCannyImg, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 80, 80, 30 );//Ч����

	int angletemp;
	int lineMUN[10000];
	int linenum = 0;
	//���ƽǶ���-45�ȵ�45��֮�䣬��ֱ�ߺŴ���lineMUN[]������
	for(int k=0; k<(lines->total); k++)
	{

		if (lines->total!=0  && lines!=NULL)
		{
			CvPoint* line = (CvPoint*)cvGetSeqElem(lines,k);//ȡ��
			tx=line[0].x-line[1].x;
			ty=line[0].y-line[1].y;

			if(ty<0.000001 && ty>=0) ty=0.000001;
			if(ty>-0.000001 && ty<0) ty=-0.000001;
			angletemp = atan(tx/ty)*180/3.1415926; //�Ƕ�ֵ��˼���б״̬��0���˼���ֱ����90���˼�ˮƽ����>0���˼�����б����<0���˼����㣩��
			if(angletemp>-25 && angletemp<25)
			{
				lineMUN[linenum] = k;
				linenum++;
			}
		}	
	}

	//---��������ֱ�߼����ȡ����һ��
	if(linenum>1)
	{
		int linemun_xmin = 0;
		int line_xmin = 10000;
		int line_xmin_temp = 0;
		int cx1, cx2;
		int linexmun[1000] = {0};//��x�����С����������ֱ�ߺ� 
		int linemun_xdmax = 0;
		int cx_dmax = 0;		
		//��x��С����ֱ�ߺţ������linexmun[]��
		int j;  
		for(j=0; j<linenum; j++)
		{
			line_xmin = 10000;
			for(int k=0; k<linenum; k++)
			{				
				CvPoint* line1 = (CvPoint*)cvGetSeqElem(lines,lineMUN[k]);	
				tx=line1[0].x-line1[1].x;
				ty=line1[0].y-line1[1].y;
				if(ty<0.000001 && ty>=0) ty=0.000001;
				if(ty>-0.000001 && ty<0) ty=-0.000001;
				cx1 = 240*(tx/ty)+line1[0].x-(tx/ty)*line1[0].y;//ȡYΪ240ʱ��ֱ�߹�Y�ύ��X��ֵ
				if(cx1 < line_xmin && cx1 > line_xmin_temp)
				{
					linemun_xmin = lineMUN[k];
					line_xmin = cx1;
				}
			}
			line_xmin_temp = line_xmin;
			linexmun[j] = linemun_xmin;
		}

		linemun_xdmax = 0;
		cx_dmax = 0;
		//�ҵ�x�������һ��ֱ��
		for(j=0; j<(linenum-1); j++)
		{
			CvPoint* line1 = (CvPoint*)cvGetSeqElem(lines,linexmun[j]);
			CvPoint* line2 = (CvPoint*)cvGetSeqElem(lines,linexmun[j+1]);	
			tx=line1[0].x-line1[1].x;
			ty=line1[0].y-line1[1].y;
			if(ty<0.000001 && ty>=0) ty=0.000001;
			if(ty>-0.000001 && ty<0) ty=-0.000001;
			cx1 = (240*(tx/ty)+line1[0].x-(tx/ty)*line1[0].y);//ȡYΪ240ʱ��ֱ�߹�Y�ύ��X��ֵ
			tx=line2[0].x-line2[1].x;
			ty=line2[0].y-line2[1].y;
			if(ty<0.000001 && ty>=0) ty=0.000001;
			if(ty>-0.000001 && ty<0) ty=-0.000001;
			cx2 = (240*(tx/ty)+line2[0].x-(tx/ty)*line2[0].y);//ȡYΪ240ʱ��ֱ�߹�Y�ύ��X��ֵ

			if((cx2-cx1)>cx_dmax)
			{
				cx_dmax = (cx2-cx1);
				linemun_xdmax = j;
			}
		}

		//��ʾ���ҵ�ֱ��
		CvPoint* line = (CvPoint*)cvGetSeqElem(lines,linexmun[linemun_xdmax]);
		if (lines->total!=0  && line!=NULL)
		{ 
			cvLine( pFrame, line[0], line[1], CV_RGB(0,255,0), 3, CV_AA, 0 );
			tx=line[0].x-line[1].x;
			ty=line[0].y-line[1].y;
			if(ty<0.000001 && ty>=0) ty=0.000001;
			if(ty>-0.000001 && ty<0) ty=-0.000001;
			m_dwz_bzq=atan(tx/ty)*180/3.1415926;
		}

		line = (CvPoint*)cvGetSeqElem(lines,linexmun[linemun_xdmax+1]);
		if (lines->total!=0  && line!=NULL)
		{ 
			cvLine( pFrame, line[0], line[1], CV_RGB(0,255,0), 3, CV_AA, 0 );
			tx=line[0].x-line[1].x;
			ty=line[0].y-line[1].y;
			if(ty<0.000001 && ty>=0) ty=0.000001;
			if(ty>-0.000001 && ty<0) ty=-0.000001;
			m_dwz_bzq=atan(tx/ty)*180/3.1415926;
		}
	}

	//��ʾͼ��
	cvShowImage("camera", pFrame);//IDD_DIALOG_TOOL
	cvShowImage("outline", pCannyImg);

	// 	cvDestroyWindow("����");
	// 	cvDestroyWindow("����");
	// 	//�ͷ�ͼ��;���
	// 	cvReleaseImage(&pFrame);
	// 	cvReleaseImage(&pCannyImg);		  
}

void CToolDlg::OnCamera() 
{
	// TODO: Add your control notification handler code here
	// ����ͷ
	HANDLE hThread1;
	DWORD ThreadID1;
	hThread1 = CreateThread(NULL,
		0,
		(LPTHREAD_START_ROUTINE)Fun1Proc,
		NULL,
		0,
		&ThreadID1
		);
	CloseHandle(hThread1);
}

//////////////////////////////////////////////////////////////////////////
//������ͷ�豸��DevNum��ѡ1��2����Ӧ��ļ���
//////////////////////////////////////////////////////////////////////////
BOOL CToolDlg::CamDevOpen(int DevNum)
{
	int ExDevNum1, ExDevNum2; //���ڸ����ı�isRunCam��ֵ��
	CString strtemp;	// = (CString)(DevNum + 48);
	switch (DevNum)
	{
	case 1:
		ExDevNum1 = 0;
		ExDevNum2 = 1;
		strtemp = '1';
		break;
	case 2:
		ExDevNum1 = 1;
		ExDevNum2 = 0;
		strtemp = '2';
		break;
	}
	if (isRunCam[ExDevNum1] == FALSE)
	{
		if (isRunCam[ExDevNum2] == FALSE)
		{
			//	cvNamedWindow("camera");
			//��������
			cvNamedWindow("camera", 0);
			cvNamedWindow("outline", 0);

			cvResizeWindow("camera",360,240);
			cvResizeWindow("outline",360,240);

			cvMoveWindow("camera", 1000, 100);
			cvMoveWindow("outline", 1000, 380);


			//�򿪵�һ������ͷ
			//if(! camera.OpenCamera(0, true)) //��������ѡ�񴰿�
			if(! camera.OpenCamera(ExDevNum2, false, 320,240)) //����������ѡ�񴰿ڣ��ô����ƶ�ͼ���͸�
			{
				AfxMessageBox("Can not open camera" + strtemp + "!");
			}

			SetTimer(4, 150, NULL);
		}

		isRunCam[ExDevNum1] = TRUE;	

		if (isRunCam[ExDevNum2] == TRUE)
		{
			camera.CloseCamera();
			KillTimer(4);

			//�򿪵�һ������ͷ
			//if(! camera.OpenCamera(0, true)) //��������ѡ�񴰿�
			if(! camera.OpenCamera(ExDevNum2, false, 320,240)) //����������ѡ�񴰿ڣ��ô����ƶ�ͼ���͸�
			{
				AfxMessageBox("Can not open camera" + strtemp + "!");
			}

			SetTimer(4, 150, NULL);
		}

		isRunCam[ExDevNum2] = FALSE;
		return TRUE;
	}	
	else
	{
		isRunCam[ExDevNum2] = FALSE;
		AfxMessageBox("Camera" + strtemp + " is running!");
		return FALSE;
	}
}

BEGIN_EVENTSINK_MAP(CToolDlg, CDialog)
	//{{AFX_EVENTSINK_MAP(CToolDlg)
	ON_EVENT(CToolDlg, IDC_MSCOMM_CAN, 1 /* OnComm */, OnCommMscommCan, VTS_NONE)
	//}}AFX_EVENTSINK_MAP
END_EVENTSINK_MAP()

//�����¼�����
void CToolDlg::OnCommMscommCan() 
{
	// TODO: Add your control notification handler code here
	//�����¼�����	
	COleVariant myVar;
	COleSafeArray safearray_inp;
	LONG len,k;
	BYTE rxdata[2048];								//����BYTE����
	CString strRecv,strtemp;

	/*��������*/
	if(m_ctrlCanCom.GetCommEvent()==2)				//�¼�ֵΪ2��ʾ���ջ����������ַ�
	{
		myVar.Attach(m_ctrlCanCom.GetInput());		//��������
		safearray_inp=myVar;						//VARIANT�ͱ���ת��ΪColeSafeArray�ͱ���
		len=safearray_inp.GetOneDimSize();			//�õ���Ч���ݳ���
		for(k=0;k<len;k++)
			safearray_inp.GetElement(&k,rxdata+k);	//ת��ΪBYTE������
		for(k=0;k<len;k++)							//������ת��ΪCstring�ͱ���
		{
			BYTE bt=*(char*)(rxdata+k);				//�ַ���
			strtemp.Format("%02X",bt);			//���ַ�������ʱ����strtemp���
			strRecv+=strtemp;					//������ձ༭���Ӧ�ַ���    
		}
	}		
	strRecv+="\n";
	isUltraCan = 3;	//�������ͨѶֻ��������ǲ���ͨ��������ж���������Ϊ��ͨѶ����

	if (strRecv.GetLength() >= 40)
	{
		CANString = strRecv.Left(40);
		if (CANString.Left(4) == "AA55")
		{				
			CString strtemp1 = CANString.Mid(10,8);	//���ݵ�ַ�жϹؽں�
			//			CString strtemp2 = CANString.Mid(20,16);	//��ȡ��Ӧ�ؽ���ת��������ĩ��ִ�����ķ���ֵ
			isUltraCan = 2;	//ͨ������ص��ж���������Ϊ���������շ�״̬

			if (strcmp(ENDEFFECTOR1_ID, strtemp1) == 0)
			{
				sscanf(CANString.Mid(20,4), "%x",&ultra_dis1[0]);	//ֱ�Ӵ�4λ16�����ַ�תΪ10������ֵ
				sscanf(CANString.Mid(24,4), "%x",&ultra_dis2[0]);
			}
			else if (strcmp(ENDEFFECTOR2_ID, strtemp1) == 0)
			{
				sscanf(CANString.Mid(20,4), "%x",&ultra_dis1[1]);	//ֱ�Ӵ�4λ16�����ַ�תΪ10������ֵ
				sscanf(CANString.Mid(24,4), "%x",&ultra_dis2[1]);	
			}
		}
		CANString = "";
	}
}

/******************************************************************************
* ������MasterCanSendTran(INT JNum, INT DataCount, CHAR DataArray[8])
* ���ܣ���һ���ַ�����Ϊʮ�����ƴ�ת��Ϊһ���ֽ����飬�ֽڼ���ÿո�ָ���
*		 ����ת������ֽ����鳤�ȣ�ͬʱ�ֽ����鳤���Զ����á�
*
* ���룺CString str - �����ַ���
* �����CByteArray &senddata - ���ʮ�����Ƶ�����
*
* ���أ�int- �ַ����鳤��
******************************************************************************/
int CToolDlg::String2Hex(CString str, CByteArray &senddata)
{
	int hexdata,lowhexdata;
	int hexdatalen=0;
	int len=str.GetLength();
	senddata.SetSize(len/2);
	for(int i=0;i<len;)
	{
		char lstr,hstr=str[i];
		if(hstr==' ')
		{
			i++;
			continue;
		}
		i++;
		if(i>=len)
			break;
		lstr=str[i];
		hexdata=ConvertHexChar(hstr);
		lowhexdata=ConvertHexChar(lstr);
		if((hexdata==16)||(lowhexdata==16))
			break;
		else 
			hexdata=hexdata*16+lowhexdata;
		i++;
		senddata[hexdatalen]=(char)hexdata;
		hexdatalen++;
	}
	senddata.SetSize(hexdatalen);
	return hexdatalen;
}

/******************************************************************************
* ������ConvertHexChar(char ch)
* ���ܣ���ʮ��������ת��char�����ַ���
*
* ���룺char ch - ʮ��������
* �����
*
* ���أ�char - char�����ַ�
******************************************************************************/
char CToolDlg::ConvertHexChar(char ch) 
{
	if((ch>='0') && (ch<='9'))
		return ch-0x30;
	else if((ch>='A') && (ch<='F'))
		return ch-'A'+10;
	else if((ch>='a') && (ch<='f'))
		return ch-'a'+10;
	else return (-1);
}

/******************************************************************************
* ������MasterCanSendTran(INT JNum, INT DataCount, CHAR DataArray[8])
* ���ܣ����ؽڱ��JNum�����ݸ���DataCount��������DataArray[8]������תCAN��Լת
*		 ���ַ���SString
*
* ���룺INT JNum - �ؽںţ�ƥ����Ӧ��CAN ID
*       INT DataCount - ���͵����ݳ���
*       CHAR DataArray[8] - ���͵���������
* �����
*
* ���أ�CString���͵��ַ�����CAN ת ���� Э��
******************************************************************************/
CString CToolDlg::UltraCanSendTran(INT EndID, INT DataCount, CHAR DataArray[8]) 
{
	CString SString;
	CString strDC, strDA, strCheck;
	CString strtemp;
	INT frameCheck = 0, sumCheck = 0;
	strDC.Format("%x", DataCount);
	strDC = "0" + strDC;
	strDA = "";
	int i;
	for (i=0; i<8; i++)
	{
		strtemp.Format("%x",DataArray[i]);
		if (DataArray[i] < 0x10)
		{
			strDA += ("0" + strtemp);
		}	
		else
		{
			strDA += strtemp;
		}
	}

	switch (EndID)
	{
	case 1:
		strtemp = ENDEFFECTOR1_ID;
		break;
	case 2:
		strtemp = ENDEFFECTOR2_ID;
		break;
	default:
		break;
	}
	SString = "AA55010101" + strtemp + strDC + strDA + "00";

	for (i=4; i<38; i=i+2)
	{
		strtemp = SString.Mid(i,2);
		sscanf(strtemp,"%x",&frameCheck);
		sumCheck += frameCheck;
	}
	sumCheck = sumCheck%256;
	strCheck.Format("%x", sumCheck);

	SString += strCheck;	

	return SString;
}

/******************************************************************************
* ������MasterCanSendCommand(int numDev, char command)
* ���ܣ�����ָ��
*
* ���룺int numDev - �ؽںţ�ƥ����Ӧ��CAN ID
*       char command - �·���ָ��
* �����
*
* ���أ�BOOL - TRUE�ɹ�,
******************************************************************************/
BOOL CToolDlg::UltraCanSendCommand(int numDev, char command)
{
	CString Sent_CanCommand;// = "AA 55 01 01 01 07 00 00 00 01 08 00 00 00 00 00 00 00 00 13";
	CByteArray hexdata;
	char comArray[8] = {0};//{command, 0, 0, 0, 0, 0, 0, 0};
	int numDevtmp;
	//	char package_Data[40] = {0};
	if (0 == CANIDEX);
	else if (1 == CANIDEX)
	{
		switch (numDev)
		{
		case 1:
			numDevtmp = 2;
			break;
		case 2:
			numDevtmp = 1;
			break;
		}
		numDev = numDevtmp;
	}

	if (numDev == 1)
	{
		comArray[0] = check1_1;
		comArray[1] = check1_2;
		comArray[2] = check1_3;
		if (command == pabics)
			comArray[4] = pabi1cs;			
	} 
	else if (numDev == 2)
	{
		comArray[0] = check2_1;
		comArray[1] = check2_2;
		comArray[2] = check2_3;
		if (command == pabics)
			comArray[4] = pabi1cs;
	}
	else
	{}

	Sent_CanCommand = UltraCanSendTran(numDev, 5, comArray);
	int len = String2Hex(Sent_CanCommand, hexdata); //�˴����ص�len�������ڼ��㷢���˶��ٸ�ʮ��������			
	m_ctrlCanCom.SetOutput(COleVariant(hexdata)); //����ʮ����������
	//	memcpy(package_Data, Sent_CanCommand, Sent_CanCommand.GetLength());
	//	connect2.send(package_Data, 40);

	return TRUE;	
}


void CToolDlg::OnButtonChangeSensor() 
{
	// TODO: Add your control notification handler code here
	if (m_bTool1 == true && m_bTool2 == false)	//����1�н�������2�ɿ����Ѽ���2�ϵĴ��������
	{
		m_exSensorSet = 2;	
		CamDevOpen(2);
	} 
	else if (m_bTool2 == true && m_bTool1 == false)	//����2�н�������1�ɿ����Ѽ���1�ϵĴ��������
	{
		m_exSensorSet = 1;	
		CamDevOpen(1);
	} 
	else if (m_bTool2 == true && m_bTool1 == true)	//ͬʱ�н������ı�
	{
	} 
	else	//ͬʱ�ɿ���ر����д�����������
	{
		AfxMessageBox("���棺�����г��������ɿ�״̬");
		if (isRunCam[0] == TRUE ||
			isRunCam[1] == TRUE)
		{
			KillTimer(4);
			camera.CloseCamera();
			//	cvDestroyWindow("camera");
			//	cvDestroyWindow("outline");
			isRunCam[0] = FALSE;
			isRunCam[1] = FALSE;
		}
		if (isUltraCan > 0)
		{
			isUltraCan = false;
			m_ctrlCanCom.SetPortOpen(FALSE);
		}
		if (isUrg1 > 0 || isUrg2 > 0)	
		{	
			m_urgControl.scip_uart_disconnect(USB_URG_NUM_G1);
			isUrg1 = 0;
			m_urgControl.scip_uart_disconnect(USB_URG_NUM_G2);
			isUrg2 = 0;
		}		
	}
}

void CToolDlg::OnButtonChangeSensor2() 
{
	// TODO: Add your control notification handler code here
	//	CamDevOpen(2);
}

void CToolDlg::OnButtonCamClose() 
{
	// TODO: Add your control notification handler code here
	KillTimer(4);
	camera.CloseCamera();
	cvDestroyWindow("camera");
	cvDestroyWindow("outline");
	isRunCam[0] = FALSE;
	isRunCam[1] = FALSE;
}

int stepcount = 0;
#define ROLLSTEPS 4
#define TURNSTEPS 4
#define TURNSTEPS2 2
#define INCHSTEPS 1
//��ת��̬
void CToolDlg::OnButtonRoll() 
{
	// TODO: Add your control notification handler code here
	// ����Ŀ��λ��
	//	Robot_IncreTransTool(currpos, increpos, CPos);
	//if ((!m_bTool1) && (m_bTool2))
	//{
	//}
	//else
	//{
	//	return;
	//}
	//if (Robot::I_Task.Get_RunState() == 1)
	//{
	//	return;
	//}

	//double tPos[ROLLSTEPS][MAX_AXIS_NUM] = 
	//{
	//	{0,		22.5,	-45,	22.5,	0,		0},	//��λλ��
	//	{0,		22.5,	-45,	30,		0,		0},	//���߼����뿪�˼�
	//	{0,		22.5,	-90,	30,		90,		0},	//��Բ����ת�е�
	//	{7,		22.5,	-45,	25,		175,	0},	//�������
	//	//	{0,		22.5,	-45,	22.5,	180,	0}	//��ת��̬Ŀ���
	//};

	////	for (int i=0; i<5; i++)
	//if (stepcount < ROLLSTEPS)
	//{

	//	// �˶�
	//	int vel = (int)(m_sliderVel.GetPos());
	//	CMotionData data;
	//	data.Line = 1;
	//	data.Mode = MODE_INTERP;
	//	data.Interp.Mode = INTERP_JOINT; //PTP
	//	data.Interp.IfLinkage = 0;
	//	for (int j=0; j<MAX_AXIS_NUM; j++)
	//		data.Interp.JEnd[j] = tPos[stepcount][j]; //deg
	//	data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
	//	data.Interp.Acc = MAX_AUTOACC_JOINT;
	//	data.Interp.Jerk = MAX_JERK_JOINT;

	//	Robot::I_Task.AddMotionData(data);	
	//	Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

	//	//	while (Robot::I_Task.Get_RunState() == 1);

	//	stepcount++;
	//	if (stepcount == ROLLSTEPS)
	//	{
	//		return;
	//	}
	//}

	CMotionData data;
	m_MotionChoose = 1;
	double s31=180;
	double s35=180; 
	double s41=180; 
	double s45=180;

	((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(false);
	((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(false);
	((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(false);
	((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(false);
	((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(false);

	if (BST_CHECKED == IsDlgButtonChecked(IDC_CHECK_ROLLDIR))
	{
		SetDlgItemText(IDC_CHECK_ROLLDIR, "����");
		s31 = 180;
		s41 = 180;
		s35 = 180;
		s45 = 180;

	} 
	else if(BST_UNCHECKED == IsDlgButtonChecked(IDC_CHECK_ROLLDIR))
	{
		SetDlgItemText(IDC_CHECK_ROLLDIR, "����");
		s31 = -180;
		s41 = -180;
		s35 = -180;
		s45 = -180;
	}

	double tPos2[ROLLSTEPS][MAX_AXIS_NUM] = //G2�н�ʱ��ʾ�̵�
	{
		{0,	     0,	    0,	   0,	    0,	  0},	//��λλ��
		{0,		10,	  -40,	  30,		0,	  0},	//���߼����뿪�˼�
		{0,		10,	  -40,	  30,	  s35,	  0},	//��ת�˶�
		{0,		0,  	0,     0,	  s45,	  0},	//�гָ˼�
	};
	double tPos1[ROLLSTEPS][MAX_AXIS_NUM] = //G1�н�ʱ��ʾ�̵�
	{
		{0,	      0,	 0,	   0,	    0,	  0},	//��λλ��
		{0,		 30,   -40,	  10,		0,	  0},	//���߼����뿪�˼�
		{s31,    30,   -40,	  10,	    0,	  0},	//��ת�˶�
		{s41,	  0,  	0,     0,	    0,	  0},	//�гָ˼�
	};

	if ((!m_bTool1) && (m_bTool2) && FLAG_ERROR != Robot::I_Task.Get_RunState() && TRUE == m_bg1tog2)         //GU160519ʹ�ڲ�ͬ���Ӽн�����¾�����ɲ�̬�˶�
	{
		KillTimer(8);
		if (stepcount <= ROLLSTEPS)
		{
			double JCurrPos[MAX_AXIS_NUM];
			Robot::I_Monit.Get_JointPos(JCurrPos);
			// �˶�
			int vel = (int)(m_sliderVel.GetPos());
			data.Line = 1;
			data.Mode = MODE_INTERP;
			data.Interp.Mode = INTERP_JOINT; //PTP
			data.Interp.IfLinkage = 0;
			for (int j=0; j<MAX_AXIS_NUM; j++)
			{
				data.Interp.JEnd[j] = tPos2[stepcount][j]; //deg
				data.Interp.JStart[j]=JCurrPos[j];
			}
			data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
			data.Interp.Acc = MAX_AUTOACC_JOINT;
			data.Interp.Jerk = MAX_JERK_JOINT;

			Robot::I_Task.AddMotionData(data);	
			Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

			stepcount++;
			if (stepcount == ROLLSTEPS)
			{
				for (int i=0; i<6; i++)
				{
					tPos_temp[i] = tPos2[3][i];

				}
				SetTimer(8,500,NULL);
				m_bg1tog2 = FALSE;
				stepcount = 0;
				//  m_MotionChoose = 0;
				return;
			}
		}
	} 
	else if((m_bTool1) && (!m_bTool2) && FLAG_ERROR != Robot::I_Task.Get_RunState() && FALSE == m_bg1tog2)
	{
		KillTimer(8);
		if (stepcount <= ROLLSTEPS)
		{
			// �˶�
			int vel = (int)(m_sliderVel.GetPos());
			data.Line = 1;
			data.Mode = MODE_INTERP;
			data.Interp.Mode = INTERP_JOINT; //PTP
			data.Interp.IfLinkage = 0;
			for (int j=0; j<MAX_AXIS_NUM; j++)
				data.Interp.JEnd[j] = tPos1[stepcount][j]; //deg
			data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
			data.Interp.Acc = MAX_AUTOACC_JOINT;
			data.Interp.Jerk = MAX_JERK_JOINT;

			Robot::I_Task.AddMotionData(data);	
			Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

			//	while (Robot::I_Task.Get_RunState() == 1);

			stepcount++;
			if (stepcount == ROLLSTEPS)
			{
				for (int i=0; i<6; i++)
				{
					tPos_temp[i] = tPos1[3][i];

				}
				SetTimer(8,500,NULL);
				m_bg1tog2 = TRUE;
				stepcount = 0;
				//m_MotionChoose = 0;
				return;
			}
		}

	}
	else
	{
		((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(true);
		AfxMessageBox("����ִ�иò�̬");
		//m_MotionChoose = 0;
		stepcount = 0;
		return;
	}

}

//��ת��̬
void CToolDlg::OnButtonTurn() 
{
	// TODO: Add your control notification handler code here
	//if((m_bTool1) && (!m_bTool2))
	//{
	//}
	//else
	//{
	//	return;
	//}
	//if (Robot::I_Task.Get_RunState() == 1)
	//{
	//	return;
	//}

	//double tPos[TURNSTEPS][MAX_AXIS_NUM] = 
	//{
	//	{0,		22.5,	-45,	22.5,	180,	0},	//��λλ��
	//	{0,		10,		100,	105,	180,	0},	//ת���ؽ�3��4����������
	//	{0,		70,		90,		105,	180,	0},	//��Բ����ת�е�
	//	{3,		105,	60,		110,	175,	0}	//�������
	//	//	{0,		105,	60,		105,	180,	0}	//��ת��̬Ŀ���	
	//};

	////	for (int i=0; i<4; i++)
	//if (stepcount >= ROLLSTEPS && stepcount < ROLLSTEPS+TURNSTEPS)
	//{
	//	// �˶�
	//	int vel = (int)(m_sliderVel.GetPos());
	//	CMotionData data;
	//	data.Line = 1;
	//	data.Mode = MODE_INTERP;
	//	data.Interp.Mode = INTERP_JOINT; //PTP
	//	data.Interp.IfLinkage = 0;
	//	for (int j=0; j<MAX_AXIS_NUM; j++)
	//		data.Interp.JEnd[j] = tPos[stepcount-ROLLSTEPS][j]; //deg
	//	data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
	//	data.Interp.Acc = MAX_AUTOACC_JOINT;
	//	data.Interp.Jerk = MAX_JERK_JOINT;

	//	Robot::I_Task.AddMotionData(data);	
	//	Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ
	//	stepcount++;
	//	if (stepcount == ROLLSTEPS+TURNSTEPS)
	//	{
	//		return;
	//	}
	//}

	CMotionData data;
	m_MotionChoose = 3;

	((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(false);
	((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(false);
	((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(false);
	((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(false);
	((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(false);
	((CButton*)GetDlgItem(IDC_CHECK_ROLLDIR))->EnableWindow(false);

	double tPos[ROLLSTEPS][MAX_AXIS_NUM] = //G2�н�ʱ��ʾ�̵�
	{
		{0,	     0,	    0,	   0,	    0,	  0},	//��λλ��
		{0,		20,	   -10,	  20,		0,	  0},	//���߼����뿪�˼�
		{0,	   160,	   10,	 160,	    0,	  0},	//��ת�˶�
		{0,	   180,  	0,   180,	    0,	  0},	//�гָ˼�
	};

	if ((!m_bTool1) && (m_bTool2) && FLAG_ERROR != Robot::I_Task.Get_RunState() && TRUE == m_bg1tog2)         //GU160519ʹ�ڲ�ͬ���Ӽн�����¾�����ɲ�̬�˶�
	{
		KillTimer(8);
		if (stepcount <= ROLLSTEPS)
		{
			// 			double JCurrPos[MAX_AXIS_NUM];
			// 			Robot::I_Monit.Get_JointPos(JCurrPos);
			// �˶�
			int vel = (int)(m_sliderVel.GetPos());
			data.Line = 1;
			data.Mode = MODE_INTERP;
			data.Interp.Mode = INTERP_JOINT; //PTP
			data.Interp.IfLinkage = 0;
			for (int j=0; j<MAX_AXIS_NUM; j++)
			{
				data.Interp.JEnd[j] = tPos[stepcount][j]; //deg
				//data.Interp.JStart[j]=JCurrPos[j];
			}
			data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
			data.Interp.Acc = MAX_AUTOACC_JOINT;
			data.Interp.Jerk = MAX_JERK_JOINT;

			Robot::I_Task.AddMotionData(data);	
			Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

			stepcount++;
			if (stepcount == ROLLSTEPS)
			{
				for (int i=0; i<6; i++)
				{
					tPos_temp[i] = tPos[3][i];

				}
				SetTimer(8,500,NULL);
				m_bg1tog2 = FALSE;
				stepcount = 0;
				//	m_MotionChoose = 0;

				//if (m_bTurn==FALSE)
				//{
				//	m_bTurn = TRUE;
				//} 
				//else
				//{
				//	m_bTurn = FALSE;
				//}
				return;
			}
		}
	} 
	else if((m_bTool1) && (!m_bTool2) && FLAG_ERROR != Robot::I_Task.Get_RunState() && FALSE == m_bg1tog2)
	{
		KillTimer(8);
		if (stepcount <= ROLLSTEPS)
		{
			// �˶�
			int vel = (int)(m_sliderVel.GetPos());
			data.Line = 1;
			data.Mode = MODE_INTERP;
			data.Interp.Mode = INTERP_JOINT; //PTP
			data.Interp.IfLinkage = 0;
			for (int j=0; j<MAX_AXIS_NUM; j++)
				data.Interp.JEnd[j] = tPos[stepcount][j]; //deg
			data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
			data.Interp.Acc = MAX_AUTOACC_JOINT;
			data.Interp.Jerk = MAX_JERK_JOINT;

			Robot::I_Task.AddMotionData(data);	
			Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

			//	while (Robot::I_Task.Get_RunState() == 1);

			stepcount++;
			if (stepcount == ROLLSTEPS)
			{
				for (int i=0; i<6; i++)
				{
					tPos_temp[i] = tPos[3][i];

				}
				SetTimer(8,500,NULL);
				m_bg1tog2 = TRUE;
				stepcount = 0;
				//m_MotionChoose = 0;

				//if (m_bTurn==FALSE)
				//{
				//	m_bTurn = TRUE;
				//} 
				//else
				//{
				//	m_bTurn = FALSE;
				//}
				return;
			}
		}

	}
	else
	{
		((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_CHECK_ROLLDIR))->EnableWindow(true);
		AfxMessageBox("����ִ�иò�̬");
		//m_MotionChoose = 0;
		stepcount = 0;
		return;
	}
}

//��ת2
void CToolDlg::OnButtonTurn2() 
{
	// TODO: Add your control notification handler code here
	//if((!m_bTool1) && (m_bTool2))
	//{
	//}
	//else
	//{
	//	return;
	//}
	//if (Robot::I_Task.Get_RunState() == 1)
	//{
	//	return;
	//}

	//double tPos[TURNSTEPS2][MAX_AXIS_NUM] = 
	//{
	//	//	{0,		105,	60,		105,	180,	0}
	//	{4,		50,		-90,	50,		178,	0},
	//	{4,		3,		0,		10,		178,	0}	//��λλ��
	//	//	{0,		0,		0,		0,		180,	0}	
	//};

	////	for (int i=0; i<4; i++)
	//if (stepcount >= ROLLSTEPS+TURNSTEPS && stepcount < ROLLSTEPS+TURNSTEPS+TURNSTEPS2)
	//{
	//	// �˶�
	//	int vel = (int)(m_sliderVel.GetPos());
	//	CMotionData data;
	//	data.Line = 1;
	//	data.Mode = MODE_INTERP;
	//	data.Interp.Mode = INTERP_JOINT; //PTP
	//	data.Interp.IfLinkage = 0;
	//	for (int j=0; j<MAX_AXIS_NUM; j++)
	//		data.Interp.JEnd[j] = tPos[stepcount-ROLLSTEPS-TURNSTEPS][j]; //deg
	//	data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
	//	data.Interp.Acc = MAX_AUTOACC_JOINT;
	//	data.Interp.Jerk = MAX_JERK_JOINT;

	//	Robot::I_Task.AddMotionData(data);	
	//	Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ
	//	stepcount++;
	//	if (stepcount == ROLLSTEPS+TURNSTEPS+TURNSTEPS2)
	//	{
	//		return;
	//	}
	//}

	OnBnClickedButtonBturn();
}

//��󶲽̬
void CToolDlg::OnButtonInchworm() 
{
	// TODO: Add your control notification handler code here
	//if((m_bTool1) && (!m_bTool2))
	//{
	//}
	//else
	//{
	//	return;
	//}
	//if (Robot::I_Task.Get_RunState() == 1)
	//{
	//	return;
	//}

	//double tPos[INCHSTEPS][MAX_AXIS_NUM] = 
	//{
	//	//	{0,		0,		0,		0,		180,	0}
	//	{2,		48,		-90,	47,		175,	0},	//��λλ��
	//	//	{0,		45,		-90,	45,		180,	0}	
	//};

	////	for (int i=0; i<4; i++)
	//if (stepcount >= ROLLSTEPS+TURNSTEPS+TURNSTEPS2 && stepcount < ROLLSTEPS+TURNSTEPS+TURNSTEPS2+INCHSTEPS)
	//{
	//	// �˶�
	//	int vel = (int)(m_sliderVel.GetPos());
	//	CMotionData data;
	//	data.Line = 1;
	//	data.Mode = MODE_INTERP;
	//	data.Interp.Mode = INTERP_JOINT; //PTP
	//	data.Interp.IfLinkage = 0;
	//	for (int j=0; j<MAX_AXIS_NUM; j++)
	//		data.Interp.JEnd[j] = tPos[stepcount-ROLLSTEPS-TURNSTEPS-TURNSTEPS2][j]; //deg
	//	data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
	//	data.Interp.Acc = MAX_AUTOACC_JOINT;
	//	data.Interp.Jerk = MAX_JERK_JOINT;

	//	Robot::I_Task.AddMotionData(data);	
	//	Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ
	//	stepcount++;
	//	if (stepcount == ROLLSTEPS+TURNSTEPS+TURNSTEPS2+INCHSTEPS)
	//	{
	//		return;
	//	}
	//}

		CMotionData data;
	m_MotionChoose = 5;
	KillTimer(8);
	double tPos2[ROLLSTEPS][MAX_AXIS_NUM] = //G2�н�ʱ��ʾ�̵�
	{
		{0,		35,	   -90,	    55,		0,	  0},	
		{0,		55,	  -110,	    55,	    0,	  0},	
		{0,		55,	   -90,	    35,	    0,	  0},	
		{0,		 0,  	 0,      0,	    0,	  0},	
	};
	double tPos1[ROLLSTEPS][MAX_AXIS_NUM] = //G1�н�ʱ��ʾ�̵�
	{
		{0,		55,	   -90,	    35,		0,	  0},	
		{0,		55,	  -110,	    55,	    0,	  0},	
		{0,		35,	   -90,	    55,	    0,	  0},	
		{0,		 0,  	 0,      0,	    0,	  0},
	};

	if ((!m_bTool1) && (m_bTool2) && FLAG_ERROR != Robot::I_Task.Get_RunState() && TRUE == m_bg1tog2 && false==m_bInSecStep)         //GU160519ʹ�ڲ�ͬ���Ӽн�����¾�����ɲ�̬�˶�
	{
		((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(false);
		//((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_CHECK_ROLLDIR))->EnableWindow(false);

		if (stepcount <= ROLLSTEPS)
		{
			// �˶�
			int vel = (int)(m_sliderVel.GetPos());
			data.Line = 1;
			data.Mode = MODE_INTERP;
			data.Interp.Mode = INTERP_JOINT; //PTP
			data.Interp.IfLinkage = 0;
			for (int j=0; j<MAX_AXIS_NUM; j++)
			{
				data.Interp.JEnd[j] = tPos2[stepcount][j]; //deg
			}
			data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
			data.Interp.Acc = MAX_AUTOACC_JOINT;
			data.Interp.Jerk = MAX_JERK_JOINT;

			Robot::I_Task.AddMotionData(data);	
			Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

			stepcount++;
			if (stepcount == 2)
			{
				AfxMessageBox("��պϼг���1���򿪼г���2!");
				//((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(false);
				m_bInSecStep = true;
			}
		}
	} 
	else if ((m_bTool1) && (!m_bTool2) && FLAG_ERROR != Robot::I_Task.Get_RunState() && TRUE == m_bg1tog2 && (stepcount == 2 || stepcount == 3) && true==m_bInSecStep)
	{
		((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(true);

		((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(false);
		//((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_CHECK_ROLLDIR))->EnableWindow(false);

		if (stepcount <= ROLLSTEPS)
		{
			// �˶�
			int vel = (int)(m_sliderVel.GetPos());
			data.Line = 1;
			data.Mode = MODE_INTERP;
			data.Interp.Mode = INTERP_JOINT; //PTP
			data.Interp.IfLinkage = 0;
			for (int j=0; j<MAX_AXIS_NUM; j++)
			{
				data.Interp.JEnd[j] = tPos2[stepcount][j]; //deg
			}
			data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
			data.Interp.Acc = MAX_AUTOACC_JOINT;
			data.Interp.Jerk = MAX_JERK_JOINT;

			Robot::I_Task.AddMotionData(data);	
			Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

			stepcount++;
			if (stepcount == ROLLSTEPS)
			{
				((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_CHECK_ROLLDIR))->EnableWindow(true);
				stepcount = 0;
				//m_MotionChoose = 0;
				m_bInSecStep = false;
				return;
			}
		}
	}
	else if((m_bTool1) && (!m_bTool2) && FLAG_ERROR != Robot::I_Task.Get_RunState() && FALSE == m_bg1tog2 && false==m_bInSecStep)
	{
		((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(false);
		//((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_CHECK_ROLLDIR))->EnableWindow(false);
		if (stepcount <= ROLLSTEPS)
		{
			// �˶�
			int vel = (int)(m_sliderVel.GetPos());
			data.Line = 1;
			data.Mode = MODE_INTERP;
			data.Interp.Mode = INTERP_JOINT; //PTP
			data.Interp.IfLinkage = 0;
			for (int j=0; j<MAX_AXIS_NUM; j++)
				data.Interp.JEnd[j] = tPos1[stepcount][j]; //deg
			data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
			data.Interp.Acc = MAX_AUTOACC_JOINT;
			data.Interp.Jerk = MAX_JERK_JOINT;

			Robot::I_Task.AddMotionData(data);	
			Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

			//	while (Robot::I_Task.Get_RunState() == 1);

			stepcount++;
			if (stepcount == 2)
			{
				AfxMessageBox("��պϼг���2���򿪼г���1!");
				//((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(false);
				m_bInSecStep = true;
			}
		}

	}
	else if ((!m_bTool1) && (m_bTool2) && FLAG_ERROR != Robot::I_Task.Get_RunState() && FALSE == m_bg1tog2 && (stepcount == 2 || stepcount == 3) && true==m_bInSecStep)
	{
		((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(true);

		((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(false);
		//((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_CHECK_ROLLDIR))->EnableWindow(false);

		if (stepcount <= ROLLSTEPS)
		{
			// �˶�
			int vel = (int)(m_sliderVel.GetPos());
			data.Line = 1;
			data.Mode = MODE_INTERP;
			data.Interp.Mode = INTERP_JOINT; //PTP
			data.Interp.IfLinkage = 0;
			for (int j=0; j<MAX_AXIS_NUM; j++)
			{
				data.Interp.JEnd[j] = tPos1[stepcount][j]; //deg
			}
			data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
			data.Interp.Acc = MAX_AUTOACC_JOINT;
			data.Interp.Jerk = MAX_JERK_JOINT;

			Robot::I_Task.AddMotionData(data);	
			Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

			stepcount++;
			if (stepcount == ROLLSTEPS)
			{
				((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_CHECK_ROLLDIR))->EnableWindow(true);
				stepcount = 0;
				//m_MotionChoose = 0;
				m_bInSecStep = false;
				return;
			}
		}
	}
	else
	{
		((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_CHECK_ROLLDIR))->EnableWindow(true);
		AfxMessageBox("����ִ�иò�̬");
		//m_MotionChoose = 0;
		/*	if (false==m_bInSecStep)
		{
		stepcount = 0;
		} 
		else
		{
		stepcount = 2;
		}*/
		m_bInSecStep = false;
		stepcount = 0;
		
		return;
	}
}

DWORD WINAPI FunIntelligentGraspProc(LPVOID lpParameter)  // thread data
{
	CToolDlg* pThis = (CToolDlg*)lpParameter;  //�����߳�ָ��
	//�˼����
	//PoleNum=1;

	double currpos[6],TagGripP[3], TagGripDir[3], t; //t��ʾ�˼��ϵĵ�λ��
	double n[3], o[3], a[3], m[3];    //������ӵ�ǰ��̬
	Robot::I_Monit.Get_CPos(currpos);//���ܣ���ȡ�����ռ�λ�ˣ������double pos[]�����ռ�λ��, λ��mm, ��̬deg**current��������ϵ��λ�����꣨mm����RPY�ǣ�deg��ŷ����
	// 	if (PoleNum<1)                      692.9--0--0--0--0--180
	// 	{
	// 		return 0;  //���û�м�⵽�˼�,���˳�
	// 	}
	/////////////////////////////////////////////
	float Tempn,NearestPole=2000;
	int NearestPoleN = 0;
	float CurrPos[3];
	for (int i=0;i<3;i++)
	{
		CurrPos[i]=currpos[i];
	}

	//ȷ����������ĸ˼�
	for (int i=0;i<PoleNum;i++)
	{
		Tempn = PtoLineDis(CurrPos,PoleCentralLine[i][0],PoleCentralLine[i][1]);// �㵽ֱ�ߵľ��뺯��
		if (Tempn<NearestPole)
		{
			NearestPole = Tempn;
			NearestPoleN = i;
		}
	}


	// 	NearestPoleN =0;
	// 	PoleStyle[0]=Square;
	MtxKine TagPosM; //Ŀ��гֵ���̬����
	double tagpos[6]={0};   // Ԥ Ŀ��гֵ�λ�ˣ�GU��
	double tagposend[6]={0};   //Ŀ��гֵ�λ�ˣ�GU��
	double tagajpos[6]={0};  //�������λ��
	double tag1pos[6]={0};   //Ŀ��гֵ�λ��
	double tag2pos[6]={0};   //Ŀ��гֵ�λ��
	double prepos[6] = {0};   //Ԥ�гֵ�λ��
	double prepostemp[6] = {0};  //ѡ��Ԥ�гֵ�
	double increpos[6]={0}, increpostemp[6]={0},dPos1[6] = {0}, dPos[6]={0}, FKPos[6]={0};//�����������ʼ��Ϊ0
	//double  dposend[6],dposendtemp[6];
	RealtimeCon = 1;
	CToolDlg RedetPole;
	/*double dposenddifval[6] = {0};*/
	//int RedetSucCount = 0;//���¼������һ�μ����ͬ�ĳɹ�������>=3ʱ��������ץ��

	//�г�Բ��
	if (PoleStyle[NearestPoleN]==Round)
	{
		/*while(RealtimeCon)
		{*/
			//���Ŀ��гֵ㣨Tcp--���㣩
			t = ( PoleCentralLine[NearestPoleN][0][0]*(currpos[0]-PoleCentralLine[NearestPoleN][1][0])
				+PoleCentralLine[NearestPoleN][0][1]*(currpos[1]-PoleCentralLine[NearestPoleN][1][1])
				+PoleCentralLine[NearestPoleN][0][2]*(currpos[2]-PoleCentralLine[NearestPoleN][1][2]) )
				/(PoleCentralLine[NearestPoleN][0][0]*PoleCentralLine[NearestPoleN][0][0]
			+PoleCentralLine[NearestPoleN][0][1]*PoleCentralLine[NearestPoleN][0][1]
			+PoleCentralLine[NearestPoleN][0][2]*PoleCentralLine[NearestPoleN][0][2]);

			for (int i=0;i<3;i++)
			{
				TagGripP[i] = PoleCentralLine[NearestPoleN][1][i] + t*PoleCentralLine[NearestPoleN][0][i]; //���㣬�����
				TagGripDir[i] = PoleCentralLine[NearestPoleN][0][i];
			}

			//for (int i=0; i<PoleNum; i++)//GU
			//{
			//	TagGripP[0] = PoleCentralLine[NearestPoleN][1][0] - PoleLen[i]/4-120;
			//}
			
			//TagGripP[0] = 690;//GU��֤����
			/////////////////////////////////////////////////////////
			//���Ŀ��гֵ㳬����⵽�ĸ˼���Χ����ȥץ��
			if (sqrt(pow(TagGripP[0]-PoleCentralLine[NearestPoleN][1][0],2)+pow(TagGripP[1]-PoleCentralLine[NearestPoleN][1][1],2)
				+pow(TagGripP[2]-PoleCentralLine[NearestPoleN][1][2],2))>(PoleLen[NearestPoleN]*1.2))
			{
				AfxMessageBox("�˼�ץ�е㳬��");
				return 0;
			}
			//���ӵ�n������˼�������������
			for (int i=0;i<3;i++)
			{
				n[i] = TagGripDir[i];
			}
			VectorUnitizationD(n);
			//����5DOF�����˺�Ŀ��㣬�������a��o; ���� m = [-TagGripP[1],TagGripP[0],0]
			a[0] = TagGripP[0]*n[2];
			a[1] = TagGripP[1]*n[2];
			a[2] = -TagGripP[0]*n[0]-TagGripP[1]*n[1];
			VectorUnitizationD(a);
			//ȷ��n��a�ķ���
			MtxKine CoCM;   //��ǰ������λ���µ�����任����
			Trans_PosToMtx(currpos, &CoCM, 0);
			if ((n[0]*CoCM.R11+n[1]*CoCM.R21+n[2]*CoCM.R31)<0) //��������˵��n����ӵ�n�нǴ���90��
			{
				for (int i=0;i<3;i++)
				{
					n[i] = -n[i];
				}
			}
			if ((a[0]*CoCM.R13+a[1]*CoCM.R23+a[2]*CoCM.R33)<0) //��������˵��a����ӵ�a�нǴ���90��
			{
				for (int i=0;i<3;i++)
				{
					a[i] = -a[i];
				}
			}

			o[0] = a[1]*n[2]-a[2]*n[1];
			o[1] = n[0]*a[2]-n[2]*a[0];
			o[2] = a[0]*n[1]-a[1]*n[0];
			VectorUnitizationD(o);
			if ((o[0]*CoCM.R12+o[1]*CoCM.R22+o[2]*CoCM.R32)<0) //��������˵��o����ӵ�o�нǴ���90��
			{
				for (int i=0;i<3;i++)
				{
					o[i] = -o[i];
				}
			}
			////////////////////////////////////////////////////
			//�г��˶�

			TagPosM.R11 = n[0];
			TagPosM.R12 = o[0];
			TagPosM.R13 = a[0];
			TagPosM.R21 = n[1];
			TagPosM.R22 = o[1];
			TagPosM.R23 = a[1];
			TagPosM.R31 = n[2];
			TagPosM.R32 = o[2];
			TagPosM.R33 = a[2];
			TagPosM.X = TagGripP[0];
			TagPosM.Y = TagGripP[1];
			TagPosM.Z = TagGripP[2];
			Trans_MtxToPos(&TagPosM, tagpos);//tagpos:x,y,z,r,p,y  ���ⲽ����ȷ


			ofstream f;
			f.open("F:\\VS2010��RobotController\\testdataoptimize.txt",ios::app);
			for (int ii=0;ii<PoleNum;ii++)
			{
				f<<"TargetPos:  "<<endl;
				f<<tagpos[0]<<"  "<<tagpos[1]<<"  "<<tagpos[2]<<"  "<<tagpos[3]<<"  "<<tagpos[4]<<"  "<<tagpos[5]<<endl;
			}
			f.close();

			for (int i=0; i<6; i++)//GU
			{
				tagposend[i] = tagpos[i];
			}
			

			///////////////////////////////////////////////////////////////////
			//����Ԥ�гֵ�λ��
			//////////////////////////////////////////////////////////////////
			for (int i=0;i<6;i++)
			{
				increpos[i] = 0;
				increpostemp[i] = 0;
			}
			//increpos[2] = -100; //��������a��z��������100
			//Robot_IncreTransTool(tagpos, increpos, prepos);
			//Robot::I_Monit.Get_JointPos(dPos1);

			//if((bTool1) && (!bTool2))//���ݣ����ӵļн�/�ɿ�״̬
			//{
			//	if(0 != Robot::I_Task.m_iKine_CR_G1.IKine(prepos, dPos1, dPos))//dposΪ�ؽ�ת��
			//	{
			//		AfxMessageBox("����λ�λ���");
			//		return 0;
			//	}
			//}
			//else if ((!bTool1) && (bTool2))
			//{
			//	if(0 != Robot::I_Task.m_iKine_CR_G2.IKine(prepos, dPos1, dPos))
			//	{
			//		AfxMessageBox("����λ�λ���");
			//		return 0;
			//	}
			//}
			//else
			//{
			//	return 0;
			//}

			

			//increpos[2] = -100; //��������a��z��������100
			Robot_IncreTransTool(tagpos, increpos, prepos);
			increpostemp[0] = -StepLen;

			Robot::I_Monit.Get_JointPos(dPos1);

			Robot_IncreTransTool(tagpos, increpostemp, prepostemp);

			//����ѡ���ĸ����򿿽�������ԭ�㣬distancetobaseС��0�����ظ˼�������淽��ȡ�㣻
			double distancetobase = sqrt(prepostemp[0]*prepostemp[0] + prepostemp[1]*prepostemp[1]+prepostemp[2]*prepostemp[2]) - sqrt(prepos[0]*prepos[0] + prepos[1]*prepos[1] + prepos[2]*prepos[2]);
			
			increpos[2] = -130; //��������a��z��������100
			Robot_IncreTransTool(tagpos, increpos, prepos);

			while(ChooseGraspPoint)
			{
				if((bTool1) && (!bTool2))//���ݣ����ӵļн�/�ɿ�״̬
				{
					if(0 != Robot::I_Task.m_iKine_CR_G1.IKine(prepos, dPos1, dPos) || 
						0 != Robot::I_Task.m_iKine_CR_G1.IKine(tagposend, dPos1, dposend))//dposΪ�ؽ�ת��
					{
						if (distancetobase <= 0)
						{
							increpos[0] -= StepLen;
							increpostemp[0] -= StepLen;
							Robot_IncreTransTool(tagpos, increpos, prepos);
							Robot_IncreTransTool(tagpos, increpostemp, tagposend);
							for (int i = 0; i< PoleNum; i++)
							{
								if (abs(increpos[0]) > PoleLen[i]/4)
								{
									//increpos[0] += StepLen;
									//increpostemp[0] += StepLen;
									//Robot_IncreTransTool(tagpos, increpos, prepos);
									//Robot_IncreTransTool(tagpos, increpostemp, tagposend);
									//if (abs(increpos[0]) > PoleLen[i]/4)
									//{
										AfxMessageBox("����λ�λ���");
										for (int k=0; k<6; k++)
										{
											dPos[k] = dPos1[k];
										}
										return 0;
									//} 

								}
							}
			
						}
						else
						{
							increpos[0] += StepLen;
							increpostemp[0] += StepLen;
							Robot_IncreTransTool(tagpos, increpos, prepos);
							Robot_IncreTransTool(tagpos, increpostemp, tagposend);
							for (int i = 0; i< PoleNum; i++)
							{
								if (abs(increpos[0]) > PoleLen[i]/4)
								{
// 									increpos[0] -= StepLen;
// 									increpostemp[0] -= StepLen;
// 									Robot_IncreTransTool(tagpos, increpos, prepos);
// 									Robot_IncreTransTool(tagpos, increpostemp, tagposend);
// 									if (abs(increpos[0]) > PoleLen[i]/4)
// 									{
										AfxMessageBox("����λ�λ���");
										for (int k=0; k<6; k++)
										{
											dPos[k] = dPos1[k];
										}
										return 0;
									//} 
								}
							}
						}

					}
					else
					{
						ChooseGraspPoint = false;
					}
				}
				else if ((!bTool1) && (bTool2))
				{
					if(0 != Robot::I_Task.m_iKine_CR_G2.IKine(prepos, dPos1, dPos)|| 
						0 != Robot::I_Task.m_iKine_CR_G2.IKine(tagposend, dPos1, dposend))
					{
						if (distancetobase <= 0)
						{
							increpos[0] -= StepLen;
							increpostemp[0] -= StepLen;
							Robot_IncreTransTool(tagpos, increpos, prepos);
							Robot_IncreTransTool(tagpos, increpostemp, tagposend);
							for (int i = 0; i< PoleNum; i++)
							{
								if (abs(increpos[0]) > PoleLen[i]/4)
								{
// 									increpos[0] += StepLen;
// 									increpostemp[0] += StepLen;
// 									Robot_IncreTransTool(tagpos, increpos, prepos);
// 									Robot_IncreTransTool(tagpos, increpostemp, tagposend);
// 									if (abs(increpos[0]) > PoleLen[i]/4)
// 									{
										AfxMessageBox("����λ�λ���");
										for (int k=0; k<6; k++)
										{
											dPos[k] = dPos1[k];
										}
										return 0;
									//}
								}
							}

						}
						else
						{
							increpos[0] += StepLen;
							increpostemp[0] += StepLen;
							Robot_IncreTransTool(tagpos, increpos, prepos);
							Robot_IncreTransTool(tagpos, increpostemp, tagposend);
							for (int i = 0; i< PoleNum; i++)
							{
								if (abs(increpos[0]) > PoleLen[i]/4)
								{
// 									increpos[0] -= StepLen;
// 									increpostemp[0] -= StepLen;
// 									Robot_IncreTransTool(tagpos, increpos, prepos);
// 									Robot_IncreTransTool(tagpos, increpostemp, tagposend);
// 									if (abs(increpos[0]) > PoleLen[i]/4)
// 									{
										AfxMessageBox("����λ�λ���");
										for (int k=0; k<6; k++)
										{
											dPos[k] = dPos1[k];
										}
										return 0;
									//}
								}
							}
						}
					}
					else
					{
						ChooseGraspPoint = false;
					}
				}
				else
				{
					return 0;
				}
			}

			
			f.open("F:\\VS2010��RobotController\\testdataoptimize.txt",ios::app);
			for (int ii=0;ii<PoleNum;ii++)
			{
				f<<"JPosPre:  "<<endl;
				f<<dPos[0]<<"  "<<dPos[1]<<"  "<<dPos[2]<<"  "<<dPos[3]<<"  "<<dPos[4]<<endl;
			}
			f.close();

			
			f.open("F:\\VS2010��RobotController\\testdataoptimize.txt",ios::app);
			for (int ii=0;ii<PoleNum;ii++)
			{
				f<<"JPosEnd:  "<<endl;
				f<<dposend[0]<<"  "<<dposend[1]<<"  "<<dposend[2]<<"  "<<dposend[3]<<"  "<<dposend[4]<<endl;
			}
			f.close();

			//if (ReDetectionSaveData==true)
			//{
				for (int i=0; i<6; i++)
				{
					dposendtemp[i] = dposend[i];
				}
				ReDetectionSaveData = false;
			//}

			//GU


			// 	dPos[0] = -dPos[0];
			// 	dPos[4] = -dPos[4];	
			// �˶�
			// 			int vel =sliderVel;
			// 			CMotionData data;
			// 			data.Line = 1;
			// 			data.Mode = MODE_INTERP;
			// 			data.Interp.Mode = INTERP_JOINT; //PTP
			// 			data.Interp.IfLinkage = 0;
			// 			for(i=0; i<JOINT_NUM; i++)
			// 				data.Interp.JEnd[i] = dPos[i]; //deg
			// 			data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 80; //deg/s
			// 			data.Interp.Acc = MAX_AUTOACC_JOINT;
			// 			data.Interp.Jerk = MAX_JERK_JOINT;
			// 
			// 			Robot::I_Task.AddMotionData(data);	
			// 			Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ
			// 			Sleep(12000);
			// 
			// 			 for (i=0;i<6;i++)
			// 			 {
			// 			 	increpos[i] = 0;
			// 			 }
			// 			 increpos[2] = 100; //��������a��z��������100
			// 			 Robot_IncreTransTool(prepos, increpos, tag1pos);

			//ʾ��ģʽ���������ؽڵ��˶����ͷ���
			double ln_vel[6];//GU:double ln_vel[6] = {0};->
			int ln_dir[6] = {0};
			Robot::I_Task.m_iKine_CR_G1.FKine(dPos,FKPos);//FKPosΪ����λ��, (x,y,z,w,p,r)

			//dPos[] = {0 24 };

			//for (int i=0;i<6;i++)
			//{
			//	ln_dir[i] = ((dPos[i]-dPos1[i]>0) ? 1 : -1);
			//	//ln_vel[i] = ((abs(dPos[i]-dPos1[i])>2) ? abs(dPos[i]-dPos1[i])*sliderVel/10 : 0);
			//	//ln_vel[i] = (abs(dPos[i]-dPos1[i])>2) ? abs(dPos[i]-dPos1[i])*sliderVel/10 : 0;//5 : 0;
			//	ln_vel[i] = ((abs(dPos[i]-dPos1[i])>1) ? sliderVel : 0);
			//}
			////if ((ln_vel[0]+ln_vel[1]+ln_vel[2]+ln_vel[3]+ln_vel[4]+ln_vel[5])==0)//GU
			////{
			////	RealtimeCon = 0;
			////}

			//if ((ln_vel[0]+ln_vel[1]+ln_vel[2]+ln_vel[3]+ln_vel[4])==0)
			//{
			//	RealtimeCon = 0;
			//	ChooseGraspPoint = true;
			//}

			//// 		if (m_nTrack == 1) // λ�ø���
			//// 		{
			//// 			// ��ȡ��λ��
			//// 			if (m_nCommData[0] = 1)
			//// 			{	
			//// 				double ld_temp = sqrt((double)(SQUARE(m_nCommData[1]) + SQUARE(m_nCommData[2])));
			//// 				double velpct1 = (ld_temp>10) ? abs(m_nCommData[1])/ld_temp : 0;
			//// 				double velpct2 = (ld_temp>10) ? abs(m_nCommData[2])/ld_temp : 0;			
			//// 
			//// 				ln_vel[1] = (int)(sliderVel*velpct1);
			//// 				ln_vel[2] = (int)(m_sliderVel.GetPos()*velpct2);
			//// 				ln_dir[1] = (m_nCommData[1] > 0) ? 1 : -1;
			//// 				ln_dir[2] = (m_nCommData[2] > 0) ? 1 : -1;
			//// 
			//// 				ln_vel[5] = (fabs(m_dwz)>5) ? 20 : 0;
			//// 				ln_dir[5] = (m_dwz > 0) ? 1 : -1;
			//// 			}
			//// 		}
			//// 		else if (m_nTrack == 2) // ��̬����
			//// 		{	
			//// 			ln_vel[5] = (fabs(m_dwz)>5) ? 20 : 0;
			//// 			ln_dir[5] = (m_dwz > 0) ? 1 : -1;
			//// 		}

			//if (RUNMODE_TEACH == Robot::runMode)   // ʾ��ģʽ
			//{
			//	// ����ʾ������
			//	Robot::I_Task.Teach(COORDINATE_JOINT, ln_vel, ln_dir);//GU   ��ԭ��ѡ��ѿ���ʾ�̵�ģʽ������һֱ�ﵽ����Ŀ��λ�õ�����
			//}

			for (int k=0; k<6; k++)
			{
				DetectJointPosition[k] = dPos[k];
			}

			CMotionData data;
			data.Line = 1;
			data.Mode = MODE_INTERP;
			data.Interp.Mode = INTERP_JOINT; //PTP
			data.Interp.IfLinkage = 0;
			for(int i=0; i<JOINT_NUM; i++)
				data.Interp.JEnd[i] = dPos[i]; //deg
			data.Interp.Vel = sliderVel; //deg/s
			data.Interp.Acc = MAX_AUTOACC_JOINT;
			data.Interp.Jerk = MAX_JERK_JOINT;

			Robot::I_Task.AddMotionData(data);	
			Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

			RealtimeCon = 0;

			Sleep(500);//500GU
			// 	dPos[0] = -dPos[0];
			// 	dPos[4] = -dPos[4];	
			// �˶�
			// 		vel =sliderVel;
			// 		data.Line = 1;
			// 		data.Mode = MODE_INTERP;
			// 		data.Interp.Mode = INTERP_JOINT; //PTP
			// 		data.Interp.IfLinkage = 0;
			// 		for(i=0; i<JOINT_NUM; i++)
			// 			data.Interp.JEnd[i] = dPos[i]; //deg
			// 		data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 80; //deg/s
			// 		data.Interp.Acc = MAX_AUTOACC_JOINT;
			// 		data.Interp.Jerk = MAX_JERK_JOINT;
			// 
			// 		Robot::I_Task.AddMotionData(data);	
			// 		Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ
			// 		Sleep(8000);
			///////////////////////////////////////////////			
		//}

		//������ʵ�����������У������˵�ʵ�ʻ�����ϵ�����۵Ĵ���������Ҫ��ζ�Ŀ��˼����м���Ե��������˵�ץ��λ�˴Ӷ�׼ȷץ��  GU
		//˼·���ڻ����˵���Ԥ�гֵ�ʱ���ٴζԸ˼����м�⣬��3�μ��õ��ļгֵ�λ�˶�����һ�����������ץ��
		/*while(ReDetection)
		{*/
			//RedetPole.RedetectPole();
			//for (int i=0;i<6;i++)
			//{
			//	dposenddifval[i] = ((abs(dposend[i]-dposendtemp[i])>0.2) ? 1 : 0);
			//}
			//if (dposenddifval[0]+dposenddifval[1]+dposenddifval[2]+dposenddifval[3]+dposenddifval[4] == 0)//dposΪ�ؽ�ת��
			//{
			//	RedetSucCount++;
			//	ReDetection = true;
			//} 
			//else
			//{
			//	RedetSucCount = 0;
			//	ReDetection = false;
			//}
			//if (RedetSucCount==3)
			//{
				//Robot::I_Monit.Get_JointPos(dPos1);

				//if((bTool1) && (!bTool2))
				//{
				//	if(0 != Robot::I_Task.m_iKine_CR_G1.IKine(tagposend, dPos1, dPos))
				//	{
				//		AfxMessageBox("����λ�λ���");
				//		return 0;
				//	}
				//}
				//else if ((!bTool1) && (bTool2))
				//{
				//	if(0 != Robot::I_Task.m_iKine_CR_G2.IKine(tagposend, dPos1, dPos))
				//	{
				//		AfxMessageBox("����λ�λ���");
				//		return 0;
				//	}
				//}
				//else
				//{
				//	return 0;
				//}

				//for (int i=0; i<6; i++)    //GU
				//{
				//	jpostarget[i] = dPos[i];
				//}
				////�˶�
				//int vel =sliderVel;
				//CMotionData data;
				//data.Line = 1;
				//data.Mode = MODE_INTERP;
				//data.Interp.Mode = INTERP_JOINT; //PTP
				//data.Interp.IfLinkage = 0;//��������־
				//for(int i=0; i<JOINT_NUM; i++)
				//{
				//	data.Interp.JEnd[i] = dPos[i]; //deg
				//	data.Interp.JStart[i] = dPos1[i];//deg  GU
				//}
				//data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 80; //deg/s
				//data.Interp.Acc = MAX_AUTOACC_JOINT;
				//data.Interp.Jerk = MAX_JERK_JOINT;
				//// 
				//Robot::I_Task.AddMotionData(data);	
				//Robot::I_Task.Set_GivenPosRunMode(TRUE); // ����λ��ʾ��ģʽ
				//Sleep(8000);
				
				//ReDetection = false;

		ChooseGraspPoint = true;
		for (int i=0;i<6;i++)
		{
			ln_dir[i] =fabs(dPos[i]-dPos1[i]);
		}
		if ((ln_dir[0]+ln_dir[1]+ln_dir[2]+ln_dir[3]+ln_dir[4]+ln_dir[5])<=0.01)//GU
		{
			m_bReachTarget = true;
			ReDetection = true;
		}
		else
		{
			m_bReachTarget = false;
			ReDetection = false;
		}
		

		//if ((abs(dPos[0]-dPos1[0])<0.1)&&(abs(dPos[1]-dPos1[1])<0.1)&&(abs(dPos[2]-dPos1[2])<0.1)&&
		//	(abs(dPos[3]-dPos1[3])<0.1)&&(abs(dPos[4]-dPos1[4])<0.1))
		//{
		//	ReDetection = true;
		//	return 0;
		//}

			/*}*/
			//ReDetectionSaveData = true;
		//}
	} 
	//////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////
	//�гַ���
	//if (PoleStyle[NearestPoleN]==Square)
	//{
	//	//���ڷ�����ȷ��Ŀ����̬
	//	for (i=0;i<3;i++)
	//	{
	//		TagGripDir[i] = PoleCentralLine[NearestPoleN][0][i];
	//	}
	//	//���ӵ�n������˼�������������
	//	for (i=0;i<3;i++)
	//	{
	//		n[i] = TagGripDir[i];
	//	}
	//	n[0]=1;
	//	n[1]=0;
	//	n[2]=0;
	//	VectorUnitizationD(n);
	//	//���ݷ��˼г�λ���ص㣬�������a��o;
	//	t = ( PoleCentralLine[NearestPoleN][0][0]*(SConf[NearestPoleN][0]-PoleCentralLine[NearestPoleN][1][0])
	//		+PoleCentralLine[NearestPoleN][0][1]*(SConf[NearestPoleN][1]-PoleCentralLine[NearestPoleN][1][1])
	//		+PoleCentralLine[NearestPoleN][0][2]*(SConf[NearestPoleN][2]-PoleCentralLine[NearestPoleN][1][2]) )
	//		/(PoleCentralLine[NearestPoleN][0][0]*PoleCentralLine[NearestPoleN][0][0]
	//	+PoleCentralLine[NearestPoleN][0][1]*PoleCentralLine[NearestPoleN][0][1]
	//	+PoleCentralLine[NearestPoleN][0][2]*PoleCentralLine[NearestPoleN][0][2] );
	//	//ȷ��a
	//	for (i=0;i<3;i++)
	//	{
	//		a[i] = PoleCentralLine[NearestPoleN][1][i]+t*PoleCentralLine[NearestPoleN][0][i]-SConf[NearestPoleN][i];
	//	}
	//	a[0]=0;
	//	a[1]=0;
	//	a[2]=-1;
	//	VectorUnitizationD(a);
	//	//ȷ��n��a�ķ���
	//	MtxKine CoCM;   //��ǰ������λ���µ�����任����
	//	Trans_PosToMtx(currpos, &CoCM, 0);
	//	if ((n[0]*CoCM.R11+n[1]*CoCM.R21+n[2]*CoCM.R31)<0) //��������˵��n����ӵ�n�нǴ���90��
	//	{
	//		for (i=0;i<3;i++)
	//		{
	//			n[i] = -n[i];
	//		}
	//	}
	//	if ((a[0]*CoCM.R13+a[1]*CoCM.R23+a[2]*CoCM.R33)<0) //��������˵��a����ӵ�a�нǴ���90��
	//	{
	//		for (i=0;i<3;i++)
	//		{
	//			a[i] = -a[i];
	//		}
	//	}

	//	o[0] = a[1]*n[2]-a[2]*n[1];
	//	o[1] = n[0]*a[2]-n[2]*a[0];
	//	o[2] = a[0]*n[1]-a[1]*n[0];
	//	VectorUnitizationD(o);
	//	if ((o[0]*CoCM.R12+o[1]*CoCM.R22+o[2]*CoCM.R32)<0) //��������˵��o����ӵ�o�нǴ���90��
	//	{
	//		for (i=0;i<3;i++)
	//		{
	//			o[i] = -o[i];
	//		}
	//	}
	//	//���������ƽ��m��ֵ
	//	m[0] = a[1];
	//	m[1] = -a[0];
	//	m[2] = 0;
	//	////////////////////////////////////////////////////
	//	//����Ŀ��˼��ϵ�Ŀ��гֵ㣬�õ��ʾ��������ֱ��m
	//	t = -(PoleCentralLine[NearestPoleN][1][0]*m[0]+PoleCentralLine[NearestPoleN][1][1]*m[1]+
	//		PoleCentralLine[NearestPoleN][1][2]*m[2])/(PoleCentralLine[NearestPoleN][0][0]*m[0]+
	//		PoleCentralLine[NearestPoleN][0][1]*m[1]+PoleCentralLine[NearestPoleN][0][2]*m[2]);
	//	for (i=0;i<3;i++)
	//	{
	//		TagGripP[i] = PoleCentralLine[NearestPoleN][1][i] + t*PoleCentralLine[NearestPoleN][0][i];
	//	}
	//	TagGripP[0]=670;
	//	TagGripP[1]=-230;
	//	TagGripP[2]=0;
	//	//���Ŀ��гֵ㳬����⵽�ĸ˼���Χ����ȥץ��
	//	// 		if (sqrt(pow(TagGripP[0]-PoleCentralLine[NearestPoleN][1][0],2)+pow(TagGripP[1]-PoleCentralLine[NearestPoleN][1][1],2)
	//	// 			+pow(TagGripP[2]-PoleCentralLine[NearestPoleN][1][2],2))>(PoleLen[0]*1.2))
	//	// 		{
	//	// 			AfxMessageBox("�˼�ץ�е㳬��");
	//	// 			return 0;
	//	// 		}
	//	//�г��˶�
	//	MtxKine TagPosM;    //Ŀ��гֵ���̬����
	//	double tagpos[6];   //Ŀ��гֵ�λ��
	//	double tagajpos[6];  //�������λ��
	//	double tag1pos[6];   //Ŀ��гֵ�λ��
	//	double tag2pos[6];   //Ŀ��гֵ�λ��
	//	double prepos[6];   //Ԥ�гֵ�λ��
	//	TagPosM.R11 = n[0];
	//	TagPosM.R12 = o[0];
	//	TagPosM.R13 = a[0];
	//	TagPosM.R21 = n[1];
	//	TagPosM.R22 = o[1];
	//	TagPosM.R23 = a[1];
	//	TagPosM.R31 = n[2];
	//	TagPosM.R32 = o[2];
	//	TagPosM.R33 = a[2];
	//	TagPosM.X = TagGripP[0];
	//	TagPosM.Y = TagGripP[1];
	//	TagPosM.Z = TagGripP[2];
	//	Trans_MtxToPos(&TagPosM, tagpos);
	//	double increpos[6], dPos1[6], dPos[6];
	//	///////////////////////////////////////////////////////////////////
	//	//����Ԥ�гֵ�λ��
	//	//////////////////////////////////////////////////////////////////
	//	for (i=0;i<6;i++)
	//	{
	//		increpos[i] = 0;
	//	}
	//	increpos[2] = -100; //��������a��z��������100
	//	Robot_IncreTransTool(tagpos, increpos, prepos);
	//	Robot::I_Monit.Get_JointPos(dPos1);

	//	if((bTool1) && (!bTool2))
	//	{
	//		if(0 != Robot::I_Task.m_iKine_CR_G1.IKine(prepos, dPos1, dPos))
	//		{
	//			AfxMessageBox("����λ�λ���");
	//			return 0;
	//		}
	//	}
	//	else if ((!bTool1) && (bTool2))
	//	{
	//		if(0 != Robot::I_Task.m_iKine_CR_G2.IKine(prepos, dPos1, dPos))
	//		{
	//			AfxMessageBox("����λ�λ���");
	//			return 0;
	//		}
	//	}
	//	else
	//	{
	//		return 0;
	//	}
	//	// 	dPos[0] = -dPos[0];
	//	// 	dPos[4] = -dPos[4];	
	//	// �˶�
	//	int vel =sliderVel;
	//	CMotionData data;
	//	data.Line = 1;
	//	data.Mode = MODE_INTERP;
	//	data.Interp.Mode = INTERP_JOINT; //PTP
	//	data.Interp.IfLinkage = 0;//��������־
	//	for(i=0; i<JOINT_NUM; i++)
	//		data.Interp.JEnd[i] = dPos[i]; //deg
	//	data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 80; //deg/s
	//	data.Interp.Acc = MAX_AUTOACC_JOINT;
	//	data.Interp.Jerk = MAX_JERK_JOINT;

	//	Robot::I_Task.AddMotionData(data);	
	//	Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ
	//	Sleep(12000);

	//	// 	for (i=0;i<6;i++)
	//	// 	{
	//	// 		increpos[i] = 0;
	//	// 	}
	//	// 	increpos[2] = 100; //��������a��z��������100
	//	//     Robot_IncreTransTool(prepos, increpos, tag1pos);
	//	Robot::I_Monit.Get_JointPos(dPos1);

	//	if((bTool1) && (!bTool2))
	//	{
	//		if(0 != Robot::I_Task.m_iKine_CR_G1.IKine(tagpos, dPos1, dPos))
	//		{
	//			AfxMessageBox("����λ�λ���");
	//			return 0;
	//		}
	//	}
	//	else if ((!bTool1) && (bTool2))
	//	{
	//		if(0 != Robot::I_Task.m_iKine_CR_G2.IKine(tagpos, dPos1, dPos))
	//		{
	//			AfxMessageBox("����λ�λ���");
	//			return 0;
	//		}
	//	}
	//	else
	//	{
	//		return 0;
	//	}
	//	// 	dPos[0] = -dPos[0];
	//	// 	dPos[4] = -dPos[4];	
	//	// �˶�
	//	vel =sliderVel;
	//	data.Line = 1;
	//	data.Mode = MODE_INTERP;
	//	data.Interp.Mode = INTERP_JOINT; //PTP
	//	data.Interp.IfLinkage = 0;
	//	for(i=0; i<JOINT_NUM; i++)
	//		data.Interp.JEnd[i] = dPos[i]; //deg
	//	data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 80; //deg/s
	//	data.Interp.Acc = MAX_AUTOACC_JOINT;
	//	data.Interp.Jerk = MAX_JERK_JOINT;

	//	Robot::I_Task.AddMotionData(data);	
	//	Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ
	//	Sleep(8000);
	//	///////////////////////////////////////////////
	//	return 0;
	//}
	if (PoleStyle[NearestPoleN]==Square)//GU  ��ԭ����ץ�з������ε���
	{
		AfxMessageBox("ץ�з�������");
	}
}
DWORD WINAPI FunPoleDetProc(LPVOID lpParameter)  // thread data �������̺߳���
{
	CToolDlg* pThis = (CToolDlg*)lpParameter;  //�����߳�ָ��
	int i;
	while (RealtimeCon)
	{
		long t1=GetTickCount();
		//PoleNum = pThis->DetectRoundPoles(PoleCentralLine[0][0],Dmin,PoleDim,PoleLen,PoleBend);
		//a = PolesDet(r,poleline,che,dim,len,bend,s);
		long t2=GetTickCount();
		long DetTime = t2-t1;
		/////////////////////////////////////////////
		//�������ת������������ϵ��
		//������ת��
		for (i=0;i<PoleNum;i++)
		{
			PointCoordinateC(&CPrimeSenseDet::PUntifyMtx,PoleCentralLine[i][1],PoleCentralLine[i][1]);
			PointCoordinateC(&CPrimeSenseDet::PUntifyMtx,SConf[i],SConf[i]);
		}
		//��������任����Ҫ��ת������ľ���ֵ�����
		for (i=0;i<PoleNum;i++)
		{
			VectorCoordinateC(&CPrimeSenseDet::PUntifyMtx,PoleCentralLine[i][0],PoleCentralLine[i][0]);
		}
		/////////////////////////////////////////////////////////////
		ofstream f;
		f.open("F:\\VS2010��RobotController\\rounddata.txt",ios::app);
		f<<"DetTime: "<<DetTime<<endl;
		for (int i=0;i<PoleNum;i++)
		{
			PoleStyle[i] = Round;
			f<<"PoleStyle�� "<<PoleStyle[i]<<endl;
			f<<"PoleCentre: ["<<PoleCentralLine[i][0][0]<<","<<PoleCentralLine[i][0][1]<<","
				<<PoleCentralLine[i][0][2]<<";    "<<PoleCentralLine[i][1][0]<<","<<PoleCentralLine[i][1][1]<<","<<PoleCentralLine[i][1][2]<<"]"<<endl;
			f<<"Dmin: "<<Dmin[i]<<"   PoleDim: "<<PoleDim[i]<<"   PoleLen: "<<PoleLen[i]<<"   PoleBend: "<<PoleBend[i]<<endl;
			//		f<<"SConf: ["<<SConf[i][0]<<","<<SConf[i][1]<<","<<SConf[i][2]<<"]"<<endl;
			f<<"  "<<endl;
		}
		f.close();
	}
	return 0;
}

void GetMemory(char *p, int num)
{
	p = (char*)malloc(sizeof(char) * num);
}


void CToolDlg::OnBnClickedBtPrimesensecam()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	//char *str = NULL;
	//GetMemory(str, 100);
	////cout<<"Memory leak test!"<<endl;
	//_CrtDumpMemoryLeaks();
	ofstream f;

	bTool1 = m_bTool1;//����1�ļн��ɿ�״̬
	bTool2 = m_bTool2;//����2�ļн��ɿ�״̬
	sliderVel = MAX_TEACHVEL_JOINT*(int)(m_sliderVel.GetPos())/100;
	int i;
	int DeteCount = 0;
	int DeteCountFalse = 0;
	RealtimeCon = true;
	DeteSymbol = true;

	ReDetection = false;




	//stability_test
	//////////////////////////////////////////////////////////////////////////
	if (TRUE)
	{
		f.open("F:\\stability_test_data\\stability_test.txt",ios::app);
		OnBnClickedButtonOpendevice();
		long test_time1 = GetTickCount();
		PoleNum = m_pCprimeSenseDet->PolesDet();
		long test_time2 = GetTickCount();
		long test_time = test_time2 - test_time1;
	
		stability_test_total_count ++;

		for (int pn=0; pn<PoleNum; pn++)              //GU
		{
			for (int m=0; m<2; m++)
			{
				for (int k=0; k<3; k++)
				{
					PoleCentralLineCount[pn][m][DeteCount][k] = PCentralLineTemp[pn][m][k];
				}
			}
			PoleLen[pn] = PoleLenghtTemp[pn];
		}

		if (PoleNum >= 1)
		{
			stability_test_success_count++;
			f<<"��"<< "  "<< stability_test_total_count << "  " <<"�β��Ժ�ʱ:"<<test_time<< ";"<<endl;
			f<<"�ɹ���ȡ����:"<<stability_test_success_count<< ";"<<endl;
			f<<"��ץ�и˼���Ŀ:"<<PoleNum<< ";"<<endl;
			for (int ii=0;ii<PoleNum;ii++)
				{
					f<<"��ʱ"<<test_time<<endl;
					for (int k=0; k<1; k++)
					{
						f<<k<<"  "<<"PoleCentrePoin "<<PoleCentralLineCount[ii][1][k][0]<<"  "<<PoleCentralLineCount[ii][1][k][1]<<"  "<<PoleCentralLineCount[ii][1][k][2]<<endl;
						f<<k<<"  "<<"PoleCentreDir "<<PoleCentralLineCount[ii][0][k][0]<<"  "<<PoleCentralLineCount[ii][0][k][1]<<"  "<<PoleCentralLineCount[ii][0][k][2]<<endl;
					}
				}
			f<<"   "<<endl;
			f<<"   "<<endl;
		}
		else
		{
			stability_test_fail_count++;
			f<<"��"<< "  "<< stability_test_total_count << "  " <<"�β��Ժ�ʱ:"<<test_time<< ";"<<endl;
			f<<"ʧ�ܴ���:"<<stability_test_fail_count<< ";"<<endl;
			f<<"   "<<endl;
			f<<"   "<<endl;
		}

		f.close();
		m_pCprimeSenseDet->CloseTheDevice();
	}

	//////////////////////////////////////////////////////////////////////////

	////�״μ��
	////PoleInfo *poles_info=NULL;
	//long t1=GetTickCount();
	////PoleNum = m_pCprimeSenseDet->PolesDet(poles_info);
	//PoleNum = m_pCprimeSenseDet->PolesDet();
	//long t2=GetTickCount();
	//long DetTime = t2-t1;

	//for (int pn=0; pn<PoleNum; pn++)              //GU
	//{
	//	for (int m=0; m<2; m++)
	//	{
	//		for (int k=0; k<3; k++)
	//		{
	//			PoleCentralLine[pn][m][k] = PCentralLineTemp[pn][m][k];
	//		}
	//	}
	//	PoleLen[pn] = PoleLenghtTemp[pn];
	//}

	if (false)//stability_test modify
	{
		if (!m_deviceclosesymbol)
		{
			AfxMessageBox("���ȴ��豸");
			DeteSymbol = false;
		}


		long tt1=GetTickCount();
		long t1=GetTickCount();
		while (DeteSymbol)
		{
		
			PoleNum = m_pCprimeSenseDet->PolesDet();


			for (int pn=0; pn<PoleNum; pn++)              //GU
			{
	// 			PoleStyle[pn] = PoleStyleTemp[pn];
	// 			if (PoleStyle[pn]==Round)
	// 			{
					for (int m=0; m<2; m++)
					{
						for (int k=0; k<3; k++)
						{
							PoleCentralLineCount[pn][m][DeteCount][k] = PCentralLineTemp[pn][m][k];
						}
					}
				//}
			
				PoleLen[pn] = PoleLenghtTemp[pn];
			}
			if (PoleNum>0)
			{
				DeteCount++;
				if (DeteCount==1)//ԭΪ20�Σ������Ż�����Ϊ1Ϊ����RANSAC�������Ա�
				{
					DeteSymbol = false;
					DeteCount = 0;
				}
			}
			else
			{
				DeteCountFalse++;
				if (DeteCountFalse==100)
				{
					DeteSymbol = false;
					DeteCountFalse = 0;
				}
			}
		}
		long t2=GetTickCount();
		long DetTime = t2-t1;

		f.open("F:\\VS2010��RobotController\\testdataDir-20170209.txt",ios::app);
		for (int ii=0;ii<PoleNum;ii++)
		{
			f<<"   "<<endl;
			f<<"��ʱ"<<DetTime<<endl;
			for (int k=0; k<20; k++)
			{
				f<<k<<"  "<<"PoleCentreDir "<<PoleCentralLineCount[ii][0][k][0]<<"  "<<PoleCentralLineCount[ii][0][k][1]<<"  "<<PoleCentralLineCount[ii][0][k][2]<<endl;
				//f<<"   "<<"PoleCentrePoin "<<PoleCentralLineCount[ii][1][k][0]<<"  "<<PoleCentralLineCount[ii][1][k][1]<<"  "<<PoleCentralLineCount[ii][1][k][2]<<endl;
			}

		}
		f.close();

		f.open("F:\\VS2010��RobotController\\testdataPoin-20170209.txt",ios::app);
		for (int ii=0;ii<PoleNum;ii++)
		{
			f<<"   "<<endl;
			f<<"��ʱ"<<DetTime<<endl;
			for (int k=0; k<20; k++)
			{
				//f<<k<<"  "<<"PoleCentreDir "<<PoleCentralLineCount[ii][0][k][0]<<"  "<<PoleCentralLineCount[ii][0][k][1]<<"  "<<PoleCentralLineCount[ii][0][k][2]<<endl;
				f<<k<<"  "<<"PoleCentrePoin "<<PoleCentralLineCount[ii][1][k][0]<<"  "<<PoleCentralLineCount[ii][1][k][1]<<"  "<<PoleCentralLineCount[ii][1][k][2]<<endl;
			}

		}
		f.close();

		PoleCentralLineOptimize(PoleCentralLineCount);
	
		long tt2 = GetTickCount();
		long DetTime_Total = tt2 - tt1;//20170208���
		f.open("F:\\VS2010��RobotController\\testdataoptimize-20170209.txt",ios::app);
		for (int ii=0;ii<PoleNum;ii++)
		{
			f<<"��ʱ"<<DetTime_Total<<endl;	
			f<<"PoleCentreDir "<<PoleCentralLine[ii][0][0]<<"  "<<PoleCentralLine[ii][0][1]<<"  "<<PoleCentralLine[ii][0][2]<<endl;
			f<<"PoleCentrePoin "<<PoleCentralLine[ii][1][0]<<"  "<<PoleCentralLine[ii][1][1]<<"  "<<PoleCentralLine[ii][1][2]<<endl;
		}
		f.close();

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////OPENGL
		int openglk;
		for (int k=0; k<=PoleNum; k++)
		{
			PCentralLine[k][0][0] = PoleCentralLine[k][1][0]+220*PoleCentralLine[k][0][0];
			PCentralLine[k][0][1] = PoleCentralLine[k][1][1]+220*PoleCentralLine[k][0][1];
			PCentralLine[k][0][2] = PoleCentralLine[k][1][2]+220*PoleCentralLine[k][0][2];

			//for (int k = 0; k < 3; k++)//GU
			//{
			//	PCentralLineTemp[i][0][k] = PCentralLine[i][0][k];
			//}
			if ((bTool1) && (!bTool2))
			{
				PointCoordinateC(&CPrimeSenseDet::PUntifyMtx,PCentralLine[k][0],PCentralLine[k][0]);
			}
			else if ((!bTool1)&&(bTool2))
			{
				PointCoordinateC(&CPrimeSenseDet::G2_PUntifyMtx,PCentralLine[k][0],PCentralLine[k][0]);
			}

			//PointCoordinateC(&CPrimeSenseDet::PUntifyMtx,PCentralLine[k][0],PCentralLine[k][0]);
		
			for (openglk=0;openglk<3;openglk++)
			{
				PCentralLine[k][0][openglk] = PCentralLine[k][0][openglk]/1000.0;
			}
			PCentralLine[k][1][0] = PoleCentralLine[k][1][0]-220*PoleCentralLine[k][0][0];
			PCentralLine[k][1][1] = PoleCentralLine[k][1][1]-220*PoleCentralLine[k][0][1];
			PCentralLine[k][1][2] = PoleCentralLine[k][1][2]-220*PoleCentralLine[k][0][2];

			//for (int k = 0; k < 3; k++)//GU
			//{
			//	PCentralLineTemp[i][1][k] = PCentralLine[i][1][k];
			//}

			if ((bTool1) && (!bTool2))
			{
				PointCoordinateC(&CPrimeSenseDet::PUntifyMtx,PCentralLine[k][1],PCentralLine[k][1]);
			}
			else if ((!bTool1)&&(bTool2))
			{
				PointCoordinateC(&CPrimeSenseDet::G2_PUntifyMtx,PCentralLine[k][1],PCentralLine[k][1]);
			}

			//PointCoordinateC(&CPrimeSenseDet::PUntifyMtx,PCentralLine[k][1],PCentralLine[k][1]);
		
			for (openglk=0;openglk<3;openglk++)
			{
				PCentralLine[k][1][openglk] = PCentralLine[k][1][openglk]/1000.0;
				PonCentline[k][openglk] = PoleCentralLine[k][1][openglk]/1000.0;
			}
		}

																																												/////////////////////////////////////////////////////////////////////////////////////////OPENGL


	//for (int pn=0; pn<PoleNum; pn++)              //GU
	//{
	//	for (int m=0; m<2; m++)
	//	{
	//		for (int k=0; k<3; k++)
	//		{
	//			PoleCentralLine[pn][m][k] = PCentralLineTemp[pn][m][k];
	//		}
	//	}
	//	PoleLen[pn] = PoleLenghtTemp[pn];
	//}



	/////////////////////////////////////////////
	//�������ת������������ϵ��
	//������ת��

 	//if ((bTool1) && (!bTool2))
 	//{
 	//	for (i=0;i<PoleNum;i++)
 	//	{
 	//		PoleStyle[i] = Round;
 	//		PointCoordinateC(&CPrimeSenseDet::PUntifyMtx,PoleCentralLine[i][1],PoleCentralLine[i][1]);
 	//		PointCoordinateC(&CPrimeSenseDet::PUntifyMtx,SConf[i],SConf[i]);
 	//	}
 	//	//��������任����Ҫ��ת������ľ���ֵ�����
 	//	for (i=0;i<PoleNum;i++)
 	//	{
 	//		VectorCoordinateC(&CPrimeSenseDet::PUntifyMtx,PoleCentralLine[i][0],PoleCentralLine[i][0]);
 	//	}
 	//}
 	//else if ((!bTool1) && (bTool2))
 	//{
 	//	for (i=0;i<PoleNum;i++)
 	//	{
 	//		PoleStyle[i] = Round;
 	//		PointCoordinateC(&CPrimeSenseDet::G2_PUntifyMtx,PoleCentralLine[i][1],PoleCentralLine[i][1]);
 	//		PointCoordinateC(&CPrimeSenseDet::G2_PUntifyMtx,SConf[i],SConf[i]);
 	//	}
 	//	//��������任����Ҫ��ת������ľ���ֵ�����
 	//	for (i=0;i<PoleNum;i++)
 	//	{
 	//		VectorCoordinateC(&CPrimeSenseDet::G2_PUntifyMtx,PoleCentralLine[i][0],PoleCentralLine[i][0]);
 	//	}
 	//}


		for (i=0;i<PoleNum;i++)
		{
			PoleStyle[i] = Round;
			PointCoordinateC(&CPrimeSenseDet::PUntifyMtx,PoleCentralLine[i][1],PoleCentralLine[i][1]);
			PointCoordinateC(&CPrimeSenseDet::PUntifyMtx,SConf[i],SConf[i]);
		}
		//��������任����Ҫ��ת������ľ���ֵ�����
		for (i=0;i<PoleNum;i++)
		{
			VectorCoordinateC(&CPrimeSenseDet::PUntifyMtx,PoleCentralLine[i][0],PoleCentralLine[i][0]);
		}


		f.open("F:\\VS2010��RobotController\\testdataoptimize_2Base.txt",ios::app);
		for (int ii=0;ii<PoleNum;ii++)
		{
			f<<"ת����������ϵ�£�"<<endl;	
			f<<"PoleCentreDirB "<<PoleCentralLine[ii][0][0]<<"  "<<PoleCentralLine[ii][0][1]<<"  "<<PoleCentralLine[ii][0][2]<<endl;
			f<<"PoleCentrePoinB "<<PoleCentralLine[ii][1][0]<<"  "<<PoleCentralLine[ii][1][1]<<"  "<<PoleCentralLine[ii][1][2]<<endl;
		}
		f.close();
																/////////////////////////////////////////////////////////////
	
	//f.open("F:\\VS2010��RobotController\\rounddata.txt",ios::app);
	//f<<"DetTime: "<<DetTime<<endl;
	//for (int i=0;i<PoleNum;i++)
	//{
	//	PoleStyle[i] = Round;
	//	f<<"PoleStyle�� "<<PoleStyle[i]<<endl;
	//	f<<"PoleCentre: ["<<PoleCentralLine[i][0][0]<<","<<PoleCentralLine[i][0][1]<<","
	//		<<PoleCentralLine[i][0][2]<<";    "<<PoleCentralLine[i][1][0]<<","<<PoleCentralLine[i][1][1]<<","<<PoleCentralLine[i][1][2]<<"]"<<endl;
	//	f<<"Dmin: "<<Dmin[i]<<"   PoleDim: "<<PoleDim[i]<<"   PoleLen: "<<PoleLen[i]<<"   PoleBend: "<<PoleBend[i]<<endl;
	//	//		f<<"SConf: ["<<SConf[i][0]<<","<<SConf[i][1]<<","<<SConf[i][2]<<"]"<<endl;
	//	f<<"  "<<endl;
	//}
	//f.close();

		for (i=0;i<PoleNum;i++)
		{
			if (PoleStyle[i] == Round)
			{
				testnum++;
				if (testnum == 20)
				{
					AfxMessageBox("Next");
					testnum = 0;
				}
																													//��¼���ĵ�͸˼����߷���
			/*f.open("F:\\VS2010��RobotController\\testdata_Direction.txt",ios::app);
			for (int i=0;i<PoleNum;i++)
			{
			f<<testnum<<"  "<<"PoleCentreDir "<<PCentralLineTemp[i][0][0]<<"  "<<PCentralLineTemp[i][0][1]<<"  "<<PCentralLineTemp[i][0][2]<<endl;
			}
			f.close();

			f.open("F:\\VS2010��RobotController\\testdata_CenterPoint.txt",ios::app);
			for (int i=0;i<PoleNum;i++)
			{
			f<<testnum<<"  "<<"PoleCentrePoin "<<PCentralLineTemp[i][1][0]<<"  "<<PCentralLineTemp[i][1][1]<<"  "<<PCentralLineTemp[i][1][2]<<endl;
			}
			f.close();*/
			//��Z��ת�����ݼ�¼
			//f.open("F:\\VS2010��RobotController\\DirPre-Z.txt",ios::app);
			//for (int i=0;i<PoleNum;i++)
			//{
			//f<<testnum<<"  "<<"PoleCentreDir "<<PCentralLineTemp[i][0][0]<<"  "<<PCentralLineTemp[i][0][1]<<"  "<<PCentralLineTemp[i][0][2]<<endl;
			//}
			//f.close();


// 			f.open("F:\\VS2010��RobotController\\DirPre-Y.txt",ios::app);
// 			for (int i=0;i<PoleNum;i++)
// 			{
// 				f<<testnum<<"  "<<"PoleCentreDir "<<PCentralLineTemp[i][0][0]<<"  "<<PCentralLineTemp[i][0][1]<<"  "<<PCentralLineTemp[i][0][2]<<endl;
// 			}
// 			f.close();
			}
		}
	

		//������ʱ�����и˼����
		//SetTimer(7,1000,NULL);//��������ʱ��7û�ҵ���
		//�����̺߳���
		HANDLE hThread1;
		hThread1=CreateThread(NULL,0,FunIntelligentGraspProc,(LPVOID)this,0,NULL);
		CloseHandle(hThread1);
														/////////////////////////////////////////
	//�����̺߳���

	// 	HANDLE hThread2;
	// 	hThread2=CreateThread(NULL,0,FunPoleDetProc,(LPVOID)this,0,NULL);
	// 	CloseHandle(hThread2);
	////////////////////////////////////////////////////////////////////


	////////////////////////////////////////////////
	//����ץ�в���
	///////////////////////////////////////////////////
	/*if (PoleNum==0)
	{
	return;
	}*/
		SetTimer(7,2000,NULL);
	}
}

/*************************************************************************
������PoleCentralLineOptimize(double PloeCentralLine[5][2][20][3])
  ���ܣ��˼��������߼������ĵ���Ż�
  ���룺double PloeCentralLine[5][2][20][3]

  �����...
**************************************************************************/

void CToolDlg::PoleCentralLineOptimize(float PoleCentralLineTemp[5][2][20][3])
{
	float PoleCentralLinePointFirst[5][3],PoleCentralLineDirectFirst[5][3],PoleCentralLineCountTemp0[3],PoleCentralLineCountTemp1[3];
	//float PoleCentralLinePointEnd[5][3],PoleCentralLineDirectEnd[5][3];
	float PtoPDistance[5][20], AngleErr[5][20];
	int ErrTestPointCount[5] = {0}, ErrTestDirCount[5] = {0};
	int ErrPointCount=0, ErrDirCount=0;

	for (int i=0; i<10; i++)
	{
		for (int k=0; k<2; k++)
		{
			for (int ik=0; ik<3; ik++)
			{
				PoleCentralLine[i][k][ik] = 0;
			}
		}
	}

	//�״���ƽ��ֵ
	for (int i=0; i<PoleNum; i++)
	{
		for (int k=0; k<20; k++)
		{
			PoleCentralLinePointFirst[i][0] = PoleCentralLinePointFirst[i][0] + PoleCentralLineCount[i][1][k][0];
			PoleCentralLinePointFirst[i][1] = PoleCentralLinePointFirst[i][1] + PoleCentralLineCount[i][1][k][1];
			PoleCentralLinePointFirst[i][2] = PoleCentralLinePointFirst[i][2] + PoleCentralLineCount[i][1][k][2];

			PoleCentralLineDirectFirst[i][0] = PoleCentralLineDirectFirst[i][0] + PoleCentralLineCount[i][0][k][0];
			PoleCentralLineDirectFirst[i][1] = PoleCentralLineDirectFirst[i][1] + PoleCentralLineCount[i][0][k][1];
			PoleCentralLineDirectFirst[i][2] = PoleCentralLineDirectFirst[i][2] + PoleCentralLineCount[i][0][k][2];
		}

		PoleCentralLinePointFirst[i][0] = PoleCentralLinePointFirst[i][0]/20;
		PoleCentralLinePointFirst[i][1] = PoleCentralLinePointFirst[i][1]/20;
		PoleCentralLinePointFirst[i][2] = PoleCentralLinePointFirst[i][2]/20;

		PoleCentralLineDirectFirst[i][0] = PoleCentralLineDirectFirst[i][0]/20;
		PoleCentralLineDirectFirst[i][1] = PoleCentralLineDirectFirst[i][1]/20;
		PoleCentralLineDirectFirst[i][2] = PoleCentralLineDirectFirst[i][2]/20;
	}

	//���������ĵ�ͷ���ֱ�����õ�ƽ�����ĵ��ƽ�����������ͽǶȣ�������С�������������
	//�������뷨������ȥ���쳣ֵ��ע���ų���������ֵ�������ã�DISTANCEERROR�� ANGLEERROR����
	for (int i=0; i<PoleNum; i++)
	{
		for (int k=0; k<20; k++)
		{
			PtoPDistance[i][k] = sqrt((PoleCentralLinePointFirst[i][0]-PoleCentralLineCount[i][1][k][0])*(PoleCentralLinePointFirst[i][0]-PoleCentralLineCount[i][1][k][0])
				                  + (PoleCentralLinePointFirst[i][1]-PoleCentralLineCount[i][1][k][1])*(PoleCentralLinePointFirst[i][1]-PoleCentralLineCount[i][1][k][1])
								  + (PoleCentralLinePointFirst[i][2]-PoleCentralLineCount[i][1][k][2])*(PoleCentralLinePointFirst[i][2]-PoleCentralLineCount[i][1][k][2]));

		    AngleErr[i][k] =acos( ( PoleCentralLineDirectFirst[i][0]*PoleCentralLineCount[i][0][k][0] + PoleCentralLineDirectFirst[i][1]*PoleCentralLineCount[i][0][k][1] + 
							 PoleCentralLineDirectFirst[i][2]*PoleCentralLineCount[i][0][k][2] )/sqrt( PoleCentralLineDirectFirst[i][0]*PoleCentralLineDirectFirst[i][0]
							 + PoleCentralLineDirectFirst[i][1]*PoleCentralLineDirectFirst[i][1] + PoleCentralLineDirectFirst[i][2]*PoleCentralLineDirectFirst[i][2] )/
						     sqrt( PoleCentralLineCount[i][0][k][0]*PoleCentralLineCount[i][0][k][0] + PoleCentralLineCount[i][0][k][1]*PoleCentralLineCount[i][0][k][1]
							 + PoleCentralLineCount[i][0][k][2]*PoleCentralLineCount[i][0][k][2] ) )*PI_DEG;
			/*if ( PtoPDistance[i][k]>= 3 )
			{
				for (int m=0; m<3; m++)
				{
					PoleCentralLineCount[i][1][k][m] = 0;
				}
				ErrPointCount++;
				ErrTestPointCount[i] = ErrPointCount;
			}

			if ( AngleErr[i][k] >= 0.1)
			{
				for (int m=0; m<3; m++)
				{
					PoleCentralLineCount[i][0][k][m] = 0;
				}
				ErrDirCount++;
				ErrTestDirCount[i] = ErrDirCount;
			}*/
		}

		for (int ii=0; ii<19; ii++)
		{
			for (int kk=19; kk>ii; kk--)
			{
				if (PtoPDistance[i][kk] < PtoPDistance[i][kk-1])
				{
					for (int jj=0; jj<3; jj++)
					{
						PoleCentralLineCountTemp1[jj] = PoleCentralLineCount[i][1][kk][jj];
					}

					for (int jj=0; jj<3; jj++)
					{
						PoleCentralLineCount[i][1][kk][jj] = PoleCentralLineCount[i][1][kk-1][jj];
					}

					for (int jj=0; jj<3; jj++)
					{
						PoleCentralLineCount[i][1][kk-1][jj] = PoleCentralLineCountTemp1[jj];
					}
				}

				if (AngleErr[i][kk] < AngleErr[i][kk-1])
				{
					for (int jj=0; jj<3; jj++)
					{
						PoleCentralLineCountTemp0[jj] = PoleCentralLineCount[i][0][kk][jj];
					}

					for (int jj=0; jj<3; jj++)
					{
						PoleCentralLineCount[i][0][kk][jj] = PoleCentralLineCount[i][0][kk-1][jj];
					}

					for (int jj=0; jj<3; jj++)
					{
						PoleCentralLineCount[i][0][kk-1][jj] = PoleCentralLineCountTemp0[jj];
					}
				}
			}
		}
	}

	ofstream f;
	f.open("F:\\VS2010��RobotController\\testdatarank.txt",ios::app);
	for (int ii=0;ii<PoleNum;ii++)
	{
		for (int k=0; k<20; k++)
		{
			f<<k<<"  "<<"PoleCentreDir "<<PoleCentralLineCount[ii][0][k][0]<<"  "<<PoleCentralLineCount[ii][0][k][1]<<"  "<<PoleCentralLineCount[ii][0][k][2]<<endl;
			f<<"   "<<"PoleCentrePoin "<<PoleCentralLineCount[ii][1][k][0]<<"  "<<PoleCentralLineCount[ii][1][k][1]<<"  "<<PoleCentralLineCount[ii][1][k][2]<<endl;
		}

	}
	f.close();


	//��ȡ�����Ż�ֵ
	for (int i=0; i<PoleNum; i++)
	{
		for (int k=5; k<15; k++)
		{
			PoleCentralLine[i][1][0] = PoleCentralLine[i][1][0] + PoleCentralLineCount[i][1][k][0];
			PoleCentralLine[i][1][1] = PoleCentralLine[i][1][1] + PoleCentralLineCount[i][1][k][1];
			PoleCentralLine[i][1][2] = PoleCentralLine[i][1][2] + PoleCentralLineCount[i][1][k][2];

			PoleCentralLine[i][0][0] = PoleCentralLine[i][0][0] + PoleCentralLineCount[i][0][k][0];
			PoleCentralLine[i][0][1] = PoleCentralLine[i][0][1] + PoleCentralLineCount[i][0][k][1];
			PoleCentralLine[i][0][2] = PoleCentralLine[i][0][2] + PoleCentralLineCount[i][0][k][2];
		}

		//PoleCentralLine[i][1][0] = PoleCentralLine[i][1][0]/(20-ErrTestPointCount[i]);
		//PoleCentralLine[i][1][1] = PoleCentralLine[i][1][1]/(20-ErrTestPointCount[i]);
		//PoleCentralLine[i][1][2] = PoleCentralLine[i][1][2]/(20-ErrTestPointCount[i]);

		//PoleCentralLine[i][0][0] = PoleCentralLine[i][0][0]/(20-ErrTestDirCount[i]);
		//PoleCentralLine[i][0][1] = PoleCentralLine[i][0][1]/(20-ErrTestDirCount[i]);
		//PoleCentralLine[i][0][2] = PoleCentralLine[i][0][2]/(20-ErrTestDirCount[i]);

		PoleCentralLine[i][1][0] = PoleCentralLine[i][1][0]/10;
		PoleCentralLine[i][1][1] = PoleCentralLine[i][1][1]/10;
		PoleCentralLine[i][1][2] = PoleCentralLine[i][1][2]/10;

		PoleCentralLine[i][0][0] = PoleCentralLine[i][0][0]/10;
		PoleCentralLine[i][0][1] = PoleCentralLine[i][0][1]/10;
		PoleCentralLine[i][0][2] = PoleCentralLine[i][0][2]/10;
	}
}

void CToolDlg::RedetectandGrasp()//������void CToolDlg::OnBnClickedBtPrimesensecam()�������ƣ�ֻ��ȥ���˴����̺߳�������
{
	//CPrimeSenseDet ReSetCarmineCheck;
	//bTool1 = m_bTool1;//����1�ļн��ɿ�״̬
	//bTool2 = m_bTool2;//����2�ļн��ɿ�״̬
	////sliderVel = (int)(m_sliderVel.GetPos());
	//int i;
	//RealtimeCon = true;
	////�״μ��
	////PoleInfo *poles_info=NULL;
	//long t1=GetTickCount();
	////PoleNum = m_pCprimeSenseDet->PolesDet(poles_info);

	//ReSetCarmineCheck.CamineCheck = TRUE;//GU

	//PoleNum = m_pCprimeSenseDet->PolesDet();
	//long t2=GetTickCount();
	//long DetTime = t2-t1;

	//for (int pn=0; pn<PoleNum; pn++)              //GU
	//{
	//	for (int m=0; m<2; m++)
	//	{
	//		for (int k=0; k<3; k++)
	//		{
	//			PoleCentralLine[pn][m][k] = PCentralLineTemp[pn][m][k];
	//		}
	//	}
	//	PoleLen[pn] = PoleLenghtTemp[pn];
	//}

	///////////////////////////////////////////////
	////�������ת������������ϵ��
	////������ת��
	//for (i=0;i<PoleNum;i++)
	//{
	//	PoleStyle[i] = Round;
	//	PointCoordinateC(&CPrimeSenseDet::PUntifyMtx,PoleCentralLine[i][1],PoleCentralLine[i][1]);
	//	PointCoordinateC(&CPrimeSenseDet::PUntifyMtx,SConf[i],SConf[i]);
	//}
	////��������任����Ҫ��ת������ľ���ֵ�����
	//for (i=0;i<PoleNum;i++)
	//{
	//	VectorCoordinateC(&CPrimeSenseDet::PUntifyMtx,PoleCentralLine[i][0],PoleCentralLine[i][0]);
	//}
	double dposenddifval[6] = {0}, dPos1[6];
	/*int RedetSucCount = 0;//���¼������һ�μ����ͬ�ĳɹ�������>=3ʱ��������ץ��*/
	//RedetPole.RedetectPole();

	OnBnClickedBtPrimesensecam();

	for (int i=0;i<6;i++)
	{
		dposenddifval[i] = ((abs(dposend[i]-dposendtemp[i])>0.2) ? 1 : 0);
	}
	if (dposenddifval[0]+dposenddifval[1]+dposenddifval[2]+dposenddifval[3]+dposenddifval[4] == 0)//dposΪ�ؽ�ת��
	{
		RedetSucCount++;
		ReDetection = true;
	} 
	else
	{
		RedetSucCount = 0;
		ReDetection = false;
	}
	if (RedetSucCount==2)
	{
		Robot::I_Monit.Get_JointPos(dPos1);

		
		//�˶�
		int vel =sliderVel;
		CMotionData data;
		data.Line = 1;
		data.Mode = MODE_INTERP;
		data.Interp.Mode = INTERP_JOINT; //PTP
		data.Interp.IfLinkage = 0;//��������־
		for(int i=0; i<JOINT_NUM; i++)
		{
			data.Interp.JEnd[i] = dposend[i]; //deg
			data.Interp.JStart[i] = dPos1[i];//deg  GU
		}
		data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 80; //deg/s
		data.Interp.Acc = MAX_AUTOACC_JOINT;
		data.Interp.Jerk = MAX_JERK_JOINT;
		// 
		Robot::I_Task.AddMotionData(data);	
		Robot::I_Task.Set_GivenPosRunMode(TRUE); // ����λ��ʾ��ģʽ
		RedetSucCount = 0;
		Sleep(8000);
		KillTimer(7);
	}
}


void CToolDlg::PoleGraspArea(void)
{
	//for (int j=-200;j<200;j++)
	//{

	int i,NearestPoleN=0; 
	double currpos[6], TagGripDir[3],TagGripP[3], PoleCentralLine[2][3], n[3],o[3],a[3],t,SConf[3];
	PoleCentralLine[0][0]=1;
	PoleCentralLine[0][1]=1;
	PoleCentralLine[0][2]=0;
	PoleCentralLine[1][0]=350;
	PoleCentralLine[1][1]=-200;
	PoleCentralLine[1][2]=0;
	// 	PoleCentralLine[1][1]=j;
	// 	PoleCentralLine[1][2]=sqrt((double)(40000-j*j));
	a[0]=0;
	a[1]=0;
	a[2]=-1;
	for (i=0;i<3;i++)
	{
		TagGripDir[i] = PoleCentralLine[0][i];
	}
	//���ӵ�n������˼�������������
	for (i=0;i<3;i++)
	{
		n[i] = TagGripDir[i];
	}
	VectorUnitizationD(n);
	//���ݷ��˼г�λ���ص㣬�������a��o;
	// 	t = ( PoleCentralLine[0][0]*(SConf[0]-PoleCentralLine[1][0])
	// 		+PoleCentralLine[0][1]*(SConf[1]-PoleCentralLine[1][1])
	// 		+PoleCentralLine[0][2]*(SConf[2]-PoleCentralLine[1][2]) )
	// 		/(PoleCentralLine[0][0]*PoleCentralLine[0][0]
	// 	+PoleCentralLine[0][1]*PoleCentralLine[0][1]
	// 	+PoleCentralLine[0][2]*PoleCentralLine[0][2] );
	// 	//ȷ��a
	// 	for (i=0;i<3;i++)
	// 	{
	// 		a[i] = PoleCentralLine[1][i]+t*PoleCentralLine[0][i]-SConf[i];
	// 	}
	VectorUnitizationD(a);
	//ȷ��n��a�ķ���
	MtxKine CoCM;   //��ǰ������λ���µ�����任����
	Trans_PosToMtx(currpos, &CoCM, 0);
	if ((n[0]*CoCM.R11+n[1]*CoCM.R21+n[2]*CoCM.R31)<0) //��������˵��n����ӵ�n�нǴ���90��
	{
		for (i=0;i<3;i++)
		{
			n[i] = -n[i];
		}
	}
	if ((a[0]*CoCM.R13+a[1]*CoCM.R23+a[2]*CoCM.R33)<0) //��������˵��a����ӵ�a�нǴ���90��
	{
		for (i=0;i<3;i++)
		{
			a[i] = -a[i];
		}
	}

	o[0] = a[1]*n[2]-a[2]*n[1];
	o[1] = n[0]*a[2]-n[2]*a[0];
	o[2] = a[0]*n[1]-a[1]*n[0];
	VectorUnitizationD(o);
	if ((o[0]*CoCM.R12+o[1]*CoCM.R22+o[2]*CoCM.R32)<0) //��������˵��o����ӵ�o�нǴ���90��
	{
		for (i=0;i<3;i++)
		{
			o[i] = -o[i];
		}
	}
	//���������ƽ��m��ֵ
	double m[3];
	m[0] = a[1];
	m[1] = -a[0];
	m[2] = 0;
	////////////////////////////////////////////////////
	//����Ŀ��˼��ϵ�Ŀ��гֵ㣬�õ��ʾ��������ֱ��m
	t = -(PoleCentralLine[1][0]*m[0]+PoleCentralLine[1][1]*m[1]+
		PoleCentralLine[1][2]*m[2])/(PoleCentralLine[0][0]*m[0]+
		PoleCentralLine[0][1]*m[1]+PoleCentralLine[0][2]*m[2]);
	ofstream f;
	f.open("F:\\VS2010��RobotController\\data.txt",ios::app);
	f<<"DetTime: "<<t<<endl;
	f.close();
	for (i=0;i<3;i++)
	{
		TagGripP[i] = PoleCentralLine[1][i] + t*PoleCentralLine[0][i];
	}
	int sss=0;
	//	}
}

void CToolDlg::OnBnClickedButton6()
{
	//PoleGraspArea();
	long t1=GetTickCount();
    m_pCprimeSenseDet->DetectPoleEnvInImage();
//	LocalEnvProcess();
	//a = PolesDet(r,poleline,che,dim,len,bend,s);
	long t2=GetTickCount();
	long DetTime = t2-t1;
	int a=0;
	// TODO: �ڴ���ӿؼ�֪ͨ����������
}


void CToolDlg::OnBnClickedButtonRobotstop()//���ü�ͣ���������ķ�ʽ����
{
	// TODO: Add your control notification handler code here
	//int i;
	//for (i=0; i<6; i++)
	//	m_nTeachVel[i] = 0;
	KillTimer(7);
	RealtimeCon = 0;
	Robot::I_Task.Stop();         // ֹͣ����
	Robot::I_Task.Stop_Teach();   // ֹͣʾ��
	
}


void CToolDlg::OnBnClickedButtonRobotcontinue()
{
	// TODO: Add your control notification handler code here
	//�����˶�
	double dPos1[6];
	Robot::I_Task.ClearError();
	sliderVel = MAX_TEACHVEL_JOINT*(int)(m_sliderVel.GetPos())/100;
	
// 	if (1 == RealtimeCon)
// 	{
// 
// 	}
// 	else
// 	{
// 		double currentpos[6];
// 		Robot::I_Monit.Get_JointPos(currentpos);
// 		int vel =sliderVel;
// 		CMotionData data;
// 		data.Line = 1;
// 		data.Mode = MODE_INTERP;
// 		data.Interp.Mode = INTERP_JOINT; //PTP
// 		data.Interp.IfLinkage = 0;//��������־
// 		for(int i=0; i<JOINT_NUM; i++)
// 		{
// 			data.Interp.JEnd[i] = jpostarget[i]; //deg
// 			data.Interp.JStart[i] = currentpos[i];//deg  GU
// 		}
// 		data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 80; //deg/s
// 		data.Interp.Acc = MAX_AUTOACC_JOINT;
// 		data.Interp.Jerk = MAX_JERK_JOINT;
// 		// 
// 		Robot::I_Task.AddMotionData(data);	
// 		Robot::I_Task.Set_GivenPosRunMode(TRUE); // ����λ��ʾ��ģʽ
// 		//Sleep(8000);
// 	}
// 	if (RedetSucCount==3)
// 	{
// 		Robot::I_Monit.Get_JointPos(dPos1);
// 
// 
// 		//�˶�
// 		int vel =sliderVel;
// 		CMotionData data;
// 		data.Line = 1;
// 		data.Mode = MODE_INTERP;
// 		data.Interp.Mode = INTERP_JOINT; //PTP
// 		data.Interp.IfLinkage = 0;//��������־
// 		for(int i=0; i<JOINT_NUM; i++)
// 		{
// 			data.Interp.JEnd[i] = dposend[i]; //deg
// 			data.Interp.JStart[i] = dPos1[i];//deg  GU
// 		}
// 		data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 80; //deg/s
// 		data.Interp.Acc = MAX_AUTOACC_JOINT;
// 		data.Interp.Jerk = MAX_JERK_JOINT;
// 		// 
// 		Robot::I_Task.AddMotionData(data);	
// 		Robot::I_Task.Set_GivenPosRunMode(TRUE); // ����λ��ʾ��ģʽ
// 		RedetSucCount = 0;
// 		Sleep(8000);
// 		KillTimer(7);
// 	} 
// 	else
// 	{
// 		SetTimer(7,1000,NULL);
// 	}

	if (m_bReachTarget = false)
	{
		ReDetection = false;
	}
	else
	{
		ReDetection = true;
	}
	SetTimer(7,2000,NULL);
}






void CToolDlg::OnBnClickedButtonBroll()
{
	// TODO: Add your control notification handler code here
	CMotionData data;
	double s31=180;
	double s35=180; 
	double s41=180; 
	double s45=180;
	m_MotionChoose = 2;

	if (BST_CHECKED == IsDlgButtonChecked(IDC_CHECK_ROLLDIR))
	{
		SetDlgItemText(IDC_CHECK_ROLLDIR, "����");
		s31 = 180;
		s41 = 180;
		s35 = 180;
		s45 = 180;

	} 
	else if(BST_UNCHECKED == IsDlgButtonChecked(IDC_CHECK_ROLLDIR))
	{
		SetDlgItemText(IDC_CHECK_ROLLDIR, "����");
		s31 = -180;
		s41 = -180;
		s35 = -180;
		s45 = -180;
	}

	//((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(false);
	((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(false);
	((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(false);
	((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(false);
	((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(false);
	((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(false);

	if (BST_CHECKED == IsDlgButtonChecked(IDC_CHECK_ROLLDIR))
	{
		SetDlgItemText(IDC_CHECK_ROLLDIR, "����");
		s31 = 180;
		s41 = 180;
		s35 = 180;
		s45 = 180;

	} 
	else if(BST_UNCHECKED == IsDlgButtonChecked(IDC_CHECK_ROLLDIR))
	{
		SetDlgItemText(IDC_CHECK_ROLLDIR, "����");
		s31 = -180;
		s41 = -180;
		s35 = -180;
		s45 = -180;
	}

	double tPos2[ROLLSTEPS][MAX_AXIS_NUM] = //G2�н�ʱ��ʾ�̵�
	{
		{0,	     0,	    0,	   0,	    0,	  0},	//��λλ��
		{0,		30,	  -40,	  10,		0,	  0},	//���߼����뿪�˼�
		{s31,	30,	  -40,	  10,	    0,	  0},	//��ת�˶�
		{s41,	 0,  	0,     0,	    0,	  0},	//�гָ˼�
		//{0,	 0,  	0,     0,	 -180,	  0},
	};
	double tPos1[ROLLSTEPS][MAX_AXIS_NUM] = //G1�н�ʱ��ʾ�̵�
	{
		{0,	      0,	 0,	   0,	    0,	  0},	//��λλ��
		{0,		 10,   -40,	  30,		0,	  0},	//���߼����뿪�˼�
		{0,      10,   -40,	  30,	  s35,	  0},	//��ת�˶�
		{0,	      0,  	 0,    0,	  s45,	  0},	//�гָ˼�
		//{180,	  0,  	 0,    0,	  180,	  0},   //�ò������˲����˶���ֻ�������������Ĳ�Ҫ��
	};

	if ((m_bTool1) && (!m_bTool2) && FLAG_ERROR != Robot::I_Task.Get_RunState() && TRUE == m_bg1tog2)         //GU160519ʹ�ڲ�ͬ���Ӽн�����¾�����ɲ�̬�˶�
	{
		KillTimer(8);
		if (stepcount < ROLLSTEPS)
		{
			double JCurrPos[MAX_AXIS_NUM];
			Robot::I_Monit.Get_JointPos(JCurrPos);
			// �˶�
			int vel = (int)(m_sliderVel.GetPos());
			data.Line = 1;
			data.Mode = MODE_INTERP;
			data.Interp.Mode = INTERP_JOINT; //PTP
			data.Interp.IfLinkage = 0;
			for (int j=0; j<MAX_AXIS_NUM; j++)
			{
				data.Interp.JStart[j]=JCurrPos[j];
				data.Interp.JEnd[j] = tPos2[stepcount][j]; //deg
			}
			data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
			data.Interp.Acc = MAX_AUTOACC_JOINT;
			data.Interp.Jerk = MAX_JERK_JOINT;

			Robot::I_Task.AddMotionData(data);	
			Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

			//	while (Robot::I_Task.Get_RunState() == 1);

			stepcount++;
			if (stepcount == ROLLSTEPS)
			{
				int flag;
				//double dPos[MAX_AXIS_NUM] = {0,0,0,0,0,0};
				((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(true);


				for (int i=0; i<6; i++)
				{
					tPos_temp[i] = tPos2[3][i];

				}
				SetTimer(8,500,NULL);
				m_bg1tog2 = FALSE;
				stepcount = 0;
				//	m_MotionChoose = 0;
				return;
			}
		}
	} 
	else if((!m_bTool1) && (m_bTool2) && FLAG_ERROR != Robot::I_Task.Get_RunState() && FALSE == m_bg1tog2)
	{
		KillTimer(8);
		if (stepcount < ROLLSTEPS)
		{

			// �˶�
			int vel = (int)(m_sliderVel.GetPos());
			data.Line = 1;
			data.Mode = MODE_INTERP;
			data.Interp.Mode = INTERP_JOINT; //PTP
			data.Interp.IfLinkage = 0;
			for (int j=0; j<MAX_AXIS_NUM; j++)
				data.Interp.JEnd[j] = tPos1[stepcount][j]; //deg
			data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
			data.Interp.Acc = MAX_AUTOACC_JOINT;
			data.Interp.Jerk = MAX_JERK_JOINT;

			Robot::I_Task.AddMotionData(data);	
			Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

			//	while (Robot::I_Task.Get_RunState() == 1);

			stepcount++;
			if (stepcount == ROLLSTEPS)
			{
				int flag;
				double dPos[MAX_AXIS_NUM] = {0,0,0,0,0,0};

				((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(true);

				for (int i=0; i<JOINT_NUM; i++)
				{
					for (int i=0; i<6; i++)
					{
						tPos_temp[i] = tPos1[3][i];
					}
					SetTimer(8,500,NULL);
					m_bg1tog2 = TRUE;
					stepcount = 0;
					//	m_MotionChoose = 0;
					return;
				}
			}

		}
	}
	else
	{
		((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(true);
		AfxMessageBox("����ִ�иò�̬");
		//	m_MotionChoose = 0;
		stepcount = 0;
		return;
	}
}


void CToolDlg::OnBnClickedButtonBturn()
{
	// TODO: Add your control notification handler code here
	CMotionData data;
	m_MotionChoose = 4;

	((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(false);
	((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(false);
	((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(false);
	((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(false);
	((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(false);
	((CButton*)GetDlgItem(IDC_CHECK_ROLLDIR))->EnableWindow(false);

	double tPos[ROLLSTEPS][MAX_AXIS_NUM] = //G2�н�ʱ��ʾ�̵�
	{
		{0,	     0,	    0,	   0,	    0,	  0},	//��λλ��
		{0,		20,	   -10,	  20,		0,	  0},	//���߼����뿪�˼�
		{0,	   160,	   10,	 160,	    0,	  0},	//��ת�˶�
		{0,	   180,  	0,   180,	    0,	  0},	//�гָ˼�
	};

	if ((m_bTool1) && (!m_bTool2) && FLAG_ERROR != Robot::I_Task.Get_RunState() && TRUE == m_bg1tog2)         //GU160519ʹ�ڲ�ͬ���Ӽн�����¾�����ɲ�̬�˶�
	{
		KillTimer(8);
		if (stepcount <= ROLLSTEPS)
		{
			// 			double JCurrPos[MAX_AXIS_NUM];
			// 			Robot::I_Monit.Get_JointPos(JCurrPos);
			// �˶�
			int vel = (int)(m_sliderVel.GetPos());
			data.Line = 1;
			data.Mode = MODE_INTERP;
			data.Interp.Mode = INTERP_JOINT; //PTP
			data.Interp.IfLinkage = 0;
			for (int j=0; j<MAX_AXIS_NUM; j++)
			{
				data.Interp.JEnd[j] = tPos[stepcount][j]; //deg
				//data.Interp.JStart[j]=JCurrPos[j];
			}
			data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
			data.Interp.Acc = MAX_AUTOACC_JOINT;
			data.Interp.Jerk = MAX_JERK_JOINT;

			Robot::I_Task.AddMotionData(data);	
			Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

			stepcount++;
			if (stepcount == ROLLSTEPS)
			{
				for (int i=0; i<6; i++)
				{
					tPos_temp[i] = tPos[3][i];

				}
				SetTimer(8,500,NULL);
				m_bg1tog2 = FALSE;
				stepcount = 0;
				//m_MotionChoose = 0;

				//if (m_bTurn==FALSE)
				//{
				//	m_bTurn = TRUE;
				//} 
				//else
				//{
				//	m_bTurn = FALSE;
				//}
				return;
			}
		}
	} 
	else if((!m_bTool1) && (m_bTool2) && FLAG_ERROR != Robot::I_Task.Get_RunState() && FALSE == m_bg1tog2)
	{
		KillTimer(8);
		if (stepcount <= ROLLSTEPS)
		{
			// �˶�
			int vel = (int)(m_sliderVel.GetPos());
			data.Line = 1;
			data.Mode = MODE_INTERP;
			data.Interp.Mode = INTERP_JOINT; //PTP
			data.Interp.IfLinkage = 0;
			for (int j=0; j<MAX_AXIS_NUM; j++)
				data.Interp.JEnd[j] = tPos[stepcount][j]; //deg
			data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
			data.Interp.Acc = MAX_AUTOACC_JOINT;
			data.Interp.Jerk = MAX_JERK_JOINT;

			Robot::I_Task.AddMotionData(data);	
			Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

			//	while (Robot::I_Task.Get_RunState() == 1);

			stepcount++;
			if (stepcount == ROLLSTEPS)
			{
				for (int i=0; i<6; i++)
				{
					tPos_temp[i] = tPos[3][i];

				}
				SetTimer(8,500,NULL);
				m_bg1tog2 = TRUE;
				stepcount = 0;
				//m_MotionChoose = 0;

				//if (m_bTurn==FALSE)
				//{
				//	m_bTurn = TRUE;
				//} 
				//else
				//{
				//	m_bTurn = FALSE;
				//}

				return;
			}
		}

	}
	else
	{
		((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_CHECK_ROLLDIR))->EnableWindow(true);
		AfxMessageBox("����ִ�иò�̬");
		//m_MotionChoose = 0;
		stepcount = 0;
		return;
	}
}


void CToolDlg::OnBnClickedButtonBinchworm()
{
	// TODO: Add your control notification handler code here

	CMotionData data;
	m_MotionChoose = 6;
	KillTimer(8);
	double tPos2[ROLLSTEPS][MAX_AXIS_NUM] = //G2�н�ʱ��ʾ�̵�
	{
		{0,		35,	   -90,	    55,		0,	  0},	
		{0,		55,	  -110,	    55,	    0,	  0},	
		{0,		55,	   -90,	    35,	    0,	  0},	
		{0,		 0,  	 0,      0,	    0,	  0},	
	};
	double tPos1[ROLLSTEPS][MAX_AXIS_NUM] = //G1�н�ʱ��ʾ�̵�
	{
		{0,		55,	   -90,	    35,		0,	  0},	
		{0,		55,	  -110,	    55,	    0,	  0},	
		{0,		35,	   -90,	    55,	    0,	  0},	
		{0,		 0,  	 0,      0,	    0,	  0},
	};

	if ((m_bTool1) && (!m_bTool2) && FLAG_ERROR != Robot::I_Task.Get_RunState() && TRUE == m_bg1tog2 && false==m_bInSecStep)         //GU160519ʹ�ڲ�ͬ���Ӽн�����¾�����ɲ�̬�˶�
	{
		((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(false);
		//((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_CHECK_ROLLDIR))->EnableWindow(false);

		if (stepcount <= ROLLSTEPS)
		{
			// �˶�
			int vel = (int)(m_sliderVel.GetPos());
			data.Line = 1;
			data.Mode = MODE_INTERP;
			data.Interp.Mode = INTERP_JOINT; //PTP
			data.Interp.IfLinkage = 0;
			for (int j=0; j<MAX_AXIS_NUM; j++)
			{
				data.Interp.JEnd[j] = tPos1[stepcount][j]; //deg
			}
			data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
			data.Interp.Acc = MAX_AUTOACC_JOINT;
			data.Interp.Jerk = MAX_JERK_JOINT;

			Robot::I_Task.AddMotionData(data);	
			Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

			stepcount++;
			if (stepcount == 2)
			{
				AfxMessageBox("��պϼг���2���򿪼г���1!");
				m_bInSecStep = true;
			}
		}
	} 
	else if ((!m_bTool1) && (m_bTool2) && FLAG_ERROR != Robot::I_Task.Get_RunState() && TRUE == m_bg1tog2 && (stepcount == 2 || stepcount == 3) && true==m_bInSecStep)
	{
		((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(false);
		if (stepcount <= ROLLSTEPS)
		{
			// �˶�
			int vel = (int)(m_sliderVel.GetPos());
			data.Line = 1;
			data.Mode = MODE_INTERP;
			data.Interp.Mode = INTERP_JOINT; //PTP
			data.Interp.IfLinkage = 0;
			for (int j=0; j<MAX_AXIS_NUM; j++)
			{
				data.Interp.JEnd[j] = tPos1[stepcount][j]; //deg
			}
			data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
			data.Interp.Acc = MAX_AUTOACC_JOINT;
			data.Interp.Jerk = MAX_JERK_JOINT;

			Robot::I_Task.AddMotionData(data);	
			Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

			stepcount++;
			if (stepcount == ROLLSTEPS)
			{
				((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_CHECK_ROLLDIR))->EnableWindow(true);
				stepcount = 0;
				//m_MotionChoose = 0;
				m_bInSecStep = false;
				return;
			}
		}
	}
	else if((!m_bTool1) && (m_bTool2) && FLAG_ERROR != Robot::I_Task.Get_RunState() && FALSE == m_bg1tog2 && false==m_bInSecStep)
	{
		((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(false);
		//((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_CHECK_ROLLDIR))->EnableWindow(false);
		if (stepcount <= ROLLSTEPS)
		{
			// �˶�
			int vel = (int)(m_sliderVel.GetPos());
			data.Line = 1;
			data.Mode = MODE_INTERP;
			data.Interp.Mode = INTERP_JOINT; //PTP
			data.Interp.IfLinkage = 0;
			for (int j=0; j<MAX_AXIS_NUM; j++)
				data.Interp.JEnd[j] = tPos2[stepcount][j]; //deg
			data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
			data.Interp.Acc = MAX_AUTOACC_JOINT;
			data.Interp.Jerk = MAX_JERK_JOINT;

			Robot::I_Task.AddMotionData(data);	
			Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

			//	while (Robot::I_Task.Get_RunState() == 1);

			stepcount++;
			if (stepcount == 2)
			{
				AfxMessageBox("��պϼг���1���򿪼г���2!");
				//((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(false);
				m_bInSecStep = true;
			}
		}

	}
	else if ((m_bTool1) && (!m_bTool2) && FLAG_ERROR != Robot::I_Task.Get_RunState() && FALSE == m_bg1tog2 && (stepcount == 2 || stepcount == 3) && true==m_bInSecStep)
	{
		((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(false);
		((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(false);
		if (stepcount <= ROLLSTEPS)
		{
			// �˶�
			int vel = (int)(m_sliderVel.GetPos());
			data.Line = 1;
			data.Mode = MODE_INTERP;
			data.Interp.Mode = INTERP_JOINT; //PTP
			data.Interp.IfLinkage = 0;
			for (int j=0; j<MAX_AXIS_NUM; j++)
			{
				data.Interp.JEnd[j] = tPos2[stepcount][j]; //deg
			}
			data.Interp.Vel = MAX_TEACHVEL_JOINT * vel / 100; //deg/s
			data.Interp.Acc = MAX_AUTOACC_JOINT;
			data.Interp.Jerk = MAX_JERK_JOINT;

			Robot::I_Task.AddMotionData(data);	
			Robot::I_Task.Set_GivenPosRunMode(true); // ����λ��ʾ��ģʽ

			stepcount++;
			if (stepcount == ROLLSTEPS)
			{
				((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(true);
				((CButton*)GetDlgItem(IDC_CHECK_ROLLDIR))->EnableWindow(true);
				stepcount = 0;
				//m_MotionChoose = 0;
				m_bInSecStep = false;
				return;
			}
		}
	}
	else
	{
		((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(true);
		((CButton*)GetDlgItem(IDC_CHECK_ROLLDIR))->EnableWindow(true);
		AfxMessageBox("����ִ�иò�̬");
		//m_MotionChoose = 0;
		//if (false==m_bInSecStep)
		//{
		//	stepcount = 0;
		//} 
		//else
		//{
		//	stepcount = 2;
		//}
		m_bInSecStep = false;
		stepcount = 0;


		return;
	}
}


void CToolDlg::OnBnClickedButtonStop()
{
	// TODO: Add your control notification handler code here
	m_Stop = true;
	Robot::I_Task.Stop();         // ֹͣ����
	Robot::I_Task.Stop_Teach();   // ֹͣʾ��
}


void CToolDlg::OnBnClickedButtonContinue()
{
	// TODO: Add your control notification handler code here
	Robot::I_Task.ClearError();
	if (stepcount != 0 && m_MotionChoose!=5 && m_MotionChoose!=6)
	{
		stepcount = stepcount-1;
	}
	else
	{

	}

	if (m_Stop)
	{	
		if (m_MotionChoose==1)
		{
			if (stepcount!=0)
			{
				CToolDlg::OnButtonRoll();
			}
			else
			{
				stepcount = 3;
				if (m_bg1tog2==true)
				{
					m_bg1tog2 = false;
				}
				else
				{
					m_bg1tog2 = true;
				}
				CToolDlg::OnButtonRoll();
			}

		}
		else if(m_MotionChoose==2)
		{
			if (stepcount!=0)
			{
				CToolDlg::OnBnClickedButtonBroll();
			}
			else
			{
				stepcount = 3;
				if (m_bg1tog2==true)
				{
					m_bg1tog2 = false;
				}
				else
				{
					m_bg1tog2 = true;
				}
				CToolDlg::OnBnClickedButtonBroll();
			}
		}
		else if (m_MotionChoose==3)
		{
			if (stepcount!=0)
			{
				CToolDlg::OnButtonTurn();
			}
			else
			{
				stepcount = 3;
				if (m_bg1tog2==true)
				{
					m_bg1tog2 = false;
				}
				else
				{
					m_bg1tog2 = true;
				}
				CToolDlg::OnButtonTurn();
			}
		}
		else if (m_MotionChoose==4)
		{
			if (stepcount!=0)
			{
				CToolDlg::OnBnClickedButtonBturn();
			}
			else
			{
				stepcount = 3;
				if (m_bg1tog2==true)
				{
					m_bg1tog2 = false;
				}
				else
				{
					m_bg1tog2 = true;
				}
				CToolDlg::OnBnClickedButtonBturn();
			}
		}
		else if (m_MotionChoose==5)
		{
			if (stepcount!=0)
			{
				stepcount = stepcount-1;
				CToolDlg::OnButtonInchworm();
			}
			else
			{
				stepcount = 3;
				CToolDlg::OnButtonInchworm();
			}
		}
		else if (m_MotionChoose==6)
		{
			if (stepcount!=0)
			{
				stepcount = stepcount-1;
				CToolDlg::OnBnClickedButtonBinchworm();
			}
			else
			{
				stepcount = 3;
				CToolDlg::OnBnClickedButtonBinchworm();
			}
		}
		else
		{
			((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(true);
			((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(true);
			((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(true);
			((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(true);
			((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(true);
			((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(true);
			((CButton*)GetDlgItem(IDC_CHECK_ROLLDIR))->EnableWindow(true);
			AfxMessageBox("�޷������˶���");
		}
		m_Stop = false;
	}
	else
	{
	}
}


void CToolDlg::OnBnClickedButtonReinit()
{
	// TODO: Add your control notification handler code here
	stepcount = 0;
	((CButton*)GetDlgItem(IDC_BUTTON_ROLL))->EnableWindow(true);
	((CButton*)GetDlgItem(IDC_BUTTON_BROLL))->EnableWindow(true);
	((CButton*)GetDlgItem(IDC_BUTTON_TURN))->EnableWindow(true);
	((CButton*)GetDlgItem(IDC_BUTTON_BTURN))->EnableWindow(true);
	((CButton*)GetDlgItem(IDC_BUTTON_INCHWORM))->EnableWindow(true);
	((CButton*)GetDlgItem(IDC_BUTTON_BINCHWORM))->EnableWindow(true);
	((CButton*)GetDlgItem(IDC_CHECK_ROLLDIR))->EnableWindow(true);
	AfxMessageBox("ע��鿴Tģ��Ĺؽڷ���");
}

void CToolDlg::ChangeDir()
{
	CString str;
	double dRat[MAX_AXIS_NUM];
	int iDir[MAX_AXIS_NUM];

	if (1 == JOINT_NUM)
	{
		str.Format("%.3f", fabs(Robot::motorRat[0]));
		dRat[0] = atof(str);
	}
	else
	{
		str.Format("%.3f", fabs(Robot::motorRat[0]));
		dRat[0] = atof(str);
		str.Format("%.3f", fabs(Robot::motorRat[1]));
		dRat[1] = atof(str);
		str.Format("%.3f", fabs(Robot::motorRat[2]));
		dRat[2] = atof(str);
		str.Format("%.3f", fabs(Robot::motorRat[3]));
		dRat[3] = atof(str);
		str.Format("%.3f", fabs(Robot::motorRat[4]));
		dRat[4] = atof(str);	
	}
	if (6 == JOINT_NUM)
	{
		str.Format("%.3f", fabs(Robot::motorRat[5]));
		dRat[5] = atof(str);
	}

	int i;
	for(i=0; i<JOINT_NUM; i++)
	{
		if (dRat[i] <= 0)
		{
			AfxMessageBox("������������");
			return;
		}
	}

	if (1 == JOINT_NUM)
	{
		iDir[0] = -1;
	}
	else
	{
		iDir[0] = -1;
		iDir[1] = -1;
		iDir[2] = -1;
		iDir[3] = -1;
		iDir[4] = -1;
	}
	if (6 == JOINT_NUM)
	{
		iDir[5] = -1;
	}


	for(i=0; i<JOINT_NUM; i++)
	{
		dRat[i] *= iDir[i];
	}

	RobotFile robotfile;
	robotfile.Set_MotorRat(dRat);

}
