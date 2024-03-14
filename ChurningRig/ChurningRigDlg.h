
// ChurningRigDlg.h : header file
//

#pragma once

#include "SerialMFC.h"
#include "CWalkDlg.h"
#include <vector>
#include <string> 
#include <chrono>
#include <fstream>
#include <string>
#include <sstream>

#ifdef _UNICODE
#define tstring std::wstring
#else
#define tstring std::string
#endif

#define NUM_THERMOCOUPLES 7

class Dataline {
public:
	Dataline(int time_elapsed, std::chrono::system_clock::time_point reset_time, float speed, float temperature1, float voltage, float current, float power, float torque, float frequency, float opspeed, CString data, float temparr[], long loadarr[]);
	virtual ~Dataline();

	std::chrono::system_clock::time_point timestamp;
	int time_elapsed;
	float speed, temperature1;
	float voltage, current, power, torque, frequency, opspeed;
	float temps[NUM_THERMOCOUPLES];
	long loads[4];
	long torquelsb;
	CString data;
	CString timeString(std::chrono::system_clock::time_point zero_time);
	CString Dataline::timeString(BOOL date = TRUE, BOOL milliseconds = FALSE);
	CString Dataline::DataString(float ff);
	CString Dataline::DataString(int dd);
	CString Dataline::DataString(long dd);
};


// CChurningRigDlg dialog
class CChurningRigDlg : public CDialogEx
{
// Construction
public:
	CChurningRigDlg(CWnd* pParent = nullptr);	// standard constructor
	UINT CChurningRigDlg::runProgram(BOOL * stopPtr);
	BOOL stopWorker;
	double torquecalval, torqueoffset;

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_CHURNINGRIG_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support

// COM
	CSerialMFC mSerial;
	afx_msg LRESULT CChurningRigDlg::OnSerialMsg(WPARAM wParam, LPARAM lParam);
	std::vector< tstring > ports;
	std::vector< tstring > portnames;
	void CChurningRigDlg::DetectComPorts(size_t upperLimit = 128);
	CString selectedPort;
	BOOL incompleteMessage;
	CStringA imStr;

	BOOL recording;
	BOOL init_complete;
	BOOL connected;
	BOOL clutchEn;
	BOOL motorStarted;
	int returnspeed;
	int sync_millis;
	CMenu* m_menu;
	std::vector< Dataline > DataBuffer;
	std::chrono::system_clock::time_point reset_time;
	std::chrono::system_clock::time_point zero_time;
	BOOL next_zero;
	size_t dataExported;
	tstring exportpath;

	enum encoderError : int {
		ENCODER_DIRCHNG = 0x1,
		ENCODER_C_ERR = 0x2,
		L1_ERR = 0x4,
		L2_ERR = 0x8,
		ENCODER_SPI_NOT_RECEIVED = 0x10
	};

	void CChurningRigDlg::AppendText(CEdit& CE, TCHAR* newText);
	void CChurningRigDlg::PrintData();
	void CChurningRigDlg::AddItem(Dataline nl, BOOL ensureVisible = TRUE);
	void CChurningRigDlg::EnableDisableControls(BOOL en);
	void CChurningRigDlg::ClutchToggle();
	void CChurningRigDlg::ClutchToggle(BOOL cEn);
	void CChurningRigDlg::inverterCommsError(char* ptr);
	void CChurningRigDlg::setSpeed(int speed);
	void CChurningRigDlg::ExportData(CString fpath);
	double CChurningRigDlg::CalculateTorque(double val);

	class RigProgram
	{
	public:
		RigProgram(CChurningRigDlg& dg);
		void Initialise(tstring pth);
		tstring file_path;
		tstring file_string;
		CChurningRigDlg& dlg;

		class ProgramLine {
		public:
			ProgramLine(tstring cmd, tstring dat, bool vld, CChurningRigDlg& dg);
			bool ParseStartStop(tstring data);
			bool ParseOnOff(tstring data);
			bool ParseForwardReverse(tstring dat);
			void MotorLineExecute();
			void ClearLineExecute();
			void DirectionLineExecute();
			void RecordingLineExecute();
			void SpeedLineExecute();
			void SpeedFollowLineExecute();
			void ClutchLineExecute();
			void PathLineExecute();
			void ExportLineExecute();
			void WaitLineExecute(BOOL* stopPtr);
			void TorqueCalExecute();
			void TorqueZero();
			tstring command;
			tstring data;
			bool valid;
			CChurningRigDlg& dlg;
		};
		ProgramLine parseLine(tstring line);
		std::vector<ProgramLine> lines;
	};

	RigProgram program;


// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnFileListcomports();
	afx_msg void OnFileSerialterminal();
	afx_msg void OnBnClickedConnect();
	CButton mConnect;
	CStatic mStatus;
	CString mStatusVal;
	BOOL mSensor1Val;
	BOOL mSensor2Val;
	CProgressCtrl mProgress;
	CButton mRecordData;
	CButton mClearData;
	CButton mTimeZero;
	CButton mExportData;
	CStatic mStatus2;
	CString mStatus2Val;
	CButton mClutch;
	CSliderCtrl mSpeedSlider;
	CStatic mMotorText;
	CString mMotorTextVal;
	CStatic mEncoderText;
	CString mEncoderTextVal;
	CButton mDirection;
	BOOL mDirectionVal; // FALSE => forwards
	CButton mBraking;
	BOOL mBrakingVal;
	CButton mMotorStartStop;
	CString mSamplerateVal;
	CString mSamplerateValOld;
	CEdit mSamplerate;
	CStatic mSamplerateText;
	CButton mTempCtrlEn;
	BOOL mTempCtrlEnVal;
	CStatic mSetpoint;
	CStatic mSump;
	CEdit mTempSet;
	CEdit mTempSump;
	CString mTempSetVal;
	CString mTempSetValOld;
	CString mSumpSetVal;
	CString mSumpSetValOld;
	CListCtrl mDataList;
	afx_msg void OnBnClickedDisconnect();
	afx_msg void OnFileExit();
	afx_msg void OnBnClickedRecord();
	//afx_msg void OnBnClickedSensor1();
	//afx_msg void OnBnClickedSensor2();
	afx_msg void OnBnClickedTimezero();
	afx_msg void OnBnClickedCleardata();
	afx_msg void OnHelpAbout();
	afx_msg void OnBnClickedExport();
	afx_msg void OnCancelOverride();
	afx_msg void OnClutch();
	afx_msg void OnBnClickedDirectionBox();
	afx_msg void OnBnClickedBrakingBox();
	afx_msg void OnBnClickedMotorButton();
	afx_msg void OnBnClickedTempctrlCheck();
	afx_msg void OnEnChangeTemperatureEdit();
	afx_msg void OnNMReleasedcaptureMotorSpeedSlider(NMHDR* pNMHDR, LRESULT* pResult);
	afx_msg void OnEnKillfocusSamplerate();
	afx_msg void OnEnChangeSamplerate();
	afx_msg void OnStnClickedEncoderText();
	CButton mSpeedFollow;
	BOOL mSpeedFollowVal;
	afx_msg void OnBnClickedSpeedfollowBox();
	afx_msg void OnProgramRun();
	afx_msg void OnProgramStop();
	afx_msg void OnEnChangeTemperatureSump();
	afx_msg void OnEnKillfocusTemperatureSump();
	afx_msg void OnEnKillfocusTemperatureEdit();
	afx_msg void OnProgramLoad();
};

UINT __cdecl programWrapper(LPVOID pParam);
struct threadInfo {
	CChurningRigDlg* dlgPtr;
	BOOL* termPtr;
};

