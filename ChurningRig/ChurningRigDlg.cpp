
// ChurningRigDlg.cpp : implementation file
//

#include "pch.h"
#include "framework.h"
#include "ChurningRig.h"
#include "ChurningRigDlg.h"
#include "ComPortsDialog.h"
#include "CSerialTermDlg.h"
#include "CWalkDlg.h"
#include "afxdialogex.h"
#include "BinRes.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

//#define COM_CONFIG_ERRORS

#pragma comment(linker,"\"/manifestdependency:type='win32' \
name='Microsoft.Windows.Common-Controls' version='6.0.0.0' \
processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")

Dataline::Dataline(int time_elapsed, std::chrono::system_clock::time_point reset_time, float speed, float temperature1, float  voltage, float current, float power, float torque, float frequency, float opspeed, CString data, float temparr[], long loadarr[])
	: time_elapsed(time_elapsed)
	, speed(speed)
	, temperature1(temperature1)
	, voltage(voltage)
	, current(current)
	, power(power)
	, torque(torque)
	, frequency(frequency)
	, opspeed(opspeed)
{
	timestamp = reset_time + std::chrono::milliseconds( time_elapsed );
	for (int i = 0; i < NUM_THERMOCOUPLES; i++) temps[i] = temparr[i];
	for (int i = 0; i < 4; i++) loads[i] = loadarr[i];
	torquelsb = loads[0] + loads[1] - loads[2] - loads[3];
}

Dataline::~Dataline() {}

CString Dataline::timeString(std::chrono::system_clock::time_point zero_time) {
	CString ts = _T("");
	std::chrono::duration<float> sec = timestamp - zero_time;
	ts.Format(_T("%.1f"), sec.count());
	return ts;
}

CString Dataline::timeString(BOOL date, BOOL milliseconds) {
	CString ts = _T("");
	time_t tt = std::chrono::system_clock::to_time_t(timestamp);
	struct tm tm;
	localtime_s(&tm, &tt);
	std::chrono::system_clock::time_point tp = std::chrono::system_clock::from_time_t(tt);
	std::chrono::duration<double> sec = timestamp - tp + std::chrono::seconds(tm.tm_sec);
	if (date) {
		ts.Format(_T("%02d-%02d-%02d "),
			tm.tm_mday,
			tm.tm_mon + 1,
			tm.tm_year - 100
		);
	}
	ts.AppendFormat(_T("%02d:%02d:"),
		tm.tm_hour,
		tm.tm_min
	);
	ts.AppendFormat(milliseconds?_T("%03.1f"):_T("%02.0f"),
		sec.count()
	);
	return ts;
}

CString Dataline::DataString(float ff) {
	CString ss = _T("");
	if (ff == floor(ff)) {
		ss.Format(_T("%.2f"), ff); // currently redundant but could be used to make floats appear as integers
	}
	else {
		ss.Format(_T("%.2f"), ff);
	}
	return ss;
}

CString Dataline::DataString(int dd) {
	CString ss = _T("");
	ss.Format(_T("%d"), dd);
	return ss;
}

CString Dataline::DataString(long ld) {
	CString ss = _T("");
	ss.Format(_T("%ld"), ld);
	return ss;
}

// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CChurningRigDlg dialog

CChurningRigDlg::CChurningRigDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_CHURNINGRIG_DIALOG, pParent),
	selectedPort(_T(""))
	, mStatusVal(_T(""))
	, mSensor1Val(TRUE)
	, mSensor2Val(FALSE)
	, mStatus2Val(_T(""))
	, recording(TRUE)
	, init_complete(FALSE)
	, connected(FALSE)
	, next_zero(TRUE)
	, dataExported(0)
	, incompleteMessage(FALSE)
	, imStr("")
	, mSamplerateVal(_T("5 Hz"))
	, mSamplerateValOld(_T(""))
	, clutchEn(FALSE)
	, mDirectionVal(FALSE)
	, motorStarted(FALSE)
	, mTempSetVal(_T("40 C"))
	, mTempSetValOld(_T(""))
	, mSumpSetVal(_T("60 C"))
	, mSumpSetValOld(_T(""))
	, mMotorTextVal(_T("Motor speed: 300 rpm"))
	, mSpeedFollowVal(FALSE)
	, returnspeed(300)
	, mBrakingVal(FALSE)
	, mTempCtrlEnVal(FALSE)
	, sync_millis(0)
	, stopWorker(FALSE)
	, program(*this)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDI_IMPERIAL);
	torquecalval = 2.47E-5;
	torqueoffset = 0;
}

double CChurningRigDlg::CalculateTorque(double val)
{
	return val * torquecalval + torqueoffset;
}

void CChurningRigDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_STATUS, mStatus);
	DDX_Text(pDX, IDC_STATUS, mStatusVal);
	DDX_Control(pDX, IDC_PROGRESS, mProgress);
	DDX_Control(pDX, IDC_CONNECT, mConnect);
	//DDX_Control(pDX, IDC_SENSOR1, mSensor1);
	//DDX_Check(pDX, IDC_SENSOR1, mSensor1Val);
	//DDX_Control(pDX, IDC_SENSOR2, mSensor2);
	//DDX_Check(pDX, IDC_SENSOR2, mSensor2Val);
	DDX_Control(pDX, IDC_RECORD, mRecordData);
	DDX_Control(pDX, IDC_CLEARDATA, mClearData);
	DDX_Control(pDX, IDC_TIMEZERO, mTimeZero);
	DDX_Control(pDX, IDC_EXPORT, mExportData);
	DDX_Control(pDX, IDC_STATUS2, mStatus2);
	DDX_Text(pDX, IDC_STATUS2, mStatus2Val);
	DDX_Control(pDX, IDC_DATALIST, mDataList);
	DDX_Control(pDX, IDC_CLUTCH_BUTTON, mClutch);
	DDX_Control(pDX, IDC_MOTOR_SPEED_SLIDER, mSpeedSlider);
	DDX_Control(pDX, IDC_MOTOR_TEXT, mMotorText);
	DDX_Text(pDX, IDC_MOTOR_TEXT, mMotorTextVal);
	DDX_Control(pDX, IDC_ENCODER_TEXT, mEncoderText);
	DDX_Text(pDX, IDC_ENCODER_TEXT, mEncoderTextVal);
	DDX_Control(pDX, IDC_DIRECTION_BOX, mDirection);
	DDX_Control(pDX, IDC_BRAKING_BOX, mBraking);
	DDX_Control(pDX, IDC_MOTOR_BUTTON, mMotorStartStop);
	DDX_Control(pDX, IDC_TEMPCTRL_CHECK, mTempCtrlEn);
	DDX_Control(pDX, IDC_SETPOINT_TEXT, mSetpoint);
	DDX_Control(pDX, IDC_SUMP_TEXT, mSump);
	DDX_Control(pDX, IDC_TEMPERATURE_EDIT, mTempSet);
	DDX_Control(pDX, IDC_TEMPERATURE_SUMP, mTempSump);
	DDX_Text(pDX, IDC_SAMPLERATE, mSamplerateVal);
	DDX_Control(pDX, IDC_SAMPLERATE, mSamplerate);
	DDX_Control(pDX, IDC_SAMPLERATE_TEXT, mSamplerateText);
	DDX_Text(pDX, IDC_TEMPERATURE_EDIT, mTempSetVal);
	DDX_Text(pDX, IDC_TEMPERATURE_SUMP, mSumpSetVal);
	DDX_Control(pDX, IDC_SPEEDFOLLOW_BOX, mSpeedFollow);
}

BEGIN_MESSAGE_MAP(CChurningRigDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_SERIAL(OnSerialMsg)
	ON_COMMAND(ID_FILE_LISTCOMPORTS, &CChurningRigDlg::OnFileListcomports)
	ON_COMMAND(ID_FILE_SERIALTERMINAL, &CChurningRigDlg::OnFileSerialterminal)
	ON_BN_CLICKED(IDC_CONNECT, &CChurningRigDlg::OnBnClickedConnect)
	ON_COMMAND(ID_FILE_EXIT, &CChurningRigDlg::OnFileExit)
	ON_BN_CLICKED(IDC_RECORD, &CChurningRigDlg::OnBnClickedRecord)
	//ON_BN_CLICKED(IDC_SENSOR1, &CChurningRigDlg::OnBnClickedSensor1)
	//ON_BN_CLICKED(IDC_SENSOR2, &CChurningRigDlg::OnBnClickedSensor2)
	ON_BN_CLICKED(IDC_TIMEZERO, &CChurningRigDlg::OnBnClickedTimezero)
	ON_BN_CLICKED(IDC_CLEARDATA, &CChurningRigDlg::OnBnClickedCleardata)
	ON_COMMAND(ID_HELP_ABOUT, &CChurningRigDlg::OnHelpAbout)
	ON_COMMAND(ID_DATA_CLEAR, &CChurningRigDlg::OnBnClickedCleardata)
	ON_BN_CLICKED(IDC_EXPORT, &CChurningRigDlg::OnBnClickedExport)
	ON_COMMAND(IDCANCEL, &CChurningRigDlg::OnCancelOverride)
	ON_COMMAND(ID_DATA_EXPORT, &CChurningRigDlg::OnBnClickedExport)
	ON_BN_CLICKED(IDC_CLUTCH_BUTTON, &CChurningRigDlg::OnClutch)
	ON_BN_CLICKED(IDC_DIRECTION_BOX, &CChurningRigDlg::OnBnClickedDirectionBox)
	ON_BN_CLICKED(IDC_BRAKING_BOX, &CChurningRigDlg::OnBnClickedBrakingBox)
	ON_BN_CLICKED(IDC_MOTOR_BUTTON, &CChurningRigDlg::OnBnClickedMotorButton)
	ON_BN_CLICKED(IDC_TEMPCTRL_CHECK, &CChurningRigDlg::OnBnClickedTempctrlCheck)
	ON_EN_CHANGE(IDC_TEMPERATURE_EDIT, &CChurningRigDlg::OnEnChangeTemperatureEdit)
	ON_NOTIFY(NM_RELEASEDCAPTURE, IDC_MOTOR_SPEED_SLIDER, &CChurningRigDlg::OnNMReleasedcaptureMotorSpeedSlider)
	ON_EN_KILLFOCUS(IDC_SAMPLERATE, &CChurningRigDlg::OnEnKillfocusSamplerate)
	ON_EN_CHANGE(IDC_SAMPLERATE, &CChurningRigDlg::OnEnChangeSamplerate)
	ON_STN_CLICKED(IDC_ENCODER_TEXT, &CChurningRigDlg::OnStnClickedEncoderText)
	ON_BN_CLICKED(IDC_SPEEDFOLLOW_BOX, &CChurningRigDlg::OnBnClickedSpeedfollowBox)
	ON_COMMAND(ID_PROGRAM_RUN, &CChurningRigDlg::OnProgramRun)
	ON_COMMAND(ID_PROGRAM_STOP, &CChurningRigDlg::OnProgramStop)
	ON_EN_CHANGE(IDC_TEMPERATURE_SUMP, &CChurningRigDlg::OnEnChangeTemperatureSump)
	ON_EN_KILLFOCUS(IDC_TEMPERATURE_SUMP, &CChurningRigDlg::OnEnKillfocusTemperatureSump)
	ON_EN_KILLFOCUS(IDC_TEMPERATURE_EDIT, &CChurningRigDlg::OnEnKillfocusTemperatureEdit)
	ON_COMMAND(ID_PROGRAM_LOAD, &CChurningRigDlg::OnProgramLoad)
END_MESSAGE_MAP()

void CChurningRigDlg::DetectComPorts(size_t upperLimit /*=128*/)
{
	ports.clear(); // deletes all existing entries
	for (size_t i = 1; i <= upperLimit; i++)
	{
		TCHAR strPort[32] = { 0 };
		_stprintf_s(strPort, _T("COM%d"), static_cast<int>(i)); // changed from \\\\.\\COM&d

		DWORD dwSize = 0;
		LPCOMMCONFIG lpCC = (LPCOMMCONFIG) new BYTE[1];
		BOOL ret = GetDefaultCommConfig(strPort, lpCC, &dwSize);
		delete[] lpCC;

		lpCC = (LPCOMMCONFIG) new BYTE[dwSize];
		ret = GetDefaultCommConfig(strPort, lpCC, &dwSize);
		delete[] lpCC;

		//ret = (CSerial::CheckPort(strPort) == CSerial::EPortAvailable);

		if (ret) ports.push_back(strPort);
	}
}


// CChurningRigDlg message handlers

BOOL CChurningRigDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != nullptr)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	m_menu = GetMenu();

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here
	//mSensor1.SetCheck(TRUE);
	//mSensor2.SetCheck(TRUE);
	mStatus.SetWindowText(_T("Status: Not connected"));
	mSpeedSlider.SetRange(0, 2000, TRUE);
	mSpeedSlider.SetTic(100);
	mSpeedSlider.SetTic(300);
	mSpeedSlider.SetTic(500);
	mSpeedSlider.SetTic(1000);
	mSpeedSlider.SetTic(1500);
	mSpeedSlider.SetPos(returnspeed);

	mDirection.SetCheck(mDirectionVal);
	
	mDataList.InsertColumn(0, _T("Timestamp"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(1, _T("Time (s)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(2, _T("Speed (rpm)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(3, _T("Temperature (°C)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(4, _T("Torque (Nm)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(5, _T("Voltage (V)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(6, _T("Current (A)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(7, _T("Power (%)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(8, _T("Torque (T)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(9, _T("Frequency (Hz)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(10, _T("Motor Speed (rpm)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(11, _T("Sump R Temp (°C)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(12, _T("Sump L Temp (°C)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(13, _T("Pump Pinion Temp (°C)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(14, _T("CRB Temp (°C)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(15, _T("Bucket Temp (°C)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(16, _T("Encoder Temp (°C)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(17, _T("Stator Temp (°C)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(18, _T("Load 1 (LSB)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(19, _T("Load 2 (LSB)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(20, _T("Load 3 (LSB)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(21, _T("Load 4 (LSB)"), LVCFMT_CENTER, 70);
	mDataList.InsertColumn(22, _T("Torque (LSB)"), LVCFMT_CENTER, 70);

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CChurningRigDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CChurningRigDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CChurningRigDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CChurningRigDlg::OnFileListcomports()
{
	// TODO: Add your command handler code here
	CString sp;
	ComPortsDialog COMDlg(sp,ports,portnames);
	INT_PTR retVal = COMDlg.DoModal();
	if (retVal == IDOK) {
		selectedPort = sp;
	}
}


void CChurningRigDlg::OnFileSerialterminal()
{
	// TODO: Add your command handler code here
	CSerialTermDlg STDlg(selectedPort);
	INT_PTR retVal = STDlg.DoModal();
}


void CChurningRigDlg::OnBnClickedConnect()
{
	if (connected) {
		OnBnClickedDisconnect();
		return;
	}
	// auto scan and connect
	// disable button and show progress bar
	BOOL success = FALSE;
	mStatus.ShowWindow(SW_HIDE);
	mStatus2.SetWindowText(_T("Detecting available COM ports..."));
	mStatus2.ShowWindow(SW_SHOW);
	mProgress.ShowWindow(SW_SHOW);
	short lR=0, uR=100;
	mProgress.SetRange(lR, uR);
	mProgress.SetPos(lR);
	mProgress.SetWindowTextW(_T(""));
	mStatus2.UpdateWindow();
	mProgress.UpdateWindow();
	ComPortsDialog::DetectComPorts(ports, portnames);
	if (!selectedPort.IsEmpty()) {
		tstring h(selectedPort);
		ports.insert(ports.begin(), h);
	}
	int a = static_cast<int>(ports.size());
	if (a>0) mProgress.SetStep((uR - lR) / (2 * a));
	CString b;
	for (int i = 0; i < a; i++) {
		mProgress.StepIt();
		mSerial.Close();
		b.Format(_T("Querying port %d of %d: %s"), i + 1, a, ports[i].c_str());
		mStatus2.SetWindowText(b);
		//CString c = _T("\\\\.\\"); //
		CString c;
		c.Append(ports[i].c_str());
		LONG lLastError = mSerial.Open(c, this);
		mProgress.StepIt();
		if (lLastError != ERROR_SUCCESS) {
#ifdef COM_CONFIG_ERRORS
			MessageBox(L"Couldn't open port - is something else using it?", L"Config Error " + c);
#endif
			continue;
		}
		b.Format(_T("Waiting for device on %s to intialise..."), ports[i].c_str());
		mStatus2.SetWindowText(b);
		mStatus2.UpdateWindow();
		Sleep(500); // wait for reset - this might vary by board - try increasing if ID isn't working
		lLastError = mSerial.Setup(CSerial::EBaud115200, CSerial::EData8, CSerial::EParNone, CSerial::EStop1);
		if (lLastError != ERROR_SUCCESS)
		{
#ifdef COM_CONFIG_ERRORS
			MessageBox(L"Error setting up port", L"Config Error "+c);
#endif
			continue;
		}
		lLastError = mSerial.SetupHandshaking(CSerial::EHandshakeHardware);
		if (lLastError != ERROR_SUCCESS)
		{
#ifdef COM_CONFIG_ERRORS
			MessageBox(L"Error setting up handshaking", L"Config Error " + c);
#endif
			continue;
		}
//		lLastError = mSerial.SetupReadTimeouts(CSerial::EReadTimeoutNonblocking);
//		if (lLastError != ERROR_SUCCESS)
//		{
//#ifdef COM_CONFIG_ERRORS
//			MessageBox(L"Error setting up timeout", L"Config Error " + c);
//#endif
//			continue;
//		}
		lLastError = mSerial.SetupReadTimeouts(CSerial::EReadTimeoutBlocking);
		if (lLastError != ERROR_SUCCESS) {
#ifdef COM_CONFIG_ERRORS
			MessageBox(L"Error setting up blocking timeout", L"Config Error " + c);
#endif
			continue;
		}
		lLastError = mSerial.Write("<_DATA 0>", 0, 0, 1000);
		if (lLastError != ERROR_SUCCESS) {
#ifdef COM_CONFIG_ERRORS
			MessageBox(L"Error writing <_DATA 0> command", L"Config Error " + c);
#endif
			continue;
		}
		// Read data, until there is nothing left
		DWORD dwBytesRead = 0;
		char szBuffer[101];
		szBuffer[100] = '\0';
		do
		{
			// Flush data from the COM-port
			LONG lLastError = mSerial.Read(szBuffer, sizeof(szBuffer) - 1, &dwBytesRead, 0, 100);
		} while (dwBytesRead == sizeof(szBuffer) - 1);
		lLastError = mSerial.Write("<_ID>",0,0,1000);
		if (lLastError != ERROR_SUCCESS)
		{
#ifdef COM_CONFIG_ERRORS
			MessageBox(L"Error writing <_ID> command", L"Config Error " + c);
#endif
			continue;
		}
		// recording = TRUE;
		Sleep(500);
		do { // scan for < character
			lLastError = mSerial.Read(szBuffer, 1, &dwBytesRead, 0, 1000); // I think this is a timeout of 1s
		} while (dwBytesRead > 0 && szBuffer[0] != '<'); // Find start of header message
		lLastError = mSerial.Read(szBuffer, 9, &dwBytesRead, 0, 1000); // Read rest of header message
		szBuffer[dwBytesRead] = '\0';
		lLastError = mSerial.SetupReadTimeouts(CSerial::EReadTimeoutNonblocking);
		if (lLastError != ERROR_SUCCESS) {
#ifdef COM_CONFIG_ERRORS
			MessageBox(L"Error setting up nonblocking timeout", L"Config Error " + c);
#endif
			continue;
		}
		if (strcmp(szBuffer, "nRF52840>")==0) { // this line isn't working
			success = TRUE;
			selectedPort = ports[i].c_str();
			b.Format(_T("Status: connected to device on %s"), selectedPort.GetBuffer());
			mStatus.SetWindowText(b);
			break;
		}
#ifdef COM_CONFIG_ERRORS
		else {
			TCHAR sbuf[100];
			swprintf_s(sbuf, L"Did not receive nRF52840>\nReceived: %s (%d bytes)\non %s", CString(szBuffer).GetBuffer(), dwBytesRead, c.GetBuffer());
			MessageBox(sbuf, L"Config Error");
		}
#endif
	}
	mStatus2.ShowWindow(SW_HIDE);
	mStatus.ShowWindow(SW_SHOW);
	if (success) {
		OnEnKillfocusSamplerate();
		Sleep(10); // wait for _SET_RATE to be processed
		recording = TRUE; // but about to be set FALSE
		OnBnClickedRecord();
		mProgress.SetPos(uR);
		EnableDisableControls(TRUE);
		setSpeed(returnspeed);
		Sleep(10); // wait for _SPEED to be processed
		mDirectionVal = TRUE;
		OnBnClickedDirectionBox();
		Sleep(10);
		OnEnKillfocusTemperatureEdit();
		Sleep(10);
		OnEnKillfocusTemperatureSump();
	}
	else { // if failed, warning, explanation, instructions
		mSerial.Close();
		mStatus.SetWindowText(_T("Status: Device not found"));
		CString msg = _T("");
		/*CString msg = _T("Could not connect. There could be several reasons for this:\n\n")
			_T("The Arduino must be programmed with the correct software. If the Arduino has not yet been programmed, try using the 'Program' button in this app. The app's .exe file must be on a local drive (not a network drive) for this to work. Alternatively you may use the Arduino IDE and the code provided.\n\n")
			_T("To check if the Arduino has been programmed correctly, click File->Serial Terminal, connect to the correct COM port, wait 2 seconds and press the '<ID>' button. ")
			_T("The device should respond '<ChurningRig>' (you may also see a stream of data packets).\n\n")
			_T("The Arduino must be connected via USB, and have a COM port assigned to it - check this is the case by opening Windows' Device Manager and expanding 'Ports (COM & LPT)'. ")
			_T("If nothing resembling 'USB-SERIAL' or 'Arduino Uno' is listed, you may need to download and install a driver for your Arduino.\n\n")
			_T("Go to 'File->List Serial Ports' and check if your Arduino port is listed. ")
			_T("If it is not listed, type the port number into the box (e.g. COM5) and click 'Select'. Press Connect again - the port specified is added to the front of the queue of candidates.\n\n")
			_T("The port must be available to open. If another program is currently using the port, e.g. the Arduino Serial Monitor Window, you must close it and try to Connect again.\n\n")
			_T("As a last resort, if your Serial connection is working but this app fails to connect, you may use any other serial terminal, e.g. the Serial Monitor in the Arduino app. To turn data logging on, send <DATA ON> to start. ")
			_T("Data packets are received enclosed in <>, comma separated, with a time-stamp in milliseconds since the Arduino was last reset.");*/
		MessageBox(msg, _T("Connection Failed - What To Do Now"), MB_ICONERROR | MB_OK);
	}

	mProgress.ShowWindow(SW_HIDE);
	if (success) {
		init_complete = TRUE;
		connected = TRUE;
		mConnect.SetWindowText(_T("Disconnect"));
	}
}


void CChurningRigDlg::setSpeed(int speed) {
	CStringA a;
	a.Format("<_SPEED %d>", speed);
	mSerial.Write(a,0,0,1000);
	mSpeedSlider.SetPos(speed);
	mMotorTextVal.Format(_T("Motor speed: %d rpm"), mSpeedSlider.GetPos());
	mMotorText.SetWindowText(mMotorTextVal);
}

void CChurningRigDlg::OnBnClickedDisconnect()
{
	//init_complete = FALSE;
	recording = TRUE;
	OnBnClickedRecord();
	LONG lLastError = mSerial.Close();
	if (lLastError != ERROR_SUCCESS) mStatus.SetWindowText(_T("Error: Failed to close port"));
	else {
		mStatus.SetWindowText(_T("Status: Not connected"));
	}
	mConnect.EnableWindow(TRUE);
	EnableDisableControls(FALSE);
	connected = FALSE;
	mConnect.SetWindowText(_T("Connect"));
}


void CChurningRigDlg::OnFileExit()
{
	OnCancelOverride();
}


void CChurningRigDlg::OnBnClickedRecord()
{
	LONG lLastError = mSerial.Write(recording?"<_DATA 0>":"<_DATA 1>", 0, 0, 1000);
	if (lLastError != ERROR_SUCCESS) mStatus.SetWindowText(_T("Error: Unable to transmit command to device"));
}


//void CChurningRigDlg::OnBnClickedSensor1()
//{
//	mSensor1Val = mSensor1.GetCheck();
//	if (!(mSensor1Val || mSensor2Val)) { // prevent no sensors enabled
//		mSensor1Val = !mSensor1Val;
//		mSensor1.SetCheck(mSensor1Val);
//		return;
//	}
//	LONG lLastError = mSerial.Write(mSensor1Val ? "<SENSOR1 1>" : "<SENSOR1 0>", 0, 0, 1000);
//	if (lLastError != ERROR_SUCCESS) mStatus.SetWindowText(_T("Error: Unable to transmit command to device"));
//}
//
//
//void CChurningRigDlg::OnBnClickedSensor2()
//{
//	mSensor2Val = mSensor2.GetCheck();
//	if (!(mSensor1Val || mSensor2Val)) { // prevent no sensors enabled
//		mSensor2Val = !mSensor2Val;
//		mSensor2.SetCheck(mSensor2Val);
//		return;
//	}
//	LONG lLastError = mSerial.Write(mSensor2Val ? "<SENSOR2 1>" : "<SENSOR2 0>", 0, 0, 1000);
//	if (lLastError != ERROR_SUCCESS) mStatus.SetWindowText(_T("Error: Unable to transmit command to device"));
//}

afx_msg LRESULT CChurningRigDlg::OnSerialMsg(WPARAM wParam, LPARAM lParam)
{
	CSerial::EEvent eEvent = CSerial::EEvent(LOWORD(wParam));
	CSerial::EError eError = CSerial::EError(HIWORD(wParam));


	// Handle error event
	if (eEvent & CSerial::EEventError)
	{
		CString errMsg = _T("RX Error: ");
		switch (eError)
		{
		case CSerial::EErrorBreak:		mStatus.SetWindowTextW(errMsg + _T("Break condition"));			break;
		case CSerial::EErrorFrame:		mStatus.SetWindowTextW(errMsg + _T("Framing error"));			break;
		case CSerial::EErrorIOE:		mStatus.SetWindowTextW(errMsg + _T("IO device error"));			break;
		case CSerial::EErrorMode:		mStatus.SetWindowTextW(errMsg + _T("Unsupported mode"));		break;
		case CSerial::EErrorOverrun:	mStatus.SetWindowTextW(errMsg + _T("Buffer overrun"));			break;
		case CSerial::EErrorRxOver:		mStatus.SetWindowTextW(errMsg + _T("Input buffer overflow"));	break;
		case CSerial::EErrorParity:		mStatus.SetWindowTextW(errMsg + _T("Input parity error"));		break;
		case CSerial::EErrorTxFull:		mStatus.SetWindowTextW(errMsg + _T("Output buffer full"));		break;
		default:						mStatus.SetWindowTextW(errMsg + _T("Unknown"));					break;
		}
	}
	else {
		mStatusVal.Format(_T("Status: connected to device on %s"), selectedPort.GetBuffer());
		mStatus.SetWindowText(mStatusVal);
	}

	if (eEvent & CSerialMFC::EEventRecv && init_complete) {
		// TODO: Read data from the port

		// Read data, until there is nothing left
		DWORD dwBytesRead = 0;
		char szBuffer[2];
		CStringA message = imStr;
		BOOL starthead = FALSE;
		BOOL endhead = FALSE;
		do
		{
			// Read data from the COM-port one character at a time
			LONG lLastError = mSerial.Read(szBuffer, sizeof(szBuffer) - 1, &dwBytesRead);
			if (lLastError != ERROR_SUCCESS) {
				mStatus.SetWindowTextW(_T("RX Error: Unable to read from COM-port."));
				return mSerial.GetLastError();
			}

			if (dwBytesRead > 0)
			{
				// Finalize the data, so it is a valid string
				szBuffer[dwBytesRead] = '\0';
				message.Append(CStringA(szBuffer));
				if (szBuffer[0] == '<') starthead = TRUE;
				else if (szBuffer[0] == '\n') {
					endhead = TRUE;
					break;
				}
			}
		} while (dwBytesRead == sizeof(szBuffer) - 1);

		if (message.GetLength() < 1) return 1;

		if (!endhead && starthead) {
			incompleteMessage = TRUE;
			imStr = message;
			return 0;
		}
		else if (endhead) {
			incompleteMessage = FALSE;
			imStr = "";
		}
		else {
			return 1;
		}

		// remove < and >
		message.Delete(0, 1);
		message.Delete(message.GetLength() - 3, 3); // also have newline to contend with

		if (message[0] == '_') {
			char cc;
			char* cptr = &cc;
			char* hptr = strtok_s(message.GetBuffer(), " ", &cptr);
			char* dptr = strtok_s(NULL, " ", &cptr);
			// CStringA a; a.Format("%s %s", hptr, dptr); MessageBox(CString(a));
			if (strcmp(hptr, "_RATE") == 0) {
				CString a = CString(dptr);
				mSamplerateVal = a;
				mSamplerateVal.Append(_T(" Hz"));
				OnEnKillfocusSamplerate();
			}
			else if (strcmp(hptr, "_CLUTCH") == 0) {
				if (*dptr == '1') clutchEn = TRUE;
				else clutchEn = FALSE;
				ClutchToggle(clutchEn);
			}
			else if (strcmp(hptr, "_START") == 0) {
				motorStarted = TRUE;
				mMotorStartStop.SetWindowText(_T("Stop"));
				if (!clutchEn) { // do not allow clutch to be engaged while motor spinning
					mClutch.EnableWindow(FALSE);
				}
			}
			else if (strcmp(hptr, "_STOP") == 0 || strcmp(hptr, "_COAST") == 0) {
				motorStarted = FALSE;
				mMotorStartStop.SetWindowText(_T("Start"));
				mClutch.EnableWindow(TRUE);
				if (mSpeedFollowVal) {
					setSpeed(returnspeed);
				}
			}
			else if (strcmp(hptr, "_DIR") == 0) {
				if (*dptr == '0') mDirectionVal = TRUE;
				else mDirectionVal = FALSE;
				mDirection.SetCheck(mDirectionVal);
			}
			else if (strcmp(hptr, "_TEMP_CTRL_BOTH") == 0) {
				if (*dptr == '1') mTempCtrlEnVal = TRUE;
				else mTempCtrlEnVal = FALSE;
				mTempCtrlEn.SetCheck(mTempCtrlEnVal ? BST_CHECKED : BST_UNCHECKED);
			}
			else if (strcmp(hptr, "_SPEED") == 0) {
				if (*dptr == '0') {
					mMotorTextVal.Format(_T("Motor speed: %d rpm"), mSpeedSlider.GetPos());
					mMotorText.SetWindowText(mMotorTextVal);
				}
				else inverterCommsError(dptr);
			}
			else if (strcmp(hptr, "_SPEED_CHANGE") == 0) {
				float ff = strtof(dptr, NULL);
				int ffi = abs(int(ff * 93 / 869)) ; // GEAR RATIO APPLIED
				mSpeedSlider.SetPos(ffi);
				mMotorTextVal.Format(_T("Motor speed: %d rpm"), ffi);
				mMotorText.SetWindowText(mMotorTextVal);
			}
			else if (strcmp(hptr, "_INVERTER_ERROR") == 0) {
				inverterCommsError(dptr);
			}
			else if (strcmp(hptr, "_ENCODER_ERROR") == 0) {
				int dd = StrToIntA(dptr);
				CString aa;
				switch (dd) {
				case ENCODER_DIRCHNG:
					aa = _T("encoder direction change");
					break;
				case ENCODER_C_ERR:
					aa = _T("encoder sequence error");
					break;
				case L1_ERR:
					aa = _T("load cell 1 error");
					break;
				case L2_ERR:
					aa = _T("load cell 2 error");
					break;
				case ENCODER_SPI_NOT_RECEIVED:
					aa = _T("spi comms error");
					break;
				default:
					aa = _T("unknown error");
				}
				mEncoderTextVal = aa;
				mEncoderText.SetWindowText(mEncoderTextVal);
			}
			else if (strcmp(hptr, "_DATA") == 0) {
				recording = (*dptr == '1');
				mRecordData.SetWindowText(recording ? _T("Stop") : _T("Record Data"));
				if (!recording) {
					GotoDlgCtrl(&mExportData);
					// This clears the flags and allows the system to sleep normally.
					SetThreadExecutionState(ES_CONTINUOUS);
				}
				else {
					EnableDisableControls(TRUE);
					GotoDlgCtrl(&mTimeZero);
					// The following sets the appropriate flags to prevent system to go into sleep mode.
					SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_AWAYMODE_REQUIRED);
				}
			}
			else if (strcmp(hptr, "__") == 0) {
				int time_elapsed = 0;
				float speed, temperature1;
				float temps[NUM_THERMOCOUPLES];
				long loads[4];
				float voltage, current, power, torque, frequency, opspeed;
				uint16_t power16, torque16;
				int r = sscanf_s(dptr, "%d,%f,%f,%f,%f,%hu,%hu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%ld,%ld,%ld,%ld", &time_elapsed, &speed, &temperature1, &voltage, &current, &power16, &torque16, &frequency, &opspeed, &temps[0], &temps[1], &temps[2], &temps[3], &temps[4], &temps[5], &temps[6], &loads[0], &loads[1], &loads[2], &loads[3]);
				power = static_cast<float>(*(int16_t*)&power16);
				torque = static_cast<float>(*(int16_t*)&torque16);
				if (r < 13 + NUM_THERMOCOUPLES) {
					return 1;
				}
				// make new Dataline object
				if (DataBuffer.size() == 0) {
					sync_millis = time_elapsed;
					reset_time = std::chrono::system_clock::now();
				}
				Dataline nl = Dataline(time_elapsed - sync_millis, reset_time, speed, temperature1, voltage, current / 10, power / 10, torque / 10, frequency / 100, opspeed / 5 * 3, CString(message), temps, loads);
				// append to buffer vector
				DataBuffer.push_back(nl);
				if (next_zero) {
					zero_time = nl.timestamp;
					next_zero = FALSE;
					PrintData();
				}
				// print to RX box
				AddItem(nl);

			}
		}
		//mSensorStatusVal = _T("Status: OK");
		//mSensorStatus.SetWindowText(mSensorStatusVal);
		//AppendText(mData, lstr.GetBuffer());
	}
	//mStatus.SetWindowTextW(_T("RX: Data read from COM-port."));
	// Return successful
	return 0;
}

void CChurningRigDlg::inverterCommsError(char* ptr) {
	mMotorTextVal = CString("Inverter error: ") + CString(ptr);
	mMotorText.SetWindowText(mMotorTextVal);
}

void CChurningRigDlg::ClutchToggle() {
	clutchEn = !clutchEn;
	ClutchToggle(clutchEn);
}

void CChurningRigDlg::ClutchToggle(BOOL cEn) {
	clutchEn = cEn;
	if (clutchEn) mClutch.SetWindowText(_T("Clutch\nDisengage"));
	else {
		mClutch.SetWindowText(_T("Clutch\nEngage"));
		OnBnClickedTimezero();
		if (motorStarted) {
			mClutch.EnableWindow(FALSE);
			if (mSpeedFollowVal) {
				returnspeed = mSpeedSlider.GetPos();
				mSerial.Write(CStringA("<_SPEED_FOLLOW 1>"), 0, 0, 1000);
			}
		}
	}
}

void CChurningRigDlg::AppendText(CEdit& CE, TCHAR* newText)
{

	// move the caret to the end of the text
	int outLength = CE.GetWindowTextLength();
	CE.SendMessage(EM_SETSEL, outLength, outLength);

	// insert the text at the new caret position
	CE.SendMessage(EM_REPLACESEL, TRUE, reinterpret_cast<LPARAM>(newText));
}

void CChurningRigDlg::AddItem(Dataline nl, BOOL ensureVisible) {
	int nItem = mDataList.InsertItem(mDataList.GetItemCount(), nl.timeString(false, false));
	mDataList.SetItemText(nItem, 1, nl.timeString(zero_time));
	mDataList.SetItemText(nItem, 2, nl.DataString(nl.speed));
	mDataList.SetItemText(nItem, 3, nl.DataString(nl.temperature1));
	mDataList.SetItemText(nItem, 4, nl.DataString(static_cast<float>(CalculateTorque(nl.torquelsb))));
	mDataList.SetItemText(nItem, 5, nl.DataString(nl.voltage));
	mDataList.SetItemText(nItem, 6, nl.DataString(nl.current));
	mDataList.SetItemText(nItem, 7, nl.DataString(nl.power));
	mDataList.SetItemText(nItem, 8, nl.DataString(nl.torque));
	mDataList.SetItemText(nItem, 9, nl.DataString(nl.frequency));
	mDataList.SetItemText(nItem, 10, nl.DataString(nl.opspeed));
	for (int i=0;i<NUM_THERMOCOUPLES;i++) mDataList.SetItemText(nItem, 11+i, nl.DataString(nl.temps[i]));
	for (int i = 0; i < 4; i++) mDataList.SetItemText(nItem, 11 + NUM_THERMOCOUPLES + i, nl.DataString(nl.loads[i]));
	mDataList.SetItemText(nItem, 15 + NUM_THERMOCOUPLES, nl.DataString(nl.torquelsb));
	
	if (ensureVisible) mDataList.EnsureVisible(mDataList.GetItemCount()-1, FALSE);
}

void CChurningRigDlg::PrintData() {
	//mDataVal = _T("");
	mDataList.DeleteAllItems();
	for (std::vector<Dataline>::const_iterator ii = DataBuffer.begin(); ii < DataBuffer.end(); ii++) {
		//Dataline line = *ii;
		AddItem(*ii, FALSE);
	}
	//mData.SetWindowText(mDataVal);
	//mData.LineScroll(mData.GetLineCount());
}


void CChurningRigDlg::OnBnClickedTimezero()
{
	next_zero = TRUE;
	if (mSpeedFollowVal) { // SPEED FOLLOW ALSO ON TIME ZERO TO AVOID DOUBLE_CLUTCH
		returnspeed = mSpeedSlider.GetPos();
		mSerial.Write(CStringA("<_SPEED_FOLLOW 1>"), 0, 0, 1000);
	}
	GotoDlgCtrl(&mRecordData);
}


void CChurningRigDlg::OnBnClickedCleardata()
{
	if (!init_complete) return;
	int r;
	if (DataBuffer.size() > 0) {
		r = MessageBox(_T("All unexported data will be deleted. Continue?"), _T("Clear Data Warning"), MB_OKCANCEL | MB_ICONWARNING | MB_DEFBUTTON2);
	} else r = IDOK;
	if (r == IDOK) {
		//mData.SetWindowText(_T(""));
		mDataList.DeleteAllItems();
		DataBuffer.clear();
	}
}


void CChurningRigDlg::OnHelpAbout()
{
	CAboutDlg ADlg;
	ADlg.DoModal();
}

afx_msg void CChurningRigDlg::OnCancelOverride()
{
	if (DataBuffer.size() > dataExported) {
		int retval = MessageBox(_T("There is unsaved data. Save before closing?"), _T("Warning: Unsaved Data"), MB_YESNOCANCEL | MB_ICONEXCLAMATION | MB_DEFBUTTON1);
		if (retval == IDYES) OnBnClickedExport();
		else if (retval == IDCANCEL) return;
	}
	// call base implementation if escape is not down

	if ((GetKeyState(VK_ESCAPE) & 0x8000) == 0) {
		OnCancel();
	}
}


void CChurningRigDlg::OnBnClickedExport()
{
	if (!init_complete || DataBuffer.size()<1) return;
	CString dname = DataBuffer[0].timeString();
	dname.Append(_T(".csv"));
	dname.Replace(_T(":"), _T("_"));
	CFileDialog ExpDlg(FALSE, _T(".csv"), dname, OFN_OVERWRITEPROMPT, _T("Comma Separated Values (*.csv)|*.csv|Text Files (*.txt)|*.txt|All Files (*.*)|*.*||"));
	INT_PTR ret = ExpDlg.DoModal();
	if (ret != IDOK) return;
	CString fpath = ExpDlg.GetPathName();
	ExportData(fpath);
	MessageBox(_T("Data written to file: ")+fpath, _T("Data Export Successful"), MB_ICONINFORMATION);
}

void CChurningRigDlg::ExportData(CString fpath) {
	CStdioFile fout;
	BOOL opensuccess = fout.Open(fpath, CFile::modeCreate | CFile::modeWrite | CFile::typeText);
	if (!opensuccess) {
		mStatus.SetWindowText(_T("Error: Unable to open file for export"));
		return;
	}
	CString hs = _T("Timestamp, Time (s), Speed (rpm), Temperature (°C), Motor Voltage (V), Motor Current (A), Power Fraction (%), Torque (-), Frequency (Hz), Motor Speed (rpm), Sump R (°C), Sump L (°C), Pump Pinion (°C), CRB (°C), Bucket (°C), Encoder (°C), Stator (°C), Load 1 (LSB), Load 2 (LSB), Load 3 (LSB), Load 4 (LSB), Torque (LSB), Torque (Nm)\n");
	fout.WriteString(hs);
	for (unsigned int i = 0; i < DataBuffer.size(); i++) {
		CString dl;
		Dataline il = DataBuffer[i];
		dl.Format(_T("%s, %s, %s, %s, %s, %s, %s, %s, %s, %s"),
			il.timeString(TRUE, TRUE).GetBuffer(),
			il.timeString(zero_time).GetBuffer(),
			il.DataString(il.speed).GetBuffer(),
			il.DataString(il.temperature1).GetBuffer(),
			il.DataString(il.voltage).GetBuffer(),
			il.DataString(il.current).GetBuffer(),
			il.DataString(il.power).GetBuffer(),
			il.DataString(il.torque).GetBuffer(),
			il.DataString(il.frequency).GetBuffer(),
			il.DataString(il.opspeed).GetBuffer()
		);
		for (int j=0;j<NUM_THERMOCOUPLES;j++) dl.AppendFormat(_T(", %s"), il.DataString(il.temps[j]).GetBuffer());
		for (int j=0; j < 4; j++) dl.AppendFormat(_T(", %s"), il.DataString(il.loads[j]).GetBuffer());
		dl.AppendFormat(_T(", %s, %s"),
			il.DataString(il.torquelsb).GetBuffer(),
			il.DataString(static_cast<float>(CalculateTorque(il.torquelsb))).GetBuffer()
		);
		dl.Append(_T("\n"));
		fout.WriteString(dl);
	}
	fout.Close();
	dataExported = DataBuffer.size();
}

void CChurningRigDlg::OnClutch()
{
	CStringA a;
	if (clutchEn) { a = "<_CLUTCH 0>"; } 
	else { a = "<_CLUTCH 1>"; }
	mSerial.Write(a, 0, 0, 1000);
}

void CChurningRigDlg::EnableDisableControls(BOOL en) {
	mRecordData.EnableWindow(en);
	mSamplerate.EnableWindow(en);
	if (en || DataBuffer.size() < 1) { // only disable these if no data present
		mClearData.EnableWindow(en);
		mTimeZero.EnableWindow(en);
		mExportData.EnableWindow(en);
		mDataList.EnableWindow(en);
	}
	mClutch.EnableWindow(en);
	mSpeedSlider.EnableWindow(en);
	mSamplerate.EnableWindow(en);
	mSamplerateText.EnableWindow(en);
	mMotorStartStop.EnableWindow(en);
	mMotorText.EnableWindow(en);
	mEncoderText.EnableWindow(en);
	mTempCtrlEn.EnableWindow(en);
	mTempSet.EnableWindow(en);
	mTempSump.EnableWindow(en);
	mSetpoint.EnableWindow(en);
	mSump.EnableWindow(en);
	mDirection.EnableWindow(en);
	mBraking.EnableWindow(en);
	mSpeedFollow.EnableWindow(en);
}

void CChurningRigDlg::OnBnClickedDirectionBox()
{
	CStringA a = "<_DIR ";
	a.Append(mDirectionVal?"1>":"0>"); // no longer negates mDirectionVal (val updated on correct serial response)
	mSerial.Write(a, 0, 0, 1000);
}

void CChurningRigDlg::OnBnClickedBrakingBox()
{
	mBrakingVal = !mBrakingVal;
	mBraking.SetCheck(mBrakingVal);
}


void CChurningRigDlg::OnBnClickedMotorButton()
{
	CStringA a;
	if (motorStarted) {
		if (mBrakingVal) a = "<_SPEED_FOLLOW 0> <_STOP>";
		else a = "<_SPEED_FOLLOW 0> <_COAST>";
	} else {
		a = "<_START>";
	}
	mSerial.Write(a, 0, 0, 1000);
}


void CChurningRigDlg::OnBnClickedTempctrlCheck()
{
	// TODO: Add your control notification handler code here
	CStringA a = "<_TEMP_CTRL_BOTH ";
	a.Append(mTempCtrlEnVal ? "0>" : "1>"); // negates mTempCtrlEnVal(val updated on correct serial response)
	mSerial.Write(a, 0, 0, 1000);
	mTempCtrlEn.SetCheck(mTempCtrlEnVal?BST_CHECKED:BST_UNCHECKED);
}


void CChurningRigDlg::OnEnChangeTemperatureEdit()
{
	// parse temperature from dialog
	mTempSet.GetWindowText(mTempSetVal);
}

void CChurningRigDlg::OnEnKillfocusTemperatureEdit()
{
	if (mTempSetVal.Compare(mTempSetValOld) != 0) {
		double T;
		int ss = mTempSetVal.Find(_T("C"));
		if (ss != -1) {
			T = _tstof(mTempSetVal.Left(ss));
		}
		else {
			T = _tstof(mTempSetVal);
		}
		// send over serial
		CStringA a = "<_TEMP_CTRL_TARGET GEARBOX>";
		mSerial.Write(a, 0, 0, 1000);
		Sleep(10);
		a = "<_TEMP_SETPOINT ";
		a.AppendFormat("%f>", T);
		mSerial.Write(a, 0, 0, 1000);

		mTempSetVal.Format(_T("%3.1f C"), T);
		mTempSetValOld = mTempSetVal;
		mTempSet.SetWindowText(mTempSetVal);
	}
}

void CChurningRigDlg::OnEnChangeTemperatureSump()
{
	// parse temperature from dialog
	mTempSump.GetWindowText(mSumpSetVal);
}

void CChurningRigDlg::OnEnKillfocusTemperatureSump()
{
	if (mSumpSetVal.Compare(mSumpSetValOld) != 0) {
		double T;
		int ss = mSumpSetVal.Find(_T("C"));
		if (ss != -1) {
			T = _tstof(mSumpSetVal.Left(ss));
		}
		else {
			T = _tstof(mSumpSetVal);
		}
		// send over serial
		CStringA a = "<_TEMP_CTRL_TARGET STATOR>";
		mSerial.Write(a, 0, 0, 1000);
		Sleep(10);
		a = "<_TEMP_SETPOINT ";
		a.AppendFormat("%f>", T);
		mSerial.Write(a, 0, 0, 1000);

		mSumpSetVal.Format(_T("%3.1f C"), T);
		mSumpSetValOld = mSumpSetVal;
		mTempSump.SetWindowText(mSumpSetVal);
	}
}



void CChurningRigDlg::OnNMReleasedcaptureMotorSpeedSlider(NMHDR* pNMHDR, LRESULT* pResult)
{
	// TODO: Add your control notification handler code here
	if (pResult) *pResult = 0;
	CStringA a = "<_SPEED ";
	a.AppendFormat("%d>", mSpeedSlider.GetPos());
	mSerial.Write(a, 0, 0, 1000);
}


void CChurningRigDlg::OnEnKillfocusSamplerate()
{
	if (mSamplerateVal.Compare(mSamplerateValOld) != 0) {
		// parse contents of Samplerate edit dialog
		double ff = 0;
		int ss = mSamplerateVal.Find(_T("Hz"));
		if (ss != -1) {
			ff = _tstof(mSamplerateVal.Left(ss));
		}
		else {
			ff = _tstof(mSamplerateVal);
		}
		if (ff != 0) {
			// send over serial
			CStringA a = "<_SET_RATE ";
			a.AppendFormat("%f>", ff);
			mSerial.Write(a, 0, 0, 1000);
		}
		mSamplerateVal.Format(_T("%4.2f Hz"), ff);
		mSamplerateValOld = CString(mSamplerateVal);
		mSamplerate.SetWindowText(mSamplerateVal);
	}
}


void CChurningRigDlg::OnEnChangeSamplerate()
{
	mSamplerate.GetWindowText(mSamplerateVal);
}


void CChurningRigDlg::OnStnClickedEncoderText()
{
	mEncoderTextVal = _T("");
	mEncoderText.SetWindowText(mEncoderTextVal);
}


void CChurningRigDlg::OnBnClickedSpeedfollowBox()
{
	mSpeedFollowVal = !mSpeedFollowVal;
	mSpeedFollow.SetCheck(mSpeedFollowVal);
}

static UINT __cdecl programWrapper(LPVOID pParam) {
	UINT a = (((threadInfo *)pParam)->dlgPtr)->runProgram(((threadInfo*)pParam)->termPtr);
	if (a) {
		(((threadInfo*)pParam)->dlgPtr)->MessageBox(_T("Program interrupted"));
	}
	return a;
}

UINT CChurningRigDlg::runProgram(BOOL* stopPtr) {
	m_menu->EnableMenuItem(ID_PROGRAM_STOP, MF_ENABLED);
	for (auto element : program.lines)
	{
		if (*stopPtr) {
			m_menu->EnableMenuItem(ID_PROGRAM_STOP, MF_DISABLED);
			return 1;
		}
		if (_wcsicmp(element.command.c_str(), _T("MOTOR")) == 0) element.MotorLineExecute();
		else if (_wcsicmp(element.command.c_str(), _T("CLEAR")) == 0) element.ClearLineExecute();
		else if (_wcsicmp(element.command.c_str(), _T("DIRECTION")) == 0) element.DirectionLineExecute();
		else if (_wcsicmp(element.command.c_str(), _T("RECORDING")) == 0) element.RecordingLineExecute();
		else if (_wcsicmp(element.command.c_str(), _T("SPEEDFOLLOW")) == 0) element.SpeedFollowLineExecute();
		else if (_wcsicmp(element.command.c_str(), _T("SPEED")) == 0) element.SpeedLineExecute();
		else if (_wcsicmp(element.command.c_str(), _T("CLUTCH")) == 0) element.ClutchLineExecute();
		else if (_wcsicmp(element.command.c_str(), _T("WAIT")) == 0) element.WaitLineExecute(stopPtr);
		else if (_wcsicmp(element.command.c_str(), _T("PATH")) == 0) element.PathLineExecute();
		else if (_wcsicmp(element.command.c_str(), _T("EXPORT")) == 0) element.ExportLineExecute();
		else if (_wcsicmp(element.command.c_str(), _T("TORQUECAL")) == 0) element.TorqueCalExecute();
		else if (_wcsicmp(element.command.c_str(), _T("TORQUEZERO")) == 0) element.TorqueZero();
		else if (_wcsicmp(element.command.c_str(), _T("TIMEZERO")) == 0) element.dlg.OnBnClickedTimezero();
	}
	m_menu->EnableMenuItem(ID_PROGRAM_STOP, MF_DISABLED);
	return 0;
	/*
	// Stop motor
	while (motorStarted) {
		OnBnClickedMotorButton();
		Sleep(50);
	}
	// Clear data
	OnBnClickedCleardata();
	mDirectionVal = TRUE;
	for (int j = 0; j < 6; j+=2) { // CURRENTLY FORWARD ONLY
		// Update direction, start forwards
		while (mDirectionVal == (j % 2 == 0)) {
			OnBnClickedDirectionBox();
			Sleep(50);
		}
		// Request data
		recording = FALSE;
		while (!recording) {
			OnBnClickedRecord();
			Sleep(50);
		}
		// Set speed follow
		mSpeedFollowVal = FALSE;
		OnBnClickedSpeedfollowBox();
		// Set speed
		setSpeed(1450);
		Sleep(100);
		// Engage clutch
		clutchEn = FALSE;
		while (!clutchEn) {
			OnClutch();
			Sleep(50);
		}
		// Start Motor
		motorStarted = FALSE;
		while (!motorStarted) {
			OnBnClickedMotorButton();
			Sleep(50);
		}
		// wait 1 min
		for (int i = 0; i < 600; i++) {
			if (*stopPtr) return 1;
			Sleep(100);
		}
		// Disengage clutch
		clutchEn = TRUE;
		while (clutchEn) {
			OnClutch();
			Sleep(50);
		}
		// wait 2.5 min
		for (int i = 0; i < 1500; i++) {
			if (*stopPtr) return 1;
			Sleep(100);
		}
		while (recording) {
			OnBnClickedRecord();
			Sleep(50);
		}
		// Stop motor
		motorStarted = TRUE;
		while (motorStarted) {
			OnBnClickedMotorButton();
			Sleep(50);
		}
		// Export data
		if (DataBuffer.size() > 0) {
			CString dname = DataBuffer[0].timeString();
			dname.Append(_T(".csv"));
			dname.Replace(_T(":"), _T("_"));
			CString pp;
			pp.Format(_T("C:\\Users\\dam216\\OneDrive - Imperial College London\\PhD\\Tesla\\RIG DATA\\5\\%s_%s_%d.csv"), (j % 2 == 0) ? _T("F") : _T("R"), _T("D0_T40"), j / 2);
			ExportData(pp);
			mDataList.DeleteAllItems();
			DataBuffer.clear();
		}
	}
	return 0;
	*/
}

struct threadInfo tI; // declare global struct for passing to worker thread

void CChurningRigDlg::OnProgramRun()
{
	tI.dlgPtr = this;
	stopWorker = FALSE;
	tI.termPtr = &stopWorker;
	AfxBeginThread(programWrapper, (LPVOID) &tI);
}


void CChurningRigDlg::OnProgramStop()
{
	stopWorker = TRUE;
}


void CChurningRigDlg::OnProgramLoad()
{
	const TCHAR szFilter[] = _T("Text Files (*.txt)|*.txt|All Files (*.*)|*.*||");
	CFileDialog dlg(TRUE, _T("txt"), NULL, OFN_OVERWRITEPROMPT, szFilter, this);
	if (dlg.DoModal() == IDOK)
	{
		program.Initialise(tstring(dlg.GetPathName()));
	}
	ShowWindow(SW_MINIMIZE);
	ShowWindow(SW_SHOWNORMAL);
	SetForegroundWindow();
	SetActiveWindow();
	m_menu->EnableMenuItem(ID_PROGRAM_RUN, MF_ENABLED);
}

CChurningRigDlg::RigProgram::RigProgram(CChurningRigDlg& dg) :
dlg(dg)
{}

void CChurningRigDlg::RigProgram::Initialise(std::wstring pth)
{
	file_path = pth;
	lines.clear();
	std::wfstream progfile;
	progfile.open(file_path, std::ios::in);
	if (progfile.is_open())
	{
		tstring fline;
		while (std::getline(progfile, fline))
		{
			ProgramLine line = parseLine(fline);
			if (line.valid) lines.push_back(line);
		}
	}
}

CChurningRigDlg::RigProgram::ProgramLine::ProgramLine(tstring cmd, tstring dat, bool vld, CChurningRigDlg& dg) :
command(cmd), data(dat), valid(vld), dlg(dg) {}

bool CChurningRigDlg::RigProgram::ProgramLine::ParseStartStop(tstring data) {
	if (_wcsicmp(data.c_str(), _T("START")) == 0) return true;
	return false;
}

bool CChurningRigDlg::RigProgram::ProgramLine::ParseOnOff(tstring dat) {
	if (_wcsicmp(dat.c_str(), _T("ON")) == 0) return true;
	return false;
}

bool CChurningRigDlg::RigProgram::ProgramLine::ParseForwardReverse(tstring dat) {
	if (_wcsicmp(dat.c_str(), _T("REVERSE")) == 0) return true;
	return false;
}

void CChurningRigDlg::RigProgram::ProgramLine::MotorLineExecute()
{
	// Start/Stop motor
	BOOL start = ParseStartStop(data);
	dlg.motorStarted = !start;
	while (dlg.motorStarted!=start) {
		dlg.OnBnClickedMotorButton();
		Sleep(50);
	}
}
void CChurningRigDlg::RigProgram::ProgramLine::ClearLineExecute()
{
	// Clear data
	dlg.mDataList.DeleteAllItems();
	dlg.DataBuffer.clear();
}

void CChurningRigDlg::RigProgram::ProgramLine::DirectionLineExecute()
{
	BOOL dir = ParseForwardReverse(data);
	dlg.mDirectionVal = !dir;
	while (dlg.mDirectionVal != dir) {
		dlg.OnBnClickedDirectionBox();
		Sleep(50);
	}
}

void CChurningRigDlg::RigProgram::ProgramLine::RecordingLineExecute()
{
	// Stop/Start Recording
	BOOL rec = ParseStartStop(data);
	dlg.recording = !rec;
	while (dlg.recording!=rec) {
		dlg.OnBnClickedRecord();
		Sleep(50);
	}
}

void CChurningRigDlg::RigProgram::ProgramLine::SpeedLineExecute() {
	// Set speed
	int spd = std::stoi(data);
	dlg.setSpeed(spd);
	Sleep(100);
}

void CChurningRigDlg::RigProgram::ProgramLine::SpeedFollowLineExecute()
{
	// Set speed follow
	BOOL spdf = ParseOnOff(data);
	dlg.mSpeedFollowVal = !spdf;
	while (dlg.mSpeedFollowVal != spdf) {
		dlg.OnBnClickedSpeedfollowBox();
		Sleep(50);
	}
}

void CChurningRigDlg::RigProgram::ProgramLine::ClutchLineExecute()
{
	// Set speed follow
	BOOL cltch = ParseOnOff(data);
	dlg.clutchEn = !cltch;
	while (dlg.clutchEn!=cltch) {
		dlg.OnClutch();
		Sleep(50);
	}
}


void CChurningRigDlg::RigProgram::ProgramLine::PathLineExecute()
{
	dlg.exportpath = data;
}

void CChurningRigDlg::RigProgram::ProgramLine::ExportLineExecute()
{
	// Export data
	if (dlg.DataBuffer.size() > 0) {
		CString pp;
		pp.Format(_T("%s%s.csv"), dlg.exportpath.c_str(),data.c_str());
		dlg.ExportData(pp);
	}
}

void CChurningRigDlg::RigProgram::ProgramLine::WaitLineExecute(BOOL* stopPtr)
{
	int value = std::stoi(data) * 10;
	for (int i = 0; i < value; i++) {
		if (*stopPtr) break;
		Sleep(100);
	}
}

void CChurningRigDlg::RigProgram::ProgramLine::TorqueCalExecute() {
	// Set torque calibration value
	dlg.torquecalval = std::stod(data);
}

void CChurningRigDlg::RigProgram::ProgramLine::TorqueZero() {
	// Set torque calibration value
	if (dlg.DataBuffer.empty())
	{
		dlg.MessageBox(_T("Cannot zero torque when data empty"), _T("Data required"), MB_OK);
		return;
	}
	dlg.torqueoffset = dlg.torqueoffset - dlg.CalculateTorque(dlg.DataBuffer.back().torquelsb);
}

CChurningRigDlg::RigProgram::ProgramLine CChurningRigDlg::RigProgram::parseLine(std::wstring line)
{
	tstring cmd;
	tstring dat;
	std::wstringstream sstr(line);
	getline(sstr, cmd, _T(' '));
	getline(sstr, dat);
	return { cmd, dat, true, dlg };
}

