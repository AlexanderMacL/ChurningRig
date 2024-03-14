// CSerialTermDlg.cpp : implementation file
//

#include "pch.h"
#include "ChurningRig.h"
#include "CSerialTermDlg.h"
#include "afxdialogex.h"

RXline::RXline(std::chrono::system_clock::time_point ts, CString dt)
	: timestamp(ts)
	, data(dt) {

}

RXline::~RXline() {}

CString RXline::toString(BOOL tmstmp) {
	if (tmstmp) {
		time_t tt = std::chrono::system_clock::to_time_t(timestamp);
		struct tm tm;
		localtime_s(&tm, &tt);
		std::chrono::system_clock::time_point tp = std::chrono::system_clock::from_time_t(tt);
		std::chrono::duration<double> sec = timestamp - tp + std::chrono::seconds(tm.tm_sec);
		char ts[23];
		sprintf_s(ts, "%02d-%02d-%02d %02d:%02d:%05.3f",
			tm.tm_mday,
			tm.tm_mon + 1,
			tm.tm_year - 100,
			tm.tm_hour,
			tm.tm_min,
			sec.count()
		);
		return CString(ts) + _T("  ") + data;
	}
	else return data;
}

// CSerialTermDlg dialog

IMPLEMENT_DYNAMIC(CSerialTermDlg, CDialogEx)

CSerialTermDlg::CSerialTermDlg(LPCTSTR selPt, CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_SERIALTERM, pParent)
	, mSerTermPortVal(selPt)
	, mSerTermBaudVal(_T("115200"))
	, mSerTermStatusVal(_T("Status: not connected"))
	, mSerTermTxVal(_T(""))
	, mSerTermRxVal(_T(""))
	, doTimestamp(FALSE)
{

}

CSerialTermDlg::~CSerialTermDlg()
{
}

void CSerialTermDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_PORT, mSerTermPort);
	DDX_Text(pDX, IDC_PORT, mSerTermPortVal);
	DDX_Control(pDX, IDC_OPEN, mSerTermOpen);
	DDX_Control(pDX, IDC_CLOSE, mSerTermClose);
	DDX_Control(pDX, IDC_BAUDRATE, mSerTermBaud);
	DDX_CBString(pDX, IDC_BAUDRATE, mSerTermBaudVal);
	DDX_Control(pDX, IDC_SERSTATUS, mSerTermStatus);
	DDX_Text(pDX, IDC_SERSTATUS, mSerTermStatusVal);
	DDX_Control(pDX, IDC_TX, mSerTermTx);
	DDX_Text(pDX, IDC_TX, mSerTermTxVal);
	DDX_Control(pDX, IDC_SEND, mSerTermSend);
	DDX_Control(pDX, IDC_SENDH, mSerTermSendH);
	DDX_Control(pDX, IDC_IDH, mSerTermIDH);
	DDX_Control(pDX, IDC_RX, mSerTermRx);
	DDX_Text(pDX, IDC_RX, mSerTermRxVal);
	DDX_Control(pDX, IDC_CLEAR, mSerTermClear);
	DDX_Control(pDX, IDC_TIMESTAMP, mSerTermTimestamp);
	DDX_Control(pDX, IDC_EXIT, mSerTermExit);
}


BEGIN_MESSAGE_MAP(CSerialTermDlg, CDialogEx)
	ON_WM_SERIAL(OnSerialMsg)
	ON_BN_CLICKED(IDC_OPEN, &CSerialTermDlg::OnBnClickedOpen)
	ON_BN_CLICKED(IDC_CLOSE, &CSerialTermDlg::OnBnClickedClose)
	ON_CBN_SELCHANGE(IDC_BAUDRATE, &CSerialTermDlg::OnCbnSelchangeBaudrate)
	ON_BN_CLICKED(IDC_SEND, &CSerialTermDlg::OnBnClickedSend)
	ON_BN_CLICKED(IDC_SENDH, &CSerialTermDlg::OnBnClickedSendh)
	ON_BN_CLICKED(IDC_IDH, &CSerialTermDlg::OnBnClickedIdh)
	ON_BN_CLICKED(IDC_CLEAR, &CSerialTermDlg::OnBnClickedClear)
	ON_BN_CLICKED(IDC_EXIT, &CSerialTermDlg::OnBnClickedExit)
	ON_BN_CLICKED(IDC_TIMESTAMP, &CSerialTermDlg::OnBnClickedTimestamp)
END_MESSAGE_MAP()


void CSerialTermDlg::PrintRx() {
	mSerTermRxVal = _T("");
	for (std::vector<RXline>::const_iterator ii = RxBuffer.begin(); ii < RxBuffer.end(); ii++) {
		RXline line = *ii;
		mSerTermRxVal.Append(line.toString(doTimestamp) + (doTimestamp ? _T("\r\n") : _T("")));
	}
	mSerTermRx.SetWindowText(mSerTermRxVal);
	mSerTermRx.LineScroll(mSerTermRx.GetLineCount());
}

void CSerialTermDlg::AppendText(CEdit& CE, TCHAR* newText)
{

	// move the caret to the end of the text
	int outLength = CE.GetWindowTextLength();
	CE.SendMessage(EM_SETSEL, outLength, outLength);

	// insert the text at the new caret position
	CE.SendMessage(EM_REPLACESEL, TRUE, reinterpret_cast<LPARAM>(newText));
}

// CSerialTermDlg message handlers

BOOL CSerialTermDlg::OnInitDialog() {

	CDialog::OnInitDialog();

	// initialise baud values
	for (int i = mSerTermBaud.GetCount() - 1; i >= 0; i--) mSerTermBaud.DeleteString(i); // Delete all elements in ComboBox
	int bdr[9] = { 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
	char bdf[7];
	for (int i = 0; i < 9; i++) {
		sprintf_s(bdf, "%d", bdr[i]);
		mSerTermBaud.AddString(CString(bdf));
		mSerTermBaud.SetCurSel(mSerTermBaud.GetCount() - 1); // set to last one
	}
	SendMessage(WM_NEXTDLGCTL, (WPARAM)mSerTermOpen.m_hWnd, TRUE);
	return FALSE;
}

void CSerialTermDlg::OnOK() {
	CWnd* ff = GetFocus();
	if (ff == &mSerTermPort) {
		OnBnClickedOpen();
	}
	else if (ff == &mSerTermTx) {
		OnBnClickedSend();
	}
	else if (ff == &mSerTermRx) {
		
	}
	else if (ff == &mSerTermExit) {
		CDialog::OnOK();
	}
	return;
}

int ShowError(LONG lError, LPCTSTR lptszMessage)
{
	// Generate a message text
	TCHAR tszMessage[256];
	wsprintf(tszMessage, _T("%s\n(error code %d)"), lptszMessage, lError);

	// Display message-box and return with an error-code
	::MessageBox(0, tszMessage, _T("Listener"), MB_ICONSTOP | MB_OK);
	return 1;
}

void CSerialTermDlg::OnBnClickedOpen()
{
	mSerTermPort.GetWindowTextW(mSerTermPortVal);
	// Setup the serial port (9600,N81) using no handshaking
	LONG lLastError;
	lLastError = mSerial.Open(mSerTermPortVal, this);
	if (lLastError != ERROR_SUCCESS) {
		mSerTermStatus.SetWindowTextW(_T("Error: Unable to connect to port ") + mSerTermPortVal);
		return;
	}
	lLastError = mSerial.Setup(CSerial::EBaud115200, CSerial::EData8, CSerial::EParNone, CSerial::EStop1);
	if (lLastError != ERROR_SUCCESS) {
		mSerTermStatus.SetWindowTextW(_T("Error: Unable to set COM-port setting on ") + mSerTermPortVal);
		return;
	}
	lLastError = mSerial.SetupHandshaking(CSerial::EHandshakeHardware);
	if (lLastError != ERROR_SUCCESS) {
		mSerTermStatus.SetWindowTextW(_T("Error: Unable to set COM-port handshaking for ") + mSerTermPortVal);
		return;
	}
	//lLastError = mSerial.SetMask(CSerial::EEventBreak |
	//	CSerial::EEventCTS |
	//	CSerial::EEventDSR |
	//	CSerial::EEventError |
	//	CSerial::EEventRing |
	//	CSerial::EEventRLSD |
	//	CSerial::EEventRecv);
	lLastError = mSerial.SetupReadTimeouts(CSerial::EReadTimeoutNonblocking);
	mSerTermStatus.SetWindowTextW(_T("Status: Connected to ") + mSerTermPortVal);
}


void CSerialTermDlg::OnBnClickedClose()
{
	LONG lLastError = mSerial.Close();
	if (lLastError == ERROR_SUCCESS) {
		mSerTermStatus.SetWindowTextW(_T("Status: not connected"));
	}
	else {
		mSerTermStatus.SetWindowTextW(_T("Status: unable to close port ") + mSerTermPortVal);
	}
}

afx_msg LRESULT CSerialTermDlg::OnSerialMsg(WPARAM wParam, LPARAM lParam)
{
	
	CSerial::EEvent eEvent = CSerial::EEvent(LOWORD(wParam));
	CSerial::EError eError = CSerial::EError(HIWORD(wParam));

	// Handle error event
	if (eEvent & CSerial::EEventError)
	{
		CString errMsg = _T("RX Error: ");
		switch (eError)
		{
		case CSerial::EErrorBreak:		mSerTermStatus.SetWindowTextW(errMsg + _T("Break condition"));			break;
		case CSerial::EErrorFrame:		mSerTermStatus.SetWindowTextW(errMsg + _T("Framing error"));			break;
		case CSerial::EErrorIOE:		mSerTermStatus.SetWindowTextW(errMsg + _T("IO device error"));			break;
		case CSerial::EErrorMode:		mSerTermStatus.SetWindowTextW(errMsg + _T("Unsupported mode"));			break;
		case CSerial::EErrorOverrun:	mSerTermStatus.SetWindowTextW(errMsg + _T("Buffer overrun"));			break;
		case CSerial::EErrorRxOver:		mSerTermStatus.SetWindowTextW(errMsg + _T("Input buffer overflow"));	break;
		case CSerial::EErrorParity:		mSerTermStatus.SetWindowTextW(errMsg + _T("Input parity error"));		break;
		case CSerial::EErrorTxFull:		mSerTermStatus.SetWindowTextW(errMsg + _T("Output buffer full"));		break;
		default:						mSerTermStatus.SetWindowTextW(errMsg + _T("Unknown"));					break;
		}
	}

	if (eEvent & CSerialMFC::EEventRecv) {
		// TODO: Read data from the port

		// Read data, until there is nothing left
		DWORD dwBytesRead = 0;
		char szBuffer[101];
		do
		{
			// Read data from the COM-port
			LONG lLastError = mSerial.Read(szBuffer, sizeof(szBuffer) - 1, &dwBytesRead);
			if (lLastError != ERROR_SUCCESS) {
				mSerTermStatus.SetWindowTextW(_T("RX Error: Unable to read from COM-port."));
				return mSerial.GetLastError();
			}

			if (dwBytesRead > 0)
			{
				// Finalize the data, so it is a valid string
				szBuffer[dwBytesRead] = '\0';

				// Display the data
				// printf("%s :: %d\r\n", szBuffer, dwBytesRead);
				//mRxTextVal.Append(CString(szBuffer));
				//mRxText.SetWindowTextW(mRxTextVal);
				//mRxText.LineScroll(mRxText.GetLineCount());
				//AppendText(this->m_hWnd, IDC_RX, CString(szBuffer).GetBuffer());
				
				// make new RXline object
				RXline nl = RXline(std::chrono::system_clock::now(), CString(szBuffer));
				// append to buffer vector
				RxBuffer.push_back(nl);
				// print to RX box
				CString lstr = nl.toString(doTimestamp) + (doTimestamp ? _T("\r\n") : _T(""));
				AppendText(mSerTermRx, lstr.GetBuffer());
			}
		} while (dwBytesRead == sizeof(szBuffer) - 1);
	}
	//mSerTermStatus.SetWindowTextW(_T("RX: Data read from COM-port."));
	// Return successful
	return 0;
}

void CSerialTermDlg::OnCbnSelchangeBaudrate()
{
	enum class SBAUD : int {
		SBAUD_600		= 0,
		SBAUD_1200		= 1,
		SBAUD_2400		= 2,
		SBAUD_4800		= 3,
		SBAUD_9600		= 4,
		SBAUD_19200		= 5,
		SBAUD_38400		= 6,
		SBAUD_57600		= 7,
		SBAUD_115200	= 8
	} bd;
	// get selected item
	bd = (SBAUD)mSerTermBaud.GetCurSel();
	// switch by selected item with enum labels
	CSerial::EBaudrate Sbd;
	switch (bd) { // map from CSerial macros to list index of ComboBox
		case SBAUD::SBAUD_600:		Sbd = CSerial::EBaud600;	break;
		case SBAUD::SBAUD_1200:		Sbd = CSerial::EBaud1200;	break;
		case SBAUD::SBAUD_2400:		Sbd = CSerial::EBaud2400;	break;
		case SBAUD::SBAUD_4800:		Sbd = CSerial::EBaud4800;	break;
		case SBAUD::SBAUD_9600:		Sbd = CSerial::EBaud9600;	break;
		case SBAUD::SBAUD_19200:	Sbd = CSerial::EBaud19200;	break;
		case SBAUD::SBAUD_38400:	Sbd = CSerial::EBaud38400;	break;
		case SBAUD::SBAUD_57600:	Sbd = CSerial::EBaud57600;	break;
		case SBAUD::SBAUD_115200:	Sbd = CSerial::EBaud115200;	break;
		default: Sbd = CSerial::EBaud115200;
	}
	// assign baudrate to mSerial
	LONG lLastError = mSerial.Setup(Sbd, CSerial::EData8, CSerial::EParNone, CSerial::EStop1);
	UpdateData();
	if (lLastError != ERROR_SUCCESS) {
		mSerTermStatus.SetWindowTextW(_T("Error: Unable to set baudrate ") + mSerTermBaudVal);
	}
}


void CSerialTermDlg::OnBnClickedSend()
{
	mSerTermTx.GetWindowTextW(mSerTermTxVal);
	LONG lLastError = mSerial.Write(CStringA(mSerTermTxVal),0,0,1000);
	if (lLastError != ERROR_SUCCESS) {
		mSerTermStatus.SetWindowTextW(_T("Error: Unable to send data: ") + mSerTermTxVal);
	} else mSerTermTx.SetWindowText(_T("")); // clear Tx on sending successfully
}


void CSerialTermDlg::OnBnClickedSendh()
{
	mSerTermTx.GetWindowTextW(mSerTermTxVal);
	LONG lLastError = mSerial.Write("<"+CStringA(mSerTermTxVal)+">",0,0,1000);
	if (lLastError != ERROR_SUCCESS) {
		mSerTermStatus.SetWindowTextW(_T("Error: Unable to send data: <") + mSerTermTxVal + _T(">"));
	} else mSerTermTx.SetWindowText(_T("")); // clear Tx on sending successfully
}


void CSerialTermDlg::OnBnClickedIdh()
{
	LONG lLastError = mSerial.Write("<_ID>",0,0,1000);
	if (lLastError != ERROR_SUCCESS) {
		mSerTermStatus.SetWindowTextW(_T("Error: Unable to send data: <_ID>"));
	}
}


void CSerialTermDlg::OnBnClickedClear()
{
	RxBuffer.clear();
	PrintRx();
}

void CSerialTermDlg::OnBnClickedExit()
{
	OnOK();
}


void CSerialTermDlg::OnBnClickedTimestamp()
{
	doTimestamp = mSerTermTimestamp.GetCheck()==BST_CHECKED;
	PrintRx();
}
