#pragma once

#include "SerialMFC.h"
#include <vector>
#include <ctime>
#include <chrono>

class RXline {
public:
	RXline(std::chrono::system_clock::time_point ts, CString dt);
	virtual ~RXline();

	std::chrono::system_clock::time_point timestamp;
	CString data;
	CString toString(BOOL tmstmp = FALSE);
};

// CSerialTermDlg dialog

class CSerialTermDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CSerialTermDlg)

public:
	CSerialTermDlg(LPCTSTR selPt, CWnd* pParent = nullptr);   // standard constructor
	virtual ~CSerialTermDlg();
	BOOL OnInitDialog();
	void CSerialTermDlg::AppendText(CEdit& CE, TCHAR* newText);
	void PrintRx();
	std::vector<RXline> RxBuffer;
	BOOL doTimestamp;


// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_SERIALTERM };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	virtual void OnOK();

// Serial
public:
	CSerialMFC mSerial;
	afx_msg LRESULT CSerialTermDlg::OnSerialMsg(WPARAM wParam, LPARAM lParam);

	DECLARE_MESSAGE_MAP()
public:
	CEdit mSerTermPort;
	CString mSerTermPortVal;
	CButton mSerTermOpen;
	CButton mSerTermClose;
	CComboBox mSerTermBaud;
	CString mSerTermBaudVal;
	CStatic mSerTermStatus;
	CString mSerTermStatusVal;
	CEdit mSerTermTx;
	CString mSerTermTxVal;
	CButton mSerTermSend;
	CButton mSerTermSendH;
	CButton mSerTermIDH;
	CEdit mSerTermRx;
	CString mSerTermRxVal;
	CButton mSerTermClear;
	CButton mSerTermTimestamp;
	CButton mSerTermExit;
	afx_msg void OnBnClickedOpen();
	afx_msg void OnBnClickedClose();
	afx_msg void OnCbnSelchangeBaudrate();
	afx_msg void OnBnClickedSend();
	afx_msg void OnBnClickedSendh();
	afx_msg void OnBnClickedIdh();
	afx_msg void OnBnClickedClear();
	afx_msg void OnBnClickedExit();

	afx_msg void OnBnClickedTimestamp();
};