#pragma once

#include <SerialMFC.h>
#include <vector>
#include <string> 

#ifdef _UNICODE
#define tstring std::wstring
#else
#define tstring std::string
#endif

// ComPortsDialog dialog

class ComPortsDialog : public CDialogEx
{
	DECLARE_DYNAMIC(ComPortsDialog)

public:
	ComPortsDialog(CString& p, std::vector< tstring >& pt, std::vector< tstring >& ptn, CWnd* pParent = nullptr);   // standard constructor
	virtual ~ComPortsDialog();
	BOOL ComPortsDialog::OnInitDialog();
	CString& pp;
	std::vector< tstring >& pts;
	std::vector< tstring >& ptnms;

// Serial
public:
	CSerialMFC mSerial;
	static void ComPortsDialog::DetectComPorts(std::vector< tstring >& ports, std::vector< tstring >& portnames);

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_COMPORTS };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	CComboBox mCOMCombo;
	CString mCOMComboVal;
	CButton mCOMSelect;
	afx_msg void OnBnClickedSelectport();
};
