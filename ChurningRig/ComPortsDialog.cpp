// ComPortsDialog.cpp : implementation file
//

#include "pch.h"
#include "ChurningRig.h"
#include "ComPortsDialog.h"
#include "afxdialogex.h"
#include "enumser.h"


// ComPortsDialog dialog

IMPLEMENT_DYNAMIC(ComPortsDialog, CDialogEx)

ComPortsDialog::ComPortsDialog(CString& p, std::vector< tstring >& pt, std::vector< tstring >& ptn, CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_COMPORTS, pParent)
	, mCOMComboVal(_T(""))
	, pp(p)
	, pts(pt)
	, ptnms(ptn)
{
	// Scan Serial Ports to see which are available
	DetectComPorts(pts,ptnms);
}

ComPortsDialog::~ComPortsDialog()
{
}

void ComPortsDialog::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO1, mCOMCombo);
	DDX_CBString(pDX, IDC_COMBO1, mCOMComboVal);
	DDX_Control(pDX, IDC_SELECTPORT, mCOMSelect);
}


BEGIN_MESSAGE_MAP(ComPortsDialog, CDialogEx)
	ON_BN_CLICKED(IDC_SELECTPORT, &ComPortsDialog::OnBnClickedSelectport)
END_MESSAGE_MAP()

BOOL ComPortsDialog::OnInitDialog()
{

	CDialog::OnInitDialog();

	for (int i = mCOMCombo.GetCount() - 1; i >= 0; i--) // Delete all elements in ComboBox
	{
		mCOMCombo.DeleteString(i);
	}
	for (unsigned int i=0; i<pts.size(); i++)
	{
		CString a = pts[i].c_str();
		//a.erase(0, 4); // remove \\.\ 
		a.AppendFormat(_T(" (%s)"), ptnms[i].c_str());
		mCOMCombo.AddString(a);
		mCOMCombo.SetCurSel(mCOMCombo.GetCount()-1); // set to last one
		//mCOMCombo.SetCurSel(0); // set to first one
	}

	GotoDlgCtrl(&mCOMSelect);

	return FALSE;
}

// ComPortsDialog message handlers

//void ComPortsDialog::DetectComPorts(std::vector< tstring >& ports, size_t upperLimit /*=128*/)
//{
//	ports.clear(); // deletes all existing entries
//	for (size_t i = 1; i <= upperLimit; i++)
//	{
//		TCHAR strPort[32] = { 0 };
//		_stprintf_s(strPort, _T("COM%d"), i); // changed from \\\\.\\COM&d
//
//		DWORD dwSize = 0;
//		LPCOMMCONFIG lpCC = (LPCOMMCONFIG) new BYTE[1];
//		BOOL ret = GetDefaultCommConfig(strPort, lpCC, &dwSize);
//		delete[] lpCC;
//
//		lpCC = (LPCOMMCONFIG) new BYTE[dwSize];
//		ret = GetDefaultCommConfig(strPort, lpCC, &dwSize);
//		delete[] lpCC;
//
//		//BOOL ret = (CSerial::CheckPort(strPort) == CSerial::EPortAvailable);
//
//		if (ret) ports.push_back(strPort);
//	}
//}
void ComPortsDialog::DetectComPorts(std::vector< tstring >& ports, std::vector< tstring >& portnames) {
	ports.clear(); // deletes all existing entries
	portnames.clear();
	CEnumerateSerial::CPortAndNamesArray ptsvec;
	CEnumerateSerial::UsingSetupAPI1(ptsvec);
	CString a;
	for (const auto& p : ptsvec) {
		a.Format(_T("COM%d"),p.first);
		ports.push_back(a.GetBuffer());
		portnames.push_back(p.second.c_str());
	}
}

void ComPortsDialog::OnBnClickedSelectport()
{
	int s = mCOMCombo.GetCurSel();
	if (s == CB_ERR) mCOMCombo.GetWindowText(pp);
	else pp = pts[s].c_str();
	OnOK();
}
