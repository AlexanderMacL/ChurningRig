#pragma once


// CWalkDlg dialog

class CWalkDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CWalkDlg)

public:
	CWalkDlg(CWnd* pParent = nullptr);   // standard constructor
	virtual ~CWalkDlg();
	CWnd* mParent;
	HBRUSH CWalkDlg::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
	void CWalkDlg::PostNcDestroy();
	void CWalkDlg::OnCancel();


// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_WALKTHROUGH };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
};
