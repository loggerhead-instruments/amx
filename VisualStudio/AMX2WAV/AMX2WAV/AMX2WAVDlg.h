
// AMX2WAVDlg.h : header file
//

#pragma once


// CAMX2WAVDlg dialog
class CAMX2WAVDlg : public CDialogEx
{
// Construction
public:
	CAMX2WAVDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_AMX2WAV_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


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
	afx_msg void OnBnClickedBatchConvert();
	int SaveWave(CString root, CString amxfilename, CString wavfilename);
	int SaveDebugInfo(CString amxfilename, CString debugfilename);
	// dialog status text
	CString m_filename;
	afx_msg void OnBnClickedExportDebug();
};
