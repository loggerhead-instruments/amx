// DSG2wavDlg.h : header file
//

#pragma once


// CDSG2wavDlg dialog
class CDSG2wavDlg : public CDialog
{
// Construction
public:
	CDSG2wavDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_DSG2WAV_DIALOG };

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
	afx_msg void OnBnClickedConvert();
	int SaveWave(CString amxfilename, CString wavfilename);
	afx_msg void OnBnClickedBatch();
	CString m_prefix;
	CString m_filename;
};
