
// AMX2WAVDlg.cpp : implementation file
//

#include "stdafx.h"
#include "AMX2WAV.h"
#include "AMX2WAVDlg.h"
#include "afxdialogex.h"
#include "wav.h"
#include "amx32.h"
#include "direct.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


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


// CAMX2WAVDlg dialog



CAMX2WAVDlg::CAMX2WAVDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_AMX2WAV_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CAMX2WAVDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_FILENAME, m_filename);
}

BEGIN_MESSAGE_MAP(CAMX2WAVDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BATCH_CONVERT, &CAMX2WAVDlg::OnBnClickedBatchConvert)
	ON_BN_CLICKED(IDC_EXPORT_DEBUG, &CAMX2WAVDlg::OnBnClickedExportDebug)
END_MESSAGE_MAP()


// CAMX2WAVDlg message handlers

BOOL CAMX2WAVDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
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

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CAMX2WAVDlg::OnSysCommand(UINT nID, LPARAM lParam)
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

void CAMX2WAVDlg::OnPaint()
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
HCURSOR CAMX2WAVDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CAMX2WAVDlg::OnBnClickedBatchConvert()
{
	UpdateData(TRUE);
	m_filename = CString("Processing");
	UpdateData(FALSE);

	CString amxfilename, wavfilename, temppath, root, pathamx;
	int success;
	//launch file open dialog to get folder to read
	CFileDialog fileDlg(TRUE, NULL, TEXT("*.amx"), NULL);

	if (fileDlg.DoModal() == IDOK)
	{
		amxfilename = fileDlg.GetPathName();  //returns path + filename
		int sst;
		sst = amxfilename.ReverseFind('\\');
		root = amxfilename.Left(sst);
		//CString foldername("%s\\Script");	 
	}


	CString  wavfilenamesel("\\wav\\");
	temppath = root + wavfilenamesel;
	pathamx = root + _T("\\*.amx");
	CreateDirectory(temppath, NULL);

	AfxGetApp()->DoWaitCursor(1);
	WIN32_FIND_DATA FindFileData;
	HANDLE hFind;

	hFind = FindFirstFile(pathamx.GetBuffer(), &FindFileData);
	if (hFind == INVALID_HANDLE_VALUE)
	{
		m_filename = CString("Done");
		UpdateData(FALSE);
		AfxMessageBox(_T("Unable to find AMX files."), MB_OK, NULL);
		return;
	}

	int fsuccess = 1;
	while (fsuccess)
	{
		//m_prefix is file prefix
		m_filename = FindFileData.cFileName;
		UpdateData(FALSE);
		amxfilename = root + _T("\\") + m_filename;
		wavfilename = root + wavfilenamesel + FindFileData.cFileName;
		SaveWave(amxfilename , wavfilename);

		fsuccess = FindNextFile(hFind, &FindFileData);
	}
	FindClose(hFind);
	AfxGetApp()->DoWaitCursor(0);
	m_filename = CString("Done");
	UpdateData(FALSE);
}


int CAMX2WAVDlg::SaveWave(CString amxfilename, CString wavfilename)
{
	// There will be a wav file for each sensor with the number of channels 
	// corresponding to the number of channels in that sensor
	CFile amxFile, wavFile[7];
	AMX_DF amx_df;
	AMX_SID_REC amx_sid_rec;
	AMX_SID_SPEC amx_sid_spec[8];
	short data[4096];
	float data32[4096];
	UINT bytesread;
	CString wavfname[7];

	HdrStruct wav_hdr[7];  //wav header
	float speriod[7];
	float srate[7];
	long ptsrecorded[7];
	CFileException e;
	if (!amxFile.Open(amxfilename, CFile::modeRead, &e))
	{
		//unable to open amx file to read
		AfxMessageBox(TEXT("Unable to open amx file."));
		return(0);
	}

	// AMX version
	bytesread = amxFile.Read(&amx_df, sizeof(amx_df));

	//Read in SID_SPECs until get all 0's
	int n = 0;
	do
	{
		bytesread = amxFile.Read(&amx_sid_spec[n], sizeof(amx_sid_spec[0]));
		speriod[n] = 1.0 / (double)amx_sid_spec[n].srate;  //for open tag SP256 is in us
		srate[n] = amx_sid_spec[n].srate;
		n++;
	} while (amx_sid_spec[n - 1].nSamples != 0);
	int numsids = n - 1;

	//Save one wav file per SID
	for (n = 0; n < numsids; n++)
	{
		wavfname[n] = wavfilename;
		wavfname[n].Format(_T("%s_%c%c_HMS_%2d_%2d_%2d__DMY_%2d_%2d_%2d.wav"), wavfname[n], amx_sid_spec[n].SID[0], amx_sid_spec[n].SID[1], amx_df.RecStartTime.hour, amx_df.RecStartTime.minute, amx_df.RecStartTime.sec, amx_df.RecStartTime.day,
			amx_df.RecStartTime.month, amx_df.RecStartTime.year);

		//open wav file for writing
		if (!wavFile[n].Open(wavfname[n], CFile::modeCreate | CFile::modeWrite, &e))
		{
			//unable to open wav file to write
			AfxMessageBox(TEXT("Unable to open wav file for writing"));
			return(0);
		}

		//intialize .wav file header	
		int bitsPerSample, formatTag;
		if (amx_sid_spec[n].dForm == 2) {
			bitsPerSample = 16;
			formatTag = 1;
		}
		if (amx_sid_spec[n].dForm == 5) {
			bitsPerSample = 32;
			formatTag = 3; //http://www-mmsp.ece.mcgill.ca/Documents/AudioFormats/WAVE/WAVE.html
		}

		sprintf_s(wav_hdr[n].rId, 10, "RIFF");
		wav_hdr[n].rLen = 36;
		sprintf_s(wav_hdr[n].wId, 10, "WAVE");
		sprintf_s(wav_hdr[n].fId, 10, "fmt ");
		wav_hdr[n].fLen = 0x10;
		wav_hdr[n].nFormatTag = formatTag;
		wav_hdr[n].nChannels = amx_sid_spec[n].sensor.nChan;
		wav_hdr[n].nSamplesPerSec = srate[n];
		wav_hdr[n].nAvgBytesPerSec = srate[n] * bitsPerSample * amx_sid_spec[n].sensor.nChan / 8;
		wav_hdr[n].nBlockAlign = bitsPerSample * amx_sid_spec[n].sensor.nChan / 8;
		wav_hdr[n].nBitsPerSamples = bitsPerSample;
		sprintf_s(wav_hdr[n].dId, 10, "data");
		wav_hdr[n].dLen = 0;
		ptsrecorded[n] = 0;
		wavFile[n].Write(&wav_hdr[n], 44);
	}

	//loop through file reading and writing in chunks
	UINT nbytes, nsamples;
	do
	{
		bytesread = amxFile.Read(&amx_sid_rec.nSID, 4);  // Read SID_REC Header
		bytesread = amxFile.Read(&amx_sid_rec.NU, 12);
		n = amx_sid_rec.nSID; //easier to read

		int bytesPerSample;
		// int16
		if (amx_sid_spec[n].dForm == 2) {
			bytesread = amxFile.Read(data, amx_sid_spec[n].nSamples * 2);
			//byte swap IMU reads
			if (amx_sid_spec[n].SID[0] == '3' | amx_sid_spec[n].SID[0] == 'I') {
				for (int n = 0; n < bytesread; n++) {
					byte hibyte = (data[n] & 0xFF00) >> 8;
					byte lobyte = (data[n] & 0xFF);
					data[n] = lobyte << 8 | hibyte;
				}
			}
			nsamples = bytesread / 2;
			bytesPerSample = 2;
		}

		// float32
		if (amx_sid_spec[n].dForm == 5) {
			bytesread = amxFile.Read(data32, amx_sid_spec[n].nSamples * 4);
			nsamples = bytesread / 4;
			bytesPerSample = 4;
		}

		if ((bytesread != 0) & amx_sid_rec.nSID < numsids)
		{
			//update wav file header
			ptsrecorded[n] += nsamples;
			wav_hdr[n].rLen = 36 + (ptsrecorded[n] * bytesPerSample);
			wav_hdr[n].dLen = ptsrecorded[n] * bytesPerSample;
			wavFile[n].SeekToBegin();
			wavFile[n].Write(&wav_hdr[n], 44);
			//go to end and write data
			wavFile[n].SeekToEnd();
			if (bytesPerSample == 2) {
				wavFile[n].Write(data, bytesread);
			}
			else {
				wavFile[n].Write(data32, bytesread);
			}

		}
	} while (bytesread > 0);

	amxFile.Close();
	for (n = 0; n < numsids; n++)
	{
		wavFile[n].Close();
	}
	return(1);
}

void CAMX2WAVDlg::OnBnClickedExportDebug()
{
	UpdateData(TRUE);
	m_filename = CString("Processing Debug Info");
	UpdateData(FALSE);

	CString amxfilename, debugfilename, temppath, root, pathamx;
	int success;
	//launch file open dialog to get folder to read
	CFileDialog fileDlg(TRUE, NULL, TEXT("*.amx"), NULL);

	if (fileDlg.DoModal() == IDOK)
	{
		amxfilename = fileDlg.GetPathName();  //returns path + filename
		int sst;
		sst = amxfilename.ReverseFind('\\');
		root = amxfilename.Left(sst);
		//CString foldername("%s\\Script");	 
	}


	CString  wavfilenamesel("\\debug\\");
	temppath = root + wavfilenamesel;
	pathamx = root + _T("\\*.amx");
	CreateDirectory(temppath, NULL);

	AfxGetApp()->DoWaitCursor(1);
	WIN32_FIND_DATA FindFileData;
	HANDLE hFind;

	hFind = FindFirstFile(pathamx.GetBuffer(), &FindFileData);
	if (hFind == INVALID_HANDLE_VALUE)
	{
		m_filename = CString("Done");
		UpdateData(FALSE);
		AfxMessageBox(_T("Unable to find AMX files."), MB_OK, NULL);
		return;
	}

	int fsuccess = 1;
	while (fsuccess)
	{
		//m_prefix is file prefix
		m_filename = FindFileData.cFileName;
		UpdateData(FALSE);
		amxfilename = root + _T("\\") + m_filename;
		debugfilename = root + wavfilenamesel + FindFileData.cFileName + _T(".txt");
		SaveDebugInfo(amxfilename, debugfilename);

		fsuccess = FindNextFile(hFind, &FindFileData);
	}
	FindClose(hFind);
	AfxGetApp()->DoWaitCursor(0);
	m_filename = CString("Done");
	UpdateData(FALSE);
}

int CAMX2WAVDlg::SaveDebugInfo(CString amxfilename, CString debugfilename)
{
	// There will be a text file containing order in which records were written
	CFile amxFile;
	CStdioFile debugFile;
	AMX_DF amx_df;
	AMX_SID_REC amx_sid_rec;
	AMX_SID_SPEC amx_sid_spec[8];
	short data[4096];
	float data32[4096];
	UINT bytesread;

	float speriod[7];
	float srate[7];
	long ptsrecorded[7];
	CFileException e;

	if (!debugFile.Open(debugfilename, CStdioFile::modeCreate | CStdioFile::modeWrite, &e)) {
		AfxMessageBox(TEXT("Unable to open debug output file."));
		return(0);
	}
	if (!amxFile.Open(amxfilename, CFile::modeRead, &e))
	{
		//unable to open amx file to read
		AfxMessageBox(TEXT("Unable to open amx file."));
		debugFile.Close();
		return(0);
	}

	// AMX version
	bytesread = amxFile.Read(&amx_df, sizeof(amx_df));

	// write DF_HEAD to file
	/*
	ULONG Version; // firmware version
	ULONG UserID;  //tag type
	AMX_TIME RecStartTime;
	float voltage;
	*/
	CString buf;
	buf.Format(_T("Version: %d\n"), amx_df.Version);
	debugFile.WriteString(buf);

	buf.Format(_T("Start Time: %04d-%02d-%02dT%02d:%02d:%02d\n"), amx_df.RecStartTime.year,
		amx_df.RecStartTime.month, 
		amx_df.RecStartTime.day, 
		amx_df.RecStartTime.hour, 
		amx_df.RecStartTime.minute, 
		amx_df.RecStartTime.sec);
	debugFile.WriteString(buf);

	buf.Format(_T("voltage: %d\n"), amx_df.voltage);
	debugFile.WriteString(buf);


	//Read in SID_SPECs until get all 0's
	int n = 0;
	do
	{
		bytesread = amxFile.Read(&amx_sid_spec[n], sizeof(amx_sid_spec[0]));

		/*
			char	SID[STR_MAX];
			INT16 sidType; // 0 = raw, 1 = summary  histogram
			INT16 NU;
			ULONG 	nSamples;	  // Size in samples of this record (excluding header)
			AMX_SENSOR  sensor;	  // used to encode what data are saved: bitmask bit (accel3, mag3, gyro3, press, temperature, mic)
			ULONG	dForm;	  // short, long, or I24
			float	srate;	  // Sample rate (Hz)
		*/
		/*
			{
				char chipName[STR_MAX]; // name of sensor e.g. MPU9250
				INT16 nChan;            //number of channels used (e.g. MPU9250 might have 9 for accel, mag, and gyro)
				INT16 NU;
				char name[12][STR_MAX]; //name of each channel (e.g. accelX, gyroZ). Max of 12 channels per chip.
				char units[12][STR_MAX];// units of each channel (e.g. g, mGauss, degPerSec)
				float cal[12];          //calibration coefficient for each channel when multiplied by this value get data in specified units
			}AMX_SENSOR;
		*/
		// write SID_SPEC to file
		buf.Format(_T("\nSID: %s\n"), CString(amx_sid_spec[n].SID));
		debugFile.WriteString(buf);
		buf.Format(_T("samples: %d\n"), amx_sid_spec[n].nSamples);
		debugFile.WriteString(buf);
		buf.Format(_T("sample rate: %f\n"), amx_sid_spec[n].srate);
		debugFile.WriteString(buf);
		buf.Format(_T("chip name: %s\n"), CString(amx_sid_spec[n].sensor.chipName));
		debugFile.WriteString(buf);
		buf.Format(_T("channels: %d\n"), amx_sid_spec[n].sensor.nChan);
		debugFile.WriteString(buf);
		for (int i = 0; i < amx_sid_spec[n].sensor.nChan; i++) {
			buf.Format(_T("%s calibration: %f\n"), CString(amx_sid_spec[n].sensor.name[i]), amx_sid_spec[n].sensor.cal[i]);
			debugFile.WriteString(buf);
		}



		speriod[n] = 1.0 / (double)amx_sid_spec[n].srate;  //for open tag SP256 is in us
		srate[n] = amx_sid_spec[n].srate;
		n++;

	} while (amx_sid_spec[n - 1].nSamples != 0);
	int numsids = n - 1;

	debugFile.Close();
	
	if (!debugFile.Open(debugfilename + _T(".csv"), CStdioFile::modeCreate | CStdioFile::modeWrite, &e)) {
		AfxMessageBox(TEXT("Unable to open debug output csv file."));
		return(0);
	}
	buf.Format(_T("sensor,bufCount\n"));
	debugFile.WriteString(buf);

	//loop through file reading and writing in chunks
	UINT nbytes, nsamples;
	int previousSid = -1;
	int sidCounter = 0;
	do
	{
		bytesread = amxFile.Read(&amx_sid_rec.nSID, 4);  // Read SID_REC Header
		bytesread = amxFile.Read(&amx_sid_rec.NU, 12);
		n = amx_sid_rec.nSID; //easier to read

		// keep track of count of sequential sid buffers to help track down dropped samples
		if (n != previousSid) {
			// write out previous sidCount
			// write SID_REC Header and indicate which SID_SPEC was recorded
			if (previousSid > -1) {
				buf.Format(_T("%s, %d\n"), CString(amx_sid_spec[previousSid].SID), sidCounter);
				debugFile.WriteString(buf);
			}
			sidCounter = 1;
			previousSid = n;
		}
		else {
			sidCounter++;
		}

		int bytesPerSample;
		// int16
		if (amx_sid_spec[n].dForm == 2) {
			bytesread = amxFile.Read(data, amx_sid_spec[n].nSamples * 2);
			//byte swap IMU reads
			if (amx_sid_spec[n].SID[0] == '3' | amx_sid_spec[n].SID[0] == 'I') {
				for (int n = 0; n < bytesread; n++) {
					byte hibyte = (data[n] & 0xFF00) >> 8;
					byte lobyte = (data[n] & 0xFF);
					data[n] = lobyte << 8 | hibyte;
				}
			}
			nsamples = bytesread / 2;
			bytesPerSample = 2;
		}

		// float32
		if (amx_sid_spec[n].dForm == 5) {
			bytesread = amxFile.Read(data32, amx_sid_spec[n].nSamples * 4);
			nsamples = bytesread / 4;
			bytesPerSample = 4;
		}

	} while (bytesread > 0);

	amxFile.Close();
	debugFile.Close();
	return(1);
}