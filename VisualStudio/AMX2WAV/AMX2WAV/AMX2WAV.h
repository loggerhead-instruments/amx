
// AMX2WAV.h : main header file for the PROJECT_NAME application
//

#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"		// main symbols


// CAMX2WAVApp:
// See AMX2WAV.cpp for the implementation of this class
//

class CAMX2WAVApp : public CWinApp
{
public:
	CAMX2WAVApp();

// Overrides
public:
	virtual BOOL InitInstance();

// Implementation

	DECLARE_MESSAGE_MAP()
};

extern CAMX2WAVApp theApp;