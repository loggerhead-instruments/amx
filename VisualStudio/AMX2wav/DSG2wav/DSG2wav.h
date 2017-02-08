// DSG2wav.h : main header file for the PROJECT_NAME application
//

#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"		// main symbols


// CDSG2wavApp:
// See DSG2wav.cpp for the implementation of this class
//

class CDSG2wavApp : public CWinApp
{
public:
	CDSG2wavApp();

// Overrides
	public:
	virtual BOOL InitInstance();

// Implementation

	DECLARE_MESSAGE_MAP()
};

extern CDSG2wavApp theApp;