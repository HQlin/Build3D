////////////////////////////////////////////////////////////////////////////////////////
// 
//  Project:    Class ComAccess
//              Overlapped serial IO communication class
//  System:     Win9x WinNT
//  File:       comaccess.cpp
//  Start date:	17.11.1997
//  Update:     31.07.1998
//  Version:    1.2
//  Author:     Patrick Feuser pat@das-netz.de Germany
//  Copyright ?1997. Alle Rechte vorbehalten
//
////////////////////////////////////////////////////////////////////////////////////////
#include "StdAfx.h"
#include "comaccess.h"

////////////////////////////////////////////////////////////////////////////////////////
// 
//  Constructors
//

ComAccess::ComAccess(VOID)
{
	m_hCom = 0;
	m_lpszErrorMessage[0] = '\0';
	ZeroMemory(&m_ov, sizeof(m_ov));
}


ComAccess::ComAccess(LPCWSTR lpszPortNum)
{ 
	m_hCom = 0;
	m_lpszErrorMessage[0] = '\0';
	ZeroMemory(&m_ov, sizeof(m_ov));
	ComAccess::Open(lpszPortNum); 
}

////////////////////////////////////////////////////////////////////////////////////////
// 
//  Function:      Open(LPCSTR lpszPortNum,
//                      DWORD  dwBaudRate, 
//                      BYTE   byParity,
//                      BYTE   byStopBits,
//                      BYTE   byByteSize)
//
//  Return value:  BOOL TRUE or FALSE
//

BOOL ComAccess::Open(LPCWSTR lpszPortNum, 
                     DWORD  dwBaudRate, 
                     BYTE   byParity,
                     BYTE   byStopBits,
					 BYTE   byByteSize)
{
	DCB  dcb; // structure that defines the control setting for a serial communications device
	BOOL bSuccess;
	
	m_hCom = CreateFile(lpszPortNum,           // pointer to name of the file
	                    GENERIC_READ|GENERIC_WRITE, // access mode
	                    0,                     // comm devices must be opened w/exclusive-access 
	                    NULL,                  // no security attributs 
	                    OPEN_EXISTING,         // comm devices must use OPEN_EXISTING 
	                    FILE_FLAG_OVERLAPPED,  // overlapped I/O
	                    NULL);                 // hTemplate must be NULL for comm devices 

	if ( m_hCom == INVALID_HANDLE_VALUE ) 
	{
		// handle the error
		ComAccess::ErrorToString("Open(): CreateFile() failed, invalid handle value");
		
		return FALSE;
	}

	//
	// Omit the call to SetupComm to use the default queue sizes.
	// Get the current configuration.
	//
 
	bSuccess = GetCommState(m_hCom, &dcb);

	if ( ! bSuccess ) 
	{
		// Handle the error.
		ComAccess::ErrorToString("Open(): GetCommState() failed");
		ComAccess::Close();

		return FALSE;
	}

	//
	// Fill in the DCB: baud=9600, 8 data bits, no parity, 1 stop bit are default parameters
	//

	dcb.BaudRate = dwBaudRate;
	dcb.ByteSize = byByteSize;
	dcb.Parity   = byParity;
	dcb.StopBits = byStopBits;
	
	bSuccess = SetCommState(m_hCom, &dcb);

	if ( ! bSuccess ) 
	{
		// Handle the error. 
		ComAccess::ErrorToString("Open(): SetCommState() failed");
		ComAccess::Close();

		return FALSE;
	}

	return TRUE;
}

////////////////////////////////////////////////////////////////////////////////////////
// 
//  Function:      Close(VOID)
//
//  Return value:  VOID
//

VOID ComAccess::Close(VOID)
{
	if ( m_hCom > 0 )
	{
		CloseHandle(m_hCom);
	}

	m_hCom = 0; 
}

////////////////////////////////////////////////////////////////////////////////////////
// 
//  Function:     WriteData(LPCVOID pdata, 
//                          DWORD   len)
//
//  Return value:  DWORD -1 failed, above, num written bytes
//

DWORD ComAccess::WriteData(LPCVOID pdata, 
                           DWORD   len)
{
	BOOL  bSuccess;
	DWORD written = 0;

	if ( len < 1 )
		return(0);

	// create event for overlapped I/O
	m_ov.hEvent = CreateEvent(NULL,   // pointer to security attributes 
	                          FALSE,   // flag for manual-reset event 
	                          FALSE,  // flag for initial state 
	                          L"");    // pointer to event-object name 

	if ( m_ov.hEvent == INVALID_HANDLE_VALUE )
	{
		// Handle the error.
		ComAccess::ErrorToString("WriteData(): CreateEvent() failed");
	  
		return(-1);
	}

	bSuccess = WriteFile(m_hCom,   // handle to file to write to  
	                     pdata,    // pointer to data to write to file 
	                     len,      // number of bytes to write 
	                     &written, // pointer to number of bytes written 
	                     &m_ov);   // pointer to structure needed for overlapped I/O


	if ( ComAccess::IsNT() )
	{
		bSuccess = GetOverlappedResult(m_hCom, &m_ov, &written, TRUE);

		if ( ! bSuccess )
		{
			// Handle the error.
			CloseHandle(m_ov.hEvent);
			ComAccess::ErrorToString("WriteData(): GetOverlappedResult() failed");

			return(-1);
		}
	}
	else
	if ( len != written )
	{
		// Handle the error.
		CloseHandle(m_ov.hEvent);
		ComAccess::ErrorToString("WriteData(): WriteFile() failed");

		return(-1);
	}

	CloseHandle(m_ov.hEvent);

	return written;
}

////////////////////////////////////////////////////////////////////////////////////////
// 
//  Function:      ReadData(LPVOID pdest, 
//	                        DWORD  len, 
//                          DWORD  dwMaxWait)
//
//  Return value:  DWORD -1 failed, above, num read bytes		
//

DWORD ComAccess::ReadData(LPVOID pdest, 
                          DWORD  len, 
                          DWORD  dwMaxWait)
{
	BOOL  bSuccess;
	DWORD result = 0,
	      read   = 0, // num read bytes
	      mask   = 0; // a 32-bit variable that receives a mask 
	                  // indicating the type of event that occurred

	if ( len < 1 ) return(0);

	// create event for overlapped I/O

	m_ov.hEvent = CreateEvent(NULL,   // pointer to security attributes 
	                          FALSE,   // flag for manual-reset event 
	                          FALSE,  // flag for initial state 
	                          L"");    // pointer to event-object name 

	if ( m_ov.hEvent == INVALID_HANDLE_VALUE )
	{
		// Handle the error.
		ComAccess::ErrorToString("ReadData(): CreateEvent() failed");
	  
		return(-1);
	}

	// Specify here the event to be enabled

	bSuccess = SetCommMask(m_hCom, EV_RXCHAR);

	if ( ! bSuccess )
	{
		// Handle the error.
		CloseHandle(m_ov.hEvent);
		ComAccess::ErrorToString("ReadData(): SetCommMask() failed");
	  
		return(-1);
	}

	
	// WaitForSingleObject

	bSuccess = WaitCommEvent(m_hCom, &mask, &m_ov);

	if ( ! bSuccess )
	{
		int err = GetLastError();

		if ( err == ERROR_IO_PENDING)
		{
			result = WaitForSingleObject(m_ov.hEvent, dwMaxWait);  //wait dwMaxWait
		                                        // milli seconds before returning
			if ( result == WAIT_FAILED )
			{
				// Handle the error.
				CloseHandle(m_ov.hEvent);
				ComAccess::ErrorToString("ReadData(): WaitForSingleObject() failed");
	  
				return(-1);
			}
		}
	}
	
	// The specified event occured?
	
	if ( mask & EV_RXCHAR) 
	{
		bSuccess = ReadFile(m_hCom, // handle of file to read 
			                pdest,  // address of buffer that receives data 
				            len,    // number of bytes to read 
					        &read,  // address of number of bytes read 
						    &m_ov); // address of structure for data 

		if ( ComAccess::IsNT() )
		{
			bSuccess = GetOverlappedResult(m_hCom, &m_ov, &read, TRUE);

			if ( ! bSuccess )
			{
				// Handle the error.
				CloseHandle(m_ov.hEvent);
				ComAccess::ErrorToString("WriteData(): GetOverlappedResult() failed");

				return(-1);
			}
		}
		else
		if ( ! bSuccess )
		{
			// Handle the error.
			CloseHandle(m_ov.hEvent);
			ComAccess::ErrorToString("ReadData(): ReadFile() failed");

			return(-1);
		}
	}
	else
	{
		// Handle the error.
		CloseHandle(m_ov.hEvent);
		ComAccess::ErrorToString("Error ReadData(): No EV_RXCHAR occured\n");

		return(-1);
	}
	
	CloseHandle(m_ov.hEvent);
	
	return read;
}

////////////////////////////////////////////////////////////////////////////////////////
// 
//  Function:      ErrorToString(LPCSTR lpszMessage)
//
//  Return value:  VOID
//

VOID ComAccess::ErrorToString(LPCSTR lpszMessage)
{
	LPVOID lpMessageBuffer;
	DWORD  error = GetLastError();

	FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER |
	              FORMAT_MESSAGE_FROM_SYSTEM,      // source and processing options
	              NULL,                            // pointer to message source
	              error,                           // requested message identifie
	              MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), // the user default language.
	              (LPTSTR) &lpMessageBuffer,       // pointer to message buffer
	              0,                               // maximum size of message buffer
	              NULL);                           // address of array of message inserts 

	// and copy it in our error string
	printf(m_lpszErrorMessage,"%s: (%d) %s\n", lpszMessage, error, lpMessageBuffer);

	LocalFree(lpMessageBuffer);
}

////////////////////////////////////////////////////////////////////////////////////////
// 
//  Function:     IsNT(VOID)
//
//  Return value: BOOL True or False
//

BOOL ComAccess::IsNT(VOID)
{
    OSVERSIONINFO osvi;
    
    osvi.dwOSVersionInfoSize = sizeof(OSVERSIONINFO);
    
    GetVersionEx(&osvi);

    if (osvi.dwPlatformId == VER_PLATFORM_WIN32_NT) 
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

CString genLightContral(CString id, int lightVal)
{
	CString str;
	str.Format(L"S%s0%3d#", id, lightVal);
	return str;
}
