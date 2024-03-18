// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the NOKOVSDKCLIENT_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// NOKOVSDKCLIENT_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.

#define _LINUX
//#define _WIN32
#pragma once

#if defined(_WIN32)
#define SYSTEM_WIN32
#elif defined(_LINUX)
#define SYSTEM_LINUX
#else
#error "undefined system!"
#endif

#ifdef SYSTEM_WIN32
#ifdef NOKOVSDKCLIENT_EXPORTS
#define NOKOVSDKCLIENT_API __declspec(dllexport)
#else
#define NOKOVSDKCLIENT_API __declspec(dllimport)
#endif
#elif defined(SYSTEM_LINUX)
#define NOKOVSDKCLIENT_API __attribute ((visibility("default")))
#endif

#include "NokovSDKTypes.h"

class ClientCore;

class NOKOVSDKCLIENT_API NokovSDKClient {
public:
	NokovSDKClient();
	~NokovSDKClient();

	int Initialize(char* szServerAddress);
	int Uninitialize();
	void NokovSDKVersion(unsigned char Version[4]);
	void SetVerbosityLevel(int level);

	int WaitForForcePlateInit(long time = 0);
	int SetForcePlateCallback(void (*CallbackFunction)(sForcePlates* pForcePlate, void* pUserData), void* pUserData = 0);

	int SetDataCallback(void (*CallbackFunction)(sFrameOfMocapData* pFrameOfData, void* pUserData), void* pUserData=0);
	int SetMessageCallback(void (*CallbackFunction)(int id, char *szTraceMessage));
	int SetNotifyMsgCallback(void (*CallbackFunc)(sNotifyMsg* pNotify, void* pUserData), void* pUserData=0);

	int GetServerDescription(sServerDescription *pServerDescription);

	int GetDataDescriptions(sDataDescriptions** pDataDescriptions);
	int GetDataDescriptionsEx(sDataDescriptions** pDataDescriptions);
	int GetTposeDataDescriptions(char* name, sDataDescriptions** pDataDescriptions);
	int FreeDataDescriptions(sDataDescriptions* pDataDescriptions);

	bool DecodeTimecode(unsigned int inTimecode, unsigned int inTimecodeSubframe, int* hour, int* minute, int* second, int* frame, int* subframe);
	bool TimecodeStringify(unsigned int inTimecode, unsigned int inTimecodeSubframe, char *Buffer, int BufferSize);

	int NokovCopyFrame(const sFrameOfMocapData* pSrc, sFrameOfMocapData* pDst);
	int NokovFreeFrame(sFrameOfMocapData* pDst);
	sFrameOfMocapData* GetLastFrameOfMocapData();

private:
	ClientCore* m_pClientCore;
};
