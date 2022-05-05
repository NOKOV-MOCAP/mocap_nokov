// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the SEEKERSDKCLIENT_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// SEEKERSDKCLIENT_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.

#define _LINUX
//#define _WIN32
//系统环境
#if defined(_WIN32)
#define SYSTEM_WIN32
#elif defined(_LINUX)
#define SYSTEM_LINUX
#else
#error "undefined system!"
#endif

#ifdef SYSTEM_WIN32
#ifdef SEEKERSDKCLIENT_EXPORTS
#define SEEKERSDKCLIENT_API __declspec(dllexport)
#else
#define SEEKERSDKCLIENT_API __declspec(dllimport)
#endif
#elif defined(SYSTEM_LINUX)
#define SEEKERSDKCLIENT_API 
#endif

#include "SeekerSDKTypes.h"

class ClientCore;

// This class is exported from the SeekerSDKClient.dll
class SEEKERSDKCLIENT_API SeekerSDKClient {
public:
	SeekerSDKClient();
	~SeekerSDKClient();

	int Initialize(char* szServerAddress);
	int Uninitialize();
	void SeekerSDKVersion(unsigned char Version[4]);
	void SetVerbosityLevel(int level);

	int WaitForForcePlateInit(long time = 0);
	int SetForcePlateCallback(void (*CallbackFunction)(sForcePlates* pForcePlate, void* pUserData), void* pUserData = 0);

	int SetDataCallback(void (*CallbackFunction)(sFrameOfMocapData* pFrameOfData, void* pUserData), void* pUserData=0);
	int SetMessageCallback(void (*CallbackFunction)(int id, char *szTraceMessage));

	int GetServerDescription(sServerDescription *pServerDescription);

	int GetDataDescriptions(sDataDescriptions** pDataDescriptions);
	int FreeDataDescriptions(sDataDescriptions* pDataDescriptions);

	bool DecodeTimecode(unsigned int inTimecode, unsigned int inTimecodeSubframe, int* hour, int* minute, int* second, int* frame, int* subframe);
	bool TimecodeStringify(unsigned int inTimecode, unsigned int inTimecodeSubframe, char *Buffer, int BufferSize);

	int SeekerCopyFrame(const sFrameOfMocapData* pSrc, sFrameOfMocapData* pDst);
	int SeekerFreeFrame(sFrameOfMocapData* pDst);
	sFrameOfMocapData* GetLastFrameOfMocapData();


private:
	ClientCore* m_pClientCore;
};
