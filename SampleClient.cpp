#ifndef _LINUX
// Windows
#include <stdio.h>
#include <tchar.h>
#include <conio.h>
#include <winsock2.h>

#include <iostream>

#include <fstream>
#include <chrono>
#include <vector>
#include <unordered_map>

#pragma warning( disable : 4996 )

#else
// Linux 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#ifdef __APPLE__
#include <unistd.h>
#else
#include <linux/unistd.h>
#endif
#include <dirent.h>
#include <iostream>
#include <netdb.h>   
#include <sys/socket.h>
#include <arpa/inet.h> 
#include <net/if.h>
#include <sys/ioctl.h>
#include <vector>
#include <unordered_map>
#define MAX_PATH 256
#endif

#include "./deps/nokov_sdk/include/NokovSDKTypes.h"
#include "./deps/nokov_sdk/include/NokovSDKClient.h"
#include "./deps/nokov_sdk/include/Utility.h"
#include <thread>

using namespace std;
#ifndef _LINUX
void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData);		// receives data from the server
void __cdecl MessageHandler(int msgType, char* msg);		            // receives NokovSDK error messages
void __cdecl ForcePlateHandler(sForcePlates* pForcePlate, void* pUserData); // receives force plate data from the server
void __cdecl NotifyHandler(sNotifyMsg* data, void* pUserData);
#else
void  __attribute__((__cdecl__)) ForcePlateHandler(sForcePlates* data, void* pUserData);	// receives force plate data from the server
void  __attribute__((__cdecl__)) DataHandler(sFrameOfMocapData* data, void* pUserData);		// receives data from the server
void  __attribute__((__cdecl__)) MessageHandler(int msgType, char* msg);		            // receives NokovSDK error messages
void  __attribute__((__cdecl__)) NotifyHandler(sNotifyMsg* data, void* pUserData);

#endif
int CreateClient(char* serverIp);
void printData(sFrameOfMocapData* data, NokovSDKClient* pClient);
void printPacketDescriptions(sDataDescriptions* pData);
void getTposeDescriptions();

unsigned int MyServersDataPort = 3130;
unsigned int MyServersCommandPort = 3131;
int bExit = 0;
NokovSDKClient* theClient;
char szMyIPAddress[128] = "";
char szServerIPAddress[128] = "";

typedef std::vector<std::vector<SlideFrameArray>> TrackerArray;
// Used to calculate velocity and acceleration
TrackerArray MarkerVelocityTrackerArray;
TrackerArray MarkerAccelerationTrackerArray;
std::vector<SlideFrameArray> BoneVelocityTrackerArray;
std::vector<SlideFrameArray> BoneAccelerationTrackerArray;
typedef std::unordered_map<int, std::string> IdNameMap;
IdNameMap g_RigidBodyIdMap;
IdNameMap g_SkeletonIdMap;
std::unordered_map<int, IdNameMap> g_SkeletonRigidsIdMp;

#ifdef _LINUX
int get_localip(const char * eth_name, char *local_ip_addr)
{	
	int ret = -1;    
	register int fd;    
	struct ifreq ifr; 	
	if (local_ip_addr == NULL || eth_name == NULL)
	{
		return ret;
	}
	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) > 0)
	{
		strcpy (ifr.ifr_name, eth_name);
		if (!(ioctl(fd, SIOCGIFADDR, &ifr)))
		{
			ret = 0;
			strcpy(local_ip_addr, inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
			printf("Found %s:%s\n", eth_name, local_ip_addr);
		}
	}
	if (fd > 0)
	{
		close(fd);
	}
	return ret;
}
#endif

#ifndef _LINUX
int _tmain(int argc, _TCHAR* argv[])
#else
int main(int argc, char* argv[])
#endif
{
	int iResult = -1;

#ifdef LOST_FRAME_TEST
	memset(&s_Record, 0x00, sizeof(s_Record));
#endif

	//1.init wsa
#ifndef _LINUX	
	WSADATA wsaData;
	int ret = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (ret != 0)
	{
		return false;
	}
	//2.Get host name
	char hostname[256];
	ret = gethostname(hostname, sizeof(hostname));
	if (ret == SOCKET_ERROR)
	{
		return false;
	}
	//3.Get host IP
	HOSTENT* host = gethostbyname(hostname);
	if (host == NULL)
	{
		return false;
	}
	//strcpy(szMyIPAddress, inet_ntoa(*(in_addr*)*host->h_addr_list));

	printf("found all ip address .[Please ensure there is an ip address in the same network with the server]\n");
	for (int iCount = 0; host->h_addr_list[iCount] != NULL; ++iCount)
	{
		//strcpy(szMyIPAddress, inet_ntoa(*(in_addr*)*host->h_addr_list[iCount]));
		strcpy(szMyIPAddress, inet_ntoa(*(in_addr*)host->h_addr_list[iCount]));
		printf("%d)[%s] .\n", iCount + 1, szMyIPAddress);
	}

#else
#endif	
	//printf("Client gethostbyname szMyIPAddress[%s] .\n", szMyIPAddress);
	char ipBuf[100] = {0};
	printf("Please input the server IP\n");
	scanf("%s", &ipBuf);

	//4.Create NokovSDK Client
	iResult = CreateClient(ipBuf);
	if (iResult != ErrorCode_OK)
	{
		printf("Error initializing client.  Exiting");
#ifndef _LINUX
		getch();
#else
		getchar();
		getchar();
#endif		
		return 1;
	}
	else
	{
		printf("Client initialized and ready.\n");
	}

#ifndef _LINUX
	getch();
#else
	getchar();
	getchar();
#endif	

	// Done - clean up.
	theClient->Uninitialize();

	return ErrorCode_OK;
}

void readData(void* param)
{
	if (param == NULL) {
		return;
	}
	NokovSDKClient* client = (NokovSDKClient*)param;
	sFrameOfMocapData* data = NULL;
	while (!bExit) {
		data = client->GetLastFrameOfMocapData();
		if (data) {
			printData(data, client);
		}
	}

	return;
}


// Establish a NokovSDK Client connection
int CreateClient(char* szServerIP)
{
	// release previous server
	if(theClient)
	{
		theClient->Uninitialize();
		delete theClient;
	}

	// create NokovSDK client
	theClient = new NokovSDKClient();

	// print version info
	unsigned char ver[4];
	theClient->NokovSDKVersion(ver);
	printf("NokovSDK Sample Client 2.4.0.5156(NokovSDK ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

	// Set callback handlers
	//theClient->SetMessageCallback(MessageHandler);
	//theClient->SetVerbosityLevel(Verbosity_Error);
	// Init Client and connect to NokovSDK server
	int retCode = -1;
	retCode = theClient->Initialize(szServerIP);//szMyIPAddress��Client IP (local IP)��szServerIP��Server IP (computer IP running motion capture software)
	//Sleep(10);
	memcpy(szServerIPAddress, szServerIP, sizeof(szServerIP));

	if (retCode != ErrorCode_OK)
	{
		printf("Unable to connect to server.Error code: %d. Exiting\n", retCode);
		return ErrorCode_Internal;
	}

	// print server info
	sServerDescription ServerDescription;
	memset(&ServerDescription, 0, sizeof(ServerDescription));
	theClient->GetServerDescription(&ServerDescription);
	if(!ServerDescription.HostPresent)
	{
		printf("Unable to connect to server. Host not present. Exiting.\n");
		return 1;
	}
	printf("Successfully connected to server\n");
	printf("\n1: Callback passive receiving data\n2: Host reading data\nEnter 1 or 2 and select: ");
#ifndef _LINUX
	char ch = getch();
#else
	getchar();
	char ch = getchar();
#endif

	sDataDescriptions* ps = nullptr;
	theClient->GetDataDescriptions(&ps);
	if (ps)
	{
		printPacketDescriptions(ps);
		theClient->FreeDataDescriptions(ps);
		ps = nullptr;
	}

	//getTposeDescriptions();
	theClient->SetNotifyMsgCallback(NotifyHandler, theClient);
	theClient->WaitForForcePlateInit();
	//theClient->SetDataCallback(DataHandler, theClient);	// this function will receive data from the server
	theClient->SetForcePlateCallback(ForcePlateHandler, theClient);
	if (ch == '1') {
		printf("\nselect: %c\n", ch);
		theClient->SetDataCallback(DataHandler, theClient);	// this function will receive data from the server
	}
	else {
		printf("\nselect: %c\n", ch);
		std::thread t(readData, (void*)theClient);
		t.detach();
	}
	
	return ErrorCode_OK;
}

void printPacketDescriptions(sDataDescriptions* pData)
{
	if (!pData)
	{
		return;
	}
	for (int dsIndex = 0; dsIndex < pData->nDataDescriptions; ++dsIndex)
	{
		const auto& dataDescription = pData->arrDataDescriptions[dsIndex];

		switch (dataDescription.type)
		{
		case Descriptor_MarkerSetEx:
			printf("MarkerSetExName : %s\n", dataDescription.Data.MarkerSetData->szName);
			for (int markerIndex = 0; markerIndex < dataDescription.Data.MarkerSetData->nMarkers; ++markerIndex)
			{
				MarkerData& md = dataDescription.Data.MarkerSetData->Markers[markerIndex];
				printf("Marker[%d] : %f,%f,%f\n", markerIndex, md[0], md[1], md[2]);
			}
			break;
		case Descriptor_MarkerSet:
			printf("MarkerSetName : %s\n", dataDescription.Data.MarkerSetDescription->szName);
#ifdef LOST_FRAME_TEST
			s_Record.m_MarkerNums += dataDescription.Data.MarkerSetDescription->nMarkers;
#endif
			for (int markerIndex = 0; markerIndex < dataDescription.Data.MarkerSetDescription->nMarkers; ++markerIndex)
			{
				printf("Marker[%d] : %s\n", markerIndex, dataDescription.Data.MarkerSetDescription->szMarkerNames[markerIndex]);
			}
			break;
		case Descriptor_RigidBody:
			printf("RigidBody:%s\n", dataDescription.Data.RigidBodyDescription->szName);
			g_RigidBodyIdMap.emplace(dataDescription.Data.RigidBodyDescription->ID, dataDescription.Data.RigidBodyDescription->szName);
#ifdef LOST_FRAME_TEST
			++s_Record.m_RigNums;
#endif
			break;
		case Descriptor_Skeleton:
			printf("Skeleton name:%s, id:%d, rigids:%d\n", dataDescription.Data.SkeletonDescription->szName, 
				dataDescription.Data.SkeletonDescription->skeletonID, dataDescription.Data.SkeletonDescription->nRigidBodies);
			{
				IdNameMap rm;
				for (int boneIndex = 0; boneIndex < dataDescription.Data.SkeletonDescription->nRigidBodies; ++boneIndex)
				{
					auto boneDescription = dataDescription.Data.SkeletonDescription->RigidBodies[boneIndex];
					rm.emplace(boneDescription.ID, boneDescription.szName);
					printf("[%d] %s %d %f %f %f %f %f %f %f\n", boneDescription.ID, boneDescription.szName, boneDescription.parentID, 
						boneDescription.offsetx, boneDescription.offsety, boneDescription.offsetz, 
						boneDescription.qx, boneDescription.qy, boneDescription.qz, boneDescription.qw);
				}

				g_SkeletonRigidsIdMp.emplace(dataDescription.Data.SkeletonDescription->skeletonID, rm);
				g_SkeletonIdMap.emplace(dataDescription.Data.SkeletonDescription->skeletonID, dataDescription.Data.SkeletonDescription->szName);

			}
			break;
		case Descriptor_ForcePlate:
			for (int channelIdx = 0; channelIdx < dataDescription.Data.ForcePlateDescription->nChannels; ++channelIdx)
			{
				printf("Channel:%d %s\n", channelIdx,
					dataDescription.Data.ForcePlateDescription->szChannelNames[channelIdx]);
			}
		default:
			break;
		}
	}
}

void getTposeDescriptions()
{
	char name[100] = "HYK";
	sDataDescriptions* pData = nullptr;

	theClient->GetTposeDataDescriptions(name, &pData);
	//theClient->GetDataDescriptions(&pData);
	if (pData)
	{
		printPacketDescriptions(pData);
		theClient->FreeDataDescriptions(pData);
		pData = nullptr;
	}

	return;
}

// ForcePlateHandler receives data from the server
#ifndef _LINUX
void __cdecl ForcePlateHandler(sForcePlates* pForcePlate, void* pUserData)
#else
void  __attribute__((__cdecl__)) ForcePlateHandler(sForcePlates* pForcePlate, void* pUserData)
#endif
{
	if (nullptr != pForcePlate)
	{
		for (int plateIdx = 0; plateIdx < pForcePlate->nForcePlates; ++plateIdx)
		{
			printf("Frame[%d]:ForcePlate[%d]\nFxyz:(%lf,%lf,%lf)\nxyz:(%lf,%lf,%lf)\nMFree:%lf\n", pForcePlate->iFrame,
				plateIdx, pForcePlate->ForcePlates[plateIdx].Fxyz[0],
				pForcePlate->ForcePlates[plateIdx].Fxyz[1],
				pForcePlate->ForcePlates[plateIdx].Fxyz[2],
				pForcePlate->ForcePlates[plateIdx].xyz[0],
				pForcePlate->ForcePlates[plateIdx].xyz[1],
				pForcePlate->ForcePlates[plateIdx].xyz[2],
				pForcePlate->ForcePlates[plateIdx].Mfree);
		}
	}
}

void printData(sFrameOfMocapData* data, NokovSDKClient* pClient)
{
	int i = 0;

	// FrameOfMocapData params
	bool bIsRecording = data->params & 0x01;
	bool bTrackedModelsChanged = data->params & 0x02;
	if (bIsRecording)
		printf("RECORDING\n");
	if (bTrackedModelsChanged)
		printf("Models Changed.\n");

	// timecode - for systems with an eSync and SMPTE timecode generator - decode to values
	int hour, minute, second, frame, subframe;

	bool bValid = pClient->DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe);

	// decode to friendly string
	char szTimecode[128] = "";
	pClient->TimecodeStringify(data->Timecode, data->TimecodeSubframe, szTimecode, 128);
	printf("Timecode : %s\n", szTimecode);//print timecode stardard
	printf("FrameNO:%d\tTimeStamp:%lld\n", data->iFrame, data->iTimeStamp);

	printf("MarkerSet [Count=%d]\n", data->nMarkerSets);	//Number of Markerset

	MarkerVelocityTrackerArray.resize(data->nMarkerSets);
	MarkerAccelerationTrackerArray.resize(data->nMarkerSets);

	for (i = 0; i < data->nMarkerSets; i++)
	{
		sMarkerSetData* markerset = &data->MocapData[i];
		MarkerVelocityTrackerArray[i].resize(markerset->nMarkers);
		MarkerAccelerationTrackerArray[i].resize(markerset->nMarkers);

		printf("Markerset%d: %s [nMarkers Count=%d]\n", i + 1, markerset->szName, markerset->nMarkers);  //Output the id and name of the i-th Markerset and the total number of named markers in the Markerset
		printf("{\n");
		for (int i_Marker = 0; i_Marker < markerset->nMarkers; i_Marker++)	//Output the id and information (x, y, z) of the Marker point contained in the i-th Markerset
		{
			printf("\tMarker%d: %3.2f,%3.2f,%3.2f\n",
				i_Marker,
				markerset->Markers[i_Marker][0],
				markerset->Markers[i_Marker][1],
				markerset->Markers[i_Marker][2]);

			// calculate the velocity and acceleration
			MarkerVelocityTrackerArray[i][i_Marker].Cache(markerset->Markers[i_Marker][0]
				, markerset->Markers[i_Marker][1], markerset->Markers[i_Marker][2]);
			MarkerAccelerationTrackerArray[i][i_Marker].Cache(markerset->Markers[i_Marker][0]
				, markerset->Markers[i_Marker][1], markerset->Markers[i_Marker][2]);

			//Caution: Actually, you cat get velocity of frame 2 after you get frame 3's position, But it just has a little difference
			static CalculateVelocity method(60, 3); //FPS:60 FrameFactor:3
			static CalculateAcceleration method2(60, 3); //FPS:60 FrameFactor:5

			Vel MarkerVelocity;
			MarkerVelocityTrackerArray[i][i_Marker].tryToCalculate(MarkerVelocity, method);

			Accel MarkerAccleration;
			MarkerAccelerationTrackerArray[i][i_Marker].tryToCalculate(MarkerAccleration, method2);

			std::cout << "\t\t" << MarkerVelocity << "\t\t" << MarkerAccleration;
		}
		printf("}\n");//The data output of the i-th Markerset is completed
	}

	BoneVelocityTrackerArray.resize(data->nRigidBodies);
	BoneAccelerationTrackerArray.resize(data->nRigidBodies);

	//Print RigidBodies
	printf("Markerset.RigidBodies [Count=%d]\n", data->nRigidBodies);//Number of Markerset.Skeleton(skeleton)
	for (i = 0; i < data->nRigidBodies; i++)
	{
		printf("{\n");
		printf("\tname\tid\t\tx\ty\tz\tqx\tqy\tqz\tqw\n");
		printf("\t%s\t%d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
			g_RigidBodyIdMap[data->RigidBodies[i].ID].c_str(),
			data->RigidBodies[i].ID,
			data->RigidBodies[i].x,
			data->RigidBodies[i].y,
			data->RigidBodies[i].z,
			data->RigidBodies[i].qx,
			data->RigidBodies[i].qy,
			data->RigidBodies[i].qz,
			data->RigidBodies[i].qw);

		// calculate the velocity and acceleration
		BoneVelocityTrackerArray[i].Cache(data->RigidBodies[i].x
			, data->RigidBodies[i].y, data->RigidBodies[i].z);
		BoneAccelerationTrackerArray[i].Cache(data->RigidBodies[i].x
			, data->RigidBodies[i].y, data->RigidBodies[i].z);

		//Caution: Actually, you cat get velocity of frame 2 after you get frame 3's position, But it just has a little difference
		static CalculateVelocity method1(60, 3); //FPS:60 FrameFactor:3
		static CalculateAcceleration method2(60, 3); //FPS:60 FrameFactor:3

		Vel boneVelocity;
		BoneVelocityTrackerArray[i].tryToCalculate(boneVelocity, method1);

		Accel boneAccleration;
		BoneAccelerationTrackerArray[i].tryToCalculate(boneAccleration, method2);

		std::cout << "\t\t" << boneVelocity << "\t\t" << boneAccleration;

		printf("\tRigidBody markers [Count=%d]\n", data->RigidBodies[i].nMarkers);
		for (int iMarker = 0; iMarker < data->RigidBodies[i].nMarkers; iMarker++)//Output the id and information (x, y, z) of the marker associated with the j-th RigidBody (RigidBody)
		{
			printf("\t\t");
			if (data->RigidBodies[i].MarkerIDs)
				printf("Marker%d: ", data->RigidBodies[i].MarkerIDs[iMarker]);
			if (data->RigidBodies[i].Markers)
				printf("%3.2f,%3.2f,%3.2f\n",
					data->RigidBodies[i].Markers[iMarker][0],
					data->RigidBodies[i].Markers[iMarker][1],
					data->RigidBodies[i].Markers[iMarker][2]);
		}

		printf("}\n");//The data output of the j-th RigidBody (RigidBody) is completed
	}

	//Print Skeletons
	printf("Markerset.Skeletons [Count=%d]\n", data->nSkeletons);//Number of Markerset.Skeleton(skeleton)
	for (i = 0; i < data->nSkeletons; i++)
	{
		printf("Markerset%d [Name=%s].Skeleton [nRigidBodies Count=%d]\n", data->Skeletons[i].skeletonID, g_SkeletonIdMap[data->Skeletons[i].skeletonID].c_str(), data->Skeletons[i].nRigidBodies);//Skeleton (Skeleton) of the i-th Markerset of the motion capture data, including the number of RigidBody (RigidBody)
		printf("{\n");
		for (int j = 0; j < data->Skeletons[i].nRigidBodies; j++)//Output id and information(x, y, z, qx, qy, qz, qw) of the j-th RigidBody (RigidBody) 
		{
			printf("\tname\tid\t\tx\ty\tz\tqx\tqy\tqz\tqw\n");
			printf("\t%s\t%d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
				g_SkeletonRigidsIdMp[data->Skeletons[i].skeletonID][data->Skeletons[i].RigidBodyData[j].ID].c_str(),
				data->Skeletons[i].RigidBodyData[j].ID,
				data->Skeletons[i].RigidBodyData[j].x,
				data->Skeletons[i].RigidBodyData[j].y,
				data->Skeletons[i].RigidBodyData[j].z,
				data->Skeletons[i].RigidBodyData[j].qx,
				data->Skeletons[i].RigidBodyData[j].qy,
				data->Skeletons[i].RigidBodyData[j].qz,
				data->Skeletons[i].RigidBodyData[j].qw);

			printf("\tRigidBody markers [Count=%d]\n", data->Skeletons[i].RigidBodyData[j].nMarkers);
			for (int iMarker = 0; iMarker < data->Skeletons[i].RigidBodyData[j].nMarkers; iMarker++)//Output the id and information (x, y, z) of the marker associated with the j-th RigidBody (RigidBody)
			{
				printf("\t\t");
				if (data->Skeletons[i].RigidBodyData[j].MarkerIDs)
					printf("Marker%d: ", data->Skeletons[i].RigidBodyData[j].MarkerIDs[iMarker]);
				if (data->Skeletons[i].RigidBodyData[j].Markers)
					printf("%3.2f,%3.2f,%3.2f\n",
						data->Skeletons[i].RigidBodyData[j].Markers[iMarker][0],
						data->Skeletons[i].RigidBodyData[j].Markers[iMarker][1],
						data->Skeletons[i].RigidBodyData[j].Markers[iMarker][2]);
			}
		}
		printf("}\n");//The data output of the j-th RigidBody (RigidBody) is completed
	}
	printf("Other Markers [Count=%d]\n", data->nOtherMarkers);//Output the total number of unnamed markers contained
	printf("{\n");
	for (i = 0; i < data->nOtherMarkers; i++)//Output the id and information (x, y, z) of the unnamed marker included
	{
		printf("\tOther Marker%d: %3.2f,%3.2f,%3.2f\n",
			i,
			data->OtherMarkers[i][0],
			data->OtherMarkers[i][1],
			data->OtherMarkers[i][2]);
	}
	printf("}\n\n");//The data output of unnamed marker is completed, and the data output of one frame is completed
	printf("Analog [Count=%d]\n", data->nAnalogdatas);//Output the total number of analog data contained
	printf("{\n");
	for (int i = 0; i < data->nAnalogdatas; ++i)
	{
		printf("\tAnalogData %d: %3.2f\n", i, data->Analogdata[i]);
	}
	printf("}\n\n");//The data output of analog data is completed, and the data output of one frame is completed
}

// DataHandler receives data from the server
#ifndef _LINUX
void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData)
#else
void  __attribute__((__cdecl__)) DataHandler(sFrameOfMocapData* data, void* pUserData)
#endif
{

	NokovSDKClient* pClient = (NokovSDKClient*) pUserData;

	printData(data, pClient);
	return;
}


// MessageHandler receives NokovSDK error/debug messages
#ifndef _LINUX
void __cdecl MessageHandler(int msgType, char* msg)
#else
void  __attribute__((__cdecl__)) MessageHandler(int msgType, char* msg)
#endif
{
	printf("\n%s\n", msg);
}

// NotifyHandler receives NokovSDK notify
#ifndef _LINUX
void __cdecl NotifyHandler(sNotifyMsg* data, void* pUserData)
#else
void  __attribute__((__cdecl__)) NotifyHandler(sNotifyMsg* data, void* pUserData)
#endif
{
	printf("Notify Type: %d, Value: %d, timestamp:%llu, msg: %s, param1:%d, param2:%d, param3:%d, param4:%d\n",
		data->nType, data->nValue, data->nTimeStamp, data->sMsg, data->nParam1, data->nParam2, data->nParam3, data->nParam4);
	
	getTposeDescriptions();
}

