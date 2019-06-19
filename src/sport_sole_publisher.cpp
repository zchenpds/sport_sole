// WITH RESET

#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <mutex>
#include <algorithm>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <sys/time.h>
#include <time.h>  
#include <fcntl.h>
#include <unistd.h>

#include <math.h>
#include <string.h>
#include <inttypes.h>

#include <ros/ros.h>
#include "sport_sole/SportSole.h"

#define US_CYCLES 1000 //[us]

#define BUFFER 4096
#define PACKET_LENGTH_WIFI  51//47//45
#define PACKET_LENGTH_LOG   106//95//91//88//87//86//84
#define PACKET_LENGTH_PD    24//22
#define PACKET_LENGTH_LED    8
#define PACKET_LENGTH_TIME   16//8
#define PACKET_LENGTH_SYNC  7
#define N_PACKET_LED 20
#define RADTODEG 57.295780
#define FACTOR_SCALE_ANGLE 5000.0f
#define FACTOR_SCALE_ACCELERATION 1000.0f
#define FACTOR_SCALE_ACC_ADXL 128.0f
#define MAX_UINT16_FLOAT 65536.0f
#define MAX_UINT16 65536
#define L1_NORM_ACC_SCALE 4.0f

#define N_STR 512
#define N_MAX_MODES 50
#define N_PRESS_SENS 4

#define RESET_PORT 3461
//#define SYNC_PORT 3462
#define PRESSUREVAL_PORT 3463
#define PACKET_LENGTH_RESET_LED 6
#define PACKET_LENGTH_PRESSUREVAL 8
#define PACKET_LENGTH_MATLAB 7


#define TIME_TO_WAIT_US 11000000 // wait 11s before starting the metronome
using namespace std;

mutex dataMutex;

struct structDataPacketPureData 
{
	uint32_t timestamp;
		
	float yaw1,pitch1,roll1;
	float ax1,ay1,az1;
	
	

	uint16_t p1,p2,p3,p4,p5,p6,p7,p8;
	uint64_t Odroid_Timestamp;
	uint8_t Odroid_Trigger;
    
	uint32_t timestamp2;
	// why no sonar here?
};

struct structDataPacketPureDataRAW
{
	uint32_t timestamp;
		
	int16_t yaw1,pitch1,roll1;
	int16_t ax1,ay1,az1;
	

	uint16_t p1,p2,p3,p4,p5,p6,p7,p8;
	uint64_t Odroid_Timestamp;
	uint8_t Odroid_Trigger;

    uint32_t timestamp2;
};

struct structDataPacket 
{
	uint32_t timestamp;
		
	float qw1,qx1,qy1,qz1;
	float yaw1,pitch1,roll1;
	float ax1,ay1,az1;
	
	

	uint16_t p1,p2,p3,p4,p5,p6,p7,p8;
	uint64_t Odroid_Timestamp;
	uint8_t Odroid_Trigger;
	
	uint32_t timestamp2;

};

struct structSyncPacket 
{
	uint8_t Ext_Trigger;
};


struct structSWstat 
{
	unsigned int packetLedSent;
	unsigned int packetGuiSent;
	unsigned int packetPdSent;
	unsigned int packetErrorPdShoeL;
	unsigned int packetErrorPdShoeR;
	unsigned int packetErrorSync;
	unsigned int packetBroadSent;
	unsigned int packetReceivedPdShoeL;
	unsigned int packetReceivedPdShoeR;
	unsigned int packetReceivedSync;
	
};

inline void reconstructStruct(structDataPacketPureDataRAW dataPacketRAW, structDataPacketPureData &dataPacket)
{
	dataPacket.timestamp=dataPacketRAW.timestamp;
 
	dataPacket.yaw1=((float)dataPacketRAW.yaw1)/FACTOR_SCALE_ANGLE;
	dataPacket.pitch1=((float)dataPacketRAW.pitch1)/FACTOR_SCALE_ANGLE;
	dataPacket.roll1=((float)dataPacketRAW.roll1)/FACTOR_SCALE_ANGLE;

	dataPacket.ax1=((float)dataPacketRAW.ax1)/FACTOR_SCALE_ACCELERATION;
	dataPacket.ay1=((float)dataPacketRAW.ay1)/FACTOR_SCALE_ACCELERATION;
	dataPacket.az1=((float)dataPacketRAW.az1)/FACTOR_SCALE_ACCELERATION;

	dataPacket.p1=dataPacketRAW.p1;
	dataPacket.p2=dataPacketRAW.p2;
	dataPacket.p3=dataPacketRAW.p3;
	dataPacket.p4=dataPacketRAW.p4;
	dataPacket.p5=dataPacketRAW.p5;
	dataPacket.p6=dataPacketRAW.p6;
	dataPacket.p7=dataPacketRAW.p7;
	dataPacket.p8=dataPacketRAW.p8;
	
	dataPacket.Odroid_Timestamp=dataPacketRAW.Odroid_Timestamp;
	dataPacket.Odroid_Trigger=dataPacketRAW.Odroid_Trigger;
	
	dataPacket.timestamp2=dataPacketRAW.timestamp2;
	
}

inline void reconstructStructSyncPacket(uint8_t* recvbuffer,structSyncPacket  &dataPacket)
{
	uint8_t *pointer;
	
	//recvbuffer[0];
	//recvbuffer[1];
	//recvbuffer[2];
	
	
	pointer=(uint8_t*)&dataPacket.Ext_Trigger;
	pointer[0]=recvbuffer[3];

	
	//recvbuffer[4];
	//recvbuffer[5];
	//recvbuffer[6];

}


inline void reconstructStructPureDataRAW(uint8_t* recvbuffer,structDataPacketPureDataRAW &dataPacket)
{
	uint8_t *pointer;

	//recvbuffer[0];
	//recvbuffer[1];
	//recvbuffer[2];

	pointer=(uint8_t*)&dataPacket.timestamp;
	pointer[3]=recvbuffer[3];
	pointer[2]=recvbuffer[4];
	pointer[1]=recvbuffer[5];
	pointer[0]=recvbuffer[6];

	pointer=(uint8_t*)&dataPacket.yaw1;
	pointer[1]=recvbuffer[7];
	pointer[0]=recvbuffer[8];

	pointer=(uint8_t*)&dataPacket.pitch1;
	pointer[1]=recvbuffer[9];
	pointer[0]=recvbuffer[10];

	pointer=(uint8_t*)&dataPacket.roll1;
	pointer[1]=recvbuffer[11];
	pointer[0]=recvbuffer[12];

	pointer=(uint8_t*)&dataPacket.ax1;
	pointer[1]=recvbuffer[13];
	pointer[0]=recvbuffer[14];

	pointer=(uint8_t*)&dataPacket.ay1;
	pointer[1]=recvbuffer[15];
	pointer[0]=recvbuffer[16];

	pointer=(uint8_t*)&dataPacket.az1;
	pointer[1]=recvbuffer[17];
	pointer[0]=recvbuffer[18];

	/////////////////////////////////////////////


	/////////////////////////////////////////////

	pointer=(uint8_t*)&dataPacket.p1;
	pointer[1]=recvbuffer[19];
	pointer[0]=recvbuffer[20];

	pointer=(uint8_t*)&dataPacket.p2;
	pointer[1]=recvbuffer[21];
	pointer[0]=recvbuffer[22];

	pointer=(uint8_t*)&dataPacket.p3;
	pointer[1]=recvbuffer[23];
	pointer[0]=recvbuffer[24];

	pointer=(uint8_t*)&dataPacket.p4;
	pointer[1]=recvbuffer[25];
	pointer[0]=recvbuffer[26];

	pointer=(uint8_t*)&dataPacket.p5;
	pointer[1]=recvbuffer[27];
	pointer[0]=recvbuffer[28];

	pointer=(uint8_t*)&dataPacket.p6;
	pointer[1]=recvbuffer[29];
	pointer[0]=recvbuffer[30];

	pointer=(uint8_t*)&dataPacket.p7;
	pointer[1]=recvbuffer[31];
	pointer[0]=recvbuffer[32];

	pointer=(uint8_t*)&dataPacket.p8;
	pointer[1]=recvbuffer[33];
	pointer[0]=recvbuffer[34];
	/////////////////////////////////////////////

    //Timestamp_Odroid
	pointer=(uint8_t*)&dataPacket.Odroid_Timestamp;
	pointer[7]=recvbuffer[35];
	pointer[6]=recvbuffer[36];
	pointer[5]=recvbuffer[37];
	pointer[4]=recvbuffer[38];
	pointer[3]=recvbuffer[49];
	pointer[2]=recvbuffer[40];
	pointer[1]=recvbuffer[41];
	pointer[0]=recvbuffer[42];

	// trigger
	pointer=(uint8_t*)&dataPacket.Odroid_Trigger;
	pointer[0]=recvbuffer[43];
 
    // timestamp2 
	pointer=(uint8_t*)&dataPacket.timestamp2;
	pointer[3]=recvbuffer[44];
	pointer[2]=recvbuffer[45];
	pointer[1]=recvbuffer[46];
	pointer[0]=recvbuffer[47];	
    // //currenttime
//     pointer=(uint8_t*)&dataPacket.currenttime;
//     pointer[7]=recvbuffer[48];
// 	pointer[6]=recvbuffer[49];
// 	pointer[5]=recvbuffer[50];
// 	pointer[4]=recvbuffer[51];
// 	pointer[3]=recvbuffer[52];
// 	pointer[2]=recvbuffer[53];
// 	pointer[1]=recvbuffer[54];
// 	pointer[0]=recvbuffer[55];


	//recvbuffer[44];
	//recvbuffer[45];
	//recvbuffer[46];

}

inline uint16_t convert2uint16(float fValue,float scale)
{
	uint32_t iValue=(uint32_t)(MAX_UINT16_FLOAT*(fValue)/scale);;
	
	if (iValue>MAX_UINT16)
	{
		iValue=MAX_UINT16;
	}
	
	return (uint16_t) iValue;	
}

struct structPDShoe
{
	string ipAddress;
	string name;
	int port;
	uint8_t lastPacket[PACKET_LENGTH_WIFI];
	unsigned int packetError;
	unsigned int packetReceived;
	uint8_t id;
};

struct structSync
{
	string ipAddress;
	string name;
	int port;
	uint8_t lastPacket[PACKET_LENGTH_SYNC];
	unsigned int packetError;
	unsigned int packetReceived;
	uint8_t id;
};

struct structSensSettings{
	uint16_t maxUpToNow = 0;
	uint16_t maxInUse = 1023;
	uint16_t minUpToNow = 1023;
	uint16_t minInUse = 0;
	uint16_t perMilleThreshold = 400;	
};

struct structPressSettings
{
	structSensSettings front;
	structSensSettings heel;
};

inline bool checkPacket(uint8_t *buffer,int ret)
{
	return (buffer[0]==0x07 && buffer[1]==0x08 && buffer[2]==0x09 && buffer[48]==0xA && buffer[49]==0xB && buffer[50]==0xC && ret==PACKET_LENGTH_WIFI);
};

inline bool checkResetPacket(uint8_t *buffer,int ret)
{
	return (buffer[0]==0x01 && buffer[1]==0x02 && buffer[2]==0x03 && buffer[3]==0x4 && buffer[4]==0x5 && buffer[5]==0x6 && ret==PACKET_LENGTH_RESET_LED);
};

inline uint8_t checkMatlabIncomingPacket(uint8_t *buffer,int ret)
{
	uint8_t integrityCheck =(uint8_t)(buffer[0]==0x01 && buffer[1]==0x02 && buffer[2]==0x03 && buffer[4]==0x4 && buffer[5]==0x5 && buffer[6]==0x6 && ret==PACKET_LENGTH_MATLAB);
	return (integrityCheck * buffer[3]);
};

inline uint16_t checkPressureIncomingPacket(uint8_t *buffer,int ret)
{
	uint16_t val=65535;
	bool integrityCheck =(uint8_t)(buffer[0]==0x01 && buffer[1]==0x02 && buffer[2]==0x03 && buffer[5]==0x4 && buffer[6]==0x5 && buffer[7]==0x6 && ret==PACKET_LENGTH_PRESSUREVAL);
	//printf("\nPack: [%d %d %d] [%d %d] [%d %d %d]\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7]);
	//printf("Integrity Check: %d\n",integrityCheck);
	if (integrityCheck) {
		// little-endian 
		val = buffer[3] + (buffer[4] << 8);
		// big-endian
		//val = (buffer[3] << 8) + buffer[4];
	}
	return (val);
};


inline bool checkSyncPacket(uint8_t *buffer,int ret)
{
	return (buffer[0]==0x01 && buffer[1]==0x02 && buffer[2]==0x03 && buffer[4]==0x4 && buffer[5]==0x5 && buffer[6]==0x6 && ret==PACKET_LENGTH_SYNC);
};


// void threadCheckReset(bool* resetLED,bool* resetPressure,bool* setPressure)
// {
// 	dataMutex.lock();
//
// 	printf("Hello from ResetThread!\n");
//     printf("NO GRAVITY COMPENSATION!\n");
//
// 	unsigned int nReset=0;
// 	int sockfdServer;
// 	socklen_t len;
// 	int ret;
// 	uint8_t recvBuffer[BUFFER];
// 	uint8_t res;
//
// 	struct sockaddr_in addrServer;
// 	struct sockaddr_in addrClient;
//
// 	sockfdServer=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
// 	//printf("sockfdServer=%d\n",sockfdServer);
//
// 	bzero(&addrServer,sizeof(addrServer));
// 	addrServer.sin_family = AF_INET;
// 	addrServer.sin_addr.s_addr=htonl(INADDR_ANY);//IP_ADDRESS(127, 0, 0, 1);
// 	addrServer.sin_port=htons(RESET_PORT);
//
// 	bind(sockfdServer,(struct sockaddr *)&addrServer,sizeof(addrServer));
//
// 	len = (socklen_t)sizeof(addrClient);
//
// 	dataMutex.unlock();
// 	usleep(250);
//
// 	// No blocking mode
// 	//int opts=fcntl(sockfdServer, F_GETFL, 0);
// 	//fcntl(sockfdServer, F_SETFL, opts | O_NONBLOCK);
//
// 	while(1)
// 	{
// 		ret=recvfrom(sockfdServer,recvBuffer,PACKET_LENGTH_WIFI,0,(struct sockaddr *)&addrClient,&len);
//
// 		if (ret>1)
// 		{
// 			res = checkMatlabIncomingPacket(recvBuffer,ret);
// 			//if(checkResetPacket(recvBuffer,ret))
// 			if(res == 1)
// 			{
// 				nReset++;
// 				printf("\n\n\nReset [%d]!\n\n\n",nReset);
//
// 				dataMutex.lock();
//
// 			    *resetLED=true;
//
// 				dataMutex.unlock();
// 			}
// 			else if(res == 2)
// 			{
// 				printf("\n\n\nReset Pressure!\n\n\n");
// 				dataMutex.lock();
//
// 				*resetPressure=true;
//
// 				dataMutex.unlock();
// 			}
// 			else if(res == 3)
// 			{
// 				printf("\n\n\nSet Pressure Coefficients!\n");
// 				dataMutex.lock();
//
// 				*setPressure=true;
//
// 				dataMutex.unlock();
// 			}
// 		}
// 	}
//
// }

// void threadCheckPressureReadVal(uint16_t* pressureVal)
// {
// 	dataMutex.lock();
//
// 	printf("Hello from PressureValThread!\n");
//
// 	int sockfdServer;
// 	socklen_t len;
// 	int ret;
// 	uint8_t recvBuffer[BUFFER];
// 	uint16_t res;
//
// 	struct sockaddr_in addrServer;
// 	struct sockaddr_in addrClient;
//
// 	sockfdServer=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
//
// 	bzero(&addrServer,sizeof(addrServer));
// 	addrServer.sin_family = AF_INET;
// 	addrServer.sin_addr.s_addr=htonl(INADDR_ANY);//IP_ADDRESS(127, 0, 0, 1);
// 	addrServer.sin_port=htons(PRESSUREVAL_PORT);
//
// 	bind(sockfdServer,(struct sockaddr *)&addrServer,sizeof(addrServer));
//
// 	len = (socklen_t)sizeof(addrClient);
//
// 	dataMutex.unlock();
// 	usleep(250);
//
// 	// No blocking mode
// 	//int opts=fcntl(sockfdServer, F_GETFL, 0);
// 	//fcntl(sockfdServer, F_SETFL, opts | O_NONBLOCK);
//
// 	while(1)
// 	{
// 		ret=recvfrom(sockfdServer,recvBuffer,PACKET_LENGTH_PRESSUREVAL,0,(struct sockaddr *)&addrClient,&len);
//
// 		if (ret>1)
// 		{
// 			res = checkPressureIncomingPacket(recvBuffer,ret);
// 			if (res < 65535){
// 				//printf("\n\n\nPressure Read= %d\n\n\n",res);
// 				dataMutex.lock();
// 			    *pressureVal=res;
// 				dataMutex.unlock();
// 			}
// 			else{
// 				printf("\n\n\nWrong Pressure Packet [%d,%d].\n\n\n",ret,res);
// 			}
// 		}
// 	}
//
// }


void threadSYNCreceive(structSync* SyncBoard)
{
	dataMutex.lock();
	
	printf("Hello from Thread! [%s ip: %s - port: %d]\n",SyncBoard->name.c_str(),SyncBoard->ipAddress.c_str(),SyncBoard->port);

	//unsigned int nReset=0;
	uint8_t id;
	int sockfdServer;
	socklen_t len;
	int ret;
	uint8_t recvBuffer[BUFFER];
	
	struct sockaddr_in addrServer;
	struct sockaddr_in addrClient;
	
	sockfdServer=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
	//printf("sockfdServer=%d\n",sockfdServer);
	
	bzero(&addrServer,sizeof(addrServer));
	addrServer.sin_family = AF_INET;
	addrServer.sin_addr.s_addr=htonl(INADDR_ANY);//IP_ADDRESS(127, 0, 0, 1);
	addrServer.sin_port=htons(SyncBoard->port);
	
	bind(sockfdServer,(struct sockaddr *)&addrServer,sizeof(addrServer));
	
	len = (socklen_t)sizeof(addrClient);
	id=SyncBoard->id;
	
	dataMutex.unlock();
	usleep(250);
	
	// No blocking mode
	//int opts=fcntl(sockfdServer, F_GETFL, 0);
	//fcntl(sockfdServer, F_SETFL, opts | O_NONBLOCK); 
		
	while(1)
	{		
		ret=recvfrom(sockfdServer,recvBuffer,PACKET_LENGTH_SYNC,0,(struct sockaddr *)&addrClient,&len);
		
		if (ret>1)
		{			
			dataMutex.lock();
			
			SyncBoard->packetReceived++;
			if (checkSyncPacket(recvBuffer,ret))
			{
				memcpy(SyncBoard->lastPacket,recvBuffer,PACKET_LENGTH_SYNC);
			}
			else
			{
				SyncBoard->packetError++;	
				//printf("Ret=%d\n", ret);
			}
			
			dataMutex.unlock();
			
		}	
	}

}


void threadUDPreceive(structPDShoe* PDShoe)
{
	dataMutex.lock();
	
	printf("Hello from Thread! [%s ip: %s - port: %d]\n",PDShoe->name.c_str(),PDShoe->ipAddress.c_str(),PDShoe->port);
	
	uint8_t id;
	unsigned long int cycles=0;
	int sockfdServer;
	socklen_t len;
	int ret;
	uint8_t recvBuffer[BUFFER];
	
	struct sockaddr_in addrServer;
	struct sockaddr_in addrClient;
	
	sockfdServer=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
	//printf("sockfdServer=%d\n",sockfdServer);
	
	bzero(&addrServer,sizeof(addrServer));
	addrServer.sin_family = AF_INET;
	addrServer.sin_addr.s_addr=htonl(INADDR_ANY);//IP_ADDRESS(127, 0, 0, 1);
	addrServer.sin_port=htons(PDShoe->port);
	
	bind(sockfdServer,(struct sockaddr *)&addrServer,sizeof(addrServer));
	
	len = (socklen_t)sizeof(addrClient);
	
	id=PDShoe->id;
	dataMutex.unlock();
	usleep(250);
	
	// No blocking mode
	//int opts=fcntl(sockfdServer, F_GETFL, 0);
	//fcntl(sockfdServer, F_SETFL, opts | O_NONBLOCK); 
		
	while(1)
	{		
		ret=recvfrom(sockfdServer,recvBuffer,PACKET_LENGTH_WIFI,0,(struct sockaddr *)&addrClient,&len);
		
		if (ret>1)
		{
			//printf("Data!\n");
			
			dataMutex.lock();
			
			PDShoe->packetReceived++;
			if (checkPacket(recvBuffer,ret))
			{
				memcpy(PDShoe->lastPacket,recvBuffer,PACKET_LENGTH_WIFI);
			}
			else
			{
				PDShoe->packetError++;	
				//printf("Ret=%d\n", ret);
			}
			
			dataMutex.unlock();
			
		}
		cycles++;
	}
}

// void createPureDataShortPacket(uint8_t* buffer_out,uint16_t norm_a1_foot,uint16_t norm_a2_foot,uint16_t norm_a1_heel,uint16_t norm_a2_heel,uint16_t heelL,uint16_t frontL,uint16_t heelR,uint16_t frontR,uint8_t metrobool,uint8_t currMode)
// {
//
// 	uint8_t* pointer;
//
// 	buffer_out[0]=0x01;
// 	buffer_out[1]=0x02;
// 	buffer_out[2]=0x03;
//
// 	//LH
// 	pointer=(uint8_t*)&heelL;
// 	buffer_out[3]=pointer[1];
// 	buffer_out[4]=pointer[0];
//
// 	//LMB
// 	pointer=(uint8_t*)&frontL;
// 	buffer_out[5]=pointer[1];
// 	buffer_out[6]=pointer[0];
//
// 	//L1N foot (L)
// 	pointer=(uint8_t*)&norm_a1_foot;
// 	buffer_out[7]=pointer[1];
// 	buffer_out[8]=pointer[0];
//
// 	//L1N heel (L)
// 	pointer=(uint8_t*)&norm_a1_heel;
// 	buffer_out[9]=pointer[1];
// 	buffer_out[10]=pointer[0];
//
// 	//RH
// 	pointer=(uint8_t*)&heelR;
// 	buffer_out[11]=pointer[1];
// 	buffer_out[12]=pointer[0];
//
// 	//RMB
// 	pointer=(uint8_t*)&frontR;
// 	buffer_out[13]=pointer[1];
// 	buffer_out[14]=pointer[0];
//
// 	//L1N foot (R)
// 	pointer=(uint8_t*)&norm_a2_foot;
// 	buffer_out[15]=pointer[1];
// 	buffer_out[16]=pointer[0];
//
// 	//L1N heel (R)
// 	pointer=(uint8_t*)&norm_a2_heel;
// 	buffer_out[17]=pointer[1];
// 	buffer_out[18]=pointer[0];
//
//     //metronome
//     pointer=(uint8_t*)&metrobool;
//     buffer_out[19]=pointer[0];
//
//     //metronome2
//     pointer=(uint8_t*)&currMode;
//     buffer_out[20]=pointer[0];
//
// 	buffer_out[21]=0x4;
// 	buffer_out[22]=0x5;
// 	buffer_out[23]=0x6;
//
// }

void createLogPacket(uint8_t* buffer_out,uint8_t* buffer_01,uint8_t* buffer_02,uint8_t Odroid_Trigger,uint64_t currenttime,uint8_t Ext_Trigger)
{
	uint8_t *pointer;
	//See PureData Packet {XLS File}
	buffer_out[0]=0x01;
	buffer_out[1]=0x02;
	buffer_out[2]=0x03;

	for(int i=0;i<45;i++)
	{
		buffer_out[i+3]=buffer_01[i+3];
		buffer_out[i+48]=buffer_02[i+3];
	}


	//buffer_out[85]=Odroid_Trigger;


	//current time
	pointer=(uint8_t*)&currenttime;
	buffer_out[93]=pointer[7];
	buffer_out[94]=pointer[6];
	buffer_out[95]=pointer[5];
	buffer_out[96]=pointer[4];
	buffer_out[97]=pointer[3];
	buffer_out[98]=pointer[2];
	buffer_out[99]=pointer[1];
	buffer_out[100]=pointer[0];

    buffer_out[101]=Odroid_Trigger;
	
	buffer_out[102]=Ext_Trigger;
	// for(int i=0;i<4;i++)
	// {
	// 	buffer_out[i+44]=cycles;
	// 	buffer_out[i+97]=cycles;
	// }
	//
	// for(int i=0;i<8;i++)
	// {
	// 	buffer_out[i+48]=currenttime;
	// 	buffer_out[i+101]=currenttime;
	// }

	//     buffer_out[97]=metrobool;
	//     buffer_out[98]=currMode;
	//
	// buffer_out[99]=Sync;
	// buffer_out[100]=currState;

	// buffer_out[101]=0x4;
	// buffer_out[102]=0x5;
	// buffer_out[103]=0x6;
	buffer_out[103]=0x4;
	buffer_out[104]=0x5;
	buffer_out[105]=0x6;

	//buffer_out[47]=trigger;
}

// void createLedPacket(uint8_t* buffer_out,uint8_t trigger)
// {
// 	buffer_out[0]=0x01; buffer_out[1]=0x02; buffer_out[2]=0x03;
// 	buffer_out[3]=trigger;
// 	buffer_out[4]=0x00;
// 	buffer_out[5]=0x04; buffer_out[6]=0x05; buffer_out[7]=0x06;
// }

uint64_t getMicrosTimeStamp() 
{
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

void createTimePacket(uint8_t* buffer_out,uint64_t currenttime,uint8_t Odroid_Trigger)
{

	uint8_t *pointer;
	buffer_out[0]=0x01; buffer_out[1]=0x02; buffer_out[2]=0x03;
	
	buffer_out[3]=0x02; // 0 -> Reboot   1 -> Enable   2 -> Broadcast
	
	pointer=(uint8_t*)&currenttime;
	buffer_out[4]=pointer[7];
	buffer_out[5]=pointer[6];
	buffer_out[6]=pointer[5];
	buffer_out[7]=pointer[4];
	buffer_out[8]=pointer[3];
	buffer_out[9]=pointer[2];
	buffer_out[10]=pointer[1];
	buffer_out[11]=pointer[0];
	
	buffer_out[12]=Odroid_Trigger;

	buffer_out[13]=0x04; buffer_out[14]=0x05; buffer_out[15]=0x06;
}

// void calculatesLinearAcceleration(float &lax,float &lay,float &laz, float ax,float ay,float az, float yaw,float pitch,float roll,float alpha)
// {
// 	float g=1.0f;
//
// 	ax=-ax;
// 	ay=-ay;
// 	az=-az;
//
// 	//lax=cos(yaw)*(ay*cos(pitch)*sin(alpha)+cos(alpha)*(az*cos(pitch)+ay*cos(roll)*sin(pitch))+sin(pitch)*((-1)*az*cos(roll)*sin(alpha)+ax*sin(roll)))+(-1)*(ax*cos(roll)+((-1)*ay*cos(alpha)+az*sin(alpha))*sin(roll))*sin(yaw);
// 	//lay=cos(yaw)*((-1)*ay*cos(alpha)+az*sin(alpha))*sin(roll)+(az*cos(alpha)*cos(pitch)+ay*cos(pitch)*sin(alpha)+ax*sin(pitch)*sin(roll))*sin(yaw)+cos(roll)*(ax*cos(yaw)+(ay*cos(alpha)+(-1)*az*sin(alpha))*sin(pitch)*sin(yaw));
// 	//laz=(-1)*g+cos(pitch)*cos(roll)*(ay*cos(alpha)+(-1)*az*sin(alpha))+(-1)*(az*cos(alpha)+ay*sin(alpha))*sin(pitch)+ax*cos(pitch)*sin(roll);
//
//     lax=ax;
//     lay=ay;
//     laz=az;
//
//
// }

// void writeGPIO(uint8_t trigger, FILE * GPIOzero, FILE * GPIOone)
// {
// 		// if(trigger>0)
// //         {
// // 			fputs("1\n", GPIOone);
// // 			fflush(GPIOone);
// //         }
// //         else
// //         {
// // 			fputs("0\n", GPIOzero);
// // 			fflush(GPIOzero);
// //         }
// // 		//printf("LED: %d \n",trigger);
// }

// void updateHeelFront(uint16_t &heel, uint16_t &front, const uint16_t &p1, const uint16_t &p2, const uint16_t &p3, const uint16_t &p4, const uint16_t &p5, const uint16_t &p7, const uint16_t &p8)
// {
// 	/*
// 	4-ch configuration:
// 	name -> order -> Signal Name -> Actual placement
// 	p3 -> toe -> lball -> heel
// 	p2 -> mball -> mball -> mball
// 	p4 -> lball -> heel -> lball
// 	p1 -> heel -> toe -> toe
//
// 	heel = p3;
// 	front = max(max(p1,p2),p4);
//
// 	8-ch configuration:
// 	p1 -> Hallux
// 	p2 -> Toes
// 	p3 -> Met 1
// 	p4 -> Met 3
// 	p5 -> Met 5
// 	p6 -> Arch
// 	p7 -> Heel L
// 	p8 -> Heel R
// 	*/
// 	heel = max(p7,p8);
// 	front = max(max(max(p1,p2),max(p3,p4)),p5);
// }

// void updatePressSettings(structPressSettings &pressSettings,const uint16_t heel, const uint16_t front)
// {
//
// 	if(pressSettings.heel.maxUpToNow<heel){
// 		pressSettings.heel.maxUpToNow=heel;
// 	}
//
// 	if(pressSettings.heel.minUpToNow>heel){
// 		pressSettings.heel.minUpToNow=heel;
// 	}
//
// 	if(pressSettings.front.maxUpToNow<front){
// 		pressSettings.front.maxUpToNow=front;
// 	}
//
// 	if(pressSettings.front.minUpToNow>front){
// 		pressSettings.front.minUpToNow=front;
// 	}
//
//
// }

// bool onOffPressure(structSensSettings &sens, const uint16_t pSignal)
// {
// 	float nSignal=(float)(pSignal-sens.minInUse)/(float)(sens.maxInUse-sens.minInUse);
// 	float thr=(1/1000.0f)*(float)(sens.perMilleThreshold);
// 	return (nSignal > thr);
// }
	
// uint8_t OnOffToCode(const uint16_t &front,const uint16_t &heel)
// {
// 	uint8_t out;
//
// 	if(front)
// 	{
// 		if(heel)
// 		{
// 			out=1;
// 		}
// 		else
// 		{
// 			out=2;
// 		}
// 	}
// 	else
// 	{
// 		if(heel)
// 		{
// 			out=0;
// 		}
// 		else
// 		{
// 			out=3;
// 		}
// 	}
// 	return out;
// }
		
// void updateState(uint8_t &currState, uint16_t &heelR,uint16_t &frontR,uint16_t &heelL,uint16_t &frontL,bool &heelOnR,bool &frontOnR,bool &heelOnL,bool &frontOnL,structPressSettings &pressSettingsL,structPressSettings &pressSettingsR, const uint16_t &p1L, const uint16_t &p2L, const uint16_t &p3L, const uint16_t &p4L, const uint16_t &p5L, const uint16_t &p7L,const uint16_t &p8L, const uint16_t &p1R,const uint16_t &p2R,const uint16_t &p3R,const uint16_t &p4R,const uint16_t &p5R,const uint16_t &p7R,const uint16_t &p8R)
// {
// 	//[rh rf lh lf]
// 	//uint16_t heelL,frontL,heelR,frontR;
// 	//bool heelOnL,frontOnL,heelOnR,frontOnR;
//
// 	//get heel and front
// 	updateHeelFront(heelL,frontL,p1L,p2L,p3L,p4L,p5L,p7L,p8L);
// 	updateHeelFront(heelR,frontR,p1R,p2R,p3R,p4R,p5R,p7R,p8R);
//
// 	//update Settings
// 	updatePressSettings(pressSettingsL,heelL,frontL);
// 	updatePressSettings(pressSettingsR,heelR,frontR);
//
// 	//thresholding
// 	heelOnL=onOffPressure(pressSettingsL.heel,heelL);
// 	frontOnL=onOffPressure(pressSettingsL.front,frontL);
// 	heelOnR=onOffPressure(pressSettingsR.heel,heelR);
// 	frontOnR=onOffPressure(pressSettingsR.front,frontR);
//
// 	// compute the current state
// 	currState = 10*OnOffToCode(frontOnL,heelOnL)+OnOffToCode(frontOnR,heelOnR);
// }

int main(int argc, char* argv[])
{	
	printf("\nHello from PD Shoe (SONAR, RESET, EXT SYNC, HIP-PACK LED and 8ch)\n\n");
	//printf("WARNING: US_CYCLES 1000\n");
		
	// bool resetLED=0x00;
	// bool condResetLed;
	// bool resettingState=false;
	// uint32_t lastResetTimestamp;
    
    // metronome
	// int Tstr=0;
	// int Tstep_min=0;
	// int Tstep_max=0;
	// int rand_interval=0;
	// float RandRatio=0.0f;
	// bool metrobool=0x00;
	// uint64_t next_beat=0;
	
	// automatic feedback modes
	FILE * TemporalFile;
	char strSession[N_STR];
	char strTemporalFile[N_STR];
	// uint8_t mode[N_MAX_MODES];
	// uint8_t modePeriod[N_MAX_MODES];
	// int numMode=0;
	// uint8_t indCurrMode=0;
	// uint8_t currMode=255;
	// uint64_t nextModeSwitch=0;
	/*
	currMode=255; -> Do not force PD to change feedback mode
	currMode=0; -> Force PD to turn feedback mode off
	currMode=1..12; -> Force PD to change feedback mode off
	*/

	// TODO: 
	ros::init(argc,argv, "SportSole");
	ros::NodeHandle nh(std::string("~"));
	
	// TODO: This Modify the way the argument is parsed. Use the ROS parameter server instead to obtain the argument.
	if (argc>2)
	{
		printf("Error on input argument!\n");
		return -1;
	}
	else if (argc==1)
	{
		// No session name, simple functioning
		sprintf(strSession,"%s","");
	}
	else if (argc==2)
	{
		// Simple functioning
		sprintf(strSession,"%s",argv[1]);
        printf("Session Name: %s\n",strSession);
	}
		//     else if (argc==3)
		//     {
		// // with constant Metronome
		//         sprintf(strSession,"%s",argv[1]);
		//         Tstr = atoi (argv[2]);
		//         printf("Session Name: %s\n",strSession);
		//         printf("Metronome: %d [ms]\n",Tstr);
		//     }
		//     else if (argc==4)
		//     {
		// // with pseudorandom cues
		//         sprintf(strSession,"%s",argv[1]);
		// Tstep_min=atoi (argv[2]);// - atoi (argv[3]);
		// Tstep_max=atoi (argv[3]);// + atoi (argv[3]);
		//         printf("Session Name: %s\n",strSession);
		//         printf("Min Step Period: %d [ms]\n",Tstep_min);
		// printf("Max Step Period: %d [ms]\n",Tstep_max);
		//     }
		//     else if (argc==5)
		//     {
		// // with pseudorandom cues and fixed time periods
		// // use zeros for Tstep_min and Tstep_max if random cues not needed
		//         sprintf(strSession,"%s",argv[1]);
		// Tstep_min=atoi (argv[2]);// - atoi (argv[3]);
		// Tstep_max=atoi (argv[3]);// + atoi (argv[3]);
		// sprintf(strTemporalFile,"/media/rootfs/sd/log/%s",argv[4]);
		//
		// int n,m=1;
		// int i=0;
		//
		// ifstream read(strTemporalFile);
		// while(read>>n>>m){
		// mode[i]=n;
		// modePeriod[i]=m;
		// i++;
		// }
		// numMode=i;//number of lines
		// ifstream close(strTemporalFile);
		//
		//         printf("Session Name: %s\n",strSession);
		// if(Tstep_max > 0){
		//         printf("Min Step Period: %d [ms]\n",Tstep_min);
		// printf("Max Step Period: %d [ms]\n",Tstep_max);
		// }
		//
		// printf("\nParameter File: %s\n",strTemporalFile);
		// for(int i=0;i<numMode;i++)
		// {
		// 	printf("Period [%d]: mode %d, duration %d [s]\n",i,mode[i],modePeriod[i]);
		// }
		// //return -1;
		//     }

	
	// Load threshold levels
	// sprintf(strTemporalFile,"/media/rootfs/sd/log/thresholds.txt");
	// uint16_t nn=1;
	// int i=0;
	// uint16_t perMilleThr[]={0,0,0,0};
	// ifstream read(strTemporalFile);
	// while(read>>nn){
	// perMilleThr[i]=nn;
	// i++;
	// }
	// ifstream close(strTemporalFile);
	// printf("\nPressure Threshold Settings Loaded from: %s\n",strTemporalFile);
	// printf("Left Front: %d [per mille]\n",perMilleThr[0]);
	// printf("Left Heel: %d [per mille]\n",perMilleThr[1]);
	// printf("Right Front: %d [per mille]\n",perMilleThr[2]);
	// printf("Right Heel: %d [per mille]\n\n",perMilleThr[3]);
	
	//normalized pressure
	// bool resetPressure=0x00;
	// bool setPressure=0x00;
	// bool condResetPressure;
	// bool condSetPressure;
	// structPressSettings pressSettingsL;
	// structPressSettings pressSettingsR;
	// uint8_t currState=0;
	// uint16_t heelR,frontR,heelL,frontL;///////////
	// bool heelOnR,frontOnR,heelOnL,frontOnL;///////////
	// pressSettingsL.front.perMilleThreshold = perMilleThr[0];
	// pressSettingsL.heel.perMilleThreshold = perMilleThr[1];
	// pressSettingsR.front.perMilleThreshold = perMilleThr[2];
	// pressSettingsR.heel.perMilleThreshold = perMilleThr[3];
	

	//uint16_t pressureVal=0;
	//uint8_t PressBuff[]={0,0};
	//uint8_t *pointer;

	uint8_t bufferLog[PACKET_LENGTH_LOG];
	uint8_t bufferPd[PACKET_LENGTH_PD];
	uint8_t bufferLed[PACKET_LENGTH_LED];
	uint8_t bufferSync[PACKET_LENGTH_SYNC];
	uint8_t bufferTime[PACKET_LENGTH_TIME];
	
	thread threadPDShoeL;
	thread threadPDShoeR;
	//thread threadResetLED;
	thread threadSync;
	//thread threadPressureReadVal;
	
	structPDShoe PDShoeL;
	structPDShoe PDShoeR;
	structSWstat swStat;
	structSync Sync;
	structDataPacketPureDataRAW dataPacketRawL;
	structDataPacketPureDataRAW dataPacketRawR;
	structDataPacketPureData dataPacketL;
	structDataPacketPureData dataPacketR;
	structSyncPacket SyncPacket;
	
	swStat.packetLedSent=0;
	swStat.packetGuiSent=0;
	swStat.packetPdSent=0;
	swStat.packetErrorPdShoeL=0;
	swStat.packetErrorPdShoeR=0;
	swStat.packetErrorSync=0;
	swStat.packetBroadSent=0;
	
	
	struct timeval tv;
	uint64_t timestamp;
	uint64_t timestamp_start;
	uint32_t tCycle;
	uint64_t cycleMicrosTime=US_CYCLES;
	uint32_t cycles=0;
	
	
	char strDate[N_STR];
	char strFile[N_STR];
	
	time_t timer;
	struct tm tstruct;
	
	int sockfdPd;
	int sockfdGui;
	int sockfdLed;
	int sockfdSync;
	int sockfdBroad;
	
	struct sockaddr_in addrPd;
	struct sockaddr_in addrGui;
	struct sockaddr_in addrLed;
	struct sockaddr_in addrSync;
	struct sockaddr_in addrBroad;
	
	
	PDShoeL.name="PD Shoe [Left]";
	PDShoeL.ipAddress="192.168.1.11";
	PDShoeL.port=3456;
	//PDShoeL.frequencyError=0;
	PDShoeL.packetError=0;
	PDShoeL.packetReceived=0;
	PDShoeL.id=1;
	
	PDShoeR.name="PD Shoe [Right]";
	PDShoeR.ipAddress="192.168.1.12";
	PDShoeR.port=3457;
	//PDShoeR.frequencyError=0;
	PDShoeR.packetError=0;
	PDShoeR.packetReceived=0;
	PDShoeR.id=2;
	
	Sync.name="External Sync";
	Sync.ipAddress="192.168.1.13";
	Sync.port=3462;
	//Sync.frequencyError=0;
	Sync.packetError=0;
	Sync.packetReceived=0;
	Sync.id=3;
	

	sockfdPd=socket(AF_INET,SOCK_DGRAM,0);
	sockfdGui=socket(AF_INET,SOCK_DGRAM,0);
	sockfdLed=socket(AF_INET,SOCK_DGRAM,0);
	sockfdSync=socket(AF_INET,SOCK_DGRAM,0);
	sockfdBroad=socket(AF_INET,SOCK_DGRAM,0);
	
	//bcast_sock = socket(AF_INET, SOCK_DGRAM, 0);
	int broadcastEnable=1;
	int ret=setsockopt(sockfdBroad, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
	
	
	bzero(&addrPd,sizeof(addrPd));
	addrPd.sin_family = AF_INET;
    addrPd.sin_addr.s_addr=inet_addr("192.168.1.100");///("192.168.1.100");//("192.168.1.102");///
	addrPd.sin_port=htons(3459);
	
	bzero(&addrLed,sizeof(addrLed));
	addrLed.sin_family = AF_INET;
	addrLed.sin_addr.s_addr=inet_addr("192.168.1.13");
	addrLed.sin_port=htons(3458);
	
	bzero(&addrGui,sizeof(addrGui));
	addrGui.sin_family = AF_INET;
	addrGui.sin_addr.s_addr=inet_addr("192.168.1.101");
	addrGui.sin_port=htons(3460);
	
	bzero(&addrBroad,sizeof(addrBroad));
	addrBroad.sin_family = AF_INET;
	//addrBroad.sin_addr.s_addr=inet_addr("192.168.1.11");
	addrBroad.sin_addr.s_addr=inet_addr("192.168.1.255");
	addrBroad.sin_port=htons(3464);
	
	
	threadPDShoeL=thread(threadUDPreceive,&PDShoeL);
	threadPDShoeR=thread(threadUDPreceive,&PDShoeR);
	//threadResetLED=thread(threadCheckReset,&resetLED,&resetPressure,&setPressure);
	threadSync=thread(threadSYNCreceive,&Sync);
	//threadPressureReadVal=thread(threadCheckPressureReadVal,&pressureVal);
		
	threadPDShoeL.detach();
	threadPDShoeR.detach();
	//threadResetLED.detach();
	threadSync.detach();
	//threadPressureReadVal.detach();
	
	usleep(1000);
	
	uint8_t precTrigger;
	uint16_t normA1_foot;
	uint16_t normA2_foot;
	
	float laxr,layr,lazr;
	uint16_t normA1_heel;
	uint16_t normA2_heel;
	
	uint nPacketLed=0;
	uint64_t currenttime=0;
	
	timer=time(0);
	tstruct = *localtime(&timer);
	strftime(strDate, sizeof(strDate), "%Y-%m-%d_%H-%M-%S", &tstruct);
	sprintf(strFile,"/home/nvidia/log/%s_%s.dat",strDate,strSession);
	FILE * pFile;
	pFile = fopen (strFile, "wb");
	
#define CYCLES_WRITE 500
	
	uint8_t vFileBuffer[CYCLES_WRITE*PACKET_LENGTH_LOG];
	unsigned int idxFileWrite=0;
	
	uint8_t Odroid_Trigger=1;
	uint8_t sendSportSole=2;
	FILE * GPIOzero;
	FILE * GPIOone;
	GPIOzero = fopen("/sys/class/gpio/gpio204/value", "w");
	GPIOone = fopen("/sys/class/gpio/gpio204/value", "w");
	
	// Some initial blinking...
	// the on-board LED should stay on after an even number of loops
	for (int un=20; un>0; un--){
		if(Odroid_Trigger>0)
		{
			Odroid_Trigger=0;
		}
		else{
			Odroid_Trigger=1;
		}
		//writeGPIO(ledTrigger,GPIOzero,GPIOone);
	    usleep(75000);
	  }
	
	  /* Setup the random number generator so that it generates
	  a different sequence at each call of the program
	  */
		// 	  if(Tstep_max>0){
		// /* initialize random seed: */
		// srand (time(NULL));
		// 	  }
	
	printf("Waiting...\n");
	//RAND_MAX is (2^31-1)=2147483647
	
	bool cond=false;
	while(!cond)
	{
		dataMutex.lock();
		cond=(PDShoeL.packetReceived>0) && (PDShoeR.packetReceived>0); //&& (PDShoeR.packetReceived>0);
		dataMutex.unlock();
		
		usleep(500);
	}
	
	printf("Start!\n");
	timestamp_start=getMicrosTimeStamp();
		
	while(1)
	{
		timestamp=getMicrosTimeStamp();
		
		//CODE!
		dataMutex.lock();
				
		reconstructStructPureDataRAW(PDShoeL.lastPacket,dataPacketRawL);
		reconstructStructPureDataRAW(PDShoeR.lastPacket,dataPacketRawR);
		reconstructStructSyncPacket(Sync.lastPacket,SyncPacket);
		
		swStat.packetErrorPdShoeL=PDShoeL.packetError;
		swStat.packetErrorPdShoeR=PDShoeR.packetError;
		swStat.packetErrorSync=Sync.packetError;
		swStat.packetReceivedPdShoeL=PDShoeL.packetReceived;
		swStat.packetReceivedPdShoeR=PDShoeR.packetReceived;
		swStat.packetReceivedSync=Sync.packetReceived;
		
		dataMutex.unlock();
		
		reconstructStruct(dataPacketRawL,dataPacketL);
		reconstructStruct(dataPacketRawR,dataPacketR);
		
			
		// Pressure Normalization
		//dataMutex.lock();
		//condResetPressure=resetPressure;
		//condSetPressure=setPressure;
		//dataMutex.unlock();
		
		// if(condResetPressure){
// 		dataMutex.lock();
// 		resetPressure=0x00;
// 		dataMutex.unlock();
// 		condResetPressure = false;
// 		// reset upToNow max and min
// 		pressSettingsL.front.maxUpToNow=0;
// 		pressSettingsL.front.minUpToNow=1023;
// 		pressSettingsR.front.maxUpToNow=0;
// 		pressSettingsR.front.minUpToNow=1023;
// 		pressSettingsL.heel.maxUpToNow=0;
// 		pressSettingsL.heel.minUpToNow=1023;
// 		pressSettingsR.heel.maxUpToNow=0;
// 		pressSettingsR.heel.minUpToNow=1023;
// 		}
		
		// if(condSetPressure){
	// 	dataMutex.lock();
	// 	setPressure=0x00;
	// 	dataMutex.unlock();
	// 	condSetPressure = false;
	// 	// set max and min values for normalization
	// 	pressSettingsL.front.maxInUse=pressSettingsL.front.maxUpToNow;
	// 	pressSettingsL.front.minInUse=pressSettingsL.front.minUpToNow;
	// 	pressSettingsR.front.maxInUse=pressSettingsR.front.maxUpToNow;
	// 	pressSettingsR.front.minInUse=pressSettingsR.front.minUpToNow;
	// 	pressSettingsL.heel.maxInUse=pressSettingsL.heel.maxUpToNow;
	// 	pressSettingsL.heel.minInUse=pressSettingsL.heel.minUpToNow;
	// 	pressSettingsR.heel.maxInUse=pressSettingsR.heel.maxUpToNow;
	// 	pressSettingsR.heel.minInUse=pressSettingsR.heel.minUpToNow;
	// 	printf("Current Normalization Coefficients for Pressure Sensors:\n");
	// 	printf("|| LH[%04d:%04d] LF[%04d:%04d] || RH[%04d:%04d] RF[%04d:%04d]\n\n\n",pressSettingsL.heel.minInUse,pressSettingsL.heel.maxInUse,pressSettingsL.front.minInUse,pressSettingsL.front.maxInUse,pressSettingsR.heel.minInUse,pressSettingsR.heel.maxInUse,pressSettingsR.front.minInUse,pressSettingsR.front.maxInUse);
	// 	}
		
	//	updateState(currState,heelR,frontR,heelL,frontL,heelOnR,frontOnR,heelOnL,frontOnL,pressSettingsL,pressSettingsR,dataPacketL.p1,dataPacketL.p2,dataPacketL.p3,dataPacketL.p4,dataPacketL.p5,dataPacketL.p7,dataPacketL.p8,dataPacketR.p1,dataPacketR.p2,dataPacketR.p3,dataPacketR.p4,dataPacketR.p5,dataPacketR.p7,dataPacketR.p8);			
			
			// TODO: Populate the messages for left and right shoes with dataPacketL and dataPacketR. Then publish them.


							
		// Send data to PD
		// if ((cycles%2)==0)
		// {

		// 	        // Update Metronome
		// 	//         if((Tstr > 0) or (Tstep_max > 0))
		// 	// {
		// 	//  if(next_beat==0)
		// 	//         {
		// 	//             next_beat=timestamp + TIME_TO_WAIT_US; //allows #us before starting metronome
		// 	//         }
		// 	//         else if (timestamp >= next_beat){
		// 	// 	metrobool= !metrobool;
		// 	// if (Tstr > 0)
		// 	//         {
		// 	//             next_beat+=1000*Tstr/2;
		// 	//             //printf("metronome= %d\n",metronome);
		// 	//         }
		// 	//         else
		// 	//         {
		// 	// 	RandRatio=(float)(rand()/(RAND_MAX+1.0));
		// 	// 	rand_interval=(int)(Tstep_min+(Tstep_max-Tstep_min)*RandRatio);
		// 	// 	next_beat+=(uint64_t)(1000*rand_interval); // convert to useconds and add
		// 	// 	//printf("%d\n",rand_interval);
		// 	// 	//printf("%lld\n",(long long)next_beat);
		// 	//         }
		// 	// }
		// 	// }
			
			
	 //        // Update Feedback Mode
		// 	// 	        if(numMode != 0)
		// 	// {
		// 	//  if(nextModeSwitch==0)
		// 	// 	        {
		// 	// 	            nextModeSwitch=timestamp + TIME_TO_WAIT_US; //allows #us before starting automatic period switching
		// 	// 	        }
		// 	// 	        else if (timestamp >= nextModeSwitch){
		// 	// 	if(indCurrMode < numMode)
		// 	// 	{
		// 	// 		nextModeSwitch+=(uint64_t)(1000000*modePeriod[indCurrMode]);
		// 	// 		currMode=mode[indCurrMode];
		// 	// 		printf("Now Switching to Mode=%d for the next %d [s]\n",mode[indCurrMode],modePeriod[indCurrMode]);
		// 	// 		indCurrMode+=1;
		// 	// 	}
		// 	// 	else
		// 	// 	{
		// 	// 		// This instruction keeps the last assigned feedback mode
		// 	// 		// indefinitely
		// 	// 		currMode = 255;
		// 	// 		numMode = 0;
		// 	// 		// This turns the automatic feedback off
		// 	// 		Tstep_max = 0;
		// 	// 	}
		// 	// }
		// 	// }

		// 	//inline uint16_t convert2uint16(float fValue,float scale)
		// 	normA1_foot=convert2uint16(fabs(dataPacketL.ax1)+fabs(dataPacketL.ay1)+fabs(dataPacketL.az1),L1_NORM_ACC_SCALE);
		// 	normA2_foot=convert2uint16(fabs(dataPacketR.ax1)+fabs(dataPacketR.ay1)+fabs(dataPacketR.az1),L1_NORM_ACC_SCALE);
			
		// 	//normA1_foot=(uint16_t)(MAX_UINT16_FLOAT*(fabs(dataPacketL.ax1)+fabs(dataPacketL.ay1)+fabs(dataPacketL.az1))/L1_NORM_ACC_SCALE);
		// 	//normA2_foot=(uint16_t)(MAX_UINT16_FLOAT*(fabs(dataPacketR.ax1)+fabs(dataPacketR.ay1)+fabs(dataPacketR.az1))/L1_NORM_ACC_SCALE);
			
		// 	//void calculatesLinearAcceleration(float &lax,float &lay,float &laz, float ax,float ay,float az, float yaw,float pitch,float roll)
		// 	//calculatesLinearAcceleration(laxr,layr,lazr,dataPacketL.axr,dataPacketL.ayr,dataPacketL.azr,dataPacketL.yaw1,dataPacketL.pitch1,dataPacketL.roll1,-0.5236);
            
		// 	normA1_heel=(uint16_t)(MAX_UINT16_FLOAT*(fabs(laxr)+fabs(layr)+fabs(lazr))/L1_NORM_ACC_SCALE);
		// 	//printf("acc %3.2f %3.2f %3.2f\n",dataPacketL.axr,dataPacketL.ayr,dataPacketL.azr);
		// 	//printf("lacc %3.2f %3.2f %3.2f\n",laxr,layr,lazr);

		// 	//calculatesLinearAcceleration(laxr,layr,lazr,dataPacketR.axr,dataPacketR.ayr,dataPacketR.azr,dataPacketR.yaw1,dataPacketR.pitch1,dataPacketR.roll1,-0.5236);
		// 	//normA2_heel=(uint16_t)(MAX_UINT16_FLOAT*(fabs(laxr)+fabs(layr)+fabs(lazr))/L1_NORM_ACC_SCALE);
  //           normA2_heel=convert2uint16(fabs(laxr)+fabs(layr)+fabs(lazr),L1_NORM_ACC_SCALE);
			
		// 	//createPureDataShortPacket(bufferPd,normA1_foot,normA2_foot,normA1_heel,normA2_heel,heelL,frontL,heelR,frontL,(uint8_t)(metrobool),currMode);
		// 	///createPureDataShortPacket(bufferPd,bufferLog,normA1_foot,normA2_foot,normA1_heel,normA2_heel,metronome,SyncPacket.trigger);
			
		// 	sendto(sockfdPd,bufferPd,sizeof(bufferPd),0,(struct sockaddr *)&addrPd,sizeof(addrPd));
		// 	swStat.packetPdSent++;
			
		// }
		
		if ((cycles%70)==0)
		{
			sendto(sockfdGui,bufferLog,sizeof(bufferLog),0,(struct sockaddr *)&addrGui,sizeof(addrGui));
			swStat.packetGuiSent++;
		}
		
		if ((cycles%1000)==0)
		{
			if(Odroid_Trigger>0)
			{
				Odroid_Trigger=0;
			}
			else{
				Odroid_Trigger=1;
			}
			currenttime = getMicrosTimeStamp()-timestamp_start;
			createTimePacket(bufferTime,currenttime,Odroid_Trigger);
			//writeGPIO(ledTrigger, GPIOzero, GPIOone);	
			sendto(sockfdBroad,bufferTime,PACKET_LENGTH_TIME,0,(struct sockaddr *)&addrBroad,sizeof(addrBroad));
			swStat.packetBroadSent++;
		}
		
		
		// if (nPacketLed>0)
		// {
		// 	sendto(sockfdLed,bufferLed,PACKET_LENGTH_LED,0,(struct sockaddr *)&addrLed,sizeof(addrLed));
		// 	nPacketLed--;
		// 	writeGPIO(ledTrigger, GPIOzero, GPIOone);
		// }

		// if ((cycles%500)==0)
		// {
		// 	uint64_t currenttime;
		// 	currenttime = getMicrosTimeStamp();
		// 	createTimePacket(bufferTime,currenttime);
		// 	sendto(sockfdLed,bufferTime,PACKET_LENGTH_TIME,0,(struct sockaddr *)&addrLed,sizeof(addrLed));
		// 	nPacketLed--;
		// 	writeGPIO(ledTrigger, GPIOzero, GPIOone);
		// }

		// if(!resettingState)
		// {
		// 	dataMutex.lock();
		// 	condResetLed=resetLED;
		// 	dataMutex.unlock();
		//
		// 	if(condResetLed)
		// 	{
		// 		lastResetTimestamp=dataPacketL.timestamp;
		//
		// 		dataMutex.lock();
		// 		resetLED=0x00;
		// 		dataMutex.unlock();
		//
		// 		resettingState=true;
		// 	}
		// 	else
		// 	{
		// 		// if(precTrigger!=dataPacketL.trigger)
		// 		// {
		// 		// 	createLedPacket(bufferLed,dataPacketL.trigger);
		// 		// 	sendto(sockfdLed,bufferLed,PACKET_LENGTH_LED,0,(struct sockaddr *)&addrLed,sizeof(addrLed));
		// 		// 	swStat.packetLedSent++;
		// 		// 	nPacketLed=N_PACKET_LED;
		// 		// 	ledTrigger=dataPacketL.trigger;
		// 		// 	writeGPIO(ledTrigger, GPIOzero, GPIOone);
		// 		// }
		// 	}
		//
		// 	precTrigger=dataPacketL.trigger;
		//
		// }
		// else
		// {
		// 	precTrigger=0;
		// 	ledTrigger=0;
		// 	createLedPacket(bufferLed,precTrigger);
		// 	//check timestamp
		// 	if((dataPacketL.timestamp-lastResetTimestamp)>5000000)
		// 	{
		// 		resettingState=false;
		// 	}
		// }
		//
		
		//pointer=(uint8_t*)&pressureVal;
		//PressBuff[0]=pointer[1];
		//PressBuff[1]=pointer[0];
		
		createLogPacket(bufferLog,PDShoeL.lastPacket,PDShoeR.lastPacket,Odroid_Trigger,currenttime,SyncPacket.Ext_Trigger);
		//createLogPacket(bufferLog,PDShoeL.lastPacket,PDShoeR.lastPacket,precTrigger,(uint8_t)(metrobool),currMode,PressBuff[0],PressBuff[1]);
		if (idxFileWrite==(CYCLES_WRITE-1))
		{
			memcpy(&vFileBuffer[idxFileWrite*PACKET_LENGTH_LOG],&bufferLog,PACKET_LENGTH_LOG);
			
			//Write LOG
			fwrite(&vFileBuffer,CYCLES_WRITE*PACKET_LENGTH_LOG, 1, pFile);
			
			idxFileWrite=0;
		}
		else
		{
			memcpy(&vFileBuffer[idxFileWrite*PACKET_LENGTH_LOG],&bufferLog,PACKET_LENGTH_LOG);
			idxFileWrite++;
		}
		
		if((cycles%100)==0)
		{
			sendto(sockfdLed,bufferLed,PACKET_LENGTH_LED,0,(struct sockaddr *)&addrLed,sizeof(addrLed));
			swStat.packetLedSent++;
			//writeGPIO(ledTrigger, GPIOzero, GPIOone);
	
		}
		
		if((cycles%1000)==0)
		{
			// DEBUG
			//printf("|| LH%04d[%d] - LF%04d[%d] || RH%04d[%d] - RF%04d[%d] || Stat: %02d\n",heelL,heelOnL,frontL,frontOnL,heelR,heelOnR,frontR,frontOnR,currState);
			//printf("LEFT: p1[%d] - p2[%d] - p3[%d] - p4[%d]\n",dataPacketL.p1,dataPacketL.p2,dataPacketL.p3,dataPacketL.p4);
			//printf("RIGHT: p1[%d] - p2[%d] - p3[%d] - p4[%d]\n",dataPacketR.p1,dataPacketR.p2,dataPacketR.p3,dataPacketR.p4);
			//printf("State=%d\n",currState);

		     //        if(Tstr > 0)
		     //           {
			    // printf("cycles=%d err(L)=%d err(R)=%d p(L)=%d p(R)=%d metro ON\n",cycles,swStat.packetErrorPdShoeL,swStat.packetErrorPdShoeR,swStat.packetReceivedPdShoeL,swStat.packetReceivedPdShoeR);
		     //           }
		     //           else
		     //           {
			float currenttime_float_sec=((float)(currenttime))/1000000.0f;
				printf("cycles=%d err(L)=%d err(R)=%d err(S)=%d p(L)=%d p(R)=%d p(S)=%d p(B)=%d Tr=%d, t=%5.2f ExtSync=%d\n",cycles,swStat.packetErrorPdShoeL,swStat.packetErrorPdShoeR,swStat.packetErrorSync,swStat.packetReceivedPdShoeL,swStat.packetReceivedPdShoeR,swStat.packetReceivedSync,swStat.packetBroadSent,Odroid_Trigger,currenttime_float_sec,SyncPacket.Ext_Trigger);
				   //printf("cycles=%d err(L)=%d err(R)=%d err(S)=%d p(L)=%d p(R)=%d p(S)=%d PressVal=%d\n",cycles,swStat.packetErrorPdShoeL,swStat.packetErrorPdShoeR,swStat.packetErrorSync,swStat.packetReceivedPdShoeL,swStat.packetReceivedPdShoeR,swStat.packetReceivedSync,pressureVal);
				   //printf("[%d %d]\n",PressBuff[0],PressBuff[1]);
		               // }

	    }
		
			
		cycles++;
		
		tCycle=getMicrosTimeStamp()-timestamp;
		
		//printf("t cycle=%d\n",(uint16_t)tCycle);
		
#define US_SLEEP_CORRECTION 0 //48
		if(!(tCycle>cycleMicrosTime-US_SLEEP_CORRECTION)) usleep(cycleMicrosTime-US_SLEEP_CORRECTION-tCycle);
		//else printf("*\n");
		
		
	}
	
	//Never here, but..
	threadPDShoeL.~thread();
	threadPDShoeR.~thread();	
	//threadResetLED.~thread();
	threadSync.~thread();
	//threadPressureReadVal.~thread();
	return 0; 
} 

