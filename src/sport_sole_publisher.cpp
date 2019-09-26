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
#include <wordexp.h>

#include <ros/ros.h>
#include "sport_sole/SportSole.h"
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>


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
bool is_running = true;

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




void threadSYNCreceive(structSync* SyncBoard)
{
	dataMutex.lock();
	
	ROS_INFO("Hello from Thread! [%s ip: %s - port: %d]\n",SyncBoard->name.c_str(),SyncBoard->ipAddress.c_str(),SyncBoard->port);

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
		
	while(is_running)
	{		
		ret=recvfrom(sockfdServer,recvBuffer,PACKET_LENGTH_SYNC,MSG_DONTWAIT,(struct sockaddr *)&addrClient,&len);

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
		else
			this_thread::sleep_for(chrono::microseconds(1));
	}

}


void threadUDPreceive(structPDShoe* PDShoe)
{
	dataMutex.lock();
	
	ROS_INFO("Hello from Thread! [%s ip: %s - port: %d]\n",PDShoe->name.c_str(),PDShoe->ipAddress.c_str(),PDShoe->port);
	
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
		
	while(is_running)
	{		
		ret=recvfrom(sockfdServer,recvBuffer,PACKET_LENGTH_WIFI,MSG_DONTWAIT,(struct sockaddr *)&addrClient,&len);
		
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
		else
			this_thread::sleep_for(chrono::microseconds(1));
		cycles++;
	}
}



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


int main(int argc, char* argv[])
{	
	ROS_INFO("\nHello from PD Shoe (SONAR, RESET, EXT SYNC, HIP-PACK LED and 8ch)\n\n");
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

	// TODO: Complete CMakeLists.txt
	// create two publishers
	ros::init(argc, argv, "sport_sole_publisher");
	ros::NodeHandle n(std::string("~"));


	//publish msgs
	ros::Publisher msgLeft_pub = n.advertise<std_msgs::String> ("sport_sole_left", 0) ;
	ros::Publisher msgRight_pub = n.advertise<std_msgs::String> ("sport_sole_right", 0) ;


	//publisher created here for visualizing shoe's acceleration data
	ros::Publisher accel_left_pub = n.advertise<visualization_msgs::Marker> ("accel_left", 0);
	ros::Publisher accel_right_pub = n.advertise<visualization_msgs::Marker> ("accel_right", 0);

	
	// TODO: The value of the variable strSession is obtained from argv here. 
	// We want to modify the way the argument is parsed. 
	// Try and follow the tutorial on Parameter Server, and use the parameter server to obtain the argument supplied to rosrun, e.g. :
	// rosrun sport_sole sport_sole_publisher _session_name:=abc.
	std::string stringSession;
	if (n.getParam("session_name", stringSession)) {
		ROS_INFO("Session_name: %s\n",stringSession.c_str());
	}
	else
	{
		ROS_ERROR("Failed to get session_name");
		ros::shutdown();
		exit(1);
	}
	sprintf(strSession,"%s",stringSession.c_str());


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
		
	//threadPDShoeL.detach();
	//threadPDShoeR.detach();
	//threadResetLED.detach();
	//threadSync.detach();
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
	sprintf(strFile,"%s/log/%s_%s.dat", getenv("HOME"),strDate,strSession);
	ROS_INFO("Data will be logged in %s\n", strFile);
	FILE * pFile;
	pFile = fopen (strFile, "wb");
	if (!pFile)
	{
		ROS_ERROR_STREAM("Cannot open file for logging!");
	}
	
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
	
	ROS_INFO("Waiting...\n");
	//RAND_MAX is (2^31-1)=2147483647
	
	bool cond=false;
	while(!cond && ros::ok() && !ros::isShuttingDown())
	{
		dataMutex.lock();
		cond=(PDShoeL.packetReceived>0) && (PDShoeR.packetReceived>0); //&& (PDShoeR.packetReceived>0);
		dataMutex.unlock();
		
		usleep(500);
	}
	
	ROS_INFO("Start!\n");
	timestamp_start=getMicrosTimeStamp();
		
	while(ros::ok() && !ros::isShuttingDown())
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
			
			
		// TODO: Populate the messages for left and right shoes with dataPacketL and dataPacketR. Then publish them.
		sport_sole::SportSole msgLeft, msgRight;
		msgLeft.header.stamp = ros::Time::now();
		msgLeft.acceleration.linear.x = dataPacketL.ax1;
		msgLeft.acceleration.linear.y = dataPacketL.ay1; 
		msgLeft.acceleration.linear.z = dataPacketL.az1;
		tf::Quaternion q1_left(tf::Vector3(0, 0, 1), dataPacketL.yaw1);
		tf::Quaternion q2_left(tf::Vector3(-1, 0, 0), dataPacketL.pitch1);
		tf::Quaternion q3_left(tf::Vector3(0, 1, 0), dataPacketL.roll1);
		tf::Quaternion q_left = q1_left * q2_left * q3_left;
		tf::quaternionTFToMsg(q_left, msgLeft.quaternion);
		msgLeft.pressures[0] = dataPacketL.p1; 
		msgLeft.pressures[1] = dataPacketL.p2; 
		msgLeft.pressures[2] = dataPacketL.p3;
		msgLeft.pressures[3] = dataPacketL.p4;
		msgLeft.pressures[4] = dataPacketL.p5;
		msgLeft.pressures[5] = dataPacketL.p6;
		msgLeft.pressures[6] = dataPacketL.p7;
		msgLeft.pressures[7] = dataPacketL.p8;
		msgLeft_pub.publish(msgLeft);


		msgRight.header.stamp = ros::Time::now();
		msgRight.acceleration.linear.x = dataPacketR.ax1;
		msgRight.acceleration.linear.y = dataPacketR.ay1; 
		msgRight.acceleration.linear.z = dataPacketR.az1;
		tf::Quaternion q1_right(tf::Vector3(0, 0, 1), dataPacketR.yaw1);
		tf::Quaternion q2_right(tf::Vector3(-1, 0, 0), dataPacketR.pitch1);
		tf::Quaternion q3_right(tf::Vector3(0, 1, 0), dataPacketR.roll1);
		tf::Quaternion q_right = q1_right * q2_right * q3_right;
		tf::quaternionTFToMsg(q_right, msgRight.quaternion);
		msgRight.pressures[0] = dataPacketR.p1; 
		msgRight.pressures[1] = dataPacketR.p2; 
		msgRight.pressures[2] = dataPacketR.p3;
		msgRight.pressures[3] = dataPacketR.p4;
		msgRight.pressures[4] = dataPacketR.p5;
		msgRight.pressures[5] = dataPacketR.p6;
		msgRight.pressures[6] = dataPacketR.p7;
		msgRight.pressures[7] = dataPacketR.p8;
		msgRight_pub.publish(msgRight);


		// Populate data and use Publisher accel_left_pub to represent the acceleration
		visualization_msgs::Marker markerLeft, markerRight;
		geometry_msgs:: Point temp_point_left, temp_point_right; 
		markerLeft.header.frame_id = "shoe_left";
		markerLeft.header.stamp = ros::Time::now();
		markerLeft.ns = "~";
		markerLeft.id = 0;
		markerLeft.type = visualization_msgs::Marker::ARROW;
		markerLeft.action = visualization_msgs::Marker::ADD; 
		temp_point_left.x = temp_point_left.y = temp_point_left.z = 0;  //origin of arrow
		markerLeft.points.push_back(temp_point_left);  //creates the start of the arrow
		temp_point_left.x = msgLeft.acceleration.linear.x;
		temp_point_left.y = msgLeft.acceleration.linear.y;
		temp_point_left.z = msgLeft.acceleration.linear.z;
		markerLeft.points.push_back(temp_point_left);  //end of the arrow
		markerLeft.scale.x = 0.1;
		markerLeft.scale.y = 0.2;
		markerLeft.scale.z = 0.3;
		markerLeft.color.a = 1.0; 
		markerLeft.color.r = 1.0;
		markerLeft.color.g = 0.0;
		markerLeft.color.b = 0.0;
		accel_left_pub.publish(markerLeft);


		markerRight.header.frame_id = "shoe_right";
		markerRight.header.stamp = ros::Time::now();
		markerRight.ns = "~";
		markerRight.id = 1;
		markerRight.type = visualization_msgs::Marker::ARROW;
		markerRight.action = visualization_msgs::Marker::ADD; 
		temp_point_right.x = temp_point_right.y = temp_point_right.z = 0;  //origin of arrow
		markerRight.points.push_back(temp_point_right);  //creates the start of the arrow
		temp_point_right.x = msgRight.acceleration.linear.x;
		temp_point_right.y = msgRight.acceleration.linear.y;
		temp_point_right.z = msgRight.acceleration.linear.z;
		markerRight.points.push_back(temp_point_right);  //end of the arrow
		markerRight.scale.x = 0.1;
		markerRight.scale.y = 0.2;
		markerRight.scale.z = 0.3;
		markerRight.color.a = 1.0; 
		markerRight.color.r = 1.0;
		markerRight.color.g = 0.0;
		markerRight.color.b = 0.0;
		accel_right_pub.publish(markerRight);

		//incorporates rviz to visualize left shoe orientation (RPY)
		static tf::TransformBroadcaster br_left, br_right;
		tf::Transform transformL, transformR;
		transformL.setOrigin( tf::Vector3(0.75, 0, 0) );
		transformL.setRotation(q_left);
		br_left.sendTransform(tf::StampedTransform(transformL, ros::Time::now(), "map", "shoe_left"));

		transformR.setOrigin( tf::Vector3(-0.75, 0, 0) );
		transformR.setRotation(q_right);
		br_right.sendTransform(tf::StampedTransform(transformR, ros::Time::now(), "map", "shoe_right"));
		

		// Send data to PD
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
		
		createLogPacket(bufferLog,PDShoeL.lastPacket,PDShoeR.lastPacket,Odroid_Trigger,currenttime,SyncPacket.Ext_Trigger);
		//createLogPacket(bufferLog,PDShoeL.lastPacket,PDShoeR.lastPacket,precTrigger,(uint8_t)(metrobool),currMode,PressBuff[0],PressBuff[1]);
		if (idxFileWrite==(CYCLES_WRITE-1))
		{
			memcpy(&vFileBuffer[idxFileWrite*PACKET_LENGTH_LOG],&bufferLog,PACKET_LENGTH_LOG);
			
			//Write LOG
			if (pFile)
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
				ROS_INFO("cycles=%d err(L)=%d err(R)=%d err(S)=%d p(L)=%d p(R)=%d p(S)=%d p(B)=%d Tr=%d, t=%5.2f ExtSync=%d\n",cycles,swStat.packetErrorPdShoeL,swStat.packetErrorPdShoeR,swStat.packetErrorSync,swStat.packetReceivedPdShoeL,swStat.packetReceivedPdShoeR,swStat.packetReceivedSync,swStat.packetBroadSent,Odroid_Trigger,currenttime_float_sec,SyncPacket.Ext_Trigger);
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

	is_running = false;
	
	threadPDShoeL.join();
	threadPDShoeR.join();	
	threadSync.join();
	return 0; 
} 

