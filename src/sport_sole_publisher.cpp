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
#include <ros/time.h>
#include "sport_sole/SportSole.h"
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>


#define US_CYCLES 1000 //[us]

#define BUFFER 4096
#define PACKET_LENGTH_WIFI  77
#define PACKET_LENGTH_LOG   166
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
const int PUB_PERIOD_MS = 10;

#define TIME_TO_WAIT_US 11000000 // wait 11s before starting the metronome
using namespace std;

mutex dataMutex;
bool is_running = true;

// enum left or right
enum left_right_t {
	LEFT=0,
	RIGHT,
	LEFT_RIGHT
};

struct structDataPacketPureData 
{
	uint32_t timestamp;
		
	float yaw1,pitch1,roll1;
	float ax1,ay1,az1;
	
	float qw, qx, qy, qz;
	float wx, wy, wz;
	float r_ax, r_ay, r_az;
	float mx, my, mz;

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
	
	int16_t qw, qx, qy, qz;
	int16_t wx, wy, wz;
	int16_t r_ax, r_ay, r_az;
	int16_t mx, my, mz;

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
	
	float qw, qx, qy, qz;
	float wx, wy, wz;
	float r_ax, r_ay, r_az;
	float mx, my, mz;

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


// Gait Phase Finite State Machine
struct GaitPhaseFSM
{
	enum class GaitPhase : uint8_t {
		Swing = 0b00, // Swing
		Stance1 = 0b10, // Heel contact
		Stance2 = 0b11, // Foot flat
		Stance3 = 0b01 // Heel off
	};

	GaitPhase gait_phase;
	const double p_threshold = 100.0;
	
	GaitPhaseFSM():
		gait_phase(GaitPhase::Stance2)
	{
	}
	
	void update(const structDataPacketPureDataRAW & data)
	{
		double p_hind_sum = data.p6 + data.p7;
		double p_fore_sum = data.p1 + data.p2 + data.p3 + data.p4 + data.p5;

		switch (gait_phase) {
			case GaitPhase::Swing:
				if (p_hind_sum > p_threshold) 
					gait_phase = GaitPhase::Stance1;
				break;
			case GaitPhase::Stance1:
				if (p_fore_sum > p_threshold) 
					gait_phase = GaitPhase::Stance2;
				break;
			case GaitPhase::Stance2:
				if (p_hind_sum <= p_threshold) 
					gait_phase = GaitPhase::Stance3;
				break;
			case GaitPhase::Stance3:
				if (p_fore_sum <= p_threshold) 
					gait_phase = GaitPhase::Swing;
				break;
		}
	}

	uint8_t getGaitPhase()
	{
		return static_cast<uint8_t>(gait_phase);
	}
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

	dataPacket.qw = ((float)dataPacketRAW.qw)/FACTOR_SCALE_ANGLE;
	dataPacket.qx = ((float)dataPacketRAW.qx)/FACTOR_SCALE_ANGLE;
	dataPacket.qy = ((float)dataPacketRAW.qy)/FACTOR_SCALE_ANGLE;
	dataPacket.qz = ((float)dataPacketRAW.qz)/FACTOR_SCALE_ANGLE;

	dataPacket.wx = dataPacketRAW.wx / 900.0f;
	dataPacket.wy = dataPacketRAW.wy / 900.0f;
	dataPacket.wz = dataPacketRAW.wz / 900.0f;

	dataPacket.r_ax = dataPacketRAW.r_ax / 1000.0f;
	dataPacket.r_ay = dataPacketRAW.r_ay / 1000.0f;
	dataPacket.r_az = dataPacketRAW.r_az / 1000.0f;

	dataPacket.mx = dataPacketRAW.mx / 1000.0f;
	dataPacket.my = dataPacketRAW.my / 1000.0f;
	dataPacket.mz = dataPacketRAW.mz / 1000.0f;
	
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
	// Extra data field

	const size_t idx0 = 19;
	size_t idx = idx0;

	// quaternion
	pointer=(uint8_t*)&dataPacket.qw;
	pointer[1]=recvbuffer[idx++];
	pointer[0]=recvbuffer[idx++];
	
	pointer=(uint8_t*)&dataPacket.qx;
	pointer[1]=recvbuffer[idx++];
	pointer[0]=recvbuffer[idx++];

	pointer=(uint8_t*)&dataPacket.qy;
	pointer[1]=recvbuffer[idx++];
	pointer[0]=recvbuffer[idx++];

	pointer=(uint8_t*)&dataPacket.qz;
	pointer[1]=recvbuffer[idx++];
	pointer[0]=recvbuffer[idx++];

	// raw gyro
	pointer=(uint8_t*)&dataPacket.wx;
	pointer[1]=recvbuffer[idx++];
	pointer[0]=recvbuffer[idx++];

	pointer=(uint8_t*)&dataPacket.wy;
	pointer[1]=recvbuffer[idx++];
	pointer[0]=recvbuffer[idx++];

	pointer=(uint8_t*)&dataPacket.wz;
	pointer[1]=recvbuffer[idx++];
	pointer[0]=recvbuffer[idx++];

	// raw acc
	pointer=(uint8_t*)&dataPacket.r_ax;
	pointer[1]=recvbuffer[idx++];
	pointer[0]=recvbuffer[idx++];

	pointer=(uint8_t*)&dataPacket.r_ay;
	pointer[1]=recvbuffer[idx++];
	pointer[0]=recvbuffer[idx++];

	pointer=(uint8_t*)&dataPacket.r_az;
	pointer[1]=recvbuffer[idx++];
	pointer[0]=recvbuffer[idx++];

	// raw magnetometer
	pointer=(uint8_t*)&dataPacket.mx;
	pointer[1]=recvbuffer[idx++];
	pointer[0]=recvbuffer[idx++];

	pointer=(uint8_t*)&dataPacket.my;
	pointer[1]=recvbuffer[idx++];
	pointer[0]=recvbuffer[idx++];

	pointer=(uint8_t*)&dataPacket.mz;
	pointer[1]=recvbuffer[idx++];
	pointer[0]=recvbuffer[idx++];

	size_t idx_cnt = idx - idx0; // Must be 26

	/////////////////////////////////////////////

	pointer=(uint8_t*)&dataPacket.p1;
	pointer[1]=recvbuffer[idx_cnt + 19];
	pointer[0]=recvbuffer[idx_cnt + 20];

	pointer=(uint8_t*)&dataPacket.p2;
	pointer[1]=recvbuffer[idx_cnt + 21];
	pointer[0]=recvbuffer[idx_cnt + 22];

	pointer=(uint8_t*)&dataPacket.p3;
	pointer[1]=recvbuffer[idx_cnt + 23];
	pointer[0]=recvbuffer[idx_cnt + 24];

	pointer=(uint8_t*)&dataPacket.p4;
	pointer[1]=recvbuffer[idx_cnt + 25];
	pointer[0]=recvbuffer[idx_cnt + 26];

	pointer=(uint8_t*)&dataPacket.p5;
	pointer[1]=recvbuffer[idx_cnt + 27];
	pointer[0]=recvbuffer[idx_cnt + 28];

	pointer=(uint8_t*)&dataPacket.p6;
	pointer[1]=recvbuffer[idx_cnt + 29];
	pointer[0]=recvbuffer[idx_cnt + 30];

	pointer=(uint8_t*)&dataPacket.p7;
	pointer[1]=recvbuffer[idx_cnt + 31];
	pointer[0]=recvbuffer[idx_cnt + 32];

	pointer=(uint8_t*)&dataPacket.p8;
	pointer[1]=recvbuffer[idx_cnt + 33];
	pointer[0]=recvbuffer[idx_cnt + 34];
	/////////////////////////////////////////////

    //Timestamp_Odroid
	pointer=(uint8_t*)&dataPacket.Odroid_Timestamp;
	pointer[7]=recvbuffer[idx_cnt + 35];
	pointer[6]=recvbuffer[idx_cnt + 36];
	pointer[5]=recvbuffer[idx_cnt + 37];
	pointer[4]=recvbuffer[idx_cnt + 38];
	pointer[3]=recvbuffer[idx_cnt + 49];
	pointer[2]=recvbuffer[idx_cnt + 40];
	pointer[1]=recvbuffer[idx_cnt + 41];
	pointer[0]=recvbuffer[idx_cnt + 42];

	// trigger
	pointer=(uint8_t*)&dataPacket.Odroid_Trigger;
	pointer[0]=recvbuffer[idx_cnt + 43];
 
    // timestamp2 
	pointer=(uint8_t*)&dataPacket.timestamp2;
	pointer[3]=recvbuffer[idx_cnt + 44];
	pointer[2]=recvbuffer[idx_cnt + 45];
	pointer[1]=recvbuffer[idx_cnt + 46];
	pointer[0]=recvbuffer[idx_cnt + 47];	// 73
    // //currenttime


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
	return (buffer[0]==0x07 && buffer[1]==0x08 && buffer[2]==0x09 && 
		buffer[PACKET_LENGTH_WIFI - 3]==0xA && buffer[PACKET_LENGTH_WIFI - 2]==0xB && buffer[PACKET_LENGTH_WIFI - 1]==0xC && ret==PACKET_LENGTH_WIFI);
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
	
	ROS_INFO("Hello from Thread! [%s ip: %s - port: %d]",SyncBoard->name.c_str(),SyncBoard->ipAddress.c_str(),SyncBoard->port);

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
			this_thread::sleep_for(chrono::milliseconds(1));
	}

}


void threadUDPreceive(structPDShoe* PDShoe)
{
	dataMutex.lock();
	
	ROS_INFO("Hello from Thread! [%s ip: %s - port: %d]",PDShoe->name.c_str(),PDShoe->ipAddress.c_str(),PDShoe->port);
	
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
			this_thread::sleep_for(chrono::milliseconds(1));
		cycles++;
	}
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
	ROS_INFO("Hello from PD Shoe (SONAR, RESET, EXT SYNC, HIP-PACK LED and 8ch)");

	// TODO: Complete CMakeLists.txt
	// create two publishers
	ros::init(argc, argv, "sport_sole_publisher");
	ros::NodeHandle n(std::string("~"));
	string global_frame_ids[LEFT_RIGHT];
	for (size_t lr: {LEFT, RIGHT})
	{
		n.param<std::string>(string("global_frame_id_") + (lr == LEFT ? "l" : "r"), global_frame_ids[lr], "map");
	}

	// Publish time offset variations.
	ros::Publisher pub_l_to_ros = n.advertise<std_msgs::Float32>("l_to_ros", 1);
	ros::Publisher pub_r_to_ros = n.advertise<std_msgs::Float32>("r_to_ros", 1);

	//publish msg
	ros::Publisher pub_sport_sole = n.advertise<sport_sole::SportSole> ("sport_sole", 10) ;

	//publisher created here for visualizing shoe's acceleration data and orientation
	ros::Publisher pub_markers = n.advertise<visualization_msgs::MarkerArray> ("sport_sole_markers", 1);

	// Transform broadcaster
	tf2_ros::TransformBroadcaster tf_broadcaster;

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

	GaitPhaseFSM gait_phase_fsms[LEFT_RIGHT];
	
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

	// Gravitational acceleration in Hoboken.
	const double GRAVITATIONAL_ACCELERATION = 9.81772;
	
	ros::Time ros_stamp_base;
	ros::Duration transmission_delay(0.002);
	ros::Duration l_to_ros_offset(0, 0);
	ros::Duration r_to_ros_offset(0, 0);

	constexpr double alpha_low_pass = 0.02;

	auto getRosTimestampL = [&]()->ros::Time{
		if (l_to_ros_offset.isZero()) 
			l_to_ros_offset = ros::Time::now() - ros::Time(dataPacketL.timestamp * 1e-6);
		ros::Duration l_to_ros = ros::Time::now() - ros::Time(dataPacketL.timestamp * 1e-6);
		std_msgs::Float32 msg; 
		msg.data = (l_to_ros - l_to_ros_offset).toSec();
		pub_l_to_ros.publish(msg);
		l_to_ros_offset = l_to_ros_offset + (l_to_ros - l_to_ros_offset) * alpha_low_pass;
		return ros::Time(dataPacketL.timestamp * 1e-6) + l_to_ros_offset;
	};

	auto getRosTimestampR = [&]()->ros::Time{
		if (r_to_ros_offset.isZero()) 
			r_to_ros_offset = ros::Time::now() - ros::Time(dataPacketR.timestamp * 1e-6);
		ros::Duration r_to_ros = ros::Time::now() - ros::Time(dataPacketR.timestamp * 1e-6);
		std_msgs::Float32 msg;
		msg.data = (r_to_ros - r_to_ros_offset).toSec();
		pub_r_to_ros.publish(msg);
		r_to_ros_offset = r_to_ros_offset + (r_to_ros - r_to_ros_offset) * alpha_low_pass;
		return ros::Time(dataPacketR.timestamp * 1e-6) + r_to_ros_offset;
	};
	
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

	// Send an enable packet
	currenttime = getMicrosTimeStamp()-timestamp_start;
	createTimePacket(bufferTime,currenttime,Odroid_Trigger);
	bufferTime[3] = 1;
	sendto(sockfdBroad,bufferTime,PACKET_LENGTH_TIME,0,(struct sockaddr *)&addrBroad,sizeof(addrBroad));
	ROS_INFO("Enable packet sent");
	
	ROS_INFO("Waiting...");
	//RAND_MAX is (2^31-1)=2147483647
	
	bool cond=false;
	while(!cond && ros::ok() && !ros::isShuttingDown())
	{
		dataMutex.lock();
		cond=(PDShoeL.packetReceived>0) && (PDShoeR.packetReceived>0); //&& (PDShoeR.packetReceived>0);
		dataMutex.unlock();

		// Set ROS stamp base
		if (cond)
			ros_stamp_base = ros::Time::now() - transmission_delay;
		
		usleep(500);
	}
	
	ROS_INFO("Start!");
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
			
		sport_sole::SportSole msg;
		uint32_t l_stamp_curr = dataPacketL.timestamp;
		static uint32_t l_stamp_desired = l_stamp_curr;
		//ROS_INFO_STREAM("Time difference: " << (getRosTimestampL() - getRosTimestampR()).nsec);
		//ROS_INFO_STREAM("Stamp: " << getRosTimestampL());
		// if (cycles % PUB_PERIOD_MS == 0)
		if (l_stamp_curr >= l_stamp_desired)
		{
			l_stamp_desired = l_stamp_curr + 1000; // up to 1000 Hz
			ros::Time l_stamp_ros = getRosTimestampL();
			ros::Time r_stamp_ros = getRosTimestampR();
			
			// Define the function to get the quaternion
			auto assignQuaternion = [](const structDataPacketPureData & data_packet, geometry_msgs::Quaternion & q_msg) {
				#if 0
					// Use Euler angles
					tf::Quaternion q1(tf::Vector3(0, 0, 1), data_packet.yaw1);
					tf::Quaternion q2(tf::Vector3(0, 1, 0), data_packet.pitch1);
					tf::Quaternion q3(tf::Vector3(1, 0, 0), data_packet.roll1);
					tf::Quaternion q = q1 * q3 * q2;
					tf::quaternionTFToMsg(q, q_msg);
				#else
					// User quaternion
					tf::Quaternion q(-data_packet.qy, data_packet.qx, data_packet.qz, data_packet.qw);
					static const tf::Quaternion q2{{1, 0, 0}, M_PI};
					q = q * q2;
					q_msg.w = q.w();
					q_msg.x = q.x();
					q_msg.y = q.y();
					q_msg.z = q.z();
				#endif
			};
			
			// Populate the SportSole message
			msg.header.stamp = l_stamp_ros;
			msg.raw_acceleration[0].linear.x = -dataPacketL.r_ay * GRAVITATIONAL_ACCELERATION;
			msg.raw_acceleration[0].linear.y = dataPacketL.r_ax * GRAVITATIONAL_ACCELERATION;
			msg.raw_acceleration[0].linear.z = dataPacketL.r_az * GRAVITATIONAL_ACCELERATION;
			msg.angular_velocity[0].x = -dataPacketL.wy;
			msg.angular_velocity[0].y = dataPacketL.wx;
			msg.angular_velocity[0].z = dataPacketL.wz;
			msg.acceleration[0].linear.x = -dataPacketL.ay1 * GRAVITATIONAL_ACCELERATION;
			msg.acceleration[0].linear.y = dataPacketL.ax1 * GRAVITATIONAL_ACCELERATION;
			msg.acceleration[0].linear.z = dataPacketL.az1 * GRAVITATIONAL_ACCELERATION;
			assignQuaternion(dataPacketL, msg.quaternion[0]);
			int p_index = 0;
			msg.pressures[p_index++] = dataPacketL.p1; 
			msg.pressures[p_index++] = dataPacketL.p2; 
			msg.pressures[p_index++] = dataPacketL.p3;
			msg.pressures[p_index++] = dataPacketL.p4;
			msg.pressures[p_index++] = dataPacketL.p5;
			msg.pressures[p_index++] = dataPacketL.p6;
			msg.pressures[p_index++] = dataPacketL.p7;
			msg.pressures[p_index++] = dataPacketL.p8;


			msg.raw_acceleration[1].linear.x = -dataPacketR.r_ay * GRAVITATIONAL_ACCELERATION;
			msg.raw_acceleration[1].linear.y = dataPacketR.r_ax * GRAVITATIONAL_ACCELERATION;
			msg.raw_acceleration[1].linear.z = dataPacketR.r_az * GRAVITATIONAL_ACCELERATION;
			msg.angular_velocity[1].x = -dataPacketR.wy;
			msg.angular_velocity[1].y = dataPacketR.wx;
			msg.angular_velocity[1].z = dataPacketR.wz;
			msg.acceleration[1].linear.x = -dataPacketR.ay1 * GRAVITATIONAL_ACCELERATION;
			msg.acceleration[1].linear.y = dataPacketR.ax1 * GRAVITATIONAL_ACCELERATION; 
			msg.acceleration[1].linear.z = dataPacketR.az1 * GRAVITATIONAL_ACCELERATION;
			assignQuaternion(dataPacketR, msg.quaternion[1]);
			msg.pressures[p_index++] = dataPacketR.p1; 
			msg.pressures[p_index++] = dataPacketR.p2; 
			msg.pressures[p_index++] = dataPacketR.p3;
			msg.pressures[p_index++] = dataPacketR.p4;
			msg.pressures[p_index++] = dataPacketR.p5;
			msg.pressures[p_index++] = dataPacketR.p6;
			msg.pressures[p_index++] = dataPacketR.p7;
			msg.pressures[p_index++] = dataPacketR.p8;

			// TO-DO
			gait_phase_fsms[LEFT].update(dataPacketRawL);
			gait_phase_fsms[RIGHT].update(dataPacketRawR);
			
			// Ignore the first 4 sec of data because the gravity was not removed yet.
			if ((l_stamp_ros - ros_stamp_base).toSec() > 4.5)
			{
				msg.gait_state = 0;
				msg.gait_state |= 
					(gait_phase_fsms[LEFT].getGaitPhase() << 2) |
					(gait_phase_fsms[RIGHT].getGaitPhase() << 0);
				pub_sport_sole.publish(msg);
			}
			
			// Construct and publish marker array and tfs for visualization
			if (pub_markers.getNumSubscribers() > 0)
			{
				// Define the message for pub_marker
				visualization_msgs::MarkerArrayPtr marker_array_ptr(new visualization_msgs::MarkerArray);
				
				// Tail of the arrows
				geometry_msgs:: Point arrow_tails[LEFT_RIGHT];
				if (global_frame_ids[LEFT].compare(global_frame_ids[RIGHT]) == 0)
				{
					arrow_tails[LEFT].y = 0.5;
					arrow_tails[RIGHT].y = -0.5;
				}

				// Draw arrows to represent acceleration
				for (size_t lr: {LEFT, RIGHT})
				{
					visualization_msgs::MarkerPtr marker_ptr(new visualization_msgs::Marker);
				
					marker_ptr->header.stamp = (lr == LEFT) ? l_stamp_ros : r_stamp_ros;
					marker_ptr->header.frame_id = global_frame_ids[lr];
					marker_ptr->ns = "~";
					marker_ptr->id = lr; 
					marker_ptr->lifetime = ros::Duration(1.0);
					marker_ptr->type = visualization_msgs::Marker::ARROW;
					marker_ptr->action = visualization_msgs::Marker::ADD; 

					
					marker_ptr->points.push_back(arrow_tails[lr]);

					// Head of the arrow
					geometry_msgs::Point arrow_head;
					const double arrow_scale_factor = 0.2;
					arrow_head.x = arrow_tails[lr].x + msg.acceleration[lr].linear.x * arrow_scale_factor;
					arrow_head.y = arrow_tails[lr].y + msg.acceleration[lr].linear.y * arrow_scale_factor;
					arrow_head.z = arrow_tails[lr].z + msg.acceleration[lr].linear.z * arrow_scale_factor;
					marker_ptr->points.push_back(arrow_head); 

					marker_ptr->scale.x = 0.05;
					marker_ptr->scale.y = 0.1;
					marker_ptr->scale.z = 0.2;
					marker_ptr->color.a = 1.0; 
					marker_ptr->color.r = 1.0;
					marker_ptr->color.g = 0.0;
					marker_ptr->color.b = 0.0;
					marker_array_ptr->markers.push_back(*marker_ptr);
				}

				// Draw a rectangular prism to represent orientation
				for (size_t lr: {LEFT, RIGHT})
				{
					visualization_msgs::MarkerPtr marker_ptr(new visualization_msgs::Marker);
					
					marker_ptr->header.stamp = (lr == LEFT) ? l_stamp_ros : r_stamp_ros;
					marker_ptr->header.frame_id = global_frame_ids[lr];
					marker_ptr->lifetime = ros::Duration(0.13);
					marker_ptr->ns = n.getNamespace();
					marker_ptr->type = visualization_msgs::Marker::CUBE;
					marker_ptr->id = lr + 2;

					marker_ptr->pose.position = arrow_tails[lr];

					// 
					marker_ptr->pose.orientation = msg.quaternion[lr];

					marker_ptr->scale.x = 0.3;
					marker_ptr->scale.y = 0.1;
					marker_ptr->scale.z = 0.05;

					marker_ptr->color.a = 1.0;
					marker_ptr->color.r = 1.0;
					marker_ptr->color.g = 0.0;
					marker_ptr->color.b = 1.0;
					marker_array_ptr->markers.push_back(*marker_ptr);
				}

				// Publish the markers
				pub_markers.publish(*marker_array_ptr);
			

				// Broadcast transforms
				for (size_t lr: {LEFT, RIGHT})
				{
					geometry_msgs::TransformStamped msg_tf; 
					msg_tf.header.stamp = (lr == LEFT) ? l_stamp_ros : r_stamp_ros;
					msg_tf.header.frame_id = global_frame_ids[lr];
					msg_tf.child_frame_id = (lr == LEFT) ? "imu_left" : "imu_right";
					msg_tf.transform.translation.x = arrow_tails[lr].x;
					msg_tf.transform.translation.y = arrow_tails[lr].y;
					msg_tf.transform.translation.z = arrow_tails[lr].z;
					msg_tf.transform.rotation = msg.quaternion[lr];
					tf_broadcaster.sendTransform(msg_tf);
				}
			}

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
		
		if((cycles%1000)==0)
		{

			float currenttime_float_sec=((float)(currenttime))/1000000.0f;
			ROS_INFO("cycles=%d err(L)=%d err(R)=%d err(S)=%d p(L)=%d p(R)=%d p(S)=%d p(B)=%d Tr=%d, t=%5.2f ExtSync=%d",cycles,swStat.packetErrorPdShoeL,swStat.packetErrorPdShoeR,swStat.packetErrorSync,swStat.packetReceivedPdShoeL,swStat.packetReceivedPdShoeR,swStat.packetReceivedSync,swStat.packetBroadSent,Odroid_Trigger,currenttime_float_sec,SyncPacket.Ext_Trigger);
			//printf("cycles=%d err(L)=%d err(R)=%d err(S)=%d p(L)=%d p(R)=%d p(S)=%d PressVal=%d",cycles,swStat.packetErrorPdShoeL,swStat.packetErrorPdShoeR,swStat.packetErrorSync,swStat.packetReceivedPdShoeL,swStat.packetReceivedPdShoeR,swStat.packetReceivedSync,pressureVal);

	    }
		
		cycles++;
		
		tCycle=getMicrosTimeStamp()-timestamp;
		
		//printf("t cycle=%d\n",(uint16_t)tCycle);
		
#define US_SLEEP_CORRECTION 0 //48
		if(!(tCycle>cycleMicrosTime-US_SLEEP_CORRECTION)) usleep(cycleMicrosTime-US_SLEEP_CORRECTION-tCycle);
		//else printf("*\n");
		
		
	}

	// Send an reset packet
	currenttime = getMicrosTimeStamp()-timestamp_start;
	createTimePacket(bufferTime,currenttime,Odroid_Trigger);
	bufferTime[3] = 0;
	sendto(sockfdBroad,bufferTime,PACKET_LENGTH_TIME,0,(struct sockaddr *)&addrBroad,sizeof(addrBroad));
	ROS_INFO("Reset packet sent!");
	
	is_running = false;
	
	threadPDShoeL.join();
	threadPDShoeR.join();	
	threadSync.join();
	return 0; 
} 

