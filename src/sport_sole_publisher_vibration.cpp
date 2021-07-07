//Modifited by Qingya Zhao, Jun 2021

// WITH RESET

#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <mutex>
#include <algorithm>
#include <cassert>

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
#include <owt.h>

#include "sport_sole/sport_sole_common.h"
using namespace sport_sole;

#include <condition_variable>
std::condition_variable cond_var;

#include <Eigen/Dense>

#define US_CYCLES 1000 //[us]

#define BUFFER 4096
#define PACKET_LENGTH_WIFI  97 //57 //47//47//45
#define PACKET_LENGTH_LOG   214 //118//141//98//95//91//88//87//86//84
#define PACKET_LENGTH_PD    24//22
#define PACKET_LENGTH_WRIST 29 //25
#define PACKET_LENGTH_LED    8
#define PACKET_LENGTH_TIME   16//8
#define PACKET_LENGTH_SYNC   7
#define N_PACKET_LED 20
#define RADTODEG 57.295780
#define FACTOR_SCALE_ANGLE 5000.0f
#define FACTOR_SCALE_QUAT 5000.0f
#define FACTOR_SCALE_ACCELERATION 1000.0f
#define FACTOR_SCALE_ANG_VEL 900.0f
#define FACTOR_SCALE_MAG 1000.0f
#define FACTOR_SCALE_ACC_ADXL 128.0f
#define MAX_UINT16_FLOAT 65536.0f
#define MAX_UINT16 65536
#define L1_NORM_ACC_SCALE 4.0f

#define FACTOR_SCALE_AFO 10000.0f
#define FACTOR_SCALE_SV  1000.0f
#define FACTOR_SCALE_OMEGA  1000.0f
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

#define ADXL345_LSB_G_2G            256.0f
#define ADXL345_LSB_G_4G            128.0f
#define ADXL345_LSB_G_8G             64.0f

#define TIME_TO_WAIT_US 11000000 // wait 11s before starting the metronome
const double pi = 3.14159265358979323846;
using namespace std;
using namespace Eigen;

mutex dataMutex;
bool is_running = true;


struct structDataPacketPureData 
{
	uint32_t timestamp;
		
	float yaw1,pitch1,roll1;
	float ax1,ay1,az1;
	float qw1,qx1,qy1,qz1;
	float gx1,gy1,gz1;
	float r_ax1,r_ay1,r_az1;
	float mx1,my1,mz1;

	uint16_t p1,p2,p3,p4,p5,p6,p7,p8;
	uint64_t Odroid_Timestamp;
	uint8_t Odroid_Trigger;
	uint32_t timestamp2;

	// why no sonar here?
    float AFO;
	float AFOc;
	float thetaM;
	float SV;
	float SL;
	uint32_t Gait_State; 
	float Omega_m;
	float Delta_theta_d;
	float AFO_omega;
};

struct structDataPacketPureDataRAW
{
	uint32_t timestamp;
		
	int16_t yaw1,pitch1,roll1;
	int16_t ax1,ay1,az1;
	int16_t qw1,qx1,qy1,qz1;
	int16_t gx1,gy1,gz1;
	int16_t r_ax1,r_ay1,r_az1;
	int16_t mx1,my1,mz1;

	uint16_t p1,p2,p3,p4,p5,p6,p7,p8;
	uint64_t Odroid_Timestamp;
	uint8_t Odroid_Trigger;
	uint32_t timestamp2;
	uint16_t AFO;
	uint16_t AFOc;
	uint16_t thetaM;
	uint16_t SV;
	uint16_t SL;
	uint32_t Gait_State;
	uint16_t Omega_m;
	int16_t  Delta_theta_d;
	int16_t AFO_omega;
};

struct structDataPacketPureDataWrist
{
	uint32_t timestamp;

	float ax, ay, az;

	uint64_t Odroid_Timestamp;
	uint8_t Odroid_Trigger;

	uint32_t timestamp2;
};

struct structDataPacketPureDataRAWWrist
{
	uint32_t timestamp;

	int16_t ax, ay, az;

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
	float AFO;
	float AFOc;
	float thetaM;
	float SV;
	float SL;
	uint32_t Gait_State; 
	float Omega_m;
	float Delta_theta_d;	
	float AFO_omega;
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
	unsigned int packetErrorWristR;
	unsigned int packetErrorSync;
	unsigned int packetBroadSent;
	unsigned int packetReceivedPdShoeL;
	unsigned int packetReceivedPdShoeR;
	unsigned int packetReceivedWristR;
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

	dataPacket.qw1=((float)dataPacketRAW.qw1)/FACTOR_SCALE_QUAT;
	dataPacket.qx1=((float)dataPacketRAW.qx1)/FACTOR_SCALE_QUAT;
	dataPacket.qy1=((float)dataPacketRAW.qy1)/FACTOR_SCALE_QUAT;
	dataPacket.qz1=((float)dataPacketRAW.qz1)/FACTOR_SCALE_QUAT;

	dataPacket.gx1 = ((float)dataPacketRAW.gx1)/FACTOR_SCALE_ANG_VEL;
	dataPacket.gy1 = ((float)dataPacketRAW.gy1)/FACTOR_SCALE_ANG_VEL;
	dataPacket.gz1 = ((float)dataPacketRAW.gz1)/FACTOR_SCALE_ANG_VEL;

	dataPacket.r_ax1=((float)dataPacketRAW.r_ax1)/FACTOR_SCALE_ACCELERATION;
	dataPacket.r_ay1=((float)dataPacketRAW.r_ay1)/FACTOR_SCALE_ACCELERATION;
	dataPacket.r_az1=((float)dataPacketRAW.r_az1)/FACTOR_SCALE_ACCELERATION;

	dataPacket.mx1 = ((float)dataPacketRAW.mx1)/FACTOR_SCALE_MAG;
	dataPacket.my1 = ((float)dataPacketRAW.my1)/FACTOR_SCALE_MAG;
	dataPacket.mz1 = ((float)dataPacketRAW.mz1)/FACTOR_SCALE_MAG;

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
	dataPacket.AFO=((float)dataPacketRAW.AFO)/FACTOR_SCALE_AFO;	
	dataPacket.AFOc=((float)dataPacketRAW.AFOc)/FACTOR_SCALE_AFO;
	dataPacket.thetaM=((float)dataPacketRAW.thetaM)/FACTOR_SCALE_AFO;
	dataPacket.SV=((float)dataPacketRAW.SV)/FACTOR_SCALE_SV;
	dataPacket.SL=((float)dataPacketRAW.SL)/FACTOR_SCALE_SV;
	dataPacket.Gait_State=dataPacketRAW.Gait_State;
	dataPacket.Omega_m=((float)dataPacketRAW.Omega_m)/FACTOR_SCALE_OMEGA;
	dataPacket.Delta_theta_d=((float)dataPacketRAW.Delta_theta_d)/FACTOR_SCALE_OMEGA;
    dataPacket.AFO_omega=((float)dataPacketRAW.AFO_omega)/FACTOR_SCALE_OMEGA;
}

inline void reconstructStructWrist(structDataPacketPureDataRAWWrist dataPacketRAW, structDataPacketPureDataWrist &dataPacket)
{
	dataPacket.timestamp = dataPacketRAW.timestamp;

	dataPacket.ax = ((float)dataPacketRAW.ax) / ADXL345_LSB_G_4G;
	dataPacket.ay = ((float)dataPacketRAW.ay) / ADXL345_LSB_G_4G;
	dataPacket.az = ((float)dataPacketRAW.az) / ADXL345_LSB_G_4G;

	dataPacket.Odroid_Timestamp = dataPacketRAW.Odroid_Timestamp;
	dataPacket.Odroid_Trigger = dataPacketRAW.Odroid_Trigger;

	dataPacket.timestamp2 = dataPacketRAW.timestamp2;
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
// Debug weird pressure values
#define DEBUG_PRESSURE_VALUES(error_code) \
  for (size_t k = 0; k < 16; k+=2) { int16_t pressure_val = recvbuffer[19 + k] * 256 + recvbuffer[20 + k]; \
    if (pressure_val > 0x1100) { recvbuffer[19 + k] = 10; recvbuffer[20 + k] = error_code; }}
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

	// Quaternion
	pointer=(uint8_t*)&dataPacket.qw1;
	pointer[1]=recvbuffer[19];
	pointer[0]=recvbuffer[20];

	pointer=(uint8_t*)&dataPacket.qx1;
	pointer[1]=recvbuffer[21];
	pointer[0]=recvbuffer[22];

	pointer=(uint8_t*)&dataPacket.qy1;
	pointer[1]=recvbuffer[23];
	pointer[0]=recvbuffer[24];

	pointer=(uint8_t*)&dataPacket.qz1;
	pointer[1]=recvbuffer[25];
	pointer[0]=recvbuffer[26];

	// Gyroscope
	pointer=(uint8_t*)&dataPacket.gx1;
	pointer[1]=recvbuffer[27];
	pointer[0]=recvbuffer[28];

	pointer=(uint8_t*)&dataPacket.gy1;
	pointer[1]=recvbuffer[29];
	pointer[0]=recvbuffer[30];

	pointer=(uint8_t*)&dataPacket.gz1;
	pointer[1]=recvbuffer[31];
	pointer[0]=recvbuffer[32];

	// Accelerometer
	pointer=(uint8_t*)&dataPacket.r_ax1;
	pointer[1]=recvbuffer[33];
	pointer[0]=recvbuffer[34];

	pointer=(uint8_t*)&dataPacket.r_ay1;
	pointer[1]=recvbuffer[35];
	pointer[0]=recvbuffer[36];

	pointer=(uint8_t*)&dataPacket.r_az1;
	pointer[1]=recvbuffer[37];
	pointer[0]=recvbuffer[38];

	// Magnetometer
	pointer=(uint8_t*)&dataPacket.mx1;
	pointer[1]=recvbuffer[39];
	pointer[0]=recvbuffer[40];

	pointer=(uint8_t*)&dataPacket.my1;
	pointer[1]=recvbuffer[41];
	pointer[0]=recvbuffer[42];

	pointer=(uint8_t*)&dataPacket.mz1;
	pointer[1]=recvbuffer[43];
	pointer[0]=recvbuffer[44];


	/////////////////////////////////////////////

	pointer=(uint8_t*)&dataPacket.p1;
	pointer[1]=recvbuffer[45];
	pointer[0]=recvbuffer[46];

	pointer=(uint8_t*)&dataPacket.p2;
	pointer[1]=recvbuffer[47];
	pointer[0]=recvbuffer[48];

	pointer=(uint8_t*)&dataPacket.p3;
	pointer[1]=recvbuffer[49];
	pointer[0]=recvbuffer[50];

	pointer=(uint8_t*)&dataPacket.p4;
	pointer[1]=recvbuffer[51];
	pointer[0]=recvbuffer[52];

	pointer=(uint8_t*)&dataPacket.p5;
	pointer[1]=recvbuffer[53];
	pointer[0]=recvbuffer[54];

	pointer=(uint8_t*)&dataPacket.p6;
	pointer[1]=recvbuffer[55];
	pointer[0]=recvbuffer[56];

	pointer=(uint8_t*)&dataPacket.p7;
	pointer[1]=recvbuffer[57];
	pointer[0]=recvbuffer[58];

	pointer=(uint8_t*)&dataPacket.p8;
	pointer[1]=recvbuffer[59];
	pointer[0]=recvbuffer[60];
	/////////////////////////////////////////////


	/////////////////////////////////////////////

    //Timestamp_Odroid
	pointer=(uint8_t*)&dataPacket.Odroid_Timestamp;
	pointer[7]=recvbuffer[61];
	pointer[6]=recvbuffer[62];
	pointer[5]=recvbuffer[63];
	pointer[4]=recvbuffer[64];
	pointer[3]=recvbuffer[65];
	pointer[2]=recvbuffer[66];
	pointer[1]=recvbuffer[67];
	pointer[0]=recvbuffer[68];

	// trigger
	pointer=(uint8_t*)&dataPacket.Odroid_Trigger;
	pointer[0]=recvbuffer[69];

	// timestamp2
	pointer=(uint8_t*)&dataPacket.timestamp2;
	pointer[3]=recvbuffer[70];
	pointer[2]=recvbuffer[71];
	pointer[1]=recvbuffer[72];
	pointer[0]=recvbuffer[73];

	// AFO
	pointer=(uint8_t*)&dataPacket.AFO;
	pointer[1]=recvbuffer[74];
	pointer[0]=recvbuffer[75];
	// AFO corrected
	pointer=(uint8_t*)&dataPacket.AFOc;
	pointer[1]=recvbuffer[76];
	pointer[0]=recvbuffer[77];
	
    // theta_m
	pointer=(uint8_t*)&dataPacket.thetaM;
	pointer[1]=recvbuffer[78];
	pointer[0]=recvbuffer[79];	
  	// SV
	pointer=(uint8_t*)&dataPacket.SV;
	pointer[1]=recvbuffer[80];
	pointer[0]=recvbuffer[81];
    // SL
	pointer=(uint8_t*)&dataPacket.SL;
	pointer[1]=recvbuffer[82];
	pointer[0]=recvbuffer[83];	
			
	// Gait_State
	pointer=(uint8_t*)&dataPacket.Gait_State;
	pointer[3]=recvbuffer[84];
	pointer[2]=recvbuffer[85];
	pointer[1]=recvbuffer[86];
	pointer[0]=recvbuffer[87];
    // Omega_m
	pointer=(uint8_t*)&dataPacket.Omega_m;
	pointer[1]=recvbuffer[88];
	pointer[0]=recvbuffer[89];	

	// Delta_theta_d
	pointer=(uint8_t*)&dataPacket.Delta_theta_d;
	pointer[1]=recvbuffer[90];
	pointer[0]=recvbuffer[91];

	// AFO_omega
	pointer=(uint8_t*)&dataPacket.AFO_omega;
	pointer[1]=recvbuffer[92];
	pointer[0]=recvbuffer[93];	

	//recvbuffer[74];
	//recvbuffer[75];
	//recvbuffer[76];

}

inline void reconstructStructPureDataRAWWrist(uint8_t* recvbuffer, structDataPacketPureDataRAWWrist &dataPacket)
{
	uint8_t *pointer;

	//recvbuffer[0];
	//recvbuffer[1];
	//recvbuffer[2];

	pointer = (uint8_t*)&dataPacket.timestamp;
	pointer[3] = recvbuffer[3];
	pointer[2] = recvbuffer[4];
	pointer[1] = recvbuffer[5];
	pointer[0] = recvbuffer[6];

	pointer = (uint8_t*)&dataPacket.ax;
	pointer[1] = recvbuffer[7];
	pointer[0] = recvbuffer[8];

	pointer = (uint8_t*)&dataPacket.ay;
	pointer[1] = recvbuffer[9];
	pointer[0] = recvbuffer[10];

	pointer = (uint8_t*)&dataPacket.az;
	pointer[1] = recvbuffer[11];
	pointer[0] = recvbuffer[12];


	/////////////////////////////////////////////

	//Timestamp_Odroid
	pointer = (uint8_t*)&dataPacket.Odroid_Timestamp;
	pointer[7] = recvbuffer[13];
	pointer[6] = recvbuffer[14];
	pointer[5] = recvbuffer[15];
	pointer[4] = recvbuffer[16];
	pointer[3] = recvbuffer[17];
	pointer[2] = recvbuffer[18];
	pointer[1] = recvbuffer[19];
	pointer[0] = recvbuffer[20];

	// trigger
	pointer = (uint8_t*)&dataPacket.Odroid_Trigger;
	pointer[0] = recvbuffer[21];

	// timestamp2
	pointer=(uint8_t*)&dataPacket.timestamp2;
	pointer[3] = recvbuffer[22];
	pointer[2] = recvbuffer[23];
	pointer[1] = recvbuffer[24];
	pointer[0] = recvbuffer[25];

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
	bool ready = false;
};

struct structWrist
{
	string ipAddress;
	string name;
	int port;
	uint8_t lastPacket[PACKET_LENGTH_WRIST];
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
	return (buffer[0]==0x07 && buffer[1]==0x08 && buffer[2]==0x09 && buffer[PACKET_LENGTH_WIFI-3]==0xA && buffer[PACKET_LENGTH_WIFI-2]==0xB && buffer[PACKET_LENGTH_WIFI-1]==0xC && ret==PACKET_LENGTH_WIFI);
};

inline bool checkPacketWrist(uint8_t *buffer, int ret)
{
	//printf("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],buffer[8],buffer[9],buffer[10],buffer[11],buffer[12],buffer[13],buffer[14],buffer[15],buffer[16],buffer[17],buffer[18],buffer[19],buffer[20],buffer[21],buffer[22],buffer[23],buffer[24]);
	return (buffer[0] == 0x07 && buffer[1] == 0x08 && buffer[2] == 0x09 && buffer[26] == 0xA && buffer[27] == 0xB && buffer[28] == 0xC && ret == PACKET_LENGTH_WRIST);
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
	std::unique_lock<std::mutex> lk(dataMutex);
	
	ROS_INFO("Hello from Thread! [%s ip: %s - port: %d, 0x%04x]",SyncBoard->name.c_str(),SyncBoard->ipAddress.c_str(),SyncBoard->port, SyncBoard->port);

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
	
	lk.unlock();
	usleep(250);
	
	// No blocking mode
	//int opts=fcntl(sockfdServer, F_GETFL, 0);
	//fcntl(sockfdServer, F_SETFL, opts | O_NONBLOCK); 
		
	while(is_running)
	{		
		ret=recvfrom(sockfdServer,recvBuffer,PACKET_LENGTH_SYNC,MSG_DONTWAIT,(struct sockaddr *)&addrClient,&len);
		
		if (ret>1)
		{
			std::lock_guard<std::mutex> lk(dataMutex);
			
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
			
		}
		else
			this_thread::sleep_for(chrono::milliseconds(1));
	}

}


void threadUDPreceive(structPDShoe* PDShoe)
{
	std::unique_lock<std::mutex> lk(dataMutex);
	
	ROS_INFO("Hello from Thread! [%s ip: %s - port: %d, 0x%04x]",PDShoe->name.c_str(),PDShoe->ipAddress.c_str(),PDShoe->port, PDShoe->port);
	
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
	lk.unlock();
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
			
			std::unique_lock<std::mutex> lk(dataMutex);
			cond_var.wait_for(lk, std::chrono::milliseconds(500), [&]{ return !PDShoe->ready; });
			if (PDShoe->ready) continue;

			PDShoe->packetReceived++;
			if (checkPacket(recvBuffer,ret))
			{
				memcpy(PDShoe->lastPacket,recvBuffer,PACKET_LENGTH_WIFI);
				PDShoe->ready = true;
			}
			else
			{
				PDShoe->packetError++;	
				//printf("Ret=%d\n", ret);
			}
			
			lk.unlock();
			cond_var.notify_all();
			
		}
		else
			this_thread::sleep_for(chrono::milliseconds(1));
		cycles++;
	}
}

void threadUDPreceiveWrist(structWrist* Wrist)
{
	dataMutex.lock();

	printf("Hello from Thread! [%s ip: %s - port: %d]\n", Wrist->name.c_str(), Wrist->ipAddress.c_str(), Wrist->port);

	uint8_t id;
	unsigned long int cycles = 0;
	int sockfdServer;
	socklen_t len;
	int ret;
	uint8_t recvBuffer[BUFFER];

	struct sockaddr_in addrServer;
	struct sockaddr_in addrClient;

	sockfdServer = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	//printf("sockfdServer=%d\n",sockfdServer);

	bzero(&addrServer, sizeof(addrServer));
	addrServer.sin_family = AF_INET;
	addrServer.sin_addr.s_addr = htonl(INADDR_ANY);//IP_ADDRESS(127, 0, 0, 1);
	addrServer.sin_port = htons(Wrist->port);

	bind(sockfdServer, (struct sockaddr *)&addrServer, sizeof(addrServer));

	len = (socklen_t)sizeof(addrClient);

	id = Wrist->id;
	dataMutex.unlock();
	usleep(250);

	// No blocking mode
	//int opts=fcntl(sockfdServer, F_GETFL, 0);
	//fcntl(sockfdServer, F_SETFL, opts | O_NONBLOCK); 

	while (1)
	{
		ret = recvfrom(sockfdServer, recvBuffer, PACKET_LENGTH_WRIST, 0, (struct sockaddr *)&addrClient, &len);

		if (ret>1)
		{
			//printf("Data!\n");

			dataMutex.lock();

			Wrist->packetReceived++;
			if (checkPacketWrist(recvBuffer, ret))
			{
				memcpy(Wrist->lastPacket, recvBuffer, PACKET_LENGTH_WRIST);
			}
			else
			{
				Wrist->packetError++;
				//printf("Ret=%d\n", ret);
			}

			dataMutex.unlock();

		}
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
struct structRL_FuzzyLogic
{
	uint8_t  Ns; // number of strides to calulate average velocity 
    bool     Ns_init;
	// AFO_omega
	float    AFO_omega;
	// Assuming Stride Time=0.1s; The number of strides for 4 minutes is = 10*60*4=2400
	float    Vh[3000];
	float    t[3000];
	uint16_t start_idx_Vb;
	uint16_t end_idx_Vb;
	uint16_t start_idx_Vg;
	uint16_t end_idx_Vg;
	uint16_t iter;
	uint8_t  flag; // 1 -> Vb; 2 -> LS; 3 -> RL 
	float    t1;  // flag = 1
	float    t2;  // flag = 2
	float    t3;  // flag = 3 
	float    Vb;  // baseline speed
    float    Vg;  // guided speed
	float    Vt;  // target speed
	float    coef_Vg;
	float    coef_Vt;
	uint8_t  lc;  // the number of hidden layer
	float    _gamma;
	uint8_t  lsMode;
	MatrixXf P0;
	VectorXf W_hat0;
	bool     rlsFlag; // false -> not invoke; true -> invoke
	float    rls_a; // forget vector if a==1, it is common RLS
	float    Vh_new;
	float    Vh_prev;
	MatrixXf P_new;
	VectorXf W_new;	
	VectorXf phi_new;	
	float    rwd_new;
	float    Q; // quality function
	float    e_c; // current error
	uint8_t  fl_mode; // default 0, Table 1 (e_c,Q);  1, Table 2 (e_c,gredient_Q)
	//MatrixXf fuzzy_table1;
	MatrixXf fuzzy_table2;
	float    fl_lambda;
	float    d_Q;
	float    d_Ea; 
	float    delta_Vg;
	float    Vg_prev;
	// parameters for epsilon decay
	float    eps;
	float    eps_min;
	float    l; // decay rate
	uint16_t k; // steps
	//float    velocityError[3000]; // Assuming 1 stride takes 1 sec, 60/Ns = 10 /min
	float    velocityErrorThreshold;
	float    maximumVelocity; //m/s, transition speed
	uint8_t  maximumVelocityCount;
	float    t_protocol2[3];
	float    veocityIncrement;
	// Initial
	structRL_FuzzyLogic(): start_idx_Vb(0),end_idx_Vb(0),start_idx_Vg(0),end_idx_Vg(0),iter(0),flag(0)
	{
		Ns        =6;//10;
		Ns_init   =0;
		AFO_omega =3.142;
		Vh[0]     =0;
		t[0]      =0;
		t1        =20;
		t2        =40;
		t3        =60;
		coef_Vg   =1.0;//1.15;
		coef_Vt   =1.5;
		lc        =4;
		_gamma    =0.7;//0.8;//0.5;
		lsMode    =1; // constant guided speed
		rls_a     =0.9;
		Vh_new    =0;
		Vh_prev   =0;
		rlsFlag   =false;
		rwd_new   =0;
		Q         =0;
		e_c       =0;
		fl_mode   =0;
		fl_lambda =0.8;//0.5;//0.00005;//0.0008;//0.00003;//0.2;
		d_Q       =0;
		d_Ea      =0;
		delta_Vg  =0;
		P0        =MatrixXf::Zero(lc,lc);
	    W_hat0    =MatrixXf::Zero(lc,1);
	  	P_new     =MatrixXf::Zero(lc,lc);	
		W_new     =MatrixXf::Zero(lc,1);	
		phi_new   =MatrixXf::Zero(lc,1);		
		//fuzzy_table1=MatrixXf::Zero(5,5);
		fuzzy_table2=MatrixXf::Zero(11,11);
		Vg        =0;
		Vg_prev   =0;
		// parameters for epsilon decay
		eps       =1.0;
		eps_min   =0.1;
		l         =0.1;
		k         =1;
		velocityErrorThreshold=5.0; // cm/s
		veocityIncrement=10.0; //cm/s
		maximumVelocity =2.0;
		maximumVelocityCount =1;	
		t_protocol2[0]=100;
		t_protocol2[1]=280;//340;//220;
		t_protocol2[2]=460;//580;//340;			
	}
}RL_FuzzyLogic;




		
void RL_FuzzylogicInitial(structRL_FuzzyLogic & RL_FuzzyLogic, structDataPacketPureData & dataPacketL, structDataPacketPureData & dataPacketR)
{
	//float V_average = (dataPacketL.SV + dataPacketR.SV)/2;
	float V_average = dataPacketR.SV; // only take the right foot 
	float t_foot = dataPacketR.timestamp/1e6;
	if (t_foot <= RL_FuzzyLogic.t3)
	{
	    // update Vh and t
		if (V_average!=RL_FuzzyLogic.Vh[RL_FuzzyLogic.iter])
		{
			RL_FuzzyLogic.iter=RL_FuzzyLogic.iter+1;
			RL_FuzzyLogic.Vh[RL_FuzzyLogic.iter]=V_average;
			RL_FuzzyLogic.t[RL_FuzzyLogic.iter]=t_foot; // Take Right shoe as reference
		}
		// update start_idx_Vb
		if (RL_FuzzyLogic.flag == 0 && RL_FuzzyLogic.t[RL_FuzzyLogic.iter] > RL_FuzzyLogic.t1)
		{
			RL_FuzzyLogic.flag = 1; 
			RL_FuzzyLogic.start_idx_Vb = RL_FuzzyLogic.iter;
		}
		// update end_idx_Vb and start_idx_Vg
		if (RL_FuzzyLogic.flag == 1 && RL_FuzzyLogic.t[RL_FuzzyLogic.iter] > RL_FuzzyLogic.t2)
		{
			RL_FuzzyLogic.flag = 2; 
			RL_FuzzyLogic.end_idx_Vb = RL_FuzzyLogic.iter-1;
			RL_FuzzyLogic.start_idx_Vg = RL_FuzzyLogic.iter;			
			//// compute baseline speed Vb
			//float V_baseline = 0;
			//for (uint16_t i = RL_FuzzyLogic.start_idx_Vb; i<=RL_FuzzyLogic.end_idx_Vb; i=i+1)
			//{
			//	V_baseline = V_baseline + RL_FuzzyLogic.Vh[i];
			//}
			//RL_FuzzyLogic.Vb = V_baseline / (RL_FuzzyLogic.end_idx_Vb-RL_FuzzyLogic.start_idx_Vb+1);
			RL_FuzzyLogic.Vg = RL_FuzzyLogic.Vb*RL_FuzzyLogic.coef_Vg;
			//RL_FuzzyLogic.Vt = RL_FuzzyLogic.Vb*RL_FuzzyLogic.coef_Vt;
			
			RL_FuzzyLogic.Vg_prev = RL_FuzzyLogic.Vg;
		}		
	}
	else
	{
		if (RL_FuzzyLogic.flag == 2 && V_average!=RL_FuzzyLogic.Vh[RL_FuzzyLogic.iter])
		{
			RL_FuzzyLogic.flag = 3;
			RL_FuzzyLogic.end_idx_Vg = RL_FuzzyLogic.iter;
			// calculate Vh_new (average)
			for (uint16_t h = RL_FuzzyLogic.end_idx_Vg-RL_FuzzyLogic.Ns+1; h<=RL_FuzzyLogic.end_idx_Vg; h++)
			{
				RL_FuzzyLogic.Vh_new=RL_FuzzyLogic.Vh_new+RL_FuzzyLogic.Vh[h];
			}
			RL_FuzzyLogic.Vh_new=RL_FuzzyLogic.Vh_new/RL_FuzzyLogic.Ns;
			RL_FuzzyLogic.iter=0;
			RL_FuzzyLogic.Vh[RL_FuzzyLogic.iter]=V_average;						
		}
				
		if (V_average!=RL_FuzzyLogic.Vh[RL_FuzzyLogic.iter])
		{
			if(RL_FuzzyLogic.Ns == 1)
			{
				RL_FuzzyLogic.iter=0;
				RL_FuzzyLogic.Vh[RL_FuzzyLogic.iter]=V_average;	
				RL_FuzzyLogic.Vh_prev=RL_FuzzyLogic.Vh_new;
				RL_FuzzyLogic.Vh_new=V_average;
				RL_FuzzyLogic.rlsFlag=true;
			}
			else
			{
				if (RL_FuzzyLogic.Ns_init==1)
				{
					RL_FuzzyLogic.iter=0;
					RL_FuzzyLogic.Vh[RL_FuzzyLogic.iter]=V_average;	
					RL_FuzzyLogic.Ns_init=0;
				}
			
				if (RL_FuzzyLogic.iter >= RL_FuzzyLogic.Ns-2)
				{
					RL_FuzzyLogic.iter=RL_FuzzyLogic.iter+1;
					RL_FuzzyLogic.Vh[RL_FuzzyLogic.iter]=V_average;
				    RL_FuzzyLogic.rlsFlag=true;
					RL_FuzzyLogic.Vh_prev=RL_FuzzyLogic.Vh_new;
					RL_FuzzyLogic.Vh_new=0;
					for (uint16_t h = 0; h<=RL_FuzzyLogic.Ns-1; h++)
					{
						RL_FuzzyLogic.Vh_new=RL_FuzzyLogic.Vh_new+RL_FuzzyLogic.Vh[h];					
					}
					RL_FuzzyLogic.Vh_new=RL_FuzzyLogic.Vh_new/RL_FuzzyLogic.Ns;
					RL_FuzzyLogic.Ns_init=1;
				}
				else
				{
					RL_FuzzyLogic.rlsFlag=false;
					RL_FuzzyLogic.iter=RL_FuzzyLogic.iter+1;
					RL_FuzzyLogic.Vh[RL_FuzzyLogic.iter]=V_average;	
				}
			}
		}
		else
		{
			RL_FuzzyLogic.rlsFlag=false;
		}
		
	}
}
void PIController(structRL_FuzzyLogic & RL_FuzzyLogic)
{
	if (RL_FuzzyLogic.rlsFlag == true)
	{
		float rwdItem1 = (RL_FuzzyLogic.Vh_new-RL_FuzzyLogic.Vg)*100.0; // cm/s
		RL_FuzzyLogic.e_c = rwdItem1;
		// mode 4:
		if (RL_FuzzyLogic.e_c > RL_FuzzyLogic.veocityIncrement)
		{
			RL_FuzzyLogic.delta_Vg = RL_FuzzyLogic.veocityIncrement/100; // m/s
		}
		else
		{
			if(RL_FuzzyLogic.e_c > RL_FuzzyLogic.velocityErrorThreshold)
			{
				RL_FuzzyLogic.delta_Vg = 0; // m/s
			}
			else
			{
				if (RL_FuzzyLogic.e_c >= - RL_FuzzyLogic.velocityErrorThreshold)
				{
					RL_FuzzyLogic.delta_Vg = RL_FuzzyLogic.veocityIncrement/100; // m/s
				}
				else
				{
					if(RL_FuzzyLogic.e_c >= - RL_FuzzyLogic.veocityIncrement)
					{
						RL_FuzzyLogic.delta_Vg = 0; // m/s
					}
					else
					{
						RL_FuzzyLogic.delta_Vg = - RL_FuzzyLogic.veocityIncrement/100; // m/s
					}
				}
			}
		}
		RL_FuzzyLogic.Vg       = RL_FuzzyLogic.Vg + RL_FuzzyLogic.delta_Vg;	
		
		// Note: 1) Vt1 = Vb + 0.5 (Vmax - Vb)
		//       2) Vt2 = Vb + 1.0 (Vmax - Vb)
		//          Vt2 - Vt1  must > RL_FuzzyLogic.veocityIncrement
		if (RL_FuzzyLogic.Vg > RL_FuzzyLogic.Vt)
		{
			if (RL_FuzzyLogic.Vg - RL_FuzzyLogic.Vt <= RL_FuzzyLogic.veocityIncrement/100)
			{
				RL_FuzzyLogic.Vg = RL_FuzzyLogic.Vt;
			}
			else // i.e, from high Vt to low Vt (e.g., 100% -> 50%)
			{
				if (RL_FuzzyLogic.e_c > RL_FuzzyLogic.veocityIncrement)
				{
					RL_FuzzyLogic.Vg=RL_FuzzyLogic.Vg - RL_FuzzyLogic.veocityIncrement/100; // i.e., delta_Vg = 0;
				}
				else
				{
					if (RL_FuzzyLogic.e_c > RL_FuzzyLogic.velocityErrorThreshold)
					{
						// delta_Vg = 0;
					}
					else
					{
						if (RL_FuzzyLogic.e_c >= - RL_FuzzyLogic.velocityErrorThreshold)
						{
							RL_FuzzyLogic.Vg=RL_FuzzyLogic.Vg - RL_FuzzyLogic.veocityIncrement/100 - RL_FuzzyLogic.veocityIncrement/100; // delta_Vg = -10 cm/s;
						}
						else
						{
							// delta_Vg = 0;
						}
					}
				}
			}
		}
				
		RL_FuzzyLogic.rlsFlag  = false;
		RL_FuzzyLogic.k        = RL_FuzzyLogic.k + 1;
	}
}

void updateVt(structRL_FuzzyLogic & RL_FuzzyLogic, float maxV, float dV, float t_foot, bool timeVaryingVt)
{
	if (t_foot <= RL_FuzzyLogic.t_protocol2[0])
	{
		RL_FuzzyLogic.Vt= RL_FuzzyLogic.Vb;
	}
	else
	{
		if (!timeVaryingVt)
		{
			// mode 2
			if (t_foot <= RL_FuzzyLogic.t_protocol2[1])
			{
			    RL_FuzzyLogic.Vt= RL_FuzzyLogic.Vb + 0.5 * dV;
			}
			else
			{
				if (t_foot <= RL_FuzzyLogic.t_protocol2[2])
				{
					RL_FuzzyLogic.Vt= RL_FuzzyLogic.Vb + 1.0 * dV;
				}
				else
				{
					RL_FuzzyLogic.Vt= RL_FuzzyLogic.Vb + 0.5 * dV;
				}
			}		
		}
		else
		{
			RL_FuzzyLogic.Vt= 0.5*(RL_FuzzyLogic.Vb+maxV)+0.5*dV*sin(1.5*pi+(t_foot-RL_FuzzyLogic.t_protocol2[0])/240*2*pi);
		}
	}
}
void createGuidedSpeedPacket(uint8_t* buffer_out,float Vg)
{

	uint8_t *pointer;
	uint16_t val;
	buffer_out[0]=0x01; buffer_out[1]=0x02; buffer_out[2]=0x03;
	
	buffer_out[3]=21; // send Vg
		
	val = uint16_t(Vg * 1000.0f);
    pointer = (uint8_t*)&val; 
    buffer_out[4] = pointer[0];
    buffer_out[5] = pointer[1];	
	buffer_out[6]=0;
	buffer_out[7]=0;
	buffer_out[8]=0;
	buffer_out[9]=0;
	buffer_out[10]=0;
	buffer_out[11]=0;
	
	buffer_out[12]=0;

	buffer_out[13]=0x04; buffer_out[14]=0x05; buffer_out[15]=0x06;
}
void createLogPacket_2nodes(uint8_t* buffer_out,uint8_t* buffer_01,uint8_t* buffer_02,uint8_t Odroid_Trigger,uint64_t currenttime,uint8_t Ext_Trigger, structRL_FuzzyLogic & RL_FuzzyLogic)
{
	uint8_t *pointer;
	//See PureData Packet {XLS File}
	buffer_out[0]=0x01;
	buffer_out[1]=0x02;
	buffer_out[2]=0x03;

	for(int i=0;i<91;i++)
	{
		buffer_out[i+3]=buffer_01[i+3];
		buffer_out[i+94]=buffer_02[i+3];
	}

	//current time
	pointer=(uint8_t*)&currenttime;
	buffer_out[185]=pointer[7];
	buffer_out[186]=pointer[6];
	buffer_out[187]=pointer[5];
	buffer_out[188]=pointer[4];
	buffer_out[189]=pointer[3];
	buffer_out[190]=pointer[2];
	buffer_out[191]=pointer[1];
	buffer_out[192]=pointer[0];

    buffer_out[193]=Odroid_Trigger;
	
	buffer_out[194]=Ext_Trigger;

	uint16_t val1;
	int16_t  val2;
	// delta_Vg 
	val2 = int16_t(RL_FuzzyLogic.delta_Vg * 10000.0f);
    pointer = (uint8_t*)&val2; 
    buffer_out[195] = pointer[1];
    buffer_out[196] = pointer[0];  	

	// Q 
	val2 = int16_t(RL_FuzzyLogic.Q); // * 10000.0f
    pointer = (uint8_t*)&val2; 
    buffer_out[197] = pointer[1];
    buffer_out[198] = pointer[0];	

    // rwd
	val1 = uint16_t(RL_FuzzyLogic.rwd_new); // * 10000.0f
    pointer = (uint8_t*)&val1; 
    buffer_out[199] = pointer[1];
    buffer_out[200] = pointer[0];		
	
	// Vg
	val1 = uint16_t(RL_FuzzyLogic.Vg * 1000.0f);
    pointer = (uint8_t*)&val1; 
    buffer_out[201] = pointer[1];
    buffer_out[202] = pointer[0];
	    // Vh_prev
	val1 = uint16_t(RL_FuzzyLogic.Vh_prev * 1000.0f);
    pointer = (uint8_t*)&val1; 
    buffer_out[203] = pointer[1];
    buffer_out[204] = pointer[0];			
	
	// e_c
	val2 = int16_t(RL_FuzzyLogic.e_c * 100.0f); // 10000
    pointer = (uint8_t*)&val2; 
    buffer_out[205] = pointer[1];
    buffer_out[206] = pointer[0];	
	
	// d_Ea
	val2 = int16_t(RL_FuzzyLogic.d_Ea * 100.0f); // 10000
    pointer = (uint8_t*)&val2; 
    buffer_out[207] = pointer[1];
    buffer_out[208] = pointer[0];	
	
	// Vh_new
	val1 = uint16_t(RL_FuzzyLogic.Vh_new * 1000.0f);
    pointer = (uint8_t*)&val1; 
    buffer_out[209] = pointer[1];
    buffer_out[210] = pointer[0];	

	buffer_out[211]=0x4;
	buffer_out[212]=0x5;
	buffer_out[213]=0x6;
}


int main(int argc, char* argv[])
{
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
	ros::Publisher pub_sport_sole = n.advertise<sport_sole::SportSole> ("sport_sole", 1000) ;

	//publisher created here for visualizing shoe's acceleration data and orientation
	ros::Publisher pub_markers = n.advertise<visualization_msgs::MarkerArray> ("sport_sole_markers", 1);

	// Transform broadcaster
	tf2_ros::TransformBroadcaster tf_broadcaster;

	printf("\nSportSole v1.9\n\n");
	printf("2 Nodes: LSole, RSole\n\n");
	printf("Include RAW Data in Polling Mode\n");
	//printf("TODO LIST: - modify htonl(INADDR_ANY) -> ex: IP_ADDRESS(127, 0, 0, 1)\n");
	//printf("WARNING: US_CYCLES 1000\n");
		
	// automatic feedback modes
	FILE * TemporalFile;
	char strDate[N_STR];
	char strFile[N_STR];
	char strSession[N_STR];
	char strTemporalFile[N_STR];
	char strFilepath[N_STR];

	// Two CLI parameters are required: session_name, filepath.
	assert(argc >= 3); 
	bool send_enable_packet = false;
	bool require_both_shoes = true;
	for (int i = 3; i < argc; ++i) {
		if (strcmp(argv[i], "-s")==0)
			send_enable_packet = true;
		if (strcmp(argv[i], "-e")==0)
			require_both_shoes = false;
	}

	{
		// Simple functioning
		sprintf(strSession, "%s", argv[1]);
		sprintf(strFilepath, "%s", argv[2]);
		//printf("Session Name: %s\n", strSession);
	}

	
	bool resetLED = 0x00;
	bool condResetLed;
	bool resettingState = false;
	uint32_t lastResetTimestamp;

	uint8_t bufferLog[PACKET_LENGTH_LOG];
	uint8_t bufferPd[PACKET_LENGTH_PD];
	uint8_t bufferLed[PACKET_LENGTH_LED];
	uint8_t bufferSync[PACKET_LENGTH_SYNC];
	uint8_t bufferTime[PACKET_LENGTH_TIME];
	uint8_t bufferGuidedSpeed[PACKET_LENGTH_TIME];
	uint8_t bufferAFOParameter[PACKET_LENGTH_TIME];
	
	thread threadPDShoeL;
	thread threadPDShoeR;
	//thread threadWristR;
	thread threadSync;
	
	structPDShoe PDShoeL;
	structPDShoe PDShoeR;
	//structWrist WristR;
	structSWstat swStat;
	structSync Sync;

	structDataPacketPureDataRAW dataPacketRawL;
	structDataPacketPureDataRAW dataPacketRawR;
	//structDataPacketPureDataRAWWrist dataPacketRawWristR;
	structDataPacketPureData dataPacketL;
	structDataPacketPureData dataPacketR;
	//structDataPacketPureDataWrist dataPacketWristR;
	structSyncPacket SyncPacket;

	GaitPhaseFSM<structDataPacketPureDataRAW> gait_phase_fsms[LEFT_RIGHT];
	
	swStat.packetLedSent=0;
	swStat.packetGuiSent=0;
	swStat.packetPdSent=0;
	swStat.packetErrorPdShoeL=0;
	swStat.packetErrorPdShoeR=0;
	//swStat.packetErrorWristR = 0;
	swStat.packetErrorSync=0;
	swStat.packetBroadSent=0;
	

	struct timeval tv;
	uint64_t timestamp;
	uint64_t timestamp_last_pulse = 0;
	uint64_t timestamp_start;
	uint32_t tCycle;
	uint64_t cycleMicrosTime=US_CYCLES;
	uint32_t cycles=0;


	// Gravitational acceleration in Hoboken.
	const double GRAVITATIONAL_ACCELERATION = 9.81772;
	
	ros::Time ros_stamp_base;
	ros::Duration transmission_delay(0.002);
#if 0
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
#else

	ros::Duration owt_window_size(10.0);
	Owt<ros::Duration> owt_l(owt_window_size), owt_r(owt_window_size);

	auto getRosTimestampL = [&]()->ros::Time{
		ros::Duration ros_time_now_since_epoch(ros::Time::now().toSec());
		auto ts_ros = owt_l(ros::Duration(dataPacketL.timestamp * 1e-6), ros_time_now_since_epoch);
		std_msgs::Float32 msg; 
		msg.data = (ts_ros - ros_time_now_since_epoch).toSec();
		pub_l_to_ros.publish(msg);
		return ros::Time(ts_ros.toSec());
	};

	auto getRosTimestampR = [&]()->ros::Time{
		ros::Duration ros_time_now_since_epoch(ros::Time::now().toSec());
		auto ts_ros = owt_r(ros::Duration(dataPacketR.timestamp * 1e-6), ros_time_now_since_epoch);
		std_msgs::Float32 msg; 
		msg.data = (ts_ros - ros_time_now_since_epoch).toSec();
		pub_r_to_ros.publish(msg);
		return ros::Time(ts_ros.toSec());
	};
#endif

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

	// WristR.name = "Wrist [Right]";
	// WristR.ipAddress = "192.168.1.16";
	// WristR.port = 3466;
	// //WristR.frequencyError=0;
	// WristR.packetError = 0;
	// WristR.packetReceived = 0;
	// WristR.id = 5;
	
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
	//threadWristR=thread(threadUDPreceiveWrist, &WristR);
	//threadResetLED=thread(threadCheckReset,&resetLED,&resetPressure,&setPressure);
	threadSync=thread(threadSYNCreceive,&Sync);
	//threadPressureReadVal=thread(threadCheckPressureReadVal,&pressureVal);
		
	//threadPDShoeL.detach();
	//threadPDShoeR.detach();
	//threadWristR.detach();
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
	sprintf(strFile,"%s/%s_%s.dat", strFilepath, strDate,strSession);
	FILE * pFile;
	pFile = fopen (strFile, "wb");

		
#define CYCLES_WRITE 500
	
	uint8_t vFileBuffer[CYCLES_WRITE*PACKET_LENGTH_LOG];
	unsigned int idxFileWrite=0;
	
	uint8_t Odroid_Trigger=1;
	uint8_t sendSportSole=2;
	
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

	char *pch;
    char *rlVbTrigger[N_STR]; 
	
	pch = strtok(strSession,"_");
	uint8_t RL_Vb=0;
	
	while(pch != NULL)
	{
		rlVbTrigger[RL_Vb]=pch;
		pch=strtok(NULL,"_");
		RL_Vb++;
	}

    bool RL_flag;
	bool protocol2=false;
	bool constantTraining=false;
	bool timeVaryingVt=false;
	float maxV;
	float dV; // dV = maxV - Vb;
    RL_FuzzyLogic.Vb = atof(rlVbTrigger[2]);
	RL_FuzzyLogic.Vg = RL_FuzzyLogic.Vb;
	RL_FuzzyLogic.Vt = 2.5;	

	if (atof(rlVbTrigger[1])==0)
	{
		RL_flag=false;
		printf("RL + FuzzyLogic is off! \n");		
	}
	if (atof(rlVbTrigger[1])==4)
	{
		RL_flag=false;
		printf("RL + FuzzyLogic is off! \n");
		printf("Constant training speed! \n");
		protocol2=true;
		constantTraining=true;
		maxV=atof(rlVbTrigger[3]);
		dV  =maxV-RL_FuzzyLogic.Vb;
		printf("The maximum speed is %5.2f\n",maxV);
		RL_FuzzyLogic.Vt = RL_FuzzyLogic.Vb;
	}
	
	printf("The baseline speed is %5.2f\n",RL_FuzzyLogic.Vb);
	printf("Waiting...\n");
	//RAND_MAX is (2^31-1)=2147483647
	


	if (send_enable_packet)
	{
		// Set to baseline mode
		currenttime = getMicrosTimeStamp()-timestamp_start;
		createTimePacket(bufferTime,currenttime,Odroid_Trigger);
		bufferTime[3] = 3;
		sendto(sockfdBroad,bufferTime,PACKET_LENGTH_TIME,0,(struct sockaddr *)&addrBroad,sizeof(addrBroad));
		ROS_INFO("Enable baseline test sent");
		this_thread::sleep_for(chrono::milliseconds(200));
	

		// Send an enable packet
		currenttime = getMicrosTimeStamp()-timestamp_start;
		createTimePacket(bufferTime,currenttime,Odroid_Trigger);
		bufferTime[3] = 1;
		sendto(sockfdBroad,bufferTime,PACKET_LENGTH_TIME,0,(struct sockaddr *)&addrBroad,sizeof(addrBroad));
		ROS_INFO("Enable packet sent");
	}

	ROS_INFO("Waiting...");
	//RAND_MAX is (2^31-1)=2147483647
	
	bool cond=false;
	while(!cond && ros::ok() && !ros::isShuttingDown())
	{
		{
			std::lock_guard<std::mutex> lk(dataMutex);
			// Require both shoes
			if (require_both_shoes)
				cond = (PDShoeL.packetReceived>0) && (PDShoeR.packetReceived>0);
			else
				cond = (PDShoeL.packetReceived>0) || (PDShoeR.packetReceived>0);
			if (cond)
				ros_stamp_base = ros::Time::now() - transmission_delay;
		}
		
		usleep(500);
	}
	
	ROS_INFO("Start!");
	timestamp_start=getMicrosTimeStamp();
		
	while(ros::ok() && !ros::isShuttingDown())
	{
        // Critical section!
		{
			std::unique_lock<std::mutex> lk(dataMutex);
			cond_var.wait_for(lk, std::chrono::milliseconds(500), [&]{ return PDShoeL.ready || PDShoeR.ready; });
			if (!PDShoeL.ready && !PDShoeR.ready) continue;
			
			timestamp=getMicrosTimeStamp();
			reconstructStructPureDataRAW(PDShoeL.lastPacket,dataPacketRawL);
			reconstructStructPureDataRAW(PDShoeR.lastPacket,dataPacketRawR);
			reconstructStructSyncPacket(Sync.lastPacket,SyncPacket);
			
			swStat.packetErrorPdShoeL=PDShoeL.packetError;
			swStat.packetErrorPdShoeR=PDShoeR.packetError;
			swStat.packetErrorSync=Sync.packetError;
			swStat.packetReceivedPdShoeL=PDShoeL.packetReceived;
			swStat.packetReceivedPdShoeR=PDShoeR.packetReceived;
			swStat.packetReceivedSync=Sync.packetReceived;

			PDShoeL.ready = false;
			PDShoeR.ready = false;

			lk.unlock();
			cond_var.notify_all();
		}
		
		reconstructStruct(dataPacketRawL,dataPacketL);
		reconstructStruct(dataPacketRawR,dataPacketR);

        			
		sport_sole::SportSole msg;
		
		static auto packets_sent = swStat.packetReceivedPdShoeL;
		//ROS_INFO_STREAM("Time difference: " << (getRosTimestampL() - getRosTimestampR()).nsec);
		//ROS_INFO_STREAM("Stamp: " << getRosTimestampL());
		// if (cycles % PUB_PERIOD_MS == 0)
		if (packets_sent < swStat.packetReceivedPdShoeL)
		{
			int new_packets_received = (int)swStat.packetReceivedPdShoeL - (int)swStat.packetErrorPdShoeL - (int)packets_sent;
			ROS_WARN_STREAM_COND(new_packets_received > 1, "Failed to forward " << new_packets_received - 1 << " packets.");
			packets_sent = swStat.packetReceivedPdShoeL; 
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
					tf::Quaternion q(-data_packet.qy1, data_packet.qx1, data_packet.qz1, data_packet.qw1);
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
			msg.header.frame_id = "odom";
			msg.raw_acceleration[0].linear.x = -dataPacketL.r_ay1 * GRAVITATIONAL_ACCELERATION;
			msg.raw_acceleration[0].linear.y = dataPacketL.r_ax1 * GRAVITATIONAL_ACCELERATION;
			msg.raw_acceleration[0].linear.z = dataPacketL.r_az1 * GRAVITATIONAL_ACCELERATION;
			msg.angular_velocity[0].x = -dataPacketL.gy1;
			msg.angular_velocity[0].y = dataPacketL.gx1;
			msg.angular_velocity[0].z = dataPacketL.gz1;
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


			msg.raw_acceleration[1].linear.x = -dataPacketR.r_ay1 * GRAVITATIONAL_ACCELERATION;
			msg.raw_acceleration[1].linear.y = dataPacketR.r_ax1 * GRAVITATIONAL_ACCELERATION;
			msg.raw_acceleration[1].linear.z = dataPacketR.r_az1 * GRAVITATIONAL_ACCELERATION;
			msg.angular_velocity[1].x = -dataPacketR.gy1;
			msg.angular_velocity[1].y = dataPacketR.gx1;
			msg.angular_velocity[1].z = dataPacketR.gz1;
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

		// reconstructStructWrist(dataPacketRawWristR, dataPacketWristR);
		RL_FuzzylogicInitial(RL_FuzzyLogic,dataPacketL,dataPacketR);		
		if (protocol2==true)
		{
            RL_FuzzyLogic.Vt= RL_FuzzyLogic.Vb;
		}
		if (RL_flag==false)
		{
			PIController(RL_FuzzyLogic);
			if (constantTraining==true)
			{
				RL_FuzzyLogic.Vg = RL_FuzzyLogic.Vt;
			}
		}
		if ((cycles%100)==0)
		{
			createGuidedSpeedPacket(bufferGuidedSpeed,RL_FuzzyLogic.Vg);
			sendto(sockfdBroad,bufferGuidedSpeed,PACKET_LENGTH_TIME,0,(struct sockaddr *)&addrBroad,sizeof(addrBroad));
		}
		if ((cycles%70)==0)
		{
			sendto(sockfdGui,bufferLog,sizeof(bufferLog),0,(struct sockaddr *)&addrGui,sizeof(addrGui));
			swStat.packetGuiSent++;
		}

		auto time_elapsed_since_last_pulse = timestamp - timestamp_last_pulse;
		bool next_pulse = time_elapsed_since_last_pulse >= 1e6;
		
		if (next_pulse)
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
		
		
		createLogPacket_2nodes(bufferLog, PDShoeL.lastPacket, PDShoeR.lastPacket, Odroid_Trigger, currenttime, SyncPacket.Ext_Trigger,RL_FuzzyLogic);
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
		// 	//writeGPIO(ledTrigger, GPIOzero, GPIOone);
		}
		
		if(next_pulse)
		{
			if (time_elapsed_since_last_pulse > 1.5e6) 
				timestamp_last_pulse = timestamp;
			else
				timestamp_last_pulse += 1e6;
			float currenttime_float_sec = ((float)(currenttime))/1000000.0f;
			// printf("Cycles=%d - Err(L)=%d - Err(R)=%d - P(L)=%d - P(R)=%d - Tr=%d - ESync=%d\n", cycles, swStat.packetErrorPdShoeL, swStat.packetErrorPdShoeR, swStat.packetReceivedPdShoeL, swStat.packetReceivedPdShoeR, Odroid_Trigger, SyncPacket.Ext_Trigger);
			
			unsigned int packets_received[2] = {swStat.packetReceivedPdShoeL, swStat.packetReceivedPdShoeR};
			static unsigned int packets_received_last[2];
			float freqs[2];
			for (int lr: {0, 1}){
				freqs[lr] = (packets_received[lr] - packets_received_last[lr]) / (1e-6 * time_elapsed_since_last_pulse);
				packets_received_last[lr] = packets_received[lr];
			}

			
			printf("[%6d, %5.1f s, [%5.1f,%5.1f] Hz, ", cycles, currenttime_float_sec, freqs[0], freqs[1]);
			printf("err=[%2d,%2d,%2d] p=[%5d,%5d,%5d] ",
				swStat.packetErrorPdShoeL, swStat.packetErrorPdShoeR, swStat.packetErrorSync,
				swStat.packetReceivedPdShoeL, swStat.packetReceivedPdShoeR, swStat.packetReceivedSync);
			printf("v(R)=%5.2f, Vg=%5.2f, Vt=%5.2f, Vh_new=%5.2f, AFOc1=%5.2f, AFOc2=%5.2f, u=%5.2f, ", 
				dataPacketR.SV,
				RL_FuzzyLogic.Vg,
				RL_FuzzyLogic.Vt,
				RL_FuzzyLogic.Vh_new,
				dataPacketR.AFOc,
				dataPacketR.AFO_omega,
				dataPacketR.Delta_theta_d);
			printf("ESync=%d\n", SyncPacket.Ext_Trigger);

	    }
		
			
		cycles++;
		
		tCycle=getMicrosTimeStamp()-timestamp;
		
		//printf("t cycle=%d\n",(uint16_t)tCycle);
		
#define US_SLEEP_CORRECTION 0 //48
		// if(!(tCycle>cycleMicrosTime-US_SLEEP_CORRECTION)) usleep(cycleMicrosTime-US_SLEEP_CORRECTION-tCycle);
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

