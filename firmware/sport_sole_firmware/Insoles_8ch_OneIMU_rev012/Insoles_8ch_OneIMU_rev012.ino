// This code is based on S. Minto's PDShoe_IMU_Sonar.
// If will work with SoleSound ver. Beta only.
//
// modified by D. Zanotto on Nov, 2014
// modified by D. Zanotto on Jun, 2015
// modified by Huanghe    on May, 2016
// modified by Huanghe    on Jun, 2017
// modified by Huanghe    on Jau, 2018
// modified by Ton        in Sep, 2019 (change IMU to polling mode)

//#include "Wire.h"
#include <i2c_t3.h>

/////////////////
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);
/////////////////


// Parameters for multiplexer
#define PIN_ENABLE 12
#define PIN_A      11
#define PIN_B      10
#define PIN_C      9
#define PIN_PRESSURE A9

#define TRUE 1
#define FALSE 0

#define MAX_YEI_DATA_PACKET 255

#define DEGTORAD  0.017453f
#define RADTODEG 57.295779f

#define NO_SLOT	255
#define READ_TARED_ORIENTATION_AS_QUATERNION 0
#define READ_TARED_ORIENTATION_AS_EULER_ANGLES 1
#define READ_UNTARED_ORIENTATION_AS_QUATERNION 6
#define READ_UNTARED_ORIENTATION_AS_EULER_ANGLES 7
#define READ_NORMALIZED_ACCELEROMETER_VECTOR  34
#define READ_CORRECTED_GYROSCOPE_VECTOR   38
#define READ_CORRECTED_ACCELEROMETER_VECTOR   39
#define READ_CORRECTED_LINEAR_ACCELERATION    41
#define READ_RAW_GYROSCOPE_VECTOR 65
#define READ_RAW_ACCELEROMETER_VECTOR 66
#define	READ_RAW_COMPASS_VECTOR 67

#define CMD_SET_BASE_OFFSET_WITH_CURRENT_ORIENTATION 22
#define CMD_SET_STREAMING_SLOT 80
#define CMD_SET_STREAMING_TIMING 82
#define CMD_GET_STREAMING_BATCH 84
#define CMD_START_STREAMING 85
#define CMD_STOP_STREAMING 86
#define CMD_TARE_WITH_CURRENT_ORIENTATION 96
#define CMD_SET_REFERENCE_VECTOR_MODE 105
#define CMD_SET_COMPASS_ENABLE 109
#define CMD_RESET_FILTER 120
#define CMD_SET_ACCELEROMETER_RANGE 121
#define CMD_SET_FILTER_MODE 123
#define CMD_SET_GYROSCOPE_RANGE 125
#define CMD_SET_COMPASS_RANGE 126

#define CMD_GET_COMPASS_ENABLED_STATE 142
#define CMD_GET_ACCELEROMETER_RANGE 148
#define CMD_GET_FILTER_MODE 152
#define CMD_GET_GYROSCOPE_RANGE 154
#define CMD_GET_COMPASS_RANGE 155
#define CMD_BEGIN_GYROSCOPE_AUTOCALIBRATION 165
#define CMD_SET_CALIBRATION_MODE 169
#define CMD_RESPONSE_HEADER_BITFIELD 221
#define CMD_SET_UART_BAUD_RATE 231
#define CMD_GET_UART_BAUD_RATE 232
#define START_SPI_DATA_TRANSFER 0xF6
#define START_NO_RESP_HEADER 0xF7
#define START_RESP_HEADER 0xF9

#define SPI_IDLE_STATE 0x00
#define SPI_READY_STATE 0x01
#define SPI_BUSY_STATE 0x02
#define SPI_ACC_STATE 0x04
#define DELAY_SPI_YEI 5 //[us]
#define DELAY_WAIT_SERIAL_YEI 1 //[us]
#define DELAY_SERIAL_YEI 10 //[us]

#define REFERENCE_VECTOR_SINGLE_STATIC_MODE 0
#define REFERENCE_VECTOR_SINGLE_AUTO_MODE 1
#define REFERENCE_VECTOR_SINGLE_AUTO_CONTINUOUS_MODE 2
#define REFERENCE_VECTOR_MULTI_REFERENCE_MODE 3

#define ACCELEROMETER_RANGE_2G 0
#define ACCELEROMETER_RANGE_4G 1
#define ACCELEROMETER_RANGE_8G 2

#define GYROSCOPE_RANGE_250 0
#define GYROSCOPE_RANGE_500 1
#define GYROSCOPE_RANGE_2000 2

#define COMPASS_RANGE_0_8 0
#define COMPASS_RANGE_1_3 1
#define COMPASS_RANGE_1_9 2
#define COMPASS_RANGE_2_5 3
#define COMPASS_RANGE_4_0 4
#define COMPASS_RANGE_4_7 5
#define COMPASS_RANGE_5_6 6
#define COMPASS_RANGE_8_1 7

#define FILTER_IMU 0
#define FILTER_KALMAN 1
#define FILTER_KALMAN_ALTERNATING 2
#define FILTER_COMPLEMENTARY 3
#define FILTER_QUATERNION_GRADIENT_DESCEND 4
#define FILTER_MAGNETORESISTIVE_QUATERNION_GRADIENT_DESCEND 5

#define CALIBRATION_MODE_BIAS 0
#define CALIBRATION_MODE_BIAS_SCALE 1
#define CALIBRATION_MODE_ORTHO 2

#define CMD_SET_AXIS_DIRECTIONS 116
#define AXIS_XR_YF_ZU 0x01 // Right-handed system
#define AXIS_XU_YR_ZF 0x02
#define AXIS_XF_YU_ZR 0x05

#define CMD_GET_FIRMWARE_VERSION 0xDF
#define CMD_GET_HARDWARE_VERSION 0xE6

#define YEI_DELAY_AFTER_COMMAND 100 //[ms]

#define BYTE_USB_PACKET 128
#define BYTE_DATA_PACKET 128
#define BYTE_PUREDATA_PACKET 77 //51
#define BYTE_WIFI_INCOMING_PACKET 16//11//7

uint8_t YEIdataPacket[MAX_YEI_DATA_PACKET];

////////////////////////////////////////////
#define US_TO_CM 0.0172
//#define PING_SENSOR 12

volatile unsigned long sonarTicDelay;
volatile unsigned long sonarDelay;
volatile bool sonarDownUpEdge = true;

//IntervalTimer sonarTimer;
#define DT_INSOLE 2000
////////////////////////////////////////////

#define ADXL345_ADDRESS 0x53

#define ADXL345_POWER_CTL 0x2D
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_RA_BW_RATE 0x2C
#define ADXL345_RESET_CTL 0x00
#define ADXL345_MEAS_CTL 0x08
#define ADXL345_DATAX0 0x32
#define ADXL345_DATAX1 0x33
#define ADXL345_DATAY0 0x34
#define ADXL345_DATAY1 0x35
#define ADXL345_DATAZ0 0x36
#define ADXL345_DATAZ1 0x37
#define ADXL345_RANGE_2G            0b00
#define ADXL345_RANGE_4G            0b01
#define ADXL345_RANGE_8G            0b10
#define ADXL345_RANGE_16G           0b11
#define ADXL345_RATE_3200           0b1111
#define ADXL345_RATE_1600           0b1110
#define ADXL345_RATE_800            0b1101
#define ADXL345_RATE_400            0b1100
#define ADXL345_RATE_200            0b1011
#define ADXL345_RATE_100            0b1010

#define ADXL345_LSB_G_2G            256.0f
#define ADXL345_LSB_G_4G            128.0f
#define ADXL345_LSB_G_8G             64.0f

//Bread Board
#define IMU1 Serial3//Serial1//Serial3 //Beta
//#define IMU2 Serial2//Serial2 //Beta
#define WIFI Serial1//Serial3


// Curve Fitting Coefficient
const float coef1[] = {4.9623, 4.9715, 4.5513, 4.9613, 5.0471, 4.8370, 4.7539, 4.8434}; // Multiply by 10000
const float coef2[] = {22.869, 28.117, 22.400, 21.504, 26.127, 22.159, 25.630, 22.922}; // Multiply by 1000
const uint16_t segmentpoint = 581;
//uint16_t ypoint[]={28813,28867,26427,28807,29305,28086,27603,28123};  // Multiply by 10

#define LED_TEENSY 13

struct structStreamingTimingInformation
{
  unsigned int interval;
  unsigned int duration;
  unsigned int delay;

} sStreamingTime;


struct structComponentLinearAcceleration
{
  //Big Endian
  float az;
  float ay;
  float ax;
};

struct structComponentRawAcceleration
{
  //Big Endian
  float r_az;
  float r_ay;
  float r_ax;
};

struct structComponentQuaternion
{
  //Big Endian
  float qw;
  float qz;
  float qy;
  float qx;
};


struct structEulerAngles
{
  //Big Endian
  float roll;
  float yaw;
  float pitch;
};

struct structComponentRawGyro
{
  // Big Endian
  float wz;
  float wy;
  float wx;
};

struct structComponentRawMag
{
  // Big Endian
  float mz;
  float my;
  float mx;
};

struct structComponentSensorData
{
  //Big Endian
  float mx;
  float my;
  float mz;

  float ax;
  float ay;
  float az;

  float gx;
  float gy;
  float gz;
};

struct structStreamingData
{
  //Big Endian
//  structComponentRawMag mag;
  structComponentRawAcceleration acc;
  structComponentRawGyro gyro;
  structComponentLinearAcceleration lAcc;
  //structEulerAngles eulerAngles;
  structComponentQuaternion q;
};

struct structSportSolePacket
{
  uint8_t Start_bytes[3];
  uint8_t val;
  uint8_t Odroid_Timestamp[8];
  uint8_t Odroid_Trigger;
  //uint8_t cycles[4];
  //uint8_t Packet_ID; //0=> Start Recording, 1=>ZeroIMU
  //uint8_t Insole_Size;//0=>S, 1=>M, 2=>L, 3=>XL
  uint8_t Stop_bytes[3];
};

union unionIncomingWifi
{
  uint8_t vData[BYTE_WIFI_INCOMING_PACKET];
  structSportSolePacket Raw;
}
Wifi_Incoming_Data;

union unionStreamingData
{
  structStreamingData sData;
  uint8_t vData[sizeof(structStreamingData)];

} uStreamingDataIMU1, uStreamingDataIMU2;


union unionComponentSensorData
{
  structComponentSensorData sData;
  uint8_t vData[sizeof(structComponentSensorData)];
} uCompSensData;

uint8_t calcCRC256(uint8_t* dataPacket, uint8_t nByte)
{
  uint16_t checksum = 0;
  for (uint8_t i = 1; i < nByte; i++)
  {
    checksum += dataPacket[i];
  }
  return (checksum % 256);
}

int waitByteCountFromSerial(HardwareSerial& serial, unsigned int bytecount)
{
  while (serial.available() < bytecount)
  {
    delayMicroseconds(DELAY_WAIT_SERIAL_YEI);
  }
}


void YEIsettingsHeader(HardwareSerial& serial)
{
  // Settings Header
  YEIdataPacket[0] = START_NO_RESP_HEADER;
  YEIdataPacket[1] = CMD_RESPONSE_HEADER_BITFIELD;
  YEIdataPacket[2] = 0x00;
  YEIdataPacket[3] = 0x00;
  YEIdataPacket[4] = 0x00;
  YEIdataPacket[5] = 0x00;
  YEIdataPacket[6] = calcCRC256(YEIdataPacket, 6);
  serial.write(YEIdataPacket, 7);
  delay(YEI_DELAY_AFTER_COMMAND);
}

void YEIwriteCommandNoDelay(HardwareSerial& serial, uint8_t cmd)
{
  YEIdataPacket[0] = START_NO_RESP_HEADER;
  YEIdataPacket[1] = cmd;
  YEIdataPacket[2] = calcCRC256(YEIdataPacket, 2);
  serial.write(YEIdataPacket, 3);
}

void YEIwriteCommand(HardwareSerial& serial, uint8_t cmd)
{
  YEIwriteCommandNoDelay(serial, cmd);
  delay(YEI_DELAY_AFTER_COMMAND);
}


void YEIwriteCommand(HardwareSerial& serial, uint8_t cmd, uint8_t value)
{
  YEIdataPacket[0] = START_NO_RESP_HEADER;
  YEIdataPacket[1] = cmd;
  YEIdataPacket[2] = value;
  YEIdataPacket[3] = calcCRC256(YEIdataPacket, 3);
  serial.write(YEIdataPacket, 4);
  delay(YEI_DELAY_AFTER_COMMAND);
}

uint8_t YEIgetValue(HardwareSerial& serial, uint8_t cmd)
{
  YEIwriteCommand(serial, cmd);
  return serial.read();
}

void YEIsetStreamingTime(HardwareSerial& serial)
{
  // Set Streaming Time

  YEIdataPacket[0] = START_NO_RESP_HEADER;
  YEIdataPacket[1] = CMD_SET_STREAMING_TIMING;

  uint8_t *pointer = (uint8_t*)&sStreamingTime.interval;
  YEIdataPacket[2] = pointer[3];
  YEIdataPacket[3] = pointer[2];
  YEIdataPacket[4] = pointer[1];
  YEIdataPacket[5] = pointer[0];

  pointer = (uint8_t*)&sStreamingTime.duration;
  YEIdataPacket[6] = pointer[3];
  YEIdataPacket[7] = pointer[2];
  YEIdataPacket[8] = pointer[1];
  YEIdataPacket[9] = pointer[0];

  pointer = (uint8_t*)&sStreamingTime.delay;
  YEIdataPacket[10] = pointer[3];
  YEIdataPacket[11] = pointer[2];
  YEIdataPacket[12] = pointer[1];
  YEIdataPacket[13] = pointer[0];

  YEIdataPacket[14] = calcCRC256(YEIdataPacket, 14);
  serial.write(YEIdataPacket, 15);

  //delay(YEI_DELAY_AFTER_COMMAND);
}

void YEIsetStreamingMode(HardwareSerial& serial, uint8_t slot1, uint8_t slot2, uint8_t slot3, uint8_t slot4, uint8_t slot5, uint8_t slot6, uint8_t slot7, uint8_t slot8)
{
  // Setting Streaming Mode
  YEIdataPacket[0] = START_NO_RESP_HEADER;
  YEIdataPacket[1] = CMD_SET_STREAMING_SLOT;

  YEIdataPacket[2] = slot1; //1st slot
  YEIdataPacket[3] = slot2; //2nd slot
  YEIdataPacket[4] = slot3; //3rd slot
  YEIdataPacket[5] = slot4; //4th slot
  YEIdataPacket[6] = slot5; //5th slot
  YEIdataPacket[7] = slot6; //6th slot
  YEIdataPacket[8] = slot7; //7th slot
  YEIdataPacket[9] = slot8; //8th slot

  YEIdataPacket[10] = calcCRC256(YEIdataPacket, 10);
  serial.write(YEIdataPacket, 11);

  delay(YEI_DELAY_AFTER_COMMAND);
}


// Polling mode
void YEIgetStreamingBatch(unionStreamingData& uStreamingDataIMU1)
{
  int nPacketStreamingData = sizeof(uStreamingDataIMU1);
  int bIMU = 0;

  YEIwriteCommandNoDelay(IMU1, CMD_GET_STREAMING_BATCH);

  while (IMU1.available() < nPacketStreamingData)
  {
    delayMicroseconds(DELAY_WAIT_SERIAL_YEI);
  }
  
  while (!(bIMU >= nPacketStreamingData))
  {
      uStreamingDataIMU1.vData[nPacketStreamingData - bIMU - 1] = IMU1.read();

      bIMU++;
  }
}

void sonarManageEdge()
{
  if (sonarDownUpEdge) //from DOWN to UP
  {
    sonarTicDelay = micros();
  }
  else // from UP to DOWN
  {
    sonarDelay = micros() - sonarTicDelay;
  }

  sonarDownUpEdge = !sonarDownUpEdge;

}

/*void functionSonarTimer()
  {
	detachInterrupt(PING_SENSOR);

	pinMode(PING_SENSOR, OUTPUT);
	digitalWrite(PING_SENSOR,HIGH);
	delayMicroseconds(10);
	digitalWrite(PING_SENSOR,LOW);

	pinMode(PING_SENSOR, INPUT);
	attachInterrupt(PING_SENSOR,sonarManageEdge,CHANGE);

  }
*/
///////////// A                     VBCBBCV BVVVVV////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct structADXL345Accelerometer
{
  //Little Endian
  float ax;
  float ay;
  float az;

} ADXL345acc;


struct structADXL345AccelerometerRAW
{
  //Little Endian
  int16_t ax;
  int16_t ay;
  int16_t az;

} ADXL345accRAW;


void ADXL345writeCommand(uint8_t address, uint8_t value) {
  Wire.beginTransmission(ADXL345_ADDRESS); // start transmission to device
  Wire.write(address);             // send register address
  Wire.write(value);                 // send value to write
  Wire.endTransmission();         // end transmission
}

// Reads num bytes starting from address register on device in to _buff array
//void ADXL345readData(uint8_t address, uint8_t nByte, byte buffer[]) {
//	Wire.beginTransmission(ADXL345_ADDRESS); // start transmission to device
//	Wire.write(address);             // sends address to read from
//	Wire.endTransmission();         // end transmission
//
//	Wire.beginTransmission(ADXL345_ADDRESS); // start transmission to device
//	Wire.requestFrom(ADXL345_ADDRESS, nByte);    // request 6 bytes from device
//
//	uint8_t i = 0;
//	while(Wire.available())         // device may send less than requested (abnormal)
//	{
//		buffer[i] = Wire.read();    // receive a byte
//		i++;
//	}
//	Wire.endTransmission();         // end transmission
//}

//void ADXL345readAccel() {
//	uint8_t buffer[6];
//
//	ADXL345readData(ADXL345_DATAX0, 0x06, buffer); //read the acceleration data from the ADXL345
//
//	//Least Significat Byte first
//	ADXL345acc.ax = (float)((int16_t)((buffer[1] << 8) | buffer[0]))/ADXL345_LSB_G_4G;
//	ADXL345acc.ay = (float)((int16_t)((buffer[3] << 8) | buffer[2]))/ADXL345_LSB_G_4G;
//	ADXL345acc.az = (float)((int16_t)((buffer[5] << 8) | buffer[4]))/ADXL345_LSB_G_4G;
//
//	//ADXL345accRAW.ax = ((int16_t)((buffer[1] << 8) | buffer[0]));
//	//ADXL345accRAW.ay = ((int16_t)((buffer[3] << 8) | buffer[2]));
//	//ADXL345accRAW.az = ((int16_t)((buffer[5] << 8) | buffer[4]));
//
//}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t dataCommunicationPacket[BYTE_PUREDATA_PACKET];
//uint8_t dataUSBCommunicationPacket[BYTE_USB_PACKET];

#ifndef SWAP_ELEMENT
#define SWAP_ELEMENT(a,b) { a^=b; b^=a; a^=b; }
#endif

uint8_t nPacketStreamingData = sizeof(structStreamingData);

unsigned long startTime;
unsigned long nowTime = 0;
unsigned long nowTime2 = 0;
unsigned long precTimestamp;

unsigned long int cycles = 0;
boolean led_teensy = 0;

boolean reset_imu;
boolean trigger;
boolean signal;

uint8_t bIMU1;

//float yaw1, pitch1, roll1;
float angle_x1, angle_y1, angle_z1;

//unsigned long nextTrigger; //[ms]
uint8_t Odroid_Timestamp[8]; //[ms]
uint8_t Odroid_Trigger; //[ms]
unsigned long currenttime;
uint8_t cmd;
uint16_t p[] = {0, 0, 0, 0, 0, 0, 0, 0};
uint16_t offset_p[] = {0, 0, 0, 0, 0, 0, 0, 0};


void createDataPacket_PureData(uint8_t *dataPacket,
                               unsigned long timestamp,
                               float yaw1, float pitch1, float roll1,
                               float ax1, float ay1, float az1,
                               float qw, float qx, float qy, float qz,
                               float wx, float wy, float wz,
                               float r_ax, float r_ay, float r_az,
                               float mx, float my, float mz,
                               uint16_t p1, uint16_t p2, uint16_t p3, uint16_t p4,
                               uint16_t p5, uint16_t p6, uint16_t p7, uint16_t p8,
                               uint8_t Odroid_Timestamp[8], uint8_t Odroid_Trigger, unsigned long timestamp2)
{
  uint8_t* pointer;
  int16_t val;

  // START
  dataPacket[0] = 0x07;
  dataPacket[1] = 0x08;
  dataPacket[2] = 0x09;

  //Timestamp
  pointer = (uint8_t*)&timestamp;
  dataPacket[3] = pointer[3];
  dataPacket[4] = pointer[2];
  dataPacket[5] = pointer[1];
  dataPacket[6] = pointer[0];

  //yaw1
  val = int16_t(yaw1 * 5000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[7] = pointer[1];
  dataPacket[8] = pointer[0];

  //pitch1
  val = int16_t(pitch1 * 5000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[9] = pointer[1];
  dataPacket[10] = pointer[0];

  //roll1
  val = int16_t(roll1 * 5000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[11] = pointer[1];
  dataPacket[12] = pointer[0];

  // Corrected linear acceleration
  val = int16_t(ax1 * 1000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[13] = pointer[1];
  dataPacket[14] = pointer[0];

  val = int16_t(ay1 * 1000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[15] = pointer[1];
  dataPacket[16] = pointer[0];

  val = int16_t(az1 * 1000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[17] = pointer[1];
  dataPacket[18] = pointer[0];

  // Quaternions
  val = int16_t(qw * 5000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[19] = pointer[1];
  dataPacket[20] = pointer[0];

  val = int16_t(qx * 5000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[21] = pointer[1];
  dataPacket[22] = pointer[0];

  val = int16_t(qy * 5000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[23] = pointer[1];
  dataPacket[24] = pointer[0];

  val = int16_t(qz * 5000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[25] = pointer[1];
  dataPacket[26] = pointer[0];

  // RAW Gyro
  val=int16_t(wx*900.0f);
  pointer=(uint8_t*)&val;
  dataPacket[27]=pointer[1];
  dataPacket[28]=pointer[0];

  val=int16_t(wy*900.0f);
  pointer=(uint8_t*)&val;
  dataPacket[29]=pointer[1];
  dataPacket[30]=pointer[0];

  val=int16_t(wz*900.0f);
  pointer=(uint8_t*)&val;
  dataPacket[31]=pointer[1];
  dataPacket[32]=pointer[0];

  // RAW Acceleration
  val = int16_t(r_ax * 1000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[33] = pointer[1];
  dataPacket[34] = pointer[0];

  val = int16_t(r_ay * 1000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[35] = pointer[1];
  dataPacket[36] = pointer[0];

  val = int16_t(r_az * 1000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[37] = pointer[1];
  dataPacket[38] = pointer[0];
  
  // RAW Magnetometer
  val = int16_t(mx * 1000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[39] = pointer[1];
  dataPacket[40] = pointer[0];

  val = int16_t(my * 1000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[41] = pointer[1];
  dataPacket[42] = pointer[0];

  val = int16_t(mz * 1000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[43] = pointer[1];
  dataPacket[44] = pointer[0];    

  //p1
  pointer = (uint8_t*)&p1;
  dataPacket[45] = pointer[1];
  dataPacket[46] = pointer[0];

  //p2
  pointer = (uint8_t*)&p2;
  dataPacket[47] = pointer[1];
  dataPacket[48] = pointer[0];

  //p3
  pointer = (uint8_t*)&p3;
  dataPacket[49] = pointer[1];
  dataPacket[50] = pointer[0];

  //p4
  pointer = (uint8_t*)&p4;
  dataPacket[51] = pointer[1];
  dataPacket[52] = pointer[0];

  //p5
  pointer = (uint8_t*)&p5;
  dataPacket[53] = pointer[1];
  dataPacket[54] = pointer[0];

  //p6
  pointer = (uint8_t*)&p6;
  dataPacket[55] = pointer[1];
  dataPacket[56] = pointer[0];

  //p7
  pointer = (uint8_t*)&p7;
  dataPacket[57] = pointer[1];
  dataPacket[58] = pointer[0];

  //p8
  pointer = (uint8_t*)&p8;
  dataPacket[59] = pointer[1];
  dataPacket[60] = pointer[0];

  //Timestamp_Odroid
  dataPacket[61] = Odroid_Timestamp[0];
  dataPacket[62] = Odroid_Timestamp[1];
  dataPacket[63] = Odroid_Timestamp[2];
  dataPacket[64] = Odroid_Timestamp[3];
  dataPacket[65] = Odroid_Timestamp[4];
  dataPacket[66] = Odroid_Timestamp[5];
  dataPacket[67] = Odroid_Timestamp[6];
  dataPacket[68] = Odroid_Timestamp[7];

  dataPacket[69] = Odroid_Trigger;
  //pointer=(uint8_t*)&Odroid_Timestamp;
  // dataPacket[35]=pointer[8];
  // dataPacket[36]=pointer[7];
  // dataPacket[37]=pointer[6];
  // dataPacket[38]=pointer[5];
  // dataPacket[39]=pointer[4];
  // dataPacket[40]=pointer[3];
  // dataPacket[41]=pointer[2];
  // dataPacket[42]=pointer[1];
  // dataPacket[43]=pointer[0];

  //Timestamp2
  pointer = (uint8_t*)&timestamp2;
  dataPacket[70] = pointer[3];
  dataPacket[71] = pointer[2];
  dataPacket[72] = pointer[1];
  dataPacket[73] = pointer[0];

  // STOP
  dataPacket[74] = 0xA;
  dataPacket[75] = 0xB;
  dataPacket[76] = 0xC;
}



void quaternion2euler(float q0, float q1, float q2, float q3, float& phi, float& theta, float& psi)
{
  //quaternConj(q0,q1,q2,q3);

  q1 = -q1;
  q2 = -q2;
  q3 = -q3;

  float R11 = 2 * q0 * q0 - 1 + 2 * q1 * q1;
  float R21 = 2 * (q1 * q2 - q0 * q3);
  float R31 = 2 * (q1 * q3 + q0 * q2);
  float R32 = 2 * (q2 * q3 - q0 * q1);
  float R33 = 2 * q0 * q0 - 1 + 2 * q3 * q3;

  phi = atan2(R32, R33);
  theta = -atan(R31 / sqrt(1 - R31 * R31) );
  psi = atan2(R21, R11 );
}


void quaternion2euler_v3(float qx,float qy, float qz, float qw, float& x, float& y, float& z)
{
  float    xx      = qx * qx;
  float    xy      = qx * qy;
  float    xz      = qx * qz;
  float    xw      = qx * qw;

  float    yy      = qy * qy;
  float    yz      = qy * qz;
  float    yw      = qy * qw;

  float   zz      = qz * qz;
  float   zw      = qz * qw;

  float m00 = 1 - 2 * ( yy + zz );
  float m01 =     2 * ( xy - zw );
  float m02 =     2 * ( xz + yw );

  //float m10 =     2 * ( xy + zw );
  //float m11 = 1 - 2 * ( xx + zz );
  float m12 =     2 * ( yz - xw );

  //float m20 =     2 * ( xz - yw );
  //float m21 =     2 * ( yz + xw );
  float m22 = 1 - 2 * ( xx + yy );

  //float tol = 1.00E-6;

  y = asin(m02);
  x = atan2(-m12/cos(y), m22/cos(y));
  z = atan2(-m01/cos(y), m00/cos(y));
}


uint16_t get_pressure(uint8_t num)
{
  uint16_t val = 0;
  //int val;
  digitalWrite(PIN_ENABLE, LOW);
  switch (num) {
    case 1:
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, HIGH);
      digitalWrite(PIN_C, LOW);
      break;
    case 2:
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, HIGH);
      digitalWrite(PIN_C, HIGH);
      break;
    case 3:
      digitalWrite(PIN_A, HIGH);
      digitalWrite(PIN_B, HIGH);
      digitalWrite(PIN_C, LOW);
      break;
    case 4:
      digitalWrite(PIN_A, HIGH);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, HIGH);
      break;
    case 5:
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, HIGH);
      break;
    case 6:
      digitalWrite(PIN_A, HIGH);
      digitalWrite(PIN_B, HIGH);
      digitalWrite(PIN_C, HIGH);
      break;
    case 7:
      digitalWrite(PIN_A, HIGH);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, LOW);
      break;
    case 8:
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, LOW);
      break;
  }
  delayMicroseconds(4);
  val = analogRead(PIN_PRESSURE);
  //val = millis() % 1000 * 1023/1000;
  val = Volt2Pressure(num, val);
  return ((uint16_t) val);
}

uint16_t Volt2Pressure(uint16_t num, uint16_t RawOutput)
{
  float val;
  if (RawOutput <= segmentpoint)
  {
    //val=coef1[num-1]/1000;
    //val=val**RawOutput;
    val = coef1[num - 1] * RawOutput;
  } else {
    //val = coef2[num-1]/1000;
    //val=val*(RawOutput-segmentpoint)+coef1[num-1]*segmentpoint)*10;
    val = coef2[num - 1] * (RawOutput - segmentpoint) + coef1[num - 1] * segmentpoint;
  }

  return ((uint16_t) val);
}

boolean checkWIFIPacket(uint8_t *buffer)
{
  return (buffer[0] == 0x01 && buffer[1] == 0x02 && buffer[2] == 0x03  && buffer[BYTE_WIFI_INCOMING_PACKET - 3] == 0x4 && buffer[BYTE_WIFI_INCOMING_PACKET - 2] == 0x5 && buffer[BYTE_WIFI_INCOMING_PACKET - 1] == 0x6);
}

boolean checkRemoteStart2(uint8_t *buffer)
{
  return (buffer[0] == 0x01 && buffer[1] == 0x02 && buffer[2] == 0x03 && buffer[BYTE_WIFI_INCOMING_PACKET - 3] == 0x4 && buffer[BYTE_WIFI_INCOMING_PACKET - 2] == 0x5 && buffer[BYTE_WIFI_INCOMING_PACKET - 1] == 0x6) && (buffer[3] == 0x01);
}

boolean checkRemoteReboot2(uint8_t *buffer)
{
  return (buffer[0] == 0x01 && buffer[1] == 0x02 && buffer[2] == 0x03 && buffer[BYTE_WIFI_INCOMING_PACKET - 3] == 0x4 && buffer[BYTE_WIFI_INCOMING_PACKET - 2] == 0x5 && buffer[BYTE_WIFI_INCOMING_PACKET - 1] == 0x6) && (buffer[3] == 0x00);
}


boolean checkOdroidTimestamp(uint8_t *buffer)
{
  return (buffer[0] == 0x01 && buffer[1] == 0x02 && buffer[2] == 0x03 && buffer[BYTE_WIFI_INCOMING_PACKET - 3] == 0x4 && buffer[BYTE_WIFI_INCOMING_PACKET - 2] == 0x5 && buffer[BYTE_WIFI_INCOMING_PACKET - 1] == 0x6) && (buffer[3] == 0x02);
}

void setup()
{
  Serial.begin(115200);
  IMU1.begin(921600);
  WIFI.begin(921600);

  Serial.setTimeout(100);
  IMU1.setTimeout(100);
  WIFI.setTimeout(100);

  delay(100);

  IMU1.clear();
  WIFI.clear();

  pinMode(LED_TEENSY, OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT);
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(PIN_C, OUTPUT);

  for (int i = 0; i < sizeof(dataCommunicationPacket); i++) dataCommunicationPacket[i] = 0x00;

  // start blinking
  for (int i = 0; i < 20; i++)
  {
    led_teensy = !led_teensy;
    digitalWrite(LED_TEENSY, led_teensy);
    delay(75);
  }
  led_teensy = TRUE;
  digitalWrite(LED_TEENSY, led_teensy);

  //Wait for incoming packet to activate device.
  boolean StartRec = FALSE;
  uint8_t bWIFI;
  while (StartRec == FALSE)
  {
    if (WIFI.available() > 0)
    {
      bWIFI = 0;
      while (!(bWIFI >= BYTE_WIFI_INCOMING_PACKET))
      {
        if ((WIFI.available() > 0) && (bWIFI < BYTE_WIFI_INCOMING_PACKET))
        {
          Wifi_Incoming_Data.vData[bWIFI] = WIFI.read();
          bWIFI++;
        }
      }
      //check integrity of the packet, process the packet
      if (checkRemoteStart2(Wifi_Incoming_Data.vData))
      {
        StartRec = TRUE;
      }
    }
    led_teensy = !led_teensy;
    digitalWrite(LED_TEENSY, led_teensy);
    delay(75);
  }

  YEIsettingsHeader(IMU1);

  YEIwriteCommand(IMU1, CMD_STOP_STREAMING);
  IMU1.clear();

  YEIwriteCommand(IMU1, CMD_SET_ACCELEROMETER_RANGE, ACCELEROMETER_RANGE_8G);
  YEIwriteCommand(IMU1, CMD_SET_GYROSCOPE_RANGE, GYROSCOPE_RANGE_2000);
  YEIwriteCommand(IMU1, CMD_SET_COMPASS_RANGE, COMPASS_RANGE_1_3);
  YEIwriteCommand(IMU1, CMD_SET_CALIBRATION_MODE, CALIBRATION_MODE_BIAS_SCALE);
  YEIwriteCommand(IMU1, CMD_SET_AXIS_DIRECTIONS,AXIS_XR_YF_ZU);
  YEIwriteCommand(IMU1, CMD_SET_REFERENCE_VECTOR_MODE, REFERENCE_VECTOR_MULTI_REFERENCE_MODE);
  YEIwriteCommand(IMU1, CMD_SET_COMPASS_ENABLE, FALSE);

  YEIwriteCommandNoDelay(IMU1, CMD_BEGIN_GYROSCOPE_AUTOCALIBRATION);
  delay(3000);

  YEIwriteCommandNoDelay(IMU1, CMD_RESET_FILTER);
  delay(1000);

//  YEIsetStreamingMode(IMU1, READ_TARED_ORIENTATION_AS_QUATERNION, READ_CORRECTED_ACCELEROMETER_VECTOR, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT);
//  YEIsetStreamingMode(IMU1, READ_TARED_ORIENTATION_AS_QUATERNION, READ_CORRECTED_ACCELEROMETER_VECTOR, READ_RAW_GYROSCOPE_VECTOR, READ_RAW_ACCELEROMETER_VECTOR, READ_RAW_COMPASS_VECTOR, NO_SLOT, NO_SLOT, NO_SLOT);
  YEIsetStreamingMode(IMU1, READ_UNTARED_ORIENTATION_AS_QUATERNION, READ_CORRECTED_ACCELEROMETER_VECTOR, READ_CORRECTED_GYROSCOPE_VECTOR, READ_CORRECTED_ACCELEROMETER_VECTOR, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT);

  sStreamingTime.interval = 2000; //10000; //[us]
  sStreamingTime.duration = 0xFFFFFFFF;
  sStreamingTime.delay = 0;	//[us]
  YEIsetStreamingTime(IMU1);

  IMU1.clear();
  IMU1.flush();

  //nextTrigger=10000000; //[us]
  reset_imu = FALSE;
  signal = FALSE;
  //trigger=TRUE;

  cmd = 0x00;

  startTime = micros();
  precTimestamp = micros();

}



void loop()
{
  uint8_t bWIFI;
  auto loop_t0 = micros();
  if (WIFI.available() >= BYTE_WIFI_INCOMING_PACKET)
  {
    bWIFI = 0;
    while (!(bWIFI >= BYTE_WIFI_INCOMING_PACKET))
    {
      if ((WIFI.available() > 0) && (bWIFI < BYTE_WIFI_INCOMING_PACKET))
      {
        Wifi_Incoming_Data.vData[bWIFI] = WIFI.read();
        bWIFI++;
      }
    }
    //Serial.println("Received!");
    //check integrity
    if (checkWIFIPacket(Wifi_Incoming_Data.vData))
    {
      //Serial.println("Received and validated!");
      if (checkOdroidTimestamp(Wifi_Incoming_Data.vData))
      {
	    	nowTime2 = micros() - startTime;  //
        for (int i = 0; i < sizeof(Odroid_Timestamp); i++) Odroid_Timestamp[i] = Wifi_Incoming_Data.Raw.Odroid_Timestamp[i];
        Odroid_Trigger = Wifi_Incoming_Data.Raw.Odroid_Trigger;
        //Serial.print(Odroid_Trigger);
        //Serial.println("This is Odroid Timestamp!");
      }
      else if (checkRemoteReboot2(Wifi_Incoming_Data.vData))
      {
        //Serial.println("This is Odroid Reboot!"); delay(1000);
        CPU_RESTART;
      }
      else
      {
        //Serial.println("Bad Packet!");
      }
    }
  }

  auto loop_t1 = micros();

  nowTime = micros() - startTime;
  if (nowTime >= 4000000)
  {

    if (reset_imu == false)
    {

      reset_imu = true;

      YEIwriteCommandNoDelay(IMU1, CMD_STOP_STREAMING);

      IMU1.flush();

//      YEIsetStreamingMode(IMU1, READ_TARED_ORIENTATION_AS_QUATERNION, READ_CORRECTED_LINEAR_ACCELERATION, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT);
//      YEIsetStreamingMode(IMU1, READ_TARED_ORIENTATION_AS_QUATERNION, READ_CORRECTED_LINEAR_ACCELERATION, READ_RAW_GYROSCOPE_VECTOR, READ_RAW_ACCELEROMETER_VECTOR, READ_RAW_COMPASS_VECTOR, NO_SLOT, NO_SLOT, NO_SLOT);
      YEIsetStreamingMode(IMU1, READ_UNTARED_ORIENTATION_AS_QUATERNION, READ_CORRECTED_LINEAR_ACCELERATION, READ_CORRECTED_GYROSCOPE_VECTOR, READ_CORRECTED_ACCELEROMETER_VECTOR, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT);

      YEIwriteCommandNoDelay(IMU1, CMD_TARE_WITH_CURRENT_ORIENTATION);

      IMU1.flush();
      
      YEIsetStreamingTime(IMU1);
      //YEIwriteCommandNoDelay(IMU1, CMD_START_STREAMING);

      signal = !signal;
    }
    else
    {
      // MODE 1 : Streaming
//      bIMU1 = 0;
//
//      while (!(bIMU1 >= nPacketStreamingData))
//      {
//        if ((IMU1.available() > 0) && (bIMU1 < nPacketStreamingData))
//        {
//          uStreamingDataIMU1.vData[nPacketStreamingData - bIMU1 - 1] = IMU1.read();
//          bIMU1++;
//        }
//      }

      // MODE 2 : Polling
      YEIgetStreamingBatch(uStreamingDataIMU1);
      //quaternion2euler(uStreamingDataIMU1.sData.q.qw, uStreamingDataIMU1.sData.q.qy, uStreamingDataIMU1.sData.q.qx, uStreamingDataIMU1.sData.q.qz, yaw1, pitch1, roll1);
      quaternion2euler_v3(uStreamingDataIMU1.sData.q.qx,uStreamingDataIMU1.sData.q.qy,uStreamingDataIMU1.sData.q.qz,uStreamingDataIMU1.sData.q.qw,angle_x1,angle_y1,angle_z1);
    }
  }
  else
  {
    YEIwriteCommandNoDelay(IMU1, CMD_GET_STREAMING_BATCH);

    waitByteCountFromSerial(IMU1, nPacketStreamingData);

    for (uint8_t idx = nPacketStreamingData; idx > 0; idx--)
    {
      uStreamingDataIMU1.vData[idx - 1] = IMU1.read();
    }

  }

  auto loop_t2 = micros();

  // Read Pressure
  for (uint8_t h = 0; h < 8; h++) {
    p[h] = get_pressure(h + 1) - offset_p[h];
  }
  // p[0]=100;p[1]=200;p[2]=300;p[3]=400;p[4]=500;p[5]=600;p[6]=700;p[7]=800;

  // if (nowTime>=nextTrigger)
  // {
  // 	trigger=!trigger;
  // 	nextTrigger+=1000000;
  // }

  //createDataPacket_PureData
  createDataPacket_PureData(dataCommunicationPacket,
                            nowTime,

                            // Calculated Euler angles
                            angle_x1,//yaw1,//-yaw1,
                            angle_y1,//pitch1,//-pitch1,
                            angle_z1,//roll1,//roll1,

                            // Corrected linear accelerations
                            uStreamingDataIMU1.sData.lAcc.ax,//uStreamingDataIMU1.sData.lAcc.az,//-uStreamingDataIMU1.sData.lAcc.az,
                            uStreamingDataIMU1.sData.lAcc.ay,//-uStreamingDataIMU1.sData.lAcc.ax,//-uStreamingDataIMU1.sData.lAcc.ax,
                            uStreamingDataIMU1.sData.lAcc.az,//uStreamingDataIMU1.sData.lAcc.ay,//-uStreamingDataIMU1.sData.lAcc.ay,

                            // Corrected quaternions
                            uStreamingDataIMU1.sData.q.qw, 
                            uStreamingDataIMU1.sData.q.qx, 
                            uStreamingDataIMU1.sData.q.qy,                             
                            uStreamingDataIMU1.sData.q.qz,

                            // RAW Gyro
                            uStreamingDataIMU1.sData.gyro.wx,
                            uStreamingDataIMU1.sData.gyro.wy,
                            uStreamingDataIMU1.sData.gyro.wz,

                            // RAW Acc
                            uStreamingDataIMU1.sData.acc.r_ax,
                            uStreamingDataIMU1.sData.acc.r_ay,
                            uStreamingDataIMU1.sData.acc.r_az,

                            // RAW Mag (No magnetometer for the insoles)
                            0,
                            0,
                            0,

                            // Pressure sensors
                            p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7],

                            Odroid_Timestamp, Odroid_Trigger, nowTime2);

  auto loop_t3 = micros();

  WIFI.write(dataCommunicationPacket, BYTE_PUREDATA_PACKET);

  auto loop_t4 = micros();

  // if (loop_t4 - loop_t0 > 42000)
  // {
  //   Serial.print(loop_t4 - loop_t3); Serial.print(", ");
  //   Serial.print(loop_t3 - loop_t2); Serial.print(", ");
  //   Serial.print(loop_t2 - loop_t1); Serial.print(", ");
  //   Serial.println(loop_t1 - loop_t0);
  // }

  if ((cycles % 250) == 0)
  {
    led_teensy = !led_teensy;
    digitalWrite(LED_TEENSY, led_teensy);
  }

  if (((cycles + 125) % 500) == 0)
  {
    led_teensy = !led_teensy;
    digitalWrite(LED_TEENSY, led_teensy);
  }

  cycles++;
}
