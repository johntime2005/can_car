#ifndef _CANBUS_H
#define _CANBUS_H

#include <stdio.h>
#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <string> 
#include <iostream>
#include <math.h>
#include <stdlib.h>    
#include <unistd.h>      
#include <sys/types.h>
#include <sys/stat.h>
// #include <serial/serial.h>
#include <fcntl.h>          
#include <stdbool.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
using namespace std;

#if defined(_WIN32) || defined(__CYGWIN__) || defined(_WIN32_WCE)
#define LIBUSB_CALL __stdcall
#else
#define LIBUSB_CALL
#endif

typedef unsigned int u32 ;
typedef unsigned char u8 ;

typedef  struct  _Can_Msg {
    u32             ID;
    u32             TimeStamp;
    char            FrameType;
    char            DataLen;
    char	        Data[8];
    char	        ExternFlag;
    char	        RemoteFlag;
    char            BusSatus;
    char            ErrSatus;
    char            TECounter;
    char            RECounter;
}Can_Msg, *P_Can_Msg;

typedef struct _Can_Status {
    char            BusSatus;
    char            ErrSatus;
    char            TECounter;
    char            RECounter;
}Can_Status, *P_Can_Status;

typedef struct _Can_Config {
    u32             baudrate;
    u32             configs;
    u32             model;
}Can_Config, *P_Can_Config;


#ifdef __cplusplus
extern "C" {
#endif

// devNum表示设备索引号，chNum表示通道号  当插入一个双通道CAN设备时 devNum为0， chNum为0或者1；
//  当插入N个双通道CAN设备时 devNum为0至(N-1)， chNum为0或者1；

int LIBUSB_CALL  Reg_HotPlug_Func(void(*pfunc)(void));		//热插拔
int LIBUSB_CALL  CAN_ScanDevice(void);		//扫描CAN设备
int LIBUSB_CALL  CAN_OpenDevice(u32 devNum, u32 chNum);		//打开CAN设备
int LIBUSB_CALL  CAN_CloseDevice(u32 devNum, u32 chNum);	  //关闭CAN设备
int LIBUSB_CALL  CAN_Init(u32 devNum, u32 chNum, P_Can_Config pInitConfig);		//初始化CAN设备
int LIBUSB_CALL  CAN_SetFilter(u32 devNum, u32 chNum, char namber, char type, u32 ftID, u32 ftMask, char enable);		//设置过滤器
int LIBUSB_CALL  CAN_Reset(u32 devNum, u32 chNum);		//复位CAN设备
int LIBUSB_CALL  CAN_GetReceiveNum(u32 devNum, u32 chNum);			//读取缓冲区未使用的帧
int LIBUSB_CALL  CAN_Transmit(u32 devNum, u32 chNum, P_Can_Msg canmsg, u32 items, u32 timeou);		//发送CAN数据
int LIBUSB_CALL  CAN_Receive(u32 devNum, u32 chNum, P_Can_Msg canmsg, u32 Len, u32 timeou);		//接受CAN数据
int LIBUSB_CALL  CAN_GetStatus(u32 devNum, u32 chNum, P_Can_Status status);		//获取设备状态
#ifdef __cplusplus
}
#endif

//Macro definition
//宏定义
#define SEND_DATA_CHECK   1          //Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK   0          //Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER      0X7B       //Frame head //帧头
#define FRAME_TAIL        0X7D       //Frame tail //帧尾
#define RECEIVE_DATA_SIZE 24         //The length of the data sent by the lower computer //下位机发送过来的数据的长度
#define SEND_DATA_SIZE    11         //The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度
#define PI 				  3.1415926f //PI //圆周率

//Relative to the range set by the IMU gyroscope, the range is ±500°, corresponding data range is ±32768
//The gyroscope raw data is converted in radian (rad) units, 1/65.5/57.30=0.00026644
//与IMU陀螺仪设置的量程有关，量程±500°，对应数据范围±32768
//陀螺仪原始数据转换位弧度(rad)单位，1/65.5/57.30=0.00026644
#define GYROSCOPE_RATIO   0.00026644f
//Relates to the range set by the IMU accelerometer, range is ±2g, corresponding data range is ±32768
//Accelerometer original data conversion bit m/s^2 units, 32768/2g=32768/19.6=1671.84	
//与IMU加速度计设置的量程有关，量程±2g，对应数据范围±32768
//加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
#define ACCEl_RATIO 	  1671.84f  	

extern sensor_msgs::Imu Mpu6050; //External variables, IMU topic data //外部变量，IMU话题数据

//Covariance matrix for speedometer topic data for robt_pose_ekf feature pack
//协方差矩阵，用于里程计话题数据，用于robt_pose_ekf功能包
const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };

const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0, 
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9 };

const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };
										      
const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0, 
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9} ;

//速度、位置数据结构体
typedef struct __Vel_Pos_Data_
{
	float X;
	float Y;
	float Z;
}Vel_Pos_Data;

//IMU数据结构体
typedef struct __MPU6050_DATA_
{
	short accele_x_data; 
	short accele_y_data; 	
	short accele_z_data; 
    short gyros_x_data; 
	short gyros_y_data; 	
	short gyros_z_data; 

}MPU6050_DATA;

//ROS向下位机发送数据的结构体
typedef struct _SEND_DATA_  
{
	    uint8_t tx[SEND_DATA_SIZE];
		float X_speed;	       
		float Y_speed;           
		float Z_speed;         
		unsigned char Frame_Tail; 
}SEND_DATA;

typedef struct _RECEIVE_DATA_     
{
	    uint8_t rx[RECEIVE_DATA_SIZE];
	    uint8_t Flag_Stop;
		unsigned char Frame_Header;
		float X_speed;  
		float Y_speed;  
		float Z_speed;  
		float Power_Voltage;	
		unsigned char Frame_Tail;
}RECEIVE_DATA;

//机器人底盘类，使用构造函数初始化数据和发布话题等
class turn_on_robot
{
	public:
		turn_on_robot();  //构造函数
		~turn_on_robot(); //析构函数
		void Control();   //循环控制代码
		//声明串口对象 
	private:
		ros::NodeHandle n;           //创建ROS节点句柄
		ros::Time _Now, _Last_Time;  //时间相关，用于积分求位移(里程)
		float Sampling_Time;         //采样时间，用于积分求位移(里程)

		ros::Subscriber Cmd_Vel_Sub; //初始化话题订阅者
		//速度话题订阅回调函数
		void Cmd_Vel_Callback(const geometry_msgs::Twist &twist_aux);              

		ros::Publisher odom_publisher, imu_publisher, voltage_publisher; //初始化话题发布者
		void Publish_Odom();      //发布里程计话题
		// void Publish_ImuSensor(); //Pub the IMU sensor topic //发布IMU传感器话题
		void Publish_Voltage();   //发布电源电压话题

        //从串口(ttyUSB)读取运动底盘速度、IMU、电源电压数据
        bool Get_Sensor_Data();   
		bool Get_Sensor_Data_New();
        unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode); //BBC校验函数
        // short IMU_Trans(uint8_t Data_High,uint8_t Data_Low);  //IMU数据转化读取
		float Odom_Trans(uint8_t Data_High,uint8_t Data_Low); //里程计数据转化读取

        string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id; //定义相关变量
        int serial_baud_rate;      //串口通信波特率
        RECEIVE_DATA Receive_Data; //串口接收数据结构体
        SEND_DATA Send_Data;       //串口发送数据结构体

        Vel_Pos_Data Robot_Pos;    //机器人的位置
        Vel_Pos_Data Robot_Vel;    //机器人的速度
        // MPU6050_DATA Mpu6050_Data; //IMU data //IMU数据
        float Power_voltage;       //电源电压
};

#endif
