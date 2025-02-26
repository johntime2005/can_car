#include"ros/ros.h"
#include"control/canbus.h"

int dev = 0;
int cpot0 = 0;
int cpot1 = 1;
Can_Msg txmsg1[100];
Can_Msg txmsg2[100];
Can_Msg txmsg3[100];
Can_Msg rxmsg[100];
int frame_counter = 0;

// 数据转换函数
float turn_on_robot::Odom_Trans(uint8_t Data_High,uint8_t Data_Low)
{
    float data_return;
    short transition_16;
    transition_16 = 0;
    transition_16 |= Data_High<<8;
    transition_16 |= Data_Low;
    data_return = (transition_16 / 1000)+(transition_16 %1000)*0.001;
    return transition_16;
}

// 发布里程计话题
void turn_on_robot::Publish_Odom()
{
    //把Z轴转角转换为四元数进行表达
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Robot_Pos.Z);
    nav_msgs::Odometry odom;    //实例化里程计话题数据
    odom.header.stamp = ros::Time::now(); 
    odom.header.frame_id = odom_frame_id;        //里程计TF父坐标
    odom.pose.pose.position.x = Robot_Pos.X;     //位置
    odom.pose.pose.position.y = Robot_Pos.Y;
    odom.pose.pose.position.z = Robot_Pos.Z;
    odom.pose.pose.orientation = odom_quat;      //姿态，通过Z轴转角转换的四元数

    odom.child_frame_id = robot_frame_id;        //里程计TF子坐标
    odom.twist.twist.linear.x =  Robot_Vel.X;    //X方向速度
    odom.twist.twist.linear.y =  Robot_Vel.Y;    //Y方向速度
    odom.twist.twist.angular.z = Robot_Vel.Z;    //绕Z轴角速度 

    //这个矩阵有两种，分别在机器人静止和运动的时候使用。扩展卡尔曼滤波官方提供的2个矩阵，用于robot_pose_ekf功能包
    if(Robot_Vel.X== 0&&Robot_Vel.Y== 0&&Robot_Vel.Z== 0)
      //如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
      memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
      memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    // else
    // //如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
    //   memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
    //   memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));       
    odom_publisher.publish(odom); //Pub odometer topic //发布里程计话题
}

//发布电压相关信息

void turn_on_robot::Publish_Voltage()
{
    std_msgs::Float32 voltage_msgs; //定义电源电压发布话题的数据类型
    static float Count_Voltage_Pub=0;
    if(Count_Voltage_Pub++>10)
      {
        Count_Voltage_Pub=0;  
        voltage_msgs.data = Power_voltage;  //电源供电的电压获取
        voltage_publisher.publish(voltage_msgs); //发布电源电压话题单位：V、伏特
      }
}

//打印报文和ID
void print_frame(Can_Msg* msg) {
    printf("Sending CAN Frame:\n");
    printf("ID: 0x%X, Data: ", msg->ID);
    for (int i = 0; i < msg->DataLen; i++) {
        printf("%02X ", msg->Data[i]);
    }
    printf("\n");
}

// 定义三个发送函数 (发送不同的信号帧)
void send_frame1() {
    memset(&txmsg1[0], 0, sizeof(txmsg1[0]));
    txmsg1[0].ID = 0x18C4D1D0;
    txmsg1[0].Data[0] = 0x83;
    txmsg1[0].Data[1] = 0x3E;
    txmsg1[0].Data[2] = 0x00;
    txmsg1[0].Data[3] = 0x00;
    txmsg1[0].Data[4] = 0x00;
    txmsg1[0].Data[5] = 0x00;
    txmsg1[0].Data[6] = 0x00;
    txmsg1[0].Data[7] = 0xBD;
    txmsg1[0].DataLen = 8;
    txmsg1[0].ExternFlag = 1; 
    // 发送数据
    CAN_Transmit(dev, cpot0, &txmsg1[0], 1, 100);
    print_frame(&txmsg1[0]);
}

void send_frame2() {
    memset(&txmsg2[0], 0, sizeof(txmsg2[0]));
    txmsg2[0].ID = 0x18C4D1D0;
    txmsg2[0].Data[0] = 0x83;
    txmsg2[0].Data[1] = 0x3E;
    txmsg2[0].Data[2] = 0x00;
    txmsg2[0].Data[3] = 0x00;
    txmsg2[0].Data[4] = 0x00;
    txmsg2[0].Data[5] = 0x00;
    txmsg2[0].Data[6] = 0x10;
    txmsg2[0].Data[7] = 0xAD;
    txmsg2[0].DataLen = 8;
    txmsg2[0].ExternFlag = 1; 
    CAN_Transmit(dev, cpot0, txmsg2, 1, 100);
    print_frame(&txmsg2[0]);  // 打印发送的数据帧
}

void send_frame3() {
    memset(&txmsg3[0], 0, sizeof(txmsg3[0]));
    txmsg3[0].ID = 0x18C4D1D0;
    txmsg3[0].Data[0] = 0x83;
    txmsg3[0].Data[1] = 0x00;
    txmsg3[0].Data[2] = 0xC0;
    txmsg3[0].Data[3] = 0x63;
    txmsg3[0].Data[4] = 0x0F;
    txmsg3[0].Data[5] = 0x00;
    txmsg3[0].Data[6] = 0x20;
    txmsg3[0].Data[7] = 0x8F;
    txmsg3[0].DataLen = 8;
    txmsg3[0].ExternFlag = 1; 
    CAN_Transmit(dev, cpot0, txmsg3, 1, 100);
}

int can2_receive_msgs(void)         //CAN2接收数据
{
    int ret,i;
    memset(&rxmsg[0],0,sizeof(rxmsg[0]));
    ret = CAN_Receive(dev,cpot1,rxmsg,100,100);
    printf("Receive msg: id = %x",rxmsg[0].ID);
    for(i = 0 ; i < rxmsg[0].DataLen; i++)
    {
        printf("%02x ",(unsigned char)rxmsg[0].Data[i]);
    }
    printf("\r\n");
    return ret;
}

// 定时器回调函数，每10毫秒执行一次
void timerCallback(const ros::TimerEvent&) {
    switch (frame_counter) {
        case 0:
            send_frame1();  // 发送第一帧
            ROS_INFO("send_frame1 = %x",txmsg1[0].ID);
        
            break;
        case 1:
            send_frame2();  // 发送第二帧
            ROS_INFO("send_frame2 = %x",txmsg2[0].ID);
            // can2_receive_msgs();
            break;
        case 2:
            send_frame3();  // 发送第三帧
            ROS_INFO("send_frame3 = %x",txmsg3[0].ID);
            // can2_receive_msgs();
            break;
    }
    frame_counter = (frame_counter + 1) % 3;  // 轮流发送三帧
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "canbus_control");  // 初始化ROS节点
    ros::NodeHandle nh;

    // 初始化CAN设备，省略初始化代码
    int devs,ret;
    Can_Config cancfg;
    devs = CAN_ScanDevice();                 //扫描CAN设备
    // printf("CAN_ScanDevice = %d\r\n",devs);
    if(devs <= 0) return -1;
    ret = CAN_OpenDevice(dev,cpot0);        //打开通道0(CAN1)
    // printf("CAN_OpenDevice0 = %d\r\n",ret);
    if(ret != 0) return -1;
    // printf("CAN_OpenDevice0 succeed!!\r\n");
    ret = CAN_OpenDevice(dev,cpot1);        //打开通道1(CAN2)
    // printf("CAN_OpenDevice1 = %d\r\n",ret);
    if(ret != 0) return -1;
    // printf("CAN_OpenDevice1 succeed!!\r\n");
    cancfg.model = 0;
    cancfg.configs = 0;
    cancfg.baudrate = 500000;  //设置波特率500k(500*1000)
    cancfg.configs |= 0x0001;  //接通内部匹配电阻
    cancfg.configs |= 0x0002;  //开启离线唤醒模式
    cancfg.configs |= 0x0004;  //开启自动重传

    ret = CAN_Init(dev,0,&cancfg);         //初始化CAN1
    // printf("CAN_Init0 = %d\r\n",ret);
    if(ret != 0) return -1;
    // printf("CAN_Init0 succeed!!\r\n");
    CAN_SetFilter(dev,cpot0,0,0,0,0,1);    //设置接收所有数据

    ret = CAN_Init(dev,cpot1,&cancfg);     //初始化CAN2
    // printf("CAN_Init1 = %d\r\n",ret);
    if(ret != 0) return -1;
    // printf("CAN_Init1 succeed!!\r\n");
    CAN_SetFilter(dev,cpot1,0,0,0,0,1);    //设置接收所有数据
    // if(ret != 0) return -1;

    // 创建一个10毫秒的定时器，周期性发送CAN消息

    ros::Timer timer = nh.createTimer(ros::Duration(0.01), timerCallback);
    ROS_INFO("send succeed");
    ros::spin();  // 进入ROS主循环

    return 0;
}


