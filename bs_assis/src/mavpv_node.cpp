// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/TwistStamped.h>
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/State.h>
// #include <mavros_msgs/Mavlink.h>


// #include <vector>
// #include <thread>
// #include <sstream>

#include "BS_InfoPS.h" 


struct mav_state 
{
    short id;
    geometry_msgs::PoseStamped p;
    geometry_msgs::TwistStamped t;
};

std::vector<mav_state> mavVec;
ros::V_Publisher posPubs;
ros::V_Publisher velPubs;

// 用于更新本地的状态
void velCallback(const geometry_msgs::TwistStamped::ConstPtr& msg, int mav_id)
{
    mavVec[mav_id].t = *msg;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int mav_id)
{
    mavVec[mav_id].p = *msg;
    // std::cout <<" id: " << mavState.id <<" pos: " << mavState.p.pose.position.z<<std::endl;
}

void constructMavData(MavData& sendData,int mav_id)
{
    mav_state tmp = mavVec[mav_id];
    sendData.system_ID(tmp.id);

    sendData.pose_x(tmp.p.pose.position.x);
    sendData.pose_y(tmp.p.pose.position.y);
    sendData.pose_z(tmp.p.pose.position.z);

    sendData.quat_w(tmp.p.pose.orientation.w);
    sendData.quat_x(tmp.p.pose.orientation.x);
    sendData.quat_y(tmp.p.pose.orientation.y);
    sendData.quat_z(tmp.p.pose.orientation.z);


    sendData.vel_x(tmp.t.twist.linear.x);
    sendData.vel_y(tmp.t.twist.linear.y);
    sendData.vel_z(tmp.t.twist.linear.z);

    sendData.ang_x(tmp.t.twist.angular.x);
    sendData.ang_y(tmp.t.twist.angular.y);
    sendData.ang_z(tmp.t.twist.angular.z);

}


// 40Hz
void runDDS(InfoPS* bsPS,int mav_id)
{
    while (true)
    {
        // std::cout<< "pub" <<std::endl;
        // std::cout<< "mavVec: "<<mavVec[0].p.pose.position.z <<std::endl;
        MavData sendData;
        constructMavData(sendData,mav_id);
        // sendData.system_ID(mavVec[mav_id].id);
        if (bsPS->publish(&sendData))
        {
            std::cout 
                << " send system_ID: " << sendData.system_ID()+1
                << " pose z: " << mavVec[mav_id].p.pose.position.z
                << std::endl;
            // std::cout << "Message: " << hello_.message() << " with index: " << hello_.index() << " SENT" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/40));
    }
}


// 先把从mavros拿到的东西，存到向量中，并且从DDS发出去
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavpv_node");
    ros::NodeHandle nh("~");



    int mav_num = 1;
    int mav_id = 1;
    nh.param<int>("mav_num",mav_num , 6);
    nh.param<int>("mav_id",mav_id , 1);

    // nh.advertise<std_msgs::String>("chatter", 1000);

    std::cout<<"mav_num:"<<mav_num<<std::endl;
    std::cout<<"mav_id:"<<mav_id<<std::endl;
    mavVec.reserve(mav_num);

    mav_id = mav_id-1;

    for (size_t i = 0; i < mav_num; i++)
    {
        mav_state temp;
        temp.id = i;
        mavVec.push_back(temp);


        // drone id 从1开始
        std::stringstream fmt;
        fmt << "/drone_"<<(i+1);

        posPubs.push_back(nh.advertise<geometry_msgs::PoseStamped>(fmt.str()+"/mavros/local_position/pose", 10));
        velPubs.push_back(nh.advertise<geometry_msgs::TwistStamped>(fmt.str()+"/mavros/local_position/velocity_local", 10));
    }
    

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                (
                    "/mavros/local_position/pose", 10,
                    boost::bind(&poseCallback, _1, mav_id)
                 );
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
                (
                    "/mavros/local_position/velocity_local", 10,
                    boost::bind(&velCallback, _1, mav_id)
                 );

    std::cout << "Starting DDS pub and sub" << std::endl;

    InfoPS* bsPS = new InfoPS();
    if(!bsPS->init())
    {
        std::cout<< "DDS init successful!" <<std::endl;
        return 1;
    }

    std::cout<< "DDS init successful!" <<std::endl;
    std::thread ddsPub_thread(runDDS, bsPS, mav_id);


    ros::spin();
    delete bsPS;

    return 0;
}
