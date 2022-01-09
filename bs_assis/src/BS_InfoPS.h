#pragma once
/*
本文件是dds接收和传输类的头文件定义
*/


#include "MavDataPubSubTypes.h"

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>


// publisher特有的文件
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>


// subscriber特有的文件
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>


#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>


// ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Mavlink.h>


#include <vector>
#include <thread>
#include <sstream>


//extern std::vector<mav_state> mavVec;
extern ros::V_Publisher velPubs;
extern ros::V_Publisher posPubs;

using namespace eprosima::fastdds::dds;


class PubListener : public DataWriterListener
{
public:

    PubListener() : matched_(0) {}

    ~PubListener() override {}

    // 可以定义一些action当新的linstener被检测到
    void on_publication_matched( DataWriter*, const PublicationMatchedStatus& info) override
    {
        if (info.current_count_change == 1)
        {
            matched_ = info.total_count;
            std::cout << "Publisher matched." << std::endl;
        }
        else if (info.current_count_change == -1)
        {
            matched_ = info.total_count;
            std::cout << "Publisher unmatched." << std::endl;
        }
        else
        {
            std::cout << info.current_count_change
                    << " is not a valid value for PublicationMatchedStatus current count change." << std::endl;
        }
    }

    std::atomic_int matched_;

};


class SubListener : public DataReaderListener
{
public:

    SubListener() : samples_(0) { }

    ~SubListener() override { }

    void on_subscription_matched( DataReader*, const SubscriptionMatchedStatus& info) override
    {
        if (info.current_count_change == 1)
        {
            std::cout << "Subscriber matched." << std::endl;
        }
        else if (info.current_count_change == -1)
        {
            std::cout << "Subscriber unmatched." << std::endl;
        }
        else
        {
            std::cout << info.current_count_change
                    << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
        }
    }

    // 接收，数据可用的时候
    void on_data_available(DataReader* reader) override
    {
        SampleInfo info;
        if (reader->take_next_sample(&recvData, &info) == ReturnCode_t::RETCODE_OK)
        {
            if (info.valid_data)
            {
                geometry_msgs::PoseStamped posMsg;
                geometry_msgs::TwistStamped velMsg;
                
                posMsg.pose.position.x  = recvData.pose_x();
                posMsg.pose.position.y  = recvData.pose_y();
                posMsg.pose.position.z  = recvData.pose_z();


                posMsg.pose.orientation.w  = recvData.quat_w();
                posMsg.pose.orientation.x  = recvData.quat_x();
                posMsg.pose.orientation.y  = recvData.quat_y();
                posMsg.pose.orientation.z  = recvData.quat_z();

                velMsg.twist.linear.x   = recvData.vel_x();
                velMsg.twist.linear.y   = recvData.vel_y();
                velMsg.twist.linear.z   = recvData.vel_z();
                velMsg.twist.angular.x  = recvData.ang_x();
                velMsg.twist.angular.y  = recvData.ang_y();
                velMsg.twist.angular.z  = recvData.ang_z();
                                
                // samples_++;
                // std::cout << " recv system_ID: " << recvData.system_ID() << std::endl;
                posPubs[recvData.system_ID()].publish(posMsg);
                velPubs[recvData.system_ID()].publish(velMsg);
            }
        }
    }

    MavData recvData;
    std::atomic_int samples_;

};

class InfoPS
{
private:
    // MavData sendData;


    DomainParticipant* participant_;

    Topic* topic_;
    TypeSupport type_;

    Subscriber* subscriber_;
    DataReader* reader_;

    Publisher* publisher_;
    DataWriter* writer_;
    
    PubListener dwListener_;
    SubListener drListener_;


public:

    InfoPS()
        : participant_(nullptr)
        , subscriber_(nullptr)
        , topic_(nullptr)
        , reader_(nullptr)
        , type_(new MavDataPubSubType())
    {}

    virtual ~InfoPS()
    {
        if (reader_ != nullptr)
        {
            subscriber_->delete_datareader(reader_);
        }
        if (topic_ != nullptr)
        {
            participant_->delete_topic(topic_);
        }
        if (subscriber_ != nullptr)
        {
            participant_->delete_subscriber(subscriber_);
        }
        DomainParticipantFactory::get_instance()->delete_participant(participant_);
    }

    //!Initialize the subscriber
    bool init();
   
    //Run the Subscriber
    void run(uint32_t samples);
    void run();

    void runPub();
    void runSub();

    bool publish();
    bool publish(MavData* sendData);
};




