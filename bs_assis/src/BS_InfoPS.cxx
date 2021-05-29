#include "BS_InfoPS.h" 


//!Initialize the subscriber
bool InfoPS::init()
{

    DomainParticipantQos participantQos;
    participantQos.name("BS_InfoPS");
    participant_ = DomainParticipantFactory::get_instance()->create_participant(0, participantQos);

    if (participant_ == nullptr) { return false; }

    // Register the Type
    // Create the subscriptions Topic
    type_.register_type(participant_);
    topic_ = participant_->create_topic("HelloWorldTopic", "MavData", TOPIC_QOS_DEFAULT);
    if (topic_ == nullptr) { return false; }


    // Create the Subscriber
    subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
    if (subscriber_ == nullptr) { return false; }
    reader_ = subscriber_->create_datareader(topic_, DATAREADER_QOS_DEFAULT, &drListener_);
    if (reader_ == nullptr) { return false; }

    // Create the Publisher
    publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
    if (publisher_ == nullptr) { return false; }
    writer_ = publisher_->create_datawriter(topic_, DATAWRITER_QOS_DEFAULT, &dwListener_);
    if (writer_ == nullptr) { return false; }

    return true;
}


void InfoPS::runPub(){}

// 40hz进行推送，对标mavros的30hz
void InfoPS::run(uint32_t samples)
{
    uint32_t samples_sent = 0;
    while (samples_sent < samples)
    {
        if (publish())
        {
            samples_sent++;
            // std::cout << "Message: " << hello_.message() << " with index: " << hello_.index() << " SENT" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }

}

// 推送当前ID的mav数据
// void InfoPS::run()
// {
//     while (true)
//     {
        // std::cout<< "pub" <<std::endl;
        // std::cout<< "mavVec: "<<mavVec[0].p.pose.position.z <<std::endl;
        // if (publish())
        // {
            
            // std::cout << "Message: " << hello_.message() << " with index: " << hello_.index() << " SENT" << std::endl;
        // }
//         std::this_thread::sleep_for(std::chrono::milliseconds(25));
//     }
// }

void InfoPS::runSub()
{
    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}



bool InfoPS::publish()
{
    // DataWriter和DataReader有匹配上则发送
    // listener_.matched_ 是匹配上的数量
    // if (dwListener_.matched_ > 0)
    // {
    //     hello_.index(hello_.index() + 1);
    //     writer_->write(&hello_);
    //     return true;
    // }
    return false;
}

bool InfoPS::publish(MavData* sendData)
{
    if (dwListener_.matched_ > 0)
    {
        writer_->write(sendData);
        return true;
    }
    return false;
}
