#include <ccpp_dds_dcps.h>
#include <string>

#include "std_msgs/dds_impl/ccpp_Int32_.h"
#include "std_msgs/Int32.h"

//#include "ccpp_ROSMsg.h"

#include "func.h"


main(int argc, char** argv)
{
    DDS::DomainParticipantFactory_var dpf_ = DDS::DomainParticipantFactory::get_instance();
    DDS::DomainId_t domain = DDS::DOMAIN_ID_DEFAULT;

    DDS::DomainParticipant_var participant_ = dpf_->create_participant(
        domain, PARTICIPANT_QOS_DEFAULT, NULL,
        DDS::STATUS_MASK_NONE);

    DDS::ReturnCode_t status;

    std::string partition_name = "ros_partition";

    DDS::TopicQos default_topic_qos;
    status = participant_->get_default_topic_qos(default_topic_qos);
    default_topic_qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;

    std_msgs::dds_impl::Int32_TypeSupport_var ros_message_ts = new std_msgs::dds_impl::Int32_TypeSupport();
    char * ros_message_type_name = ros_message_ts->get_type_name();
    status = ros_message_ts->register_type(
        participant_.in(), ros_message_type_name);

    DDS::PublisherQos publisher_qos;
    status = participant_->get_default_publisher_qos(publisher_qos);
    publisher_qos.partition.name.length(1);
    publisher_qos.partition.name[0] = partition_name.c_str();

    DDS::Publisher_var dds_publisher = participant_->create_publisher(
        publisher_qos, NULL, DDS::STATUS_MASK_NONE);

    DDS::Topic_var ros_message_topic = participant_->create_topic(
        "topic_name", ros_message_type_name, default_topic_qos, NULL,
        DDS::STATUS_MASK_NONE
    );

    DDS::DataWriter_var topic_writer = dds_publisher->create_datawriter(
        ros_message_topic.in(), DATAWRITER_QOS_USE_TOPIC_QOS,
        NULL, DDS::STATUS_MASK_NONE);

    std_msgs::dds_impl::Int32_DataWriter_var data_writer = std_msgs::dds_impl::Int32_DataWriter::_narrow(topic_writer.in());

    // ---

    std_msgs::Int32 ros_msg;
    func<std_msgs::Int32>(ros_msg);

    return 0;
}