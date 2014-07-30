#include <ccpp_dds_dcps.h>
#include <string>

#include "std_msgs/dds_idl/ccpp_Int32_.h"
#include "std_msgs/dds_idl/Int32__conversion.h"


/*
template<typename ROSmsg, typename DDSmsg>
void func(const ROSmsg& ros_msg)
{
    DDSmsg dds_msg;
    dds_idl::convert_ros_message_to_dds(ros_msg, dds_msg);
*/


/*
template<typename ROSmsg>
void func(const ROSmsg& ros_msg)
{
    typename ROSmsg::DdsType dds_dw;
*/

// global template which should never be instantiated
template<typename T>
struct Resolver
{
  typedef void* DDSMsgType;
  typedef void* DDSMsgTypeSupportType;
  typedef void* DDSMsgDataWriter;
};

// per-dds-message type defs
template<>
struct Resolver<std_msgs::Int32>
{
  typedef std_msgs::dds_idl::Int32_ DDSMsgType;
  typedef std_msgs::dds_idl::Int32_TypeSupport DDSMsgTypeSupportType;
  typedef std_msgs::dds_idl::Int32_DataWriter DDSMsgDataWriterType;
};

template<typename ROSmsg>
void func(const ROSmsg& ros_msg)
{
    typedef typename Resolver<ROSmsg>::DDSMsgType DDSMsg_t;
    typedef typename Resolver<ROSmsg>::DDSMsgTypeSupportType DDSMsgTypeSupport_t;
    typedef typename Resolver<ROSmsg>::DDSMsgDataWriterType DDSMsgDataWriter_t;

    DDSMsg_t dds_msg;
    dds_idl::DDSTypeResolver<ROSmsg>::convert_ros_message_to_dds(ros_msg, dds_msg);

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

    std_msgs::dds_idl::Int32_TypeSupport_var ros_message_ts = new DDSMsgTypeSupport_t();
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

    std_msgs::dds_idl::Int32_DataWriter_var data_writer = DDSMsgDataWriter_t::_narrow(topic_writer.in());
}
