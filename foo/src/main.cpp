#include <iostream>
#include <string>

#include "std_msgs/dds_idl/Int32_.h"
#ifndef ndds_cpp_h
  #include <ccpp_dds_dcps.h>
  #include "std_msgs/dds_idl/ccpp_Int32_.h"
#endif

#include "std_msgs/Int32.h"

//#include "ccpp_ROSMsg.h"

//#include "func.h"


#ifdef ndds_cpp_h
DDS_TypeCode *createTypeCode()
{
    DDS_TypeCodeFactory * factory = NULL;
    DDS_TypeCode * structTc = NULL;
    DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
    DDS_StructMemberSeq structMembers;
    factory = DDS_TypeCodeFactory::get_instance();

    structTc = factory->create_struct_tc("test_type", structMembers, ex);
    structTc->add_member("my_bool", DDS_TYPECODE_MEMBER_ID_INVALID, factory->get_primitive_tc(DDS_TK_BOOLEAN),
                    DDS_TYPECODE_NONKEY_REQUIRED_MEMBER, ex);
    // structTc->add_member("bool_2", DDS_TYPECODE_MEMBER_ID_INVALID, factory->get_primitive_tc(DDS_TK_BOOLEAN),
    //                 DDS_TYPECODE_NONKEY_REQUIRED_MEMBER, ex);
    // structTc->add_member("bool_3", DDS_TYPECODE_MEMBER_ID_INVALID, factory->get_primitive_tc(DDS_TK_BOOLEAN),
    //                 DDS_TYPECODE_NONKEY_REQUIRED_MEMBER, ex);
    // structTc->add_member("double_1", DDS_TYPECODE_MEMBER_ID_INVALID, factory->get_primitive_tc(DDS_TK_DOUBLE),
    //                 DDS_TYPECODE_NONKEY_REQUIRED_MEMBER, ex);
    // structTc->add_member("ulong_1", DDS_TYPECODE_MEMBER_ID_INVALID, factory->get_primitive_tc(DDS_TK_ULONG),
    //                 DDS_TYPECODE_NONKEY_REQUIRED_MEMBER, ex);
    // structTc->add_member("string_1", DDS_TYPECODE_MEMBER_ID_INVALID, factory->create_string_tc(60, ex),
    //                 DDS_TYPECODE_NONKEY_REQUIRED_MEMBER, ex);
    DDS_StructMemberSeq_finalize(&structMembers);
    return structTc;
}
#endif


main(int argc, char** argv)
{
#ifdef ndds_cpp_h
    DDSDomainParticipantFactory* dpf_ = DDSDomainParticipantFactory::get_instance();
    DDS_DomainId_t domain = 23;

    DDSDomainParticipant* participant_ = dpf_->create_participant(
        domain, DDS_PARTICIPANT_QOS_DEFAULT, NULL,
        DDS_STATUS_MASK_NONE);

    DDS_TypeCode* typeCode = createTypeCode();
    DDS_DynamicData dynamicData(typeCode, DDS_DYNAMIC_DATA_PROPERTY_DEFAULT);
    dynamicData.set_boolean("my_bool", DDS_DYNAMIC_DATA_MEMBER_ID_UNSPECIFIED, true);

    DDS_ExceptionCode_t err;
    std::cout << "Data Type" << std::endl;
    typeCode->print_IDL(0, err);
    std::cout << "Data" << std::endl;
    dynamicData.print(stdout, 1);


    DDSDynamicDataTypeSupport* ddts = new DDSDynamicDataTypeSupport(
        typeCode, DDS_DYNAMIC_DATA_TYPE_PROPERTY_DEFAULT);
    //DDS_DynamicData* dynamicData = ddts->create_data();
    DDS_ReturnCode_t status = ddts->register_type(participant_, "my_type");


    DDS_PublisherQos publisher_qos;
    status = participant_->get_default_publisher_qos(publisher_qos);

    DDSPublisher* dds_publisher = participant_->create_publisher(
        publisher_qos, NULL, DDS_STATUS_MASK_NONE);


    DDS_TopicQos default_topic_qos;
    status = participant_->get_default_topic_qos(default_topic_qos);

    DDSTopic* topic = participant_->create_topic(
        "topic_name", "my_type", default_topic_qos, NULL,
        DDS_STATUS_MASK_NONE
    );


    DDS_DataWriterQos default_datawriter_qos;
    status = participant_->get_default_datawriter_qos(default_datawriter_qos);

    DDSDataWriter* topic_writer = dds_publisher->create_datawriter(
        topic, default_datawriter_qos,
        NULL, DDS_STATUS_MASK_NONE);


    DDSDynamicDataWriter* dynamic_writer = DDSDynamicDataWriter::narrow(topic_writer);

    dynamic_writer->write(dynamicData, DDS_HANDLE_NIL);

#else
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

    std_msgs::dds_idl::Int32_TypeSupport_var ros_message_ts = new std_msgs::dds_idl::Int32_TypeSupport();
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

    std_msgs::dds_idl::Int32_DataWriter_var data_writer = std_msgs::dds_idl::Int32_DataWriter::_narrow(topic_writer.in());
#endif

    // ---

    std_msgs::Int32 ros_msg;
    //func<std_msgs::Int32>(ros_msg);

    return 0;
}
