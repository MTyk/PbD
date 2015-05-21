/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <ProactiveAssistance/Types.hpp>
#include <ProactiveAssistance/type_info/BoostSerialization.hpp>
#include <rtt/types/StructTypeInfo.hpp>




namespace orogen_typekits {
    struct ProactiveAssistanceDataTypeInfo :
        public RTT::types::StructTypeInfo< ProactiveAssistanceData >
    {
        ProactiveAssistanceDataTypeInfo()
            : RTT::types::StructTypeInfo< ProactiveAssistanceData >("/ProactiveAssistanceData") {}



    };

    RTT::types::TypeInfoGenerator* ProactiveAssistanceData_TypeInfo()
    { return new ProactiveAssistanceDataTypeInfo(); }

}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::OutputPort< ProactiveAssistanceData >;
template class RTT::InputPort< ProactiveAssistanceData >;
template class RTT::Property< ProactiveAssistanceData >;
template class RTT::Attribute< ProactiveAssistanceData >;

template class RTT::internal::DataSource< ProactiveAssistanceData >;
template class RTT::internal::ValueDataSource< ProactiveAssistanceData >;
template class RTT::internal::ConstantDataSource< ProactiveAssistanceData >;
template class RTT::internal::AssignableDataSource< ProactiveAssistanceData >;
template class RTT::internal::ReferenceDataSource< ProactiveAssistanceData >;




/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <ProactiveAssistance/Types.hpp>
#include <ProactiveAssistance/type_info/BoostSerialization.hpp>
#include <rtt/types/SequenceTypeInfo.hpp>

#include <rtt/typekit/StdStringTypeInfo.hpp>




namespace orogen_typekits {
    struct std_stringTypeInfo :
        public RTT::types::StdStringTypeInfo
    {
        std_stringTypeInfo()
            : RTT::types::StdStringTypeInfo("/std/string") {}



    };

    RTT::types::TypeInfoGenerator* std_string_TypeInfo()
    { return new std_stringTypeInfo(); }

}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::OutputPort< ::std::string >;
template class RTT::InputPort< ::std::string >;
template class RTT::Property< ::std::string >;
template class RTT::Attribute< ::std::string >;

template class RTT::internal::DataSource< ::std::string >;
template class RTT::internal::ValueDataSource< ::std::string >;
template class RTT::internal::ConstantDataSource< ::std::string >;
template class RTT::internal::AssignableDataSource< ::std::string >;
template class RTT::internal::ReferenceDataSource< ::std::string >;



