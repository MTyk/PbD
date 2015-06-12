/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <KUKAInterface/Types.hpp>
#include <KUKAInterface/type_info/BoostSerialization.hpp>
#include <rtt/types/StructTypeInfo.hpp>




namespace orogen_typekits {
    struct KUKAInterfaceDataTypeInfo :
        public RTT::types::StructTypeInfo< KUKAInterfaceData >
    {
        KUKAInterfaceDataTypeInfo()
            : RTT::types::StructTypeInfo< KUKAInterfaceData >("/KUKAInterfaceData") {}



    };

    RTT::types::TypeInfoGenerator* KUKAInterfaceData_TypeInfo()
    { return new KUKAInterfaceDataTypeInfo(); }

}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::OutputPort< KUKAInterfaceData >;
template class RTT::InputPort< KUKAInterfaceData >;
template class RTT::Property< KUKAInterfaceData >;
template class RTT::Attribute< KUKAInterfaceData >;

template class RTT::internal::DataSource< KUKAInterfaceData >;
template class RTT::internal::ValueDataSource< KUKAInterfaceData >;
template class RTT::internal::ConstantDataSource< KUKAInterfaceData >;
template class RTT::internal::AssignableDataSource< KUKAInterfaceData >;
template class RTT::internal::ReferenceDataSource< KUKAInterfaceData >;




/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <KUKAInterface/Types.hpp>
#include <KUKAInterface/type_info/BoostSerialization.hpp>
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



