/* Generated from orogen/lib/orogen/templates/typekit/Types.hpp */

#ifndef __OROGEN_GENERATED_PROACTIVEASSISTANCE_TYPES_HPP
#define __OROGEN_GENERATED_PROACTIVEASSISTANCE_TYPES_HPP





#include "ProactiveAssistance/types/ProactiveAssistance/ProactiveAssistance-types.hpp"




// This is a hack. We include it unconditionally as it may be required by some
// typekits *and* it is a standard header. Ideally, we would actually check if
// some of the types need std::vector.
#include <vector>
#include <boost/cstdint.hpp>




#ifdef ORO_CHANNEL_ELEMENT_HPP
    extern template class RTT::base::ChannelElement< ProactiveAssistanceData >;
#endif
#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::internal::DataSource< ProactiveAssistanceData >;
    extern template class RTT::internal::AssignableDataSource< ProactiveAssistanceData >;
#endif
#ifdef ORO_CORELIB_DATASOURCES_HPP
    extern template class RTT::internal::ValueDataSource< ProactiveAssistanceData >;
    extern template class RTT::internal::ConstantDataSource< ProactiveAssistanceData >;
    extern template class RTT::internal::ReferenceDataSource< ProactiveAssistanceData >;
#endif
#ifdef ORO_INPUT_PORT_HPP
    extern template class RTT::OutputPort< ProactiveAssistanceData >;
#endif
#ifdef ORO_OUTPUT_PORT_HPP
    extern template class RTT::InputPort< ProactiveAssistanceData >;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property< ProactiveAssistanceData >;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute< ProactiveAssistanceData >;
#endif

#ifdef ORO_CHANNEL_ELEMENT_HPP
    extern template class RTT::base::ChannelElement< ::std::string >;
#endif
#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::internal::DataSource< ::std::string >;
    extern template class RTT::internal::AssignableDataSource< ::std::string >;
#endif
#ifdef ORO_CORELIB_DATASOURCES_HPP
    extern template class RTT::internal::ValueDataSource< ::std::string >;
    extern template class RTT::internal::ConstantDataSource< ::std::string >;
    extern template class RTT::internal::ReferenceDataSource< ::std::string >;
#endif
#ifdef ORO_INPUT_PORT_HPP
    extern template class RTT::OutputPort< ::std::string >;
#endif
#ifdef ORO_OUTPUT_PORT_HPP
    extern template class RTT::InputPort< ::std::string >;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property< ::std::string >;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute< ::std::string >;
#endif

#ifdef ORO_CHANNEL_ELEMENT_HPP
    extern template class RTT::base::ChannelElement< ::std::vector< double > >;
#endif
#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::internal::DataSource< ::std::vector< double > >;
    extern template class RTT::internal::AssignableDataSource< ::std::vector< double > >;
#endif
#ifdef ORO_CORELIB_DATASOURCES_HPP
    extern template class RTT::internal::ValueDataSource< ::std::vector< double > >;
    extern template class RTT::internal::ConstantDataSource< ::std::vector< double > >;
    extern template class RTT::internal::ReferenceDataSource< ::std::vector< double > >;
#endif
#ifdef ORO_INPUT_PORT_HPP
    extern template class RTT::OutputPort< ::std::vector< double > >;
#endif
#ifdef ORO_OUTPUT_PORT_HPP
    extern template class RTT::InputPort< ::std::vector< double > >;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property< ::std::vector< double > >;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute< ::std::vector< double > >;
#endif


#endif

