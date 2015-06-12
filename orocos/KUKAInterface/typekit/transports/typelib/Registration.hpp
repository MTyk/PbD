/* Generated from orogen/lib/orogen/templates/typekit/typelib/Registration.hpp */

#ifndef __OROGEN_GENERATED_KUKAINTERFACE_TYPELIB_REGISTRATION_HPP
#define __OROGEN_GENERATED_KUKAINTERFACE_TYPELIB_REGISTRATION_HPP

#include <rtt/types/TypeInfoGenerator.hpp>

namespace Typelib {
    class Registry;
}

namespace orogen_transports {
    class TypelibMarshallerBase;
}

namespace orogen_typekits {
    
    /** Creates and returns a TypeInfo object for KUKAInterfaceData */
    orogen_transports::TypelibMarshallerBase* KUKAInterfaceData_TypelibMarshaller(Typelib::Registry const& registry);
    
    /** Creates and returns a TypeInfo object for ::std::string */
    orogen_transports::TypelibMarshallerBase* std_string_TypelibMarshaller(Typelib::Registry const& registry);
    
    /** Creates and returns a TypeInfo object for ::std::vector< double > */
    orogen_transports::TypelibMarshallerBase* std_vector__double__TypelibMarshaller(Typelib::Registry const& registry);
    
}

#endif


