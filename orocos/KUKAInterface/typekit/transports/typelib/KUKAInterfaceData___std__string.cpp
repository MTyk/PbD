/* Generated from orogen/lib/orogen/templates/typekit/typelib/Type.cpp */

#include "Types.hpp"
#include <rtt/typelib/TypelibMarshaller.hpp>
#include "transports/typelib/Registration.hpp"

orogen_transports::TypelibMarshallerBase* orogen_typekits::KUKAInterfaceData_TypelibMarshaller(Typelib::Registry const& registry)
{
    return new orogen_transports::TypelibMarshaller< KUKAInterfaceData >("/KUKAInterfaceData", registry);
}




/* Generated from orogen/lib/orogen/templates/typekit/typelib/Type.cpp */

#include "Types.hpp"
#include <rtt/typelib/TypelibMarshaller.hpp>
#include "transports/typelib/Registration.hpp"

orogen_transports::TypelibMarshallerBase* orogen_typekits::std_string_TypelibMarshaller(Typelib::Registry const& registry)
{
    return new orogen_transports::TypelibMarshaller< ::std::string >("/std/string", registry);
}



