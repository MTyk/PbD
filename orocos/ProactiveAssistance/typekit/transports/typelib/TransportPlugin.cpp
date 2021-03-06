/* Generated from orogen/lib/orogen/templates/typekit/typelib/TransportPlugin.cpp */

#include "Registration.hpp"
#include "TransportPlugin.hpp"
#include <rtt/typelib/TypelibMarshallerBase.hpp>
#include <rtt/Logger.hpp>
#include <rtt/types/TypekitPlugin.hpp>
using namespace RTT;
#ifdef HAS_ROSLIB
#include <ros/package.h>
#endif

#define TYPEKIT_PACKAGE_NAME_aux0(target) #target
#define TYPEKIT_PACKAGE_NAME_aux(target) "ProactiveAssistance-typekit-" TYPEKIT_PACKAGE_NAME_aux0(target)
#define TYPEKIT_PACKAGE_NAME TYPEKIT_PACKAGE_NAME_aux(OROCOS_TARGET)

std::string orogen_typekits::ProactiveAssistanceTypelibTransportPlugin::getTypelibRegistryPath()
{
#ifdef HAS_ROSLIB
    /** If ROSLIB is available, we'll use ros::package to find the path to a package with the
      same name as our typekit. This may only be enabled if IS_ROS_PACKAGE is true at build time ! **/
    using namespace ros::package;
    try {
        bool all_good = true, found = false;
        std::string ppath = getPath( "ProactiveAssistance" );
        if ( !ppath.empty() ) {
            return ppath + "/typekit" + "/ProactiveAssistance.tlb";
        } else
            log(Debug) << "Not a ros package: " << "ProactiveAssistance" << endlog();
    } catch(...) {
        log(Debug) << "Not a ros package: " << "ProactiveAssistance" << endlog();
    }
#endif
    return TYPEKIT_REGISTRY;
}

std::string orogen_typekits::ProactiveAssistanceTypelibTransportPlugin::getTlbPath() const
{
    return ProactiveAssistanceTypelibTransportPlugin::getTypelibRegistryPath();
}

orogen_typekits::ProactiveAssistanceTypelibTransportPlugin::ProactiveAssistanceTypelibTransportPlugin()
    : TypelibTransportPlugin("ProactiveAssistance") {}

bool orogen_typekits::ProactiveAssistanceTypelibTransportPlugin::registerTransport(std::string type_name, RTT::types::TypeInfo* ti)
{
    if (!m_registry)
    {
        if (!loadRegistry())
            return false;
    }
    
    if(ti->hasProtocol(orogen_transports::TYPELIB_MARSHALLER_ID))
	return false;

    
    if ("/ProactiveAssistanceData" == type_name)
    {
        try {
            ti->addProtocol(orogen_transports::TYPELIB_MARSHALLER_ID,
                ProactiveAssistanceData_TypelibMarshaller(*m_registry));
        }
        catch(std::runtime_error const& e)
        {
            log(Error) << "cannot register a typelib transport for " << type_name << endlog();
            log(Error) << "  the following error occured:" << endlog();
            log(Error) << "  " << e.what() << endlog();
            log(Error) << "  the registry can be found at " << TYPEKIT_REGISTRY << endlog();
            return false;
        }
        return true;
    }
    
    else if ("/std/string" == type_name)
    {
        try {
            ti->addProtocol(orogen_transports::TYPELIB_MARSHALLER_ID,
                std_string_TypelibMarshaller(*m_registry));
        }
        catch(std::runtime_error const& e)
        {
            log(Error) << "cannot register a typelib transport for " << type_name << endlog();
            log(Error) << "  the following error occured:" << endlog();
            log(Error) << "  " << e.what() << endlog();
            log(Error) << "  the registry can be found at " << TYPEKIT_REGISTRY << endlog();
            return false;
        }
        return true;
    }
    
    else if ("/std/vector</double>" == type_name)
    {
        try {
            ti->addProtocol(orogen_transports::TYPELIB_MARSHALLER_ID,
                std_vector__double__TypelibMarshaller(*m_registry));
        }
        catch(std::runtime_error const& e)
        {
            log(Error) << "cannot register a typelib transport for " << type_name << endlog();
            log(Error) << "  the following error occured:" << endlog();
            log(Error) << "  " << e.what() << endlog();
            log(Error) << "  the registry can be found at " << TYPEKIT_REGISTRY << endlog();
            return false;
        }
        return true;
    }
    
    return false;
}

ORO_TYPEKIT_PLUGIN(orogen_typekits::ProactiveAssistanceTypelibTransportPlugin);

