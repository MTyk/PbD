/* Generated from orogen/lib/orogen/templates/typekit/mqueue/TransportPlugin.hpp */

#ifndef __OROGEN_GENERATED_KUKAINTERFACE_MQUEUE_PLUGIN_HPP
#define __OROGEN_GENERATED_KUKAINTERFACE_MQUEUE_PLUGIN_HPP

#include <rtt/types/TransportPlugin.hpp>

namespace Typelib {
    class Registry;
}
namespace orogen_typekits {
    class KUKAInterfaceMQueueTransportPlugin
        : public RTT::types::TransportPlugin
    {
        Typelib::Registry* m_registry;

    public:
        KUKAInterfaceMQueueTransportPlugin();
        ~KUKAInterfaceMQueueTransportPlugin();

        virtual bool registerTransport(std::string type_name, RTT::types::TypeInfo* ti);
        virtual std::string getTransportName() const;
        virtual std::string getTypekitName() const;
        virtual std::string getName() const;
    };

    extern KUKAInterfaceMQueueTransportPlugin KUKAInterfaceMQueueTransport;
}

#endif

