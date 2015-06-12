/* Generated from orogen/lib/orogen/templates/typekit/Plugin.hpp */

#ifndef __OROGEN_GENERATED_KUKAINTERFACE_TYPEKIT_HPP
#define __OROGEN_GENERATED_KUKAINTERFACE_TYPEKIT_HPP

#include <rtt/types/TypekitPlugin.hpp>

namespace Typelib {
    class Registry;
}

namespace orogen_typekits {
    class KUKAInterfaceTypekitPlugin
        : public RTT::types::TypekitPlugin
    {
        Typelib::Registry* m_registry;

    public:
        KUKAInterfaceTypekitPlugin();
        ~KUKAInterfaceTypekitPlugin();
        bool loadTypes();
        bool loadOperators();
        bool loadConstructors();
        std::string getName();
    };

    extern KUKAInterfaceTypekitPlugin KUKAInterfaceTypekit;
}

#endif


