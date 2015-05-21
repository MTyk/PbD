/* Generated from orogen/lib/orogen/templates/typekit/Plugin.hpp */

#ifndef __OROGEN_GENERATED_PROACTIVEASSISTANCE_TYPEKIT_HPP
#define __OROGEN_GENERATED_PROACTIVEASSISTANCE_TYPEKIT_HPP

#include <rtt/types/TypekitPlugin.hpp>

namespace Typelib {
    class Registry;
}

namespace orogen_typekits {
    class ProactiveAssistanceTypekitPlugin
        : public RTT::types::TypekitPlugin
    {
        Typelib::Registry* m_registry;

    public:
        ProactiveAssistanceTypekitPlugin();
        ~ProactiveAssistanceTypekitPlugin();
        bool loadTypes();
        bool loadOperators();
        bool loadConstructors();
        std::string getName();
    };

    extern ProactiveAssistanceTypekitPlugin ProactiveAssistanceTypekit;
}

#endif


