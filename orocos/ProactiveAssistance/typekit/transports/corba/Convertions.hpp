/* Generated from orogen/lib/orogen/templates/typekit/corba/Convertions.hpp */

#ifndef __OROGEN_GENERATED_PROACTIVEASSISTANCE_CORBA_CONVERTIONS_HPP
#define __OROGEN_GENERATED_PROACTIVEASSISTANCE_CORBA_CONVERTIONS_HPP

#include "../../Types.hpp"
#include "ProactiveAssistance/transports/corba/ProactiveAssistanceTypesC.h"
#include <boost/cstdint.hpp>
#include <string>

namespace orogen_typekits {
    /** Converted types: */
    
    bool toCORBA( orogen::Corba::ProactiveAssistanceData& corba, ProactiveAssistanceData const& value );
    bool fromCORBA( ProactiveAssistanceData& value, orogen::Corba::ProactiveAssistanceData const& corba );
    
    bool toCORBA( char const*& corba, ::std::string const& value );
    bool fromCORBA( ::std::string& value, char const* corba );
    
    bool toCORBA( orogen::Corba::vector__double_& corba, ::std::vector< double > const& value );
    bool fromCORBA( ::std::vector< double >& value, orogen::Corba::vector__double_ const& corba );
    
    /** Array types: */
    
}

#endif

