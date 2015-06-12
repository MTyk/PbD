/* Generated from orogen/lib/orogen/templates/typekit/corba/Convertions.hpp */

#ifndef __OROGEN_GENERATED_KUKAINTERFACE_CORBA_CONVERTIONS_HPP
#define __OROGEN_GENERATED_KUKAINTERFACE_CORBA_CONVERTIONS_HPP

#include "../../Types.hpp"
#include "KUKAInterface/transports/corba/KUKAInterfaceTypesC.h"
#include <boost/cstdint.hpp>
#include <string>

namespace orogen_typekits {
    /** Converted types: */
    
    bool toCORBA( orogen::Corba::KUKAInterfaceData& corba, KUKAInterfaceData const& value );
    bool fromCORBA( KUKAInterfaceData& value, orogen::Corba::KUKAInterfaceData const& corba );
    
    bool toCORBA( char const*& corba, ::std::string const& value );
    bool fromCORBA( ::std::string& value, char const* corba );
    
    bool toCORBA( orogen::Corba::vector__double_& corba, ::std::vector< double > const& value );
    bool fromCORBA( ::std::vector< double >& value, orogen::Corba::vector__double_ const& corba );
    
    /** Array types: */
    
}

#endif

