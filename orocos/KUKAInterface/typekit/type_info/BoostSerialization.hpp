/* Generated from orogen/lib/orogen/templates/typekit/BoostSerialization.hpp */

#ifndef __OROGEN_GENERATED_KUKAINTERFACE_BOOST_SERIALIZATION_HPP
#define __OROGEN_GENERATED_KUKAINTERFACE_BOOST_SERIALIZATION_HPP

#include <KUKAInterface/Types.hpp>

#include <boost/cstdint.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/deque.hpp>
#include <boost/serialization/utility.hpp>


namespace boost
{
    namespace serialization
    {

        template<typename Archive>
        void serialize(Archive& a, KUKAInterfaceData& b, unsigned int version)
        {
            using boost::serialization::make_nvp;
            a & make_nvp("samples", b.samples);
        }

    }
}


#endif

