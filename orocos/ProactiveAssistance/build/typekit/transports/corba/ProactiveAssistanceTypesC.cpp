// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.

#include "ProactiveAssistanceTypesC.h"
#include <omniORB4/IOP_S.h>
#include <omniORB4/IOP_C.h>
#include <omniORB4/callDescriptor.h>
#include <omniORB4/callHandle.h>
#include <omniORB4/objTracker.h>


OMNI_USING_NAMESPACE(omni)

static const char* _0RL_library_version = omniORB_4_1;



void
orogen::Corba::ProactiveAssistanceData::operator>>= (cdrStream &_n) const
{
  (const vector__double_&) samples >>= _n;

}

void
orogen::Corba::ProactiveAssistanceData::operator<<= (cdrStream &_n)
{
  (vector__double_&)samples <<= _n;

}
