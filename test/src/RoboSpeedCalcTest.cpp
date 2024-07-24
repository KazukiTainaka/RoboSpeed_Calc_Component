// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  RoboSpeedCalcTest.cpp
 * @brief ModuleDescription (test code)
 *
 */
// </rtc-template>

#include "RoboSpeedCalcTest.h"

// Module specification
// <rtc-template block="module_spec">
#if RTM_MAJOR_VERSION >= 2
static const char* const robospeedcalc_spec[] =
#else
static const char* robospeedcalc_spec[] =
#endif
  {
    "implementation_id", "RoboSpeedCalcTest",
    "type_name",         "RoboSpeedCalcTest",
    "description",       "ModuleDescription",
    "version",           "1.0.0",
    "vendor",            "VenderName",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
RoboSpeedCalcTest::RoboSpeedCalcTest(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_RealSenseOut("RealSense", m_RealSense),
    m_DepthOut("Depth", m_Depth),
    m_BumperOut("Bumper", m_Bumper),
    m_VelocityIn("Velocity", m_Velocity)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
RoboSpeedCalcTest::~RoboSpeedCalcTest()
{
}



RTC::ReturnCode_t RoboSpeedCalcTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("Velocity", m_VelocityIn);
  
  // Set OutPort buffer
  addOutPort("RealSense", m_RealSenseOut);
  addOutPort("Depth", m_DepthOut);
  addOutPort("Bumper", m_BumperOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RoboSpeedCalcTest::onFinalize()
{
  return RTC::RTC_OK;
}
*/


//RTC::ReturnCode_t RoboSpeedCalcTest::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t RoboSpeedCalcTest::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t RoboSpeedCalcTest::onActivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t RoboSpeedCalcTest::onDeactivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t RoboSpeedCalcTest::onExecute(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t RoboSpeedCalcTest::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t RoboSpeedCalcTest::onError(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t RoboSpeedCalcTest::onReset(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t RoboSpeedCalcTest::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t RoboSpeedCalcTest::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


bool RoboSpeedCalcTest::runTest()
{
    return true;
}


extern "C"
{
 
  void RoboSpeedCalcTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(robospeedcalc_spec);
    manager->registerFactory(profile,
                             RTC::Create<RoboSpeedCalcTest>,
                             RTC::Delete<RoboSpeedCalcTest>);
  }
  
}
