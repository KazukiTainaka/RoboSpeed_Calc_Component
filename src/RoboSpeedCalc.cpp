// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  RoboSpeedCalc.cpp
 * @brief ModuleDescription
 *
 */
// </rtc-template>

#include "RoboSpeedCalc.h"

#include <chrono>
#include <thread>
# define PI 3.14159265359

// Module specification
// <rtc-template block="module_spec">
#if RTM_MAJOR_VERSION >= 2
static const char* const robospeedcalc_spec[] =
#else
static const char* robospeedcalc_spec[] =
#endif
  {
    "implementation_id", "RoboSpeedCalc",
    "type_name",         "RoboSpeedCalc",
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

float previous_v = 0;
float previous_d = 0;

static bool avoiding_right = false;
static bool avoiding_left = false;
static auto avoidance_start_time = std::chrono::high_resolution_clock::now();

//auto avoidance_start = 0.0;

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
RoboSpeedCalc::RoboSpeedCalc(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_RealSenseIn("RealSense", m_RealSense),
    m_DepthIn("Depth", m_Depth),
    m_BumperIn("Bumper", m_Bumper),
    m_VelocityOut("Velocity", m_Velocity)
    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
RoboSpeedCalc::~RoboSpeedCalc()
{
}



RTC::ReturnCode_t RoboSpeedCalc::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("RealSense", m_RealSenseIn);
  addInPort("Depth", m_DepthIn);
  addInPort("Bumper", m_BumperIn);
  
  // Set OutPort buffer
  addOutPort("Velocity", m_VelocityOut);

  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>

  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RoboSpeedCalc::onFinalize()
{
  return RTC::RTC_OK;
}
*/


//RTC::ReturnCode_t RoboSpeedCalc::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t RoboSpeedCalc::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t RoboSpeedCalc::onActivated(RTC::UniqueId /*ec_id*/)
{
  m_Bumper.data.length(8);
  m_RealSense.data.length(5);
  m_Depth.data.length(640);
  return RTC::RTC_OK;
}


RTC::ReturnCode_t RoboSpeedCalc::onDeactivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}

// ガウス分布メンバシップ関数
//class GaussianMembershipFunction {
//    float c, sigma;
//public:
//    GaussianMembershipFunction(float center, float stddev) : c(center), sigma(stddev) {}
//    float compute(float x) const {
//        return exp(-pow(x - c, 2) / (2 * pow(sigma, 2)));
//    }
//};

// 三角形メンバシップ関数
class TriangleMembershipFunction {
    double a, b, c;
public:
    TriangleMembershipFunction(float a, float b, float c) : a(a), b(b), c(c) {}
    float compute(float x) const {
        if (x <= a || x >= c) return 0.0;
        if (x == b) return 1.0;
        if (x < b) return (x - a) / (b - a);
        return (c - x) / (c - b);
    }
};

// ファジィ制御器クラス
class FuzzyController {
    TriangleMembershipFunction aLow, aHigh;
    TriangleMembershipFunction bLow, bHigh;
public:
    FuzzyController() :
        aLow(0, 2.5, 2.5), aHigh(0, 0, 2.5),
        bLow(0, 1.57, 1.57), bHigh(0, 0, 1.57) {}

    double fuzzyAnd(float x, float y) const {
        return std::min(x, y);
    }

    // ファジィルールに基づいて出力を計算
    void compute(float a, float b, float& v, float& d) const {
        // メンバシップ値を計算
        float aLowValue = aLow.compute(a);
        float aHighValue = aHigh.compute(a);
        float bLowValue = bLow.compute(fabs(b));
        float bHighValue = bHigh.compute(fabs(b));

        // ルール適用
        float rule1 = fuzzyAnd(aHighValue, bLowValue);//小さいほうの値を返す
        float rule2 = fuzzyAnd(aLowValue, bHighValue);//小さいほうの値を返す

        // デファジィフィケーション
        float velocity = 0.4;   //併進速度目安最大値
        float angular_vel = 1.0; //各速度目安最大値

        v = rule1 * velocity + rule2 * velocity; // aが大きいほどvが大きくなる
        if (b < 0) {
            d = rule1 * angular_vel + rule2 * angular_vel; // bが大きいほどdが大きくなる
        }
        else {
            d = -(rule1 * angular_vel + rule2 * angular_vel); // bが大きいほどdが大きくなる
        }
    }
};

RTC::ReturnCode_t RoboSpeedCalc::onExecute(RTC::UniqueId /*ec_id*/)
{
    /*
    std::cout << "input va >";
    std::cin >> m_Velocity.data.va;

    std::cout << "input vy >";
    std::cin >> m_Velocity.data.vy;
    */

    FuzzyController controller;

    if (m_RealSenseIn.isNew() || m_BumperIn.isNew()) {
        m_BumperIn.read();
        m_RealSenseIn.read();
        //std::cout << "Receive " << m_Bumper.data[1] << ", "<< m_RealSense.data[2] << "m, " << m_RealSense.data[3] << "deg" << std::endl;

        // 右バンパがtrueになったら左に回避行動を開始
        if (m_Bumper.data[0] == true && !avoiding_right) {
            avoidance_start_time = std::chrono::high_resolution_clock::now();
            avoiding_right = true;
            printf("右バンパー接触\n");
        }
        // 左に回避行動中の場合
        if (avoiding_right) {
            auto now = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - avoidance_start_time);

            if (duration.count() < 1) {
                m_Velocity.data.vx = -0.3 /3;
                m_Velocity.data.va = 0;
                m_VelocityOut.write();
                return RTC::RTC_OK;
            }
            if (duration.count() < 4 && duration.count() >= 1) {
                m_Velocity.data.vx = 0;
                m_Velocity.data.va = (PI / 6) / 3;
                m_VelocityOut.write();
                return RTC::RTC_OK;
            }
            /*if (duration.count() < 7 && duration.count() >= 4) {
                m_Velocity.data.vx = 0.3 /3;
                m_Velocity.data.va = 0;
                m_VelocityOut.write();
                return RTC::RTC_OK;
            }
            if (duration.count() < 10 && duration.count() >= 7) {
                m_Velocity.data.vx = 0; 
                m_Velocity.data.va = - (PI / 2) / 3;
                m_VelocityOut.write();
                return RTC::RTC_OK;
            }*/
            else {
                // t秒経過後に回避行動を終了
                avoiding_right = false;
            }
            
        }
        // 左バンパがtrueになったら右に回避行動を開始
        if (m_Bumper.data[2] == true && !avoiding_left) {
            avoidance_start_time = std::chrono::high_resolution_clock::now();
            avoiding_left = true;
            printf("左バンパー接触\n");
        }
        // 右に回避行動中の場合
        if (avoiding_left) {
            auto now = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - avoidance_start_time);
            if (duration.count() < 1) {
                m_Velocity.data.vx = -0.3 / 3;
                m_Velocity.data.va = 0;
                m_VelocityOut.write();
                return RTC::RTC_OK;
            }
            if (duration.count() < 4 && duration.count() >= 1) {
                m_Velocity.data.vx = 0;
                m_Velocity.data.va = -(PI / 6) / 3;
                m_VelocityOut.write();
                return RTC::RTC_OK;
            }
            /*if (duration.count() < 7 && duration.count() >= 4) {
                m_Velocity.data.vx = 0.3 / 3;
                m_Velocity.data.va = 0;
                m_VelocityOut.write();
                return RTC::RTC_OK;
            }
            if (duration.count() < 10 && duration.count() >= 7) {
                m_Velocity.data.vx = 0;
                m_Velocity.data.va = (PI / 2) / 3;
                m_VelocityOut.write();
                return RTC::RTC_OK;
            }*/
            else {
                avoiding_left = false;
            }
        }
        // 回避行動をしていない場合の通常の動作
        float a = m_RealSense.data[2]; // 入力distance
        float b = m_RealSense.data[3] * PI / 180; // 入力angle_x

        float v, d;
        controller.compute(a, b, v, d);


        std::cout << "Input a: " << a << ", Input b: " << b << std::endl;
        std::cout << "Output v: " << v << std::endl;
        std::cout << "Output d: " << d << std::endl;


        if (v == 0 && d == 0) {
            m_Velocity.data.vx = previous_v * 0.99;
            m_Velocity.data.va = previous_d * 0.9;
            previous_v = previous_v * 0.99;
            previous_d = previous_d * 0.9;
        }
        else if(previous_v < 0.05){ //急発進の抑制
            m_Velocity.data.vx = v * 0.3;
            m_Velocity.data.va = d * 0.3;
            previous_v = v * 0.3;
            previous_d = d * 0.3;
        }
        else {
            m_Velocity.data.vx = v;
            m_Velocity.data.va = d;
            previous_v = v;
            previous_d = d;
        }

        m_VelocityOut.write();
        std::cout << "Sent " << m_Velocity.data.va << "rad/s, " << m_Velocity.data.vx << "m/s" << std::endl;

    }
    else {      //マーカーを見失った場合、少しずつ減速して停止
    m_Velocity.data.vx = previous_v * 0.99;
    m_Velocity.data.va = previous_d * 0.9;
    previous_v = previous_v * 0.99;
    previous_d = previous_d * 0.9;
    m_VelocityOut.write();
    }

  return RTC::RTC_OK;
}


//RTC::ReturnCode_t RoboSpeedCalc::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t RoboSpeedCalc::onError(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t RoboSpeedCalc::onReset(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t RoboSpeedCalc::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t RoboSpeedCalc::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}



extern "C"
{
 
  void RoboSpeedCalcInit(RTC::Manager* manager)
  {
    coil::Properties profile(robospeedcalc_spec);
    manager->registerFactory(profile,
                             RTC::Create<RoboSpeedCalc>,
                             RTC::Delete<RoboSpeedCalc>);
  }
  
}
