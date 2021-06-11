#include <math.h>
#include <iostream>
#include <string>
#include <xmlrpc-c/base.hpp>
#include <xmlrpc-c/client_simple.hpp>
#include "body_control.h"
#include "common/time_keeper.h"

namespace bz_robot
{
using namespace std;
using namespace xmlrpc_c;


std::string BodyControl::server_url = "http://192.168.1.110:20000/RPC2";

bool BodyControl::set_control_mode(int mode)
{
    //return true;
    static const string method_name("SetControlMode");
    xmlrpc_c::clientSimple myClient;
    xmlrpc_c::value result;
    xmlrpc_c::paramList paramLst;
    paramLst.add(xmlrpc_c::value_int(mode));
    myClient.call(server_url, method_name, paramLst, &result);
    return bool(xmlrpc_c::value_boolean(result));
}

int BodyControl::get_control_mode()
{
    //return true;
    static const string method_name("GetControlMode");
    xmlrpc_c::clientSimple myClient;
    xmlrpc_c::value result;;
    myClient.call(server_url, method_name, &result);
    return bool(xmlrpc_c::value_int(result));
}

bool BodyControl::e_stop(int mode)
{
    bool status = false;
    try
    {
        static const string method_name("EStop");
        xmlrpc_c::clientSimple myClient;
        xmlrpc_c::value result;
        xmlrpc_c::paramList paramLst;
        paramLst.add(xmlrpc_c::value_int(mode));
        myClient.call(server_url, method_name, paramLst, &result);
        status = bool(xmlrpc_c::value_boolean(result));
    }
    catch (exception const& e) 
    {
        status = false;
        cerr << "Something failed.  " << e.what() << endl;
    }
    return status;

}
bool BodyControl::stop(int mode)
{
    bool status = false;
    try
    {
        static const string method_name("Stop");
        xmlrpc_c::clientSimple myClient;
        xmlrpc_c::value result;
        xmlrpc_c::paramList paramLst;
        paramLst.add(xmlrpc_c::value_int(mode));
        myClient.call(server_url, method_name, paramLst, &result);
        status = bool(xmlrpc_c::value_boolean(result));     
    }     
    catch (exception const& e)      
    {   
        status = false;
        cerr << "Something failed.  " << e.what() << endl;     
    }     
    return status;
}
bool BodyControl::begin(int mode)
{
    bool status = false;
    try
    {
        static const string method_name("Begin");
        xmlrpc_c::clientSimple myClient;
        xmlrpc_c::value result;
        xmlrpc_c::paramList paramLst;
        paramLst.add(xmlrpc_c::value_int(mode));
        myClient.call(server_url, method_name, paramLst, &result);
        status = bool(xmlrpc_c::value_boolean(result));     
    }     
    catch (exception const& e)      
    {
        status = false;
        cerr << "Something failed.  " << e.what() << endl;     
    }     
    return status;
}
bool BodyControl::is_ok()
{
    bool status = false;
    try
    {
        static const string method_name("IsOk");
        xmlrpc_c::clientSimple myClient;
        xmlrpc_c::value result;
        myClient.call(server_url, method_name, &result);
        status = bool(xmlrpc_c::value_boolean(result));     
    } 
    catch (exception const& e)      
    {
        status = false;
        cerr << "Something failed.  " << e.what() << endl;
    }
    return status;
}

bool BodyControl::set_velocity_and_angle(float linear_velocity, float angle, int mode)
{
    bool status = false;
    try
    {
        static const string method_name("SetSpeedAngle");
        xmlrpc_c::clientSimple myClient;
        xmlrpc_c::value result;
        xmlrpc_c::paramList paramLst;
        paramLst.add(xmlrpc_c::value_int(mode));
        int velocity = int(linear_velocity * 1000);
        int angle_d = round(angle * 10 * 180 * M_1_PI);
        paramLst.add(xmlrpc_c::value_int(velocity));
        paramLst.add(xmlrpc_c::value_int(angle_d));
        myClient.call(server_url, method_name, paramLst, &result);
        status = bool(xmlrpc_c::value_boolean(result));
    }
    catch (exception const& e)
    {
        status = false;
        cerr << "Something failed.  " << e.what() << endl;
    }
    return status;
}

bool BodyControl::set_linear_velocity(float linear_velocity, int mode)
{
    bool status = false;
    try
    {
        static const string method_name("SetVelocity");
        xmlrpc_c::clientSimple myClient;
        xmlrpc_c::value result;
        xmlrpc_c::paramList paramLst;
        paramLst.add(xmlrpc_c::value_int(mode));
        //printf("linear_velocity = %f, int(linear_velocity * 1000) = {:d}\n", linear_velocity, int(linear_velocity * 1000));
        int velocity = int(linear_velocity * 1000);
        //printf("xmlrpc velocity = {:d}\n", velocity);
        paramLst.add(xmlrpc_c::value_int(velocity));
        myClient.call(server_url, method_name, paramLst, &result);
        status = bool(xmlrpc_c::value_boolean(result));     
    }
    catch (exception const& e)      
    {         
        status = false;
        cerr << "Something failed.  " << e.what() << endl;     
    }     
    return status;
}

bool BodyControl::set_angle(float angle, int mode)
{
    bool status = false;
    try
    {
        //printf("recv cmd\n");
        static const string method_name("SetAngle");
        xmlrpc_c::clientSimple myClient;
        xmlrpc_c::value result;
        xmlrpc_c::paramList paramLst;
        paramLst.add(xmlrpc_c::value_int(mode));
        //弧度转角度
        int angle_d = round(angle * 10 * 180 * M_1_PI);
        paramLst.add(xmlrpc_c::value_int(angle_d));
        myClient.call(server_url, method_name, paramLst, &result);
        status = bool(xmlrpc_c::value_boolean(result));

        //printf("angle = {:.3f}({:d}), mode = {:d}, status = {:d}\n", angle, angle_d, mode, status);

    }     
    catch (exception const& e)      
    {         
        status = false;
        cerr << "Something failed.  " << e.what() << endl;     
    }     
    return status;
}

bool BodyControl::reset_error_state(int mode)
{
    bool status = false;
    try
    {
        static const string method_name("ResetErrorState");
        xmlrpc_c::clientSimple myClient;
        xmlrpc_c::value result;
        xmlrpc_c::paramList paramLst;
        paramLst.add(xmlrpc_c::value_int(mode));
        myClient.call(server_url, method_name, paramLst, &result);
        status = bool(xmlrpc_c::value_boolean(result));     
    }
    catch (exception const& e)      
    {         
        status = false;
        cerr << "Something failed.  " << e.what() << endl;     
    }     
    return status;
}
bool BodyControl::reset_warning_state(int mode)
{
    bool status = false;
    try
    {
        static const string method_name("ResetWarningState");
        xmlrpc_c::clientSimple myClient;
        xmlrpc_c::value result;
        xmlrpc_c::paramList paramLst;
        paramLst.add(xmlrpc_c::value_int(mode));
        myClient.call(server_url, method_name, paramLst, &result);
        status = bool(xmlrpc_c::value_boolean(result));     
    }     
    catch (exception const& e)      
    {         
        status = false;
        cerr << "Something failed.  " << e.what() << endl;     
    }     
    return status;
}
bool BodyControl::set_safe_mode_on(int mode)
{
    bool status = false;
    try
    {
        static const string method_name("SetSafeModeON");
        xmlrpc_c::clientSimple myClient;
        xmlrpc_c::value result;
        xmlrpc_c::paramList paramLst;
        paramLst.add(xmlrpc_c::value_int(mode));
        myClient.call(server_url, method_name, paramLst, &result);
        status = bool(xmlrpc_c::value_boolean(result));     
    }
    catch (exception const& e)      
    {         
        status = false;
        cerr << "Something failed.  " << e.what() << endl;     
    }     
    return status;
}
bool BodyControl::set_safe_mode_off(int mode)
{
    bool status = false;
    try
    {
        static const string method_name("SetSafeModeOFF");
        xmlrpc_c::clientSimple myClient;
        xmlrpc_c::value result;
        xmlrpc_c::paramList paramLst;
        paramLst.add(xmlrpc_c::value_int(mode));
        myClient.call(server_url, method_name, paramLst, &result);
        status = bool(xmlrpc_c::value_boolean(result));     
    }     
    catch (exception const& e)      
    {         
        status = false;
        cerr << "Something failed.  " << e.what() << endl;     
    }     
    return status;
}

void BodyControl::exit(int mode)
{
    try
    {
        static const string method_name("Exit");
        xmlrpc_c::clientSimple myClient;
        xmlrpc_c::value result;
        xmlrpc_c::paramList paramLst;
        paramLst.add(xmlrpc_c::value_int(mode));
        myClient.call(server_url, method_name, paramLst, &result);
    }
    catch (exception const& e)      
    {         
        cerr << "Something failed.  " << e.what() << endl;     
    }
}

int BodyControl::feedback_velocity()
{
    int value = 0;
    try
    {
        static const string method_name("GetFeedbackSpeed");
        xmlrpc_c::clientSimple myClient;
        xmlrpc_c::value result;
        xmlrpc_c::paramList paramLst;
        myClient.call(server_url, method_name, paramLst, &result);
        value = int(xmlrpc_c::value_int(result));
    }
    catch (exception const& e)
    {
        cerr << "Something failed.  " << e.what() << endl;
    }
    return value;
}

int BodyControl::feedback_angle()
{
    int value = 0;
    try
    {
        static const string method_name("GetFeedbackAngle");
        xmlrpc_c::clientSimple myClient;
        xmlrpc_c::value result;
        xmlrpc_c::paramList paramLst;
        myClient.call(server_url, method_name, paramLst, &result);
        value = int(xmlrpc_c::value_int(result));
    }
    catch (exception const& e)
    {
        cerr << "Something failed.  " << e.what() << endl;
    }
    return value;
}

int BodyControl::left_wheel_encoder()
{
    int value = 0;
    try
    {
        static const string method_name("GetLeftWheelEncoder");
        xmlrpc_c::clientSimple myClient;
        xmlrpc_c::value result;
        xmlrpc_c::paramList paramLst;
        myClient.call(server_url, method_name, paramLst, &result);
        value = int(xmlrpc_c::value_int(result));
    }
    catch (exception const& e)
    {
        cerr << "Something failed.  " << e.what() << endl;
    }
    return value;
}

int BodyControl::right_wheel_encoder()
{
    int value = 0;
    try
    {
        static const string method_name("GetRightWheelEncoder");
        xmlrpc_c::clientSimple myClient;
        xmlrpc_c::value result;
        xmlrpc_c::paramList paramLst;
        myClient.call(server_url, method_name, paramLst, &result);
        value = int(xmlrpc_c::value_int(result));
    }
    catch (exception const& e)
    {
        cerr << "Something failed.  " << e.what() << endl;
    }
    return value;
}


}
