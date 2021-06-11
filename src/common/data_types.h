/*
 * Author: BlindZhou
 */

#pragma once

#include <stdint.h>
#include <array>
#include <vector>
//#include <boost/archive/binary_iarchive.hpp> //二进制反序列化
//#include <boost/archive/binary_oarchive.hpp> //二进制序列化
//#include <boost/serialization/vector.hpp>
//#include <boost/serialization/array.hpp>
#include "common/geometry.h"
//#include "third_libs/rest_rpc/include/rest_rpc.hpp"

typedef struct {
    struct timeval tv;
    struct event * ev;
    void* p_args;
}EventParam;

namespace bz_robot
{
    typedef float FLOAT_T;

    enum ReturnStatus
    {
        RET_SUCCESS = 0,
        RET_TRUE = RET_SUCCESS,
        RET_FAILED = 1,
        RET_FALSE = RET_FAILED,
        RET_ERROR = 2,
        RET_TIME_OUT = 3,   //TIME_OUT表示超过规定时间还没执行完
        RET_IS_BUSY = 4     //BUSY表示正在上次还在运行，未完成，又来了一次新的请求
    };



    template<typename T>
    class Msg
    {
    public:
        int64_t time_stamp_us;
        T data;
    public:
        //MSGPACK_DEFINE(time_stamp_us, data);
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & time_stamp_us;
            ar & data;
        }
    };

    template<typename T>
    class RetMsg
    {
    public:
        ReturnStatus return_status;
        Msg<T> msg;
    public:
        //MSGPACK_DEFINE((int&)return_status, msg);
    };

    typedef std::vector<Pose<FLOAT_T>> PathData;

    typedef std::vector<VectorX2<FLOAT_T>> FootPrintData;

    class ControlData
    {
    public:
        FLOAT_T dt;
        FLOAT_T velocity;
        FLOAT_T steer_angle;
    public:
        //MSGPACK_DEFINE(dt, velocity, steer_angle);
//        template<class Archive>
//        void serialize(Archive & ar, const unsigned int version)
//        {
//            ar & dt;
//            ar & velocity;
//            ar & steer_angle;
//        }
    };

    struct LaserData
    {
        std::string name;
        Msg<Pose<FLOAT_T>> msg_location;
        std::vector<VectorX3<FLOAT_T>> points;
    };

    struct Laser2DData
    {
        std::string name;
        Msg<Pose<FLOAT_T>> msg_location;
        VectorX3<FLOAT_T> pose_offset;
        VectorX3<FLOAT_T> angle_offset;
        FLOAT_T start_angle = 0;
        FLOAT_T angle_step = 0;
        std::vector<FLOAT_T> distance_list;
    };

    struct Laser3DData
    {
        std::string name;
        Msg<Pose<FLOAT_T>> msg_location;
        VectorX3<FLOAT_T> pose_offset;
        VectorX3<FLOAT_T> angle_offset;
        std::vector<VectorX3<FLOAT_T>> points;
        //MSGPACK_DEFINE(name, msg_location, pose_offset, angle_offset, points);
    };

    struct RobotInfo
    {
        Pose<FLOAT_T> pose;
        ControlData control_data;
        std::string mode;
        std::string status;
        int battery;
    };
}

//MSGPACK_ADD_ENUM(bz_robot::ReturnStatus);

