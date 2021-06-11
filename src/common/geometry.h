#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#include <math.h>
//#include <boost/archive/binary_iarchive.hpp> //二进制反序列化
//#include <boost/archive/binary_oarchive.hpp> //二进制序列化
//#include "third_libs/rest_rpc/include/rest_rpc.hpp"


class CloudPoint
{
public:
    inline CloudPoint (const CloudPoint &p)
    {
        x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
    }

    inline CloudPoint ()
    {
        x = y = z = 0.0f;
        data[3] = 1.0f;
    }

    inline CloudPoint (float _x, float _y, float _z)
    {
        x = _x; y = _y; z = _z;
        data[3] = 1.0f;
    }
    //MSGPACK_DEFINE(data);
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & x;
        ar & y;
        ar & z;
        ar & data[3];
    }

    union
    {
        float data[4];
        struct {
          float x;
          float y;
          float z;
        };
    };
};

template <class T>
class VectorX2
{
public:
    T x;
    T y;
    VectorX2(T xi = 0, T yi = 0): x(xi), y(yi) {};

    //MSGPACK_DEFINE(x, y);
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & x;
        ar & y;
    }

    VectorX2 operator*(const double &in_value) const
    {
        VectorX2 value;
        value.x = this->x * in_value;
        value.y = this->y * in_value;
        return value;
    }

    VectorX2 operator+(const VectorX2 &in_value) const
    {
        VectorX2 value;
        value.x = in_value.x + this->x;
        value.y = in_value.y + this->y;
        return value;
    }
    VectorX2 operator+=(const VectorX2 &in_value)
    {
        this->x += in_value.x;
        this->y += in_value.y;
        return *this;
    }
    VectorX2 operator-(const VectorX2 &in_value) const
    {
        VectorX2 value;
        value.x = this->x - in_value.x;
        value.y = this->y - in_value.y;
        return value;
    }
    VectorX2 operator-() const
    {
        VectorX2 value;
        value.x = -this->x;
        value.y = -this->y;
        return value;
    }
    bool operator==(const VectorX2 &in_value) const
    {
        bool result = false;

        if(this->x == in_value.x &&
                this->y == in_value.y)
        {
            result = true;
        }

        return result;
    }
    bool operator!=(const VectorX2 &in_value) const
    {
        return !this->operator==(in_value);
    }
};

template <class T>
bool operator==(const VectorX2<T> &p1, const VectorX2<T> &p2)
{
    bool result = false;

    if(p1.x == p2.x &&
            p1.y == p2.y)
    {
        result = true;
    }

    return result;
}

template <class T>
bool operator<(const VectorX2<T> &p1, const VectorX2<T> &p2)
{
    if(p1.x != p2.x)
    {
        return p1.x < p2.x;
    }
    else if (p1.y != p2.y)
    {
        return p1.y < p2.y;
    }
    return false;
}

template <class T>
bool operator!=(const VectorX2<T> &p1, const VectorX2<T> &p2)
{
    return !operator==(p1, p2);
}
#if 1
template <class T>
class VectorX3
{
public:
    T x;
    T y;
    T z;
    VectorX3(T xi = 0, T yi = 0, T zi = 0): x(xi), y(yi), z(zi) {};

    //MSGPACK_DEFINE(x, y, z);
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & x;
        ar & y;
        ar & z;
    }

    VectorX3 operator+(const VectorX3 &in_value)
    {
        VectorX3 value;
        value.x = in_value.x + this->x;
        value.y = in_value.y + this->y;
        value.z = in_value.y + this->z;
        return value;
    }
    VectorX3 operator-(const VectorX3 &in_value) const
    {
        VectorX3 value;
        value.x = this->x - in_value.x;
        value.y = this->y - in_value.y;
        value.z = this->z - in_value.z;
        return value;
    }
    VectorX3 operator-()
    {
        VectorX3 value;
        value.x = -this->x;
        value.y = -this->y;
        value.z = -this->z;
        return value;
    }
    bool operator==(const VectorX3 &in_value)
    {
        bool result = false;

        if(this->x == in_value.x &&
                this->y == in_value.y &&
                this->z == in_value.z)
        {
            result = true;
        }

        return result;
    }
    bool operator!=(const VectorX3 &in_value)
    {
        return !this->operator==(in_value);
    }
};

template <class T>
bool operator==(const VectorX3<T> &p1, const VectorX3<T> &p2)
{
    bool result = false;

    if(p1.x == p2.x &&
            p1.y == p2.y &&
            p1.z == p2.z)
    {
        result = true;
    }

    return result;
}

template <class T>
bool operator!=(const VectorX3<T> &p1, const VectorX3<T> &p2)
{
    return !operator==(p1, p2);
}

template <class T>
bool operator<(const VectorX3<T> &p1, const VectorX3<T> &p2)
{
    if(p1.x != p2.x)
    {
        return p1.x < p2.x;
    }
    else if (p1.y != p2.y)
    {
        return p1.y < p2.y;
    }
    else
    {
        return p1.z < p2.z;
    }
    return false;
}

#endif

template <class T>
class Pose
{
public:
    VectorX2<T> position;
    T heading_angle;
    Pose(T xi = 0, T yi = 0, double angle = 0)
    {
        position.x = xi;
        position.y = yi;
        heading_angle = angle;
    };

    //MSGPACK_DEFINE(position, heading_angle);
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & position;
        ar & heading_angle;
    }
};

template <class T>
bool operator==(const Pose<T> &p1, const Pose<T> &p2)
{
    bool result = false;

    if(p1.position == p2.position &&
            round(p1.heading_angle * 1000) == round(p2.heading_angle * 1000))
    {
        result = true;
    }

    return result;
}

template <class T>
bool operator!=(const Pose<T> &p1, const Pose<T> &p2)
{
    return !operator==(p1, p2);
}

#endif
