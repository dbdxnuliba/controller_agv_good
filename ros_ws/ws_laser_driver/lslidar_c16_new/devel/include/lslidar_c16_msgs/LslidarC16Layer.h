// Generated by gencpp from file lslidar_c16_msgs/LslidarC16Layer.msg
// DO NOT EDIT!


#ifndef LSLIDAR_C16_MSGS_MESSAGE_LSLIDARC16LAYER_H
#define LSLIDAR_C16_MSGS_MESSAGE_LSLIDARC16LAYER_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <sensor_msgs/LaserScan.h>

namespace lslidar_c16_msgs
{
template <class ContainerAllocator>
struct LslidarC16Layer_
{
  typedef LslidarC16Layer_<ContainerAllocator> Type;

  LslidarC16Layer_()
    : scan_channel()  {
    }
  LslidarC16Layer_(const ContainerAllocator& _alloc)
    : scan_channel()  {
  (void)_alloc;
      scan_channel.assign( ::sensor_msgs::LaserScan_<ContainerAllocator> (_alloc));
  }



   typedef boost::array< ::sensor_msgs::LaserScan_<ContainerAllocator> , 16>  _scan_channel_type;
  _scan_channel_type scan_channel;





  typedef boost::shared_ptr< ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator> const> ConstPtr;

}; // struct LslidarC16Layer_

typedef ::lslidar_c16_msgs::LslidarC16Layer_<std::allocator<void> > LslidarC16Layer;

typedef boost::shared_ptr< ::lslidar_c16_msgs::LslidarC16Layer > LslidarC16LayerPtr;
typedef boost::shared_ptr< ::lslidar_c16_msgs::LslidarC16Layer const> LslidarC16LayerConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator1> & lhs, const ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator2> & rhs)
{
  return lhs.scan_channel == rhs.scan_channel;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator1> & lhs, const ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace lslidar_c16_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator> >
{
  static const char* value()
  {
    return "68a91988942756cb30282e7a5cad7fbd";
  }

  static const char* value(const ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x68a91988942756cbULL;
  static const uint64_t static_value2 = 0x30282e7a5cad7fbdULL;
};

template<class ContainerAllocator>
struct DataType< ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lslidar_c16_msgs/LslidarC16Layer";
  }

  static const char* value(const ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Header header\n"
"# The 0th scan is at the bottom\n"
"sensor_msgs/LaserScan[16] scan_channel\n"
"================================================================================\n"
"MSG: sensor_msgs/LaserScan\n"
"# Single scan from a planar laser range-finder\n"
"#\n"
"# If you have another ranging device with different behavior (e.g. a sonar\n"
"# array), please find or create a different message, since applications\n"
"# will make fairly laser-specific assumptions about this data\n"
"\n"
"Header header            # timestamp in the header is the acquisition time of \n"
"                         # the first ray in the scan.\n"
"                         #\n"
"                         # in frame frame_id, angles are measured around \n"
"                         # the positive Z axis (counterclockwise, if Z is up)\n"
"                         # with zero angle being forward along the x axis\n"
"                         \n"
"float32 angle_min        # start angle of the scan [rad]\n"
"float32 angle_max        # end angle of the scan [rad]\n"
"float32 angle_increment  # angular distance between measurements [rad]\n"
"\n"
"float32 time_increment   # time between measurements [seconds] - if your scanner\n"
"                         # is moving, this will be used in interpolating position\n"
"                         # of 3d points\n"
"float32 scan_time        # time between scans [seconds]\n"
"\n"
"float32 range_min        # minimum range value [m]\n"
"float32 range_max        # maximum range value [m]\n"
"\n"
"float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)\n"
"float32[] intensities    # intensity data [device-specific units].  If your\n"
"                         # device does not provide intensities, please leave\n"
"                         # the array empty.\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.scan_channel);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LslidarC16Layer_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lslidar_c16_msgs::LslidarC16Layer_<ContainerAllocator>& v)
  {
    s << indent << "scan_channel[]" << std::endl;
    for (size_t i = 0; i < v.scan_channel.size(); ++i)
    {
      s << indent << "  scan_channel[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::sensor_msgs::LaserScan_<ContainerAllocator> >::stream(s, indent + "    ", v.scan_channel[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // LSLIDAR_C16_MSGS_MESSAGE_LSLIDARC16LAYER_H
