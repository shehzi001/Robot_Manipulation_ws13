/* Auto-generated by genmsg_cpp for file /home/shehzad/catkin_ws/bosch_drivers/amtec/msg/AmtecState.msg */
#ifndef AMTEC_MESSAGE_AMTECSTATE_H
#define AMTEC_MESSAGE_AMTECSTATE_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"

namespace amtec
{
template <class ContainerAllocator>
struct AmtecState_ {
  typedef AmtecState_<ContainerAllocator> Type;

  AmtecState_()
  : header()
  , state(0)
  , position(0.0)
  , velocity(0.0)
  {
  }

  AmtecState_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , state(0)
  , position(0.0)
  , velocity(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef uint32_t _state_type;
  uint32_t state;

  typedef double _position_type;
  double position;

  typedef double _velocity_type;
  double velocity;


  typedef boost::shared_ptr< ::amtec::AmtecState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::amtec::AmtecState_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct AmtecState
typedef  ::amtec::AmtecState_<std::allocator<void> > AmtecState;

typedef boost::shared_ptr< ::amtec::AmtecState> AmtecStatePtr;
typedef boost::shared_ptr< ::amtec::AmtecState const> AmtecStateConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::amtec::AmtecState_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::amtec::AmtecState_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace amtec

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::amtec::AmtecState_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::amtec::AmtecState_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::amtec::AmtecState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "19ace19cde5aea661ada2f628e7afeab";
  }

  static const char* value(const  ::amtec::AmtecState_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x19ace19cde5aea66ULL;
  static const uint64_t static_value2 = 0x1ada2f628e7afeabULL;
};

template<class ContainerAllocator>
struct DataType< ::amtec::AmtecState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "amtec/AmtecState";
  }

  static const char* value(const  ::amtec::AmtecState_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::amtec::AmtecState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "#\n\
# Message for the state of the amtec powercube pan/tilt unit. \n\
#\n\
Header  header         # header\n\
uint32  state          # module state\n\
float64 position       # module angle in radians\n\
float64 velocity       # module velocity in radians/s\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::amtec::AmtecState_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::amtec::AmtecState_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::amtec::AmtecState_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::amtec::AmtecState_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.state);
    stream.next(m.position);
    stream.next(m.velocity);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct AmtecState_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::amtec::AmtecState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::amtec::AmtecState_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "state: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.state);
    s << indent << "position: ";
    Printer<double>::stream(s, indent + "  ", v.position);
    s << indent << "velocity: ";
    Printer<double>::stream(s, indent + "  ", v.velocity);
  }
};


} // namespace message_operations
} // namespace ros

#endif // AMTEC_MESSAGE_AMTECSTATE_H
