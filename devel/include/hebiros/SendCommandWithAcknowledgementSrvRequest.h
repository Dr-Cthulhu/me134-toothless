// Generated by gencpp from file hebiros/SendCommandWithAcknowledgementSrvRequest.msg
// DO NOT EDIT!


#ifndef HEBIROS_MESSAGE_SENDCOMMANDWITHACKNOWLEDGEMENTSRVREQUEST_H
#define HEBIROS_MESSAGE_SENDCOMMANDWITHACKNOWLEDGEMENTSRVREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <hebiros/CommandMsg.h>

namespace hebiros
{
template <class ContainerAllocator>
struct SendCommandWithAcknowledgementSrvRequest_
{
  typedef SendCommandWithAcknowledgementSrvRequest_<ContainerAllocator> Type;

  SendCommandWithAcknowledgementSrvRequest_()
    : command()  {
    }
  SendCommandWithAcknowledgementSrvRequest_(const ContainerAllocator& _alloc)
    : command(_alloc)  {
  (void)_alloc;
    }



   typedef  ::hebiros::CommandMsg_<ContainerAllocator>  _command_type;
  _command_type command;





  typedef boost::shared_ptr< ::hebiros::SendCommandWithAcknowledgementSrvRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hebiros::SendCommandWithAcknowledgementSrvRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SendCommandWithAcknowledgementSrvRequest_

typedef ::hebiros::SendCommandWithAcknowledgementSrvRequest_<std::allocator<void> > SendCommandWithAcknowledgementSrvRequest;

typedef boost::shared_ptr< ::hebiros::SendCommandWithAcknowledgementSrvRequest > SendCommandWithAcknowledgementSrvRequestPtr;
typedef boost::shared_ptr< ::hebiros::SendCommandWithAcknowledgementSrvRequest const> SendCommandWithAcknowledgementSrvRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hebiros::SendCommandWithAcknowledgementSrvRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hebiros::SendCommandWithAcknowledgementSrvRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace hebiros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'hebiros': ['/home/robot/hebi_ros_ws/src/hebiros/hebiros/msg', '/home/robot/hebi_ros_ws/devel/share/hebiros/msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::hebiros::SendCommandWithAcknowledgementSrvRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hebiros::SendCommandWithAcknowledgementSrvRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hebiros::SendCommandWithAcknowledgementSrvRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hebiros::SendCommandWithAcknowledgementSrvRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hebiros::SendCommandWithAcknowledgementSrvRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hebiros::SendCommandWithAcknowledgementSrvRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hebiros::SendCommandWithAcknowledgementSrvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dc4cfbf8596ddaff00c6c17ca78326b7";
  }

  static const char* value(const ::hebiros::SendCommandWithAcknowledgementSrvRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdc4cfbf8596ddaffULL;
  static const uint64_t static_value2 = 0x00c6c17ca78326b7ULL;
};

template<class ContainerAllocator>
struct DataType< ::hebiros::SendCommandWithAcknowledgementSrvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hebiros/SendCommandWithAcknowledgementSrvRequest";
  }

  static const char* value(const ::hebiros::SendCommandWithAcknowledgementSrvRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hebiros::SendCommandWithAcknowledgementSrvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "CommandMsg command\n\
\n\
================================================================================\n\
MSG: hebiros/CommandMsg\n\
string[] name\n\
float64[] position\n\
float64[] velocity\n\
float64[] effort\n\
hebiros/SettingsMsg settings\n\
\n\
\n\
================================================================================\n\
MSG: hebiros/SettingsMsg\n\
string[] name\n\
bool[] save_current_settings\n\
int8[] control_strategy\n\
hebiros/PidGainsMsg position_gains\n\
hebiros/PidGainsMsg velocity_gains\n\
hebiros/PidGainsMsg effort_gains\n\
\n\
\n\
================================================================================\n\
MSG: hebiros/PidGainsMsg\n\
string[] name\n\
float64[] kp\n\
float64[] ki\n\
float64[] kd\n\
float64[] feed_forward\n\
float64[] dead_zone\n\
float64[] i_clamp\n\
float64[] punch\n\
float64[] min_target\n\
float64[] max_target\n\
float64[] target_lowpass\n\
float64[] min_output\n\
float64[] max_output\n\
float64[] output_lowpass\n\
bool[] d_on_error\n\
\n\
";
  }

  static const char* value(const ::hebiros::SendCommandWithAcknowledgementSrvRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hebiros::SendCommandWithAcknowledgementSrvRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.command);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SendCommandWithAcknowledgementSrvRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hebiros::SendCommandWithAcknowledgementSrvRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hebiros::SendCommandWithAcknowledgementSrvRequest_<ContainerAllocator>& v)
  {
    s << indent << "command: ";
    s << std::endl;
    Printer< ::hebiros::CommandMsg_<ContainerAllocator> >::stream(s, indent + "  ", v.command);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HEBIROS_MESSAGE_SENDCOMMANDWITHACKNOWLEDGEMENTSRVREQUEST_H
