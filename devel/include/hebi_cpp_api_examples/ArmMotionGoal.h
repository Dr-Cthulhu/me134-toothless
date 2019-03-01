// Generated by gencpp from file hebi_cpp_api_examples/ArmMotionGoal.msg
// DO NOT EDIT!


#ifndef HEBI_CPP_API_EXAMPLES_MESSAGE_ARMMOTIONGOAL_H
#define HEBI_CPP_API_EXAMPLES_MESSAGE_ARMMOTIONGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace hebi_cpp_api_examples
{
template <class ContainerAllocator>
struct ArmMotionGoal_
{
  typedef ArmMotionGoal_<ContainerAllocator> Type;

  ArmMotionGoal_()
    : x()
    , y()
    , z()
    , tipx()
    , tipy()
    , tipz()
    , set_color(false)
    , r(0)
    , g(0)
    , b(0)  {
    }
  ArmMotionGoal_(const ContainerAllocator& _alloc)
    : x(_alloc)
    , y(_alloc)
    , z(_alloc)
    , tipx(_alloc)
    , tipy(_alloc)
    , tipz(_alloc)
    , set_color(false)
    , r(0)
    , g(0)
    , b(0)  {
  (void)_alloc;
    }



   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _x_type;
  _x_type x;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _y_type;
  _y_type y;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _z_type;
  _z_type z;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _tipx_type;
  _tipx_type tipx;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _tipy_type;
  _tipy_type tipy;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _tipz_type;
  _tipz_type tipz;

   typedef uint8_t _set_color_type;
  _set_color_type set_color;

   typedef uint8_t _r_type;
  _r_type r;

   typedef uint8_t _g_type;
  _g_type g;

   typedef uint8_t _b_type;
  _b_type b;





  typedef boost::shared_ptr< ::hebi_cpp_api_examples::ArmMotionGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hebi_cpp_api_examples::ArmMotionGoal_<ContainerAllocator> const> ConstPtr;

}; // struct ArmMotionGoal_

typedef ::hebi_cpp_api_examples::ArmMotionGoal_<std::allocator<void> > ArmMotionGoal;

typedef boost::shared_ptr< ::hebi_cpp_api_examples::ArmMotionGoal > ArmMotionGoalPtr;
typedef boost::shared_ptr< ::hebi_cpp_api_examples::ArmMotionGoal const> ArmMotionGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hebi_cpp_api_examples::ArmMotionGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hebi_cpp_api_examples::ArmMotionGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace hebi_cpp_api_examples

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'hebi_cpp_api_examples': ['/home/robot/hebi_ros_ws/devel/share/hebi_cpp_api_examples/msg', '/home/robot/hebi_ros_ws/src/hebi_cpp_api_ros_examples/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::hebi_cpp_api_examples::ArmMotionGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hebi_cpp_api_examples::ArmMotionGoal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hebi_cpp_api_examples::ArmMotionGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hebi_cpp_api_examples::ArmMotionGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hebi_cpp_api_examples::ArmMotionGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hebi_cpp_api_examples::ArmMotionGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hebi_cpp_api_examples::ArmMotionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5182f7e080f47b29c479712b9f962cb9";
  }

  static const char* value(const ::hebi_cpp_api_examples::ArmMotionGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5182f7e080f47b29ULL;
  static const uint64_t static_value2 = 0xc479712b9f962cb9ULL;
};

template<class ContainerAllocator>
struct DataType< ::hebi_cpp_api_examples::ArmMotionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hebi_cpp_api_examples/ArmMotionGoal";
  }

  static const char* value(const ::hebi_cpp_api_examples::ArmMotionGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hebi_cpp_api_examples::ArmMotionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# Goal position:\n\
float64[] x\n\
float64[] y\n\
float64[] z\n\
float64[] tipx\n\
float64[] tipy\n\
float64[] tipz\n\
\n\
# Optionally, set a color when doing the move; otherwise, clear the color.\n\
bool set_color\n\
uint8 r\n\
uint8 g\n\
uint8 b\n\
";
  }

  static const char* value(const ::hebi_cpp_api_examples::ArmMotionGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hebi_cpp_api_examples::ArmMotionGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.tipx);
      stream.next(m.tipy);
      stream.next(m.tipz);
      stream.next(m.set_color);
      stream.next(m.r);
      stream.next(m.g);
      stream.next(m.b);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ArmMotionGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hebi_cpp_api_examples::ArmMotionGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hebi_cpp_api_examples::ArmMotionGoal_<ContainerAllocator>& v)
  {
    s << indent << "x[]" << std::endl;
    for (size_t i = 0; i < v.x.size(); ++i)
    {
      s << indent << "  x[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.x[i]);
    }
    s << indent << "y[]" << std::endl;
    for (size_t i = 0; i < v.y.size(); ++i)
    {
      s << indent << "  y[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.y[i]);
    }
    s << indent << "z[]" << std::endl;
    for (size_t i = 0; i < v.z.size(); ++i)
    {
      s << indent << "  z[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.z[i]);
    }
    s << indent << "tipx[]" << std::endl;
    for (size_t i = 0; i < v.tipx.size(); ++i)
    {
      s << indent << "  tipx[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.tipx[i]);
    }
    s << indent << "tipy[]" << std::endl;
    for (size_t i = 0; i < v.tipy.size(); ++i)
    {
      s << indent << "  tipy[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.tipy[i]);
    }
    s << indent << "tipz[]" << std::endl;
    for (size_t i = 0; i < v.tipz.size(); ++i)
    {
      s << indent << "  tipz[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.tipz[i]);
    }
    s << indent << "set_color: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.set_color);
    s << indent << "r: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.r);
    s << indent << "g: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.g);
    s << indent << "b: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.b);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HEBI_CPP_API_EXAMPLES_MESSAGE_ARMMOTIONGOAL_H
