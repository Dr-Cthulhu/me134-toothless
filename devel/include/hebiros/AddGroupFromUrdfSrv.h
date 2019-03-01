// Generated by gencpp from file hebiros/AddGroupFromUrdfSrv.msg
// DO NOT EDIT!


#ifndef HEBIROS_MESSAGE_ADDGROUPFROMURDFSRV_H
#define HEBIROS_MESSAGE_ADDGROUPFROMURDFSRV_H

#include <ros/service_traits.h>


#include <hebiros/AddGroupFromUrdfSrvRequest.h>
#include <hebiros/AddGroupFromUrdfSrvResponse.h>


namespace hebiros
{

struct AddGroupFromUrdfSrv
{

typedef AddGroupFromUrdfSrvRequest Request;
typedef AddGroupFromUrdfSrvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct AddGroupFromUrdfSrv
} // namespace hebiros


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::hebiros::AddGroupFromUrdfSrv > {
  static const char* value()
  {
    return "967d0b0c0d858ded8a6a69abbce0c981";
  }

  static const char* value(const ::hebiros::AddGroupFromUrdfSrv&) { return value(); }
};

template<>
struct DataType< ::hebiros::AddGroupFromUrdfSrv > {
  static const char* value()
  {
    return "hebiros/AddGroupFromUrdfSrv";
  }

  static const char* value(const ::hebiros::AddGroupFromUrdfSrv&) { return value(); }
};


// service_traits::MD5Sum< ::hebiros::AddGroupFromUrdfSrvRequest> should match 
// service_traits::MD5Sum< ::hebiros::AddGroupFromUrdfSrv > 
template<>
struct MD5Sum< ::hebiros::AddGroupFromUrdfSrvRequest>
{
  static const char* value()
  {
    return MD5Sum< ::hebiros::AddGroupFromUrdfSrv >::value();
  }
  static const char* value(const ::hebiros::AddGroupFromUrdfSrvRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::hebiros::AddGroupFromUrdfSrvRequest> should match 
// service_traits::DataType< ::hebiros::AddGroupFromUrdfSrv > 
template<>
struct DataType< ::hebiros::AddGroupFromUrdfSrvRequest>
{
  static const char* value()
  {
    return DataType< ::hebiros::AddGroupFromUrdfSrv >::value();
  }
  static const char* value(const ::hebiros::AddGroupFromUrdfSrvRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::hebiros::AddGroupFromUrdfSrvResponse> should match 
// service_traits::MD5Sum< ::hebiros::AddGroupFromUrdfSrv > 
template<>
struct MD5Sum< ::hebiros::AddGroupFromUrdfSrvResponse>
{
  static const char* value()
  {
    return MD5Sum< ::hebiros::AddGroupFromUrdfSrv >::value();
  }
  static const char* value(const ::hebiros::AddGroupFromUrdfSrvResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::hebiros::AddGroupFromUrdfSrvResponse> should match 
// service_traits::DataType< ::hebiros::AddGroupFromUrdfSrv > 
template<>
struct DataType< ::hebiros::AddGroupFromUrdfSrvResponse>
{
  static const char* value()
  {
    return DataType< ::hebiros::AddGroupFromUrdfSrv >::value();
  }
  static const char* value(const ::hebiros::AddGroupFromUrdfSrvResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // HEBIROS_MESSAGE_ADDGROUPFROMURDFSRV_H
