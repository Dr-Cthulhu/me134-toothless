// Generated by gencpp from file moveto/IKin.msg
// DO NOT EDIT!


#ifndef MOVETO_MESSAGE_IKIN_H
#define MOVETO_MESSAGE_IKIN_H

#include <ros/service_traits.h>


#include <moveto/IKinRequest.h>
#include <moveto/IKinResponse.h>


namespace moveto
{

struct IKin
{

typedef IKinRequest Request;
typedef IKinResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct IKin
} // namespace moveto


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::moveto::IKin > {
  static const char* value()
  {
    return "1e8f025e4a7f0e73d00d1166f7ab7aff";
  }

  static const char* value(const ::moveto::IKin&) { return value(); }
};

template<>
struct DataType< ::moveto::IKin > {
  static const char* value()
  {
    return "moveto/IKin";
  }

  static const char* value(const ::moveto::IKin&) { return value(); }
};


// service_traits::MD5Sum< ::moveto::IKinRequest> should match 
// service_traits::MD5Sum< ::moveto::IKin > 
template<>
struct MD5Sum< ::moveto::IKinRequest>
{
  static const char* value()
  {
    return MD5Sum< ::moveto::IKin >::value();
  }
  static const char* value(const ::moveto::IKinRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::moveto::IKinRequest> should match 
// service_traits::DataType< ::moveto::IKin > 
template<>
struct DataType< ::moveto::IKinRequest>
{
  static const char* value()
  {
    return DataType< ::moveto::IKin >::value();
  }
  static const char* value(const ::moveto::IKinRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::moveto::IKinResponse> should match 
// service_traits::MD5Sum< ::moveto::IKin > 
template<>
struct MD5Sum< ::moveto::IKinResponse>
{
  static const char* value()
  {
    return MD5Sum< ::moveto::IKin >::value();
  }
  static const char* value(const ::moveto::IKinResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::moveto::IKinResponse> should match 
// service_traits::DataType< ::moveto::IKin > 
template<>
struct DataType< ::moveto::IKinResponse>
{
  static const char* value()
  {
    return DataType< ::moveto::IKin >::value();
  }
  static const char* value(const ::moveto::IKinResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MOVETO_MESSAGE_IKIN_H
