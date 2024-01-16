// Generated by gencpp from file ajgar_perception/percepSrv.msg
// DO NOT EDIT!


#ifndef AJGAR_PERCEPTION_MESSAGE_PERCEPSRV_H
#define AJGAR_PERCEPTION_MESSAGE_PERCEPSRV_H

#include <ros/service_traits.h>


#include <ajgar_perception/percepSrvRequest.h>
#include <ajgar_perception/percepSrvResponse.h>


namespace ajgar_perception
{

struct percepSrv
{

typedef percepSrvRequest Request;
typedef percepSrvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct percepSrv
} // namespace ajgar_perception


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ajgar_perception::percepSrv > {
  static const char* value()
  {
    return "a5c0d5df4352df7c34178b1c281fe9a4";
  }

  static const char* value(const ::ajgar_perception::percepSrv&) { return value(); }
};

template<>
struct DataType< ::ajgar_perception::percepSrv > {
  static const char* value()
  {
    return "ajgar_perception/percepSrv";
  }

  static const char* value(const ::ajgar_perception::percepSrv&) { return value(); }
};


// service_traits::MD5Sum< ::ajgar_perception::percepSrvRequest> should match
// service_traits::MD5Sum< ::ajgar_perception::percepSrv >
template<>
struct MD5Sum< ::ajgar_perception::percepSrvRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ajgar_perception::percepSrv >::value();
  }
  static const char* value(const ::ajgar_perception::percepSrvRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ajgar_perception::percepSrvRequest> should match
// service_traits::DataType< ::ajgar_perception::percepSrv >
template<>
struct DataType< ::ajgar_perception::percepSrvRequest>
{
  static const char* value()
  {
    return DataType< ::ajgar_perception::percepSrv >::value();
  }
  static const char* value(const ::ajgar_perception::percepSrvRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ajgar_perception::percepSrvResponse> should match
// service_traits::MD5Sum< ::ajgar_perception::percepSrv >
template<>
struct MD5Sum< ::ajgar_perception::percepSrvResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ajgar_perception::percepSrv >::value();
  }
  static const char* value(const ::ajgar_perception::percepSrvResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ajgar_perception::percepSrvResponse> should match
// service_traits::DataType< ::ajgar_perception::percepSrv >
template<>
struct DataType< ::ajgar_perception::percepSrvResponse>
{
  static const char* value()
  {
    return DataType< ::ajgar_perception::percepSrv >::value();
  }
  static const char* value(const ::ajgar_perception::percepSrvResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // AJGAR_PERCEPTION_MESSAGE_PERCEPSRV_H
