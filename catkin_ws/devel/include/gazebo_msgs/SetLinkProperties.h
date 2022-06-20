// Generated by gencpp from file gazebo_msgs/SetLinkProperties.msg
// DO NOT EDIT!


#ifndef GAZEBO_MSGS_MESSAGE_SETLINKPROPERTIES_H
#define GAZEBO_MSGS_MESSAGE_SETLINKPROPERTIES_H

#include <ros/service_traits.h>


#include <gazebo_msgs/SetLinkPropertiesRequest.h>
#include <gazebo_msgs/SetLinkPropertiesResponse.h>


namespace gazebo_msgs
{

struct SetLinkProperties
{

typedef SetLinkPropertiesRequest Request;
typedef SetLinkPropertiesResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetLinkProperties
} // namespace gazebo_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::gazebo_msgs::SetLinkProperties > {
  static const char* value()
  {
    return "d534ce1b36ee99de0ffa806c3a6348f0";
  }

  static const char* value(const ::gazebo_msgs::SetLinkProperties&) { return value(); }
};

template<>
struct DataType< ::gazebo_msgs::SetLinkProperties > {
  static const char* value()
  {
    return "gazebo_msgs/SetLinkProperties";
  }

  static const char* value(const ::gazebo_msgs::SetLinkProperties&) { return value(); }
};


// service_traits::MD5Sum< ::gazebo_msgs::SetLinkPropertiesRequest> should match
// service_traits::MD5Sum< ::gazebo_msgs::SetLinkProperties >
template<>
struct MD5Sum< ::gazebo_msgs::SetLinkPropertiesRequest>
{
  static const char* value()
  {
    return MD5Sum< ::gazebo_msgs::SetLinkProperties >::value();
  }
  static const char* value(const ::gazebo_msgs::SetLinkPropertiesRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::gazebo_msgs::SetLinkPropertiesRequest> should match
// service_traits::DataType< ::gazebo_msgs::SetLinkProperties >
template<>
struct DataType< ::gazebo_msgs::SetLinkPropertiesRequest>
{
  static const char* value()
  {
    return DataType< ::gazebo_msgs::SetLinkProperties >::value();
  }
  static const char* value(const ::gazebo_msgs::SetLinkPropertiesRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::gazebo_msgs::SetLinkPropertiesResponse> should match
// service_traits::MD5Sum< ::gazebo_msgs::SetLinkProperties >
template<>
struct MD5Sum< ::gazebo_msgs::SetLinkPropertiesResponse>
{
  static const char* value()
  {
    return MD5Sum< ::gazebo_msgs::SetLinkProperties >::value();
  }
  static const char* value(const ::gazebo_msgs::SetLinkPropertiesResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::gazebo_msgs::SetLinkPropertiesResponse> should match
// service_traits::DataType< ::gazebo_msgs::SetLinkProperties >
template<>
struct DataType< ::gazebo_msgs::SetLinkPropertiesResponse>
{
  static const char* value()
  {
    return DataType< ::gazebo_msgs::SetLinkProperties >::value();
  }
  static const char* value(const ::gazebo_msgs::SetLinkPropertiesResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // GAZEBO_MSGS_MESSAGE_SETLINKPROPERTIES_H