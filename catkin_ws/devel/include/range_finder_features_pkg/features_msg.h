// Generated by gencpp from file range_finder_features_pkg/features_msg.msg
// DO NOT EDIT!


#ifndef RANGE_FINDER_FEATURES_PKG_MESSAGE_FEATURES_MSG_H
#define RANGE_FINDER_FEATURES_PKG_MESSAGE_FEATURES_MSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>

namespace range_finder_features_pkg
{
template <class ContainerAllocator>
struct features_msg_
{
  typedef features_msg_<ContainerAllocator> Type;

  features_msg_()
    : points()
    , features_updated(false)  {
    }
  features_msg_(const ContainerAllocator& _alloc)
    : points(_alloc)
    , features_updated(false)  {
  (void)_alloc;
    }



   typedef std::vector< ::geometry_msgs::Point_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Point_<ContainerAllocator> >::other >  _points_type;
  _points_type points;

   typedef uint8_t _features_updated_type;
  _features_updated_type features_updated;





  typedef boost::shared_ptr< ::range_finder_features_pkg::features_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::range_finder_features_pkg::features_msg_<ContainerAllocator> const> ConstPtr;

}; // struct features_msg_

typedef ::range_finder_features_pkg::features_msg_<std::allocator<void> > features_msg;

typedef boost::shared_ptr< ::range_finder_features_pkg::features_msg > features_msgPtr;
typedef boost::shared_ptr< ::range_finder_features_pkg::features_msg const> features_msgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::range_finder_features_pkg::features_msg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::range_finder_features_pkg::features_msg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::range_finder_features_pkg::features_msg_<ContainerAllocator1> & lhs, const ::range_finder_features_pkg::features_msg_<ContainerAllocator2> & rhs)
{
  return lhs.points == rhs.points &&
    lhs.features_updated == rhs.features_updated;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::range_finder_features_pkg::features_msg_<ContainerAllocator1> & lhs, const ::range_finder_features_pkg::features_msg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace range_finder_features_pkg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::range_finder_features_pkg::features_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::range_finder_features_pkg::features_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::range_finder_features_pkg::features_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::range_finder_features_pkg::features_msg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::range_finder_features_pkg::features_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::range_finder_features_pkg::features_msg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::range_finder_features_pkg::features_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3fb405e9ac52e34e46d6e8b78d1a3ac5";
  }

  static const char* value(const ::range_finder_features_pkg::features_msg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3fb405e9ac52e34eULL;
  static const uint64_t static_value2 = 0x46d6e8b78d1a3ac5ULL;
};

template<class ContainerAllocator>
struct DataType< ::range_finder_features_pkg::features_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "range_finder_features_pkg/features_msg";
  }

  static const char* value(const ::range_finder_features_pkg::features_msg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::range_finder_features_pkg::features_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Point[] points\n"
"bool features_updated\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::range_finder_features_pkg::features_msg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::range_finder_features_pkg::features_msg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.points);
      stream.next(m.features_updated);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct features_msg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::range_finder_features_pkg::features_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::range_finder_features_pkg::features_msg_<ContainerAllocator>& v)
  {
    s << indent << "points[]" << std::endl;
    for (size_t i = 0; i < v.points.size(); ++i)
    {
      s << indent << "  points[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "    ", v.points[i]);
    }
    s << indent << "features_updated: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.features_updated);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RANGE_FINDER_FEATURES_PKG_MESSAGE_FEATURES_MSG_H
