// Generated by gencpp from file plan_env/IdxList.msg
// DO NOT EDIT!


#ifndef PLAN_ENV_MESSAGE_IDXLIST_H
#define PLAN_ENV_MESSAGE_IDXLIST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace plan_env
{
template <class ContainerAllocator>
struct IdxList_
{
  typedef IdxList_<ContainerAllocator> Type;

  IdxList_()
    : ids()  {
    }
  IdxList_(const ContainerAllocator& _alloc)
    : ids(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> _ids_type;
  _ids_type ids;





  typedef boost::shared_ptr< ::plan_env::IdxList_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::plan_env::IdxList_<ContainerAllocator> const> ConstPtr;

}; // struct IdxList_

typedef ::plan_env::IdxList_<std::allocator<void> > IdxList;

typedef boost::shared_ptr< ::plan_env::IdxList > IdxListPtr;
typedef boost::shared_ptr< ::plan_env::IdxList const> IdxListConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::plan_env::IdxList_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::plan_env::IdxList_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::plan_env::IdxList_<ContainerAllocator1> & lhs, const ::plan_env::IdxList_<ContainerAllocator2> & rhs)
{
  return lhs.ids == rhs.ids;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::plan_env::IdxList_<ContainerAllocator1> & lhs, const ::plan_env::IdxList_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace plan_env

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::plan_env::IdxList_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::plan_env::IdxList_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::plan_env::IdxList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::plan_env::IdxList_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::plan_env::IdxList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::plan_env::IdxList_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::plan_env::IdxList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4f22efebf407aadba2ecc69df353d113";
  }

  static const char* value(const ::plan_env::IdxList_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4f22efebf407aadbULL;
  static const uint64_t static_value2 = 0xa2ecc69df353d113ULL;
};

template<class ContainerAllocator>
struct DataType< ::plan_env::IdxList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "plan_env/IdxList";
  }

  static const char* value(const ::plan_env::IdxList_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::plan_env::IdxList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32[] ids\n"
;
  }

  static const char* value(const ::plan_env::IdxList_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::plan_env::IdxList_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ids);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct IdxList_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::plan_env::IdxList_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::plan_env::IdxList_<ContainerAllocator>& v)
  {
    s << indent << "ids[]" << std::endl;
    for (size_t i = 0; i < v.ids.size(); ++i)
    {
      s << indent << "  ids[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.ids[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // PLAN_ENV_MESSAGE_IDXLIST_H
