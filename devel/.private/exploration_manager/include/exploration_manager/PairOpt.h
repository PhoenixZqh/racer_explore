// Generated by gencpp from file exploration_manager/PairOpt.msg
// DO NOT EDIT!


#ifndef EXPLORATION_MANAGER_MESSAGE_PAIROPT_H
#define EXPLORATION_MANAGER_MESSAGE_PAIROPT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace exploration_manager
{
template <class ContainerAllocator>
struct PairOpt_
{
  typedef PairOpt_<ContainerAllocator> Type;

  PairOpt_()
    : from_drone_id(0)
    , to_drone_id(0)
    , stamp(0.0)
    , ego_ids()
    , other_ids()  {
    }
  PairOpt_(const ContainerAllocator& _alloc)
    : from_drone_id(0)
    , to_drone_id(0)
    , stamp(0.0)
    , ego_ids(_alloc)
    , other_ids(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _from_drone_id_type;
  _from_drone_id_type from_drone_id;

   typedef int32_t _to_drone_id_type;
  _to_drone_id_type to_drone_id;

   typedef double _stamp_type;
  _stamp_type stamp;

   typedef std::vector<int8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int8_t>> _ego_ids_type;
  _ego_ids_type ego_ids;

   typedef std::vector<int8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int8_t>> _other_ids_type;
  _other_ids_type other_ids;





  typedef boost::shared_ptr< ::exploration_manager::PairOpt_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::exploration_manager::PairOpt_<ContainerAllocator> const> ConstPtr;

}; // struct PairOpt_

typedef ::exploration_manager::PairOpt_<std::allocator<void> > PairOpt;

typedef boost::shared_ptr< ::exploration_manager::PairOpt > PairOptPtr;
typedef boost::shared_ptr< ::exploration_manager::PairOpt const> PairOptConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::exploration_manager::PairOpt_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::exploration_manager::PairOpt_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::exploration_manager::PairOpt_<ContainerAllocator1> & lhs, const ::exploration_manager::PairOpt_<ContainerAllocator2> & rhs)
{
  return lhs.from_drone_id == rhs.from_drone_id &&
    lhs.to_drone_id == rhs.to_drone_id &&
    lhs.stamp == rhs.stamp &&
    lhs.ego_ids == rhs.ego_ids &&
    lhs.other_ids == rhs.other_ids;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::exploration_manager::PairOpt_<ContainerAllocator1> & lhs, const ::exploration_manager::PairOpt_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace exploration_manager

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::exploration_manager::PairOpt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::exploration_manager::PairOpt_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::exploration_manager::PairOpt_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::exploration_manager::PairOpt_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::exploration_manager::PairOpt_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::exploration_manager::PairOpt_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::exploration_manager::PairOpt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f1b382d48f9c952cdea39f0f21949ad6";
  }

  static const char* value(const ::exploration_manager::PairOpt_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf1b382d48f9c952cULL;
  static const uint64_t static_value2 = 0xdea39f0f21949ad6ULL;
};

template<class ContainerAllocator>
struct DataType< ::exploration_manager::PairOpt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "exploration_manager/PairOpt";
  }

  static const char* value(const ::exploration_manager::PairOpt_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::exploration_manager::PairOpt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"int32 from_drone_id\n"
"int32 to_drone_id\n"
"\n"
"float64 stamp\n"
"int8[] ego_ids\n"
"int8[] other_ids\n"
;
  }

  static const char* value(const ::exploration_manager::PairOpt_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::exploration_manager::PairOpt_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.from_drone_id);
      stream.next(m.to_drone_id);
      stream.next(m.stamp);
      stream.next(m.ego_ids);
      stream.next(m.other_ids);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PairOpt_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::exploration_manager::PairOpt_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::exploration_manager::PairOpt_<ContainerAllocator>& v)
  {
    s << indent << "from_drone_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.from_drone_id);
    s << indent << "to_drone_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.to_drone_id);
    s << indent << "stamp: ";
    Printer<double>::stream(s, indent + "  ", v.stamp);
    s << indent << "ego_ids[]" << std::endl;
    for (size_t i = 0; i < v.ego_ids.size(); ++i)
    {
      s << indent << "  ego_ids[" << i << "]: ";
      Printer<int8_t>::stream(s, indent + "  ", v.ego_ids[i]);
    }
    s << indent << "other_ids[]" << std::endl;
    for (size_t i = 0; i < v.other_ids.size(); ++i)
    {
      s << indent << "  other_ids[" << i << "]: ";
      Printer<int8_t>::stream(s, indent + "  ", v.other_ids[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // EXPLORATION_MANAGER_MESSAGE_PAIROPT_H
