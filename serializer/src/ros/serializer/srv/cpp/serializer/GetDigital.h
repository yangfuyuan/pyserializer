/* auto-generated by gensrv_cpp from /home/patrick/Eclipse/serializer/src/ros/serializer/srv/GetDigital.srv.  Do not edit! */
#ifndef SRV_SERIALIZER_GETDIGITAL_H
#define SRV_SERIALIZER_GETDIGITAL_H

#include <string>
#include <cstring>
#include <vector>
#include <map>
#include "ros/message.h"
#include "ros/time.h"

namespace serializer
{

struct GetDigital
{

inline static std::string getDataType() { return "serializer/GetDigital"; }
inline static std::string getMD5Sum() { return "ad28e4611c3edea82d59f9c3743bc9b7"; }

//! \htmlinclude Request.msg.html

class Request : public ros::Message
{
public:
  typedef boost::shared_ptr<Request> Ptr;
  typedef boost::shared_ptr<Request const> ConstPtr;

  typedef uint8_t _pin_type;

  uint8_t pin;

  Request() : ros::Message(),
    pin(0)
  {
  }
  Request(const Request &copy) : ros::Message(),
    pin(copy.pin)
  {
    (void)copy;
  }
  Request &operator =(const Request &copy)
  {
    if (this == &copy)
      return *this;
    pin = copy.pin;
    return *this;
  }
  virtual ~Request() 
  {
  }
  inline static std::string __s_getDataType() { return std::string("serializer/GetDigitalRequest"); }
  inline static std::string __s_getMD5Sum() { return std::string(""); }
  inline static std::string __s_getMessageDefinition()
  {
    return std::string(
    "uint8 pin\n"
    "\n"
    );
  }
  inline virtual const std::string __getDataType() const { return __s_getDataType(); }
  inline virtual const std::string __getMD5Sum() const { return __s_getMD5Sum(); }
  inline virtual const std::string __getMessageDefinition() const { return __s_getMessageDefinition(); }
  inline static std::string __s_getServerMD5Sum() { return std::string("ad28e4611c3edea82d59f9c3743bc9b7"); }
  inline virtual const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum(); }
  inline static std::string __s_getServiceDataType() { return std::string("serializer/GetDigital"); }
  inline virtual const std::string __getServiceDataType() const { return __s_getServiceDataType(); }
  inline uint32_t serializationLength() const
  {
    unsigned __l = 0;
    __l += 1; // pin
    return __l;
  }
  virtual uint8_t *serialize(uint8_t *write_ptr,
#if defined(__GNUC__)
                             __attribute__((unused)) uint32_t seq) const
#else
                             uint32_t seq) const
#endif
  {
    SROS_SERIALIZE_PRIMITIVE(write_ptr, pin);
    return write_ptr;
  }
  virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    SROS_DESERIALIZE_PRIMITIVE(read_ptr, pin);
    return read_ptr;
  }
};

typedef boost::shared_ptr<Request> RequestPtr;
typedef boost::shared_ptr<Request const> RequestConstPtr;

//! \htmlinclude Response.msg.html

class Response : public ros::Message
{
public:
  typedef boost::shared_ptr<Response> Ptr;
  typedef boost::shared_ptr<Response const> ConstPtr;

  typedef uint8_t _value_type;

  uint8_t value;

  Response() : ros::Message(),
    value(0)
  {
  }
  Response(const Response &copy) : ros::Message(),
    value(copy.value)
  {
    (void)copy;
  }
  Response &operator =(const Response &copy)
  {
    if (this == &copy)
      return *this;
    value = copy.value;
    return *this;
  }
  virtual ~Response() 
  {
  }
  inline static std::string __s_getDataType() { return std::string("serializer/GetDigitalResponse"); }
  inline static std::string __s_getMD5Sum() { return std::string(""); }
  inline static std::string __s_getMessageDefinition()
  {
    return std::string(
    "uint8 value\n"
    "\n"
    "\n"
    );
  }
  inline virtual const std::string __getDataType() const { return __s_getDataType(); }
  inline virtual const std::string __getMD5Sum() const { return __s_getMD5Sum(); }
  inline virtual const std::string __getMessageDefinition() const { return __s_getMessageDefinition(); }
  inline static std::string __s_getServerMD5Sum() { return std::string("ad28e4611c3edea82d59f9c3743bc9b7"); }
  inline virtual const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum(); }
  inline static std::string __s_getServiceDataType() { return std::string("serializer/GetDigital"); }
  inline virtual const std::string __getServiceDataType() const { return __s_getServiceDataType(); }
  inline uint32_t serializationLength() const
  {
    unsigned __l = 0;
    __l += 1; // value
    return __l;
  }
  virtual uint8_t *serialize(uint8_t *write_ptr,
#if defined(__GNUC__)
                             __attribute__((unused)) uint32_t seq) const
#else
                             uint32_t seq) const
#endif
  {
    SROS_SERIALIZE_PRIMITIVE(write_ptr, value);
    return write_ptr;
  }
  virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    SROS_DESERIALIZE_PRIMITIVE(read_ptr, value);
    return read_ptr;
  }
};

typedef boost::shared_ptr<Response> ResponsePtr;
typedef boost::shared_ptr<Response const> ResponseConstPtr;

Request request;
Response response;

};

}

#endif