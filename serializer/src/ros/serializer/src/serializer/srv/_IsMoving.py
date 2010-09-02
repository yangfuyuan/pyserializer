# autogenerated by genmsg_py from IsMovingRequest.msg. Do not edit.
import roslib.message
import struct


class IsMovingRequest(roslib.message.Message):
  _md5sum = "d41d8cd98f00b204e9800998ecf8427e"
  _type = "serializer/IsMovingRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """

"""
  __slots__ = []
  _slot_types = []

  ## Constructor. Any message fields that are implicitly/explicitly
  ## set to None will be assigned a default value. The recommend
  ## use is keyword arguments as this is more robust to future message
  ## changes.  You cannot mix in-order arguments and keyword arguments.
  ##
  ## The available fields are:
  ##   
  ##
  ## @param self: self
  ## @param args: complete set of field values, in .msg order
  ## @param kwds: use keyword arguments corresponding to message field names
  ## to set specific fields. 
  def __init__(self, *args, **kwds):
    if args or kwds:
      super(IsMovingRequest, self).__init__(*args, **kwds)

  ## internal API method
  def _get_types(self): return self._slot_types

  ## serialize message into buffer
  ## @param buff StringIO: buffer
  def serialize(self, buff):
    try:
      pass
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  ## unpack serialized message in str into this message instance
  ## @param self: self
  ## @param str str: byte array of serialized message
  def deserialize(self, str):
    try:
      end = 0
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  ## serialize message with numpy array types into buffer
  ## @param self: self
  ## @param buff StringIO: buffer
  ## @param numpy module: numpy python module
  def serialize_numpy(self, buff, numpy):
    try:
      pass
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  ## unpack serialized message in str into this message instance using numpy for array types
  ## @param self: self
  ## @param str str: byte array of serialized message
  ## @param numpy module: numpy python module
  def deserialize_numpy(self, str, numpy):
    try:
      end = 0
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

# autogenerated by genmsg_py from IsMovingResponse.msg. Do not edit.
import roslib.message
import struct


class IsMovingResponse(roslib.message.Message):
  _md5sum = "9104f1a32b4fbf4d3c8c80d9b9493250"
  _type = "serializer/IsMovingResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """bool moving


"""
  __slots__ = ['moving']
  _slot_types = ['bool']

  ## Constructor. Any message fields that are implicitly/explicitly
  ## set to None will be assigned a default value. The recommend
  ## use is keyword arguments as this is more robust to future message
  ## changes.  You cannot mix in-order arguments and keyword arguments.
  ##
  ## The available fields are:
  ##   moving
  ##
  ## @param self: self
  ## @param args: complete set of field values, in .msg order
  ## @param kwds: use keyword arguments corresponding to message field names
  ## to set specific fields. 
  def __init__(self, *args, **kwds):
    if args or kwds:
      super(IsMovingResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.moving is None:
        self.moving = False
    else:
      self.moving = False

  ## internal API method
  def _get_types(self): return self._slot_types

  ## serialize message into buffer
  ## @param buff StringIO: buffer
  def serialize(self, buff):
    try:
      buff.write(struct.pack('<B', self.moving))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  ## unpack serialized message in str into this message instance
  ## @param self: self
  ## @param str str: byte array of serialized message
  def deserialize(self, str):
    try:
      end = 0
      start = end
      end += 1
      (self.moving,) = struct.unpack('<B',str[start:end])
      self.moving = bool(self.moving)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  ## serialize message with numpy array types into buffer
  ## @param self: self
  ## @param buff StringIO: buffer
  ## @param numpy module: numpy python module
  def serialize_numpy(self, buff, numpy):
    try:
      buff.write(struct.pack('<B', self.moving))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  ## unpack serialized message in str into this message instance using numpy for array types
  ## @param self: self
  ## @param str str: byte array of serialized message
  ## @param numpy module: numpy python module
  def deserialize_numpy(self, str, numpy):
    try:
      end = 0
      start = end
      end += 1
      (self.moving,) = struct.unpack('<B',str[start:end])
      self.moving = bool(self.moving)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

class IsMoving(roslib.message.ServiceDefinition):
  _type          = 'serializer/IsMoving'
  _md5sum = '9104f1a32b4fbf4d3c8c80d9b9493250'
  _request_class  = IsMovingRequest
  _response_class = IsMovingResponse
