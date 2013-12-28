"""autogenerated by genpy from bmc050_driver/bmc050_measurement.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class bmc050_measurement(genpy.Message):
  _md5sum = "018c77a7bfb5d925516f29f71a5163ed"
  _type = "bmc050_driver/bmc050_measurement"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header
float64[] AccelerationX #[g]
float64[] AccelerationY #[g]
float64[] AccelerationZ #[g]
float64[] Temperature #[C]
float64[] MagneticFieldIntensityX #[uT]
float64[] MagneticFieldIntensityY #[uT]
float64[] MagneticFieldIntensityZ #[uT]
uint16[] HallResistance #[ohms]

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

"""
  __slots__ = ['header','AccelerationX','AccelerationY','AccelerationZ','Temperature','MagneticFieldIntensityX','MagneticFieldIntensityY','MagneticFieldIntensityZ','HallResistance']
  _slot_types = ['std_msgs/Header','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','uint16[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,AccelerationX,AccelerationY,AccelerationZ,Temperature,MagneticFieldIntensityX,MagneticFieldIntensityY,MagneticFieldIntensityZ,HallResistance

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(bmc050_measurement, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.AccelerationX is None:
        self.AccelerationX = []
      if self.AccelerationY is None:
        self.AccelerationY = []
      if self.AccelerationZ is None:
        self.AccelerationZ = []
      if self.Temperature is None:
        self.Temperature = []
      if self.MagneticFieldIntensityX is None:
        self.MagneticFieldIntensityX = []
      if self.MagneticFieldIntensityY is None:
        self.MagneticFieldIntensityY = []
      if self.MagneticFieldIntensityZ is None:
        self.MagneticFieldIntensityZ = []
      if self.HallResistance is None:
        self.HallResistance = []
    else:
      self.header = std_msgs.msg.Header()
      self.AccelerationX = []
      self.AccelerationY = []
      self.AccelerationZ = []
      self.Temperature = []
      self.MagneticFieldIntensityX = []
      self.MagneticFieldIntensityY = []
      self.MagneticFieldIntensityZ = []
      self.HallResistance = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.AccelerationX)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.AccelerationX))
      length = len(self.AccelerationY)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.AccelerationY))
      length = len(self.AccelerationZ)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.AccelerationZ))
      length = len(self.Temperature)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.Temperature))
      length = len(self.MagneticFieldIntensityX)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.MagneticFieldIntensityX))
      length = len(self.MagneticFieldIntensityY)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.MagneticFieldIntensityY))
      length = len(self.MagneticFieldIntensityZ)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.MagneticFieldIntensityZ))
      length = len(self.HallResistance)
      buff.write(_struct_I.pack(length))
      pattern = '<%sH'%length
      buff.write(struct.pack(pattern, *self.HallResistance))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.AccelerationX = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.AccelerationY = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.AccelerationZ = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.Temperature = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.MagneticFieldIntensityX = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.MagneticFieldIntensityY = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.MagneticFieldIntensityZ = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sH'%length
      start = end
      end += struct.calcsize(pattern)
      self.HallResistance = struct.unpack(pattern, str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.AccelerationX)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.AccelerationX.tostring())
      length = len(self.AccelerationY)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.AccelerationY.tostring())
      length = len(self.AccelerationZ)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.AccelerationZ.tostring())
      length = len(self.Temperature)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.Temperature.tostring())
      length = len(self.MagneticFieldIntensityX)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.MagneticFieldIntensityX.tostring())
      length = len(self.MagneticFieldIntensityY)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.MagneticFieldIntensityY.tostring())
      length = len(self.MagneticFieldIntensityZ)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.MagneticFieldIntensityZ.tostring())
      length = len(self.HallResistance)
      buff.write(_struct_I.pack(length))
      pattern = '<%sH'%length
      buff.write(self.HallResistance.tostring())
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.AccelerationX = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.AccelerationY = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.AccelerationZ = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.Temperature = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.MagneticFieldIntensityX = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.MagneticFieldIntensityY = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.MagneticFieldIntensityZ = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sH'%length
      start = end
      end += struct.calcsize(pattern)
      self.HallResistance = numpy.frombuffer(str[start:end], dtype=numpy.uint16, count=length)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
