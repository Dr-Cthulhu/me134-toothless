# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hebiros/PidGainsMsg.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class PidGainsMsg(genpy.Message):
  _md5sum = "409cfb8df90bde199674774f996b26c5"
  _type = "hebiros/PidGainsMsg"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """string[] name
float64[] kp
float64[] ki
float64[] kd
float64[] feed_forward
float64[] dead_zone
float64[] i_clamp
float64[] punch
float64[] min_target
float64[] max_target
float64[] target_lowpass
float64[] min_output
float64[] max_output
float64[] output_lowpass
bool[] d_on_error

"""
  __slots__ = ['name','kp','ki','kd','feed_forward','dead_zone','i_clamp','punch','min_target','max_target','target_lowpass','min_output','max_output','output_lowpass','d_on_error']
  _slot_types = ['string[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','bool[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       name,kp,ki,kd,feed_forward,dead_zone,i_clamp,punch,min_target,max_target,target_lowpass,min_output,max_output,output_lowpass,d_on_error

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PidGainsMsg, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.name is None:
        self.name = []
      if self.kp is None:
        self.kp = []
      if self.ki is None:
        self.ki = []
      if self.kd is None:
        self.kd = []
      if self.feed_forward is None:
        self.feed_forward = []
      if self.dead_zone is None:
        self.dead_zone = []
      if self.i_clamp is None:
        self.i_clamp = []
      if self.punch is None:
        self.punch = []
      if self.min_target is None:
        self.min_target = []
      if self.max_target is None:
        self.max_target = []
      if self.target_lowpass is None:
        self.target_lowpass = []
      if self.min_output is None:
        self.min_output = []
      if self.max_output is None:
        self.max_output = []
      if self.output_lowpass is None:
        self.output_lowpass = []
      if self.d_on_error is None:
        self.d_on_error = []
    else:
      self.name = []
      self.kp = []
      self.ki = []
      self.kd = []
      self.feed_forward = []
      self.dead_zone = []
      self.i_clamp = []
      self.punch = []
      self.min_target = []
      self.max_target = []
      self.target_lowpass = []
      self.min_output = []
      self.max_output = []
      self.output_lowpass = []
      self.d_on_error = []

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
      length = len(self.name)
      buff.write(_struct_I.pack(length))
      for val1 in self.name:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
      length = len(self.kp)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.kp))
      length = len(self.ki)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.ki))
      length = len(self.kd)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.kd))
      length = len(self.feed_forward)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.feed_forward))
      length = len(self.dead_zone)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.dead_zone))
      length = len(self.i_clamp)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.i_clamp))
      length = len(self.punch)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.punch))
      length = len(self.min_target)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.min_target))
      length = len(self.max_target)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.max_target))
      length = len(self.target_lowpass)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.target_lowpass))
      length = len(self.min_output)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.min_output))
      length = len(self.max_output)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.max_output))
      length = len(self.output_lowpass)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.output_lowpass))
      length = len(self.d_on_error)
      buff.write(_struct_I.pack(length))
      pattern = '<%sB'%length
      buff.write(struct.pack(pattern, *self.d_on_error))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.name = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8')
        else:
          val1 = str[start:end]
        self.name.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.kp = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.ki = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.kd = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.feed_forward = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.dead_zone = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.i_clamp = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.punch = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.min_target = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.max_target = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.target_lowpass = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.min_output = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.max_output = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.output_lowpass = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sB'%length
      start = end
      end += struct.calcsize(pattern)
      self.d_on_error = struct.unpack(pattern, str[start:end])
      self.d_on_error = map(bool, self.d_on_error)
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
      length = len(self.name)
      buff.write(_struct_I.pack(length))
      for val1 in self.name:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
      length = len(self.kp)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.kp.tostring())
      length = len(self.ki)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.ki.tostring())
      length = len(self.kd)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.kd.tostring())
      length = len(self.feed_forward)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.feed_forward.tostring())
      length = len(self.dead_zone)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.dead_zone.tostring())
      length = len(self.i_clamp)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.i_clamp.tostring())
      length = len(self.punch)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.punch.tostring())
      length = len(self.min_target)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.min_target.tostring())
      length = len(self.max_target)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.max_target.tostring())
      length = len(self.target_lowpass)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.target_lowpass.tostring())
      length = len(self.min_output)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.min_output.tostring())
      length = len(self.max_output)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.max_output.tostring())
      length = len(self.output_lowpass)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.output_lowpass.tostring())
      length = len(self.d_on_error)
      buff.write(_struct_I.pack(length))
      pattern = '<%sB'%length
      buff.write(self.d_on_error.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.name = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8')
        else:
          val1 = str[start:end]
        self.name.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.kp = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.ki = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.kd = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.feed_forward = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.dead_zone = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.i_clamp = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.punch = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.min_target = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.max_target = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.target_lowpass = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.min_output = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.max_output = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.output_lowpass = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sB'%length
      start = end
      end += struct.calcsize(pattern)
      self.d_on_error = numpy.frombuffer(str[start:end], dtype=numpy.bool, count=length)
      self.d_on_error = map(bool, self.d_on_error)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
