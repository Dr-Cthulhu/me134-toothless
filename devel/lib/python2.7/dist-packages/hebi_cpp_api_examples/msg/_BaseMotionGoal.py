# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hebi_cpp_api_examples/BaseMotionGoal.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class BaseMotionGoal(genpy.Message):
  _md5sum = "28f4303134ac5c0340dc34500202a721"
  _type = "hebi_cpp_api_examples/BaseMotionGoal"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
# Goal position (relative to start)
float64 x
float64 y
float64 theta

# Optionally, set a color when doing the move; otherwise, clear the color.
bool set_color
uint8 r
uint8 g
uint8 b
"""
  __slots__ = ['x','y','theta','set_color','r','g','b']
  _slot_types = ['float64','float64','float64','bool','uint8','uint8','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       x,y,theta,set_color,r,g,b

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(BaseMotionGoal, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.theta is None:
        self.theta = 0.
      if self.set_color is None:
        self.set_color = False
      if self.r is None:
        self.r = 0
      if self.g is None:
        self.g = 0
      if self.b is None:
        self.b = 0
    else:
      self.x = 0.
      self.y = 0.
      self.theta = 0.
      self.set_color = False
      self.r = 0
      self.g = 0
      self.b = 0

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
      buff.write(_get_struct_3d4B().pack(_x.x, _x.y, _x.theta, _x.set_color, _x.r, _x.g, _x.b))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 28
      (_x.x, _x.y, _x.theta, _x.set_color, _x.r, _x.g, _x.b,) = _get_struct_3d4B().unpack(str[start:end])
      self.set_color = bool(self.set_color)
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
      buff.write(_get_struct_3d4B().pack(_x.x, _x.y, _x.theta, _x.set_color, _x.r, _x.g, _x.b))
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
      _x = self
      start = end
      end += 28
      (_x.x, _x.y, _x.theta, _x.set_color, _x.r, _x.g, _x.b,) = _get_struct_3d4B().unpack(str[start:end])
      self.set_color = bool(self.set_color)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3d4B = None
def _get_struct_3d4B():
    global _struct_3d4B
    if _struct_3d4B is None:
        _struct_3d4B = struct.Struct("<3d4B")
    return _struct_3d4B
