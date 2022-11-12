# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from nlink_parser/LinktrackNode4Tag.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import nlink_parser.msg

class LinktrackNode4Tag(genpy.Message):
  _md5sum = "52d7d856087ab9caa357ff4bd2e80950"
  _type = "nlink_parser/LinktrackNode4Tag"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """uint8 id
float32 voltage
LinktrackNode4Anchor[] anchors

================================================================================
MSG: nlink_parser/LinktrackNode4Anchor
uint8 id
float32 dis
"""
  __slots__ = ['id','voltage','anchors']
  _slot_types = ['uint8','float32','nlink_parser/LinktrackNode4Anchor[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       id,voltage,anchors

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(LinktrackNode4Tag, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.id is None:
        self.id = 0
      if self.voltage is None:
        self.voltage = 0.
      if self.anchors is None:
        self.anchors = []
    else:
      self.id = 0
      self.voltage = 0.
      self.anchors = []

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
      buff.write(_get_struct_Bf().pack(_x.id, _x.voltage))
      length = len(self.anchors)
      buff.write(_struct_I.pack(length))
      for val1 in self.anchors:
        _x = val1
        buff.write(_get_struct_Bf().pack(_x.id, _x.dis))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.anchors is None:
        self.anchors = None
      end = 0
      _x = self
      start = end
      end += 5
      (_x.id, _x.voltage,) = _get_struct_Bf().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.anchors = []
      for i in range(0, length):
        val1 = nlink_parser.msg.LinktrackNode4Anchor()
        _x = val1
        start = end
        end += 5
        (_x.id, _x.dis,) = _get_struct_Bf().unpack(str[start:end])
        self.anchors.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_Bf().pack(_x.id, _x.voltage))
      length = len(self.anchors)
      buff.write(_struct_I.pack(length))
      for val1 in self.anchors:
        _x = val1
        buff.write(_get_struct_Bf().pack(_x.id, _x.dis))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.anchors is None:
        self.anchors = None
      end = 0
      _x = self
      start = end
      end += 5
      (_x.id, _x.voltage,) = _get_struct_Bf().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.anchors = []
      for i in range(0, length):
        val1 = nlink_parser.msg.LinktrackNode4Anchor()
        _x = val1
        start = end
        end += 5
        (_x.id, _x.dis,) = _get_struct_Bf().unpack(str[start:end])
        self.anchors.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_Bf = None
def _get_struct_Bf():
    global _struct_Bf
    if _struct_Bf is None:
        _struct_Bf = struct.Struct("<Bf")
    return _struct_Bf
