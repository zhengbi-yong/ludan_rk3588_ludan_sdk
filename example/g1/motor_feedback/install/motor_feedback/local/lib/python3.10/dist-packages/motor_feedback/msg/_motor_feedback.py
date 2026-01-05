# generated from rosidl_generator_py/resource/_idl.py.em
# with input from motor_feedback:msg/MotorFeedback.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'can_data'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MotorFeedback(type):
    """Metaclass of message 'MotorFeedback'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('motor_feedback')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'motor_feedback.msg.MotorFeedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motor_feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motor_feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motor_feedback
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motor_feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motor_feedback

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MotorFeedback(metaclass=Metaclass_MotorFeedback):
    """Message class 'MotorFeedback'."""

    __slots__ = [
        '_header',
        '_motor_id',
        '_can_id',
        '_error',
        '_position',
        '_velocity',
        '_torque',
        '_temp_mos',
        '_temp_rotor',
        '_position_rad',
        '_velocity_rad_s',
        '_torque_nm',
        '_can_data',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'motor_id': 'int32',
        'can_id': 'int32',
        'error': 'int32',
        'position': 'int16',
        'velocity': 'int16',
        'torque': 'int16',
        'temp_mos': 'int8',
        'temp_rotor': 'int8',
        'position_rad': 'float',
        'velocity_rad_s': 'float',
        'torque_nm': 'float',
        'can_data': 'uint8[8]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint8'), 8),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.motor_id = kwargs.get('motor_id', int())
        self.can_id = kwargs.get('can_id', int())
        self.error = kwargs.get('error', int())
        self.position = kwargs.get('position', int())
        self.velocity = kwargs.get('velocity', int())
        self.torque = kwargs.get('torque', int())
        self.temp_mos = kwargs.get('temp_mos', int())
        self.temp_rotor = kwargs.get('temp_rotor', int())
        self.position_rad = kwargs.get('position_rad', float())
        self.velocity_rad_s = kwargs.get('velocity_rad_s', float())
        self.torque_nm = kwargs.get('torque_nm', float())
        if 'can_data' not in kwargs:
            self.can_data = numpy.zeros(8, dtype=numpy.uint8)
        else:
            self.can_data = kwargs.get('can_data')

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.motor_id != other.motor_id:
            return False
        if self.can_id != other.can_id:
            return False
        if self.error != other.error:
            return False
        if self.position != other.position:
            return False
        if self.velocity != other.velocity:
            return False
        if self.torque != other.torque:
            return False
        if self.temp_mos != other.temp_mos:
            return False
        if self.temp_rotor != other.temp_rotor:
            return False
        if self.position_rad != other.position_rad:
            return False
        if self.velocity_rad_s != other.velocity_rad_s:
            return False
        if self.torque_nm != other.torque_nm:
            return False
        if any(self.can_data != other.can_data):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def motor_id(self):
        """Message field 'motor_id'."""
        return self._motor_id

    @motor_id.setter
    def motor_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motor_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'motor_id' field must be an integer in [-2147483648, 2147483647]"
        self._motor_id = value

    @builtins.property
    def can_id(self):
        """Message field 'can_id'."""
        return self._can_id

    @can_id.setter
    def can_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'can_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'can_id' field must be an integer in [-2147483648, 2147483647]"
        self._can_id = value

    @builtins.property
    def error(self):
        """Message field 'error'."""
        return self._error

    @error.setter
    def error(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'error' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'error' field must be an integer in [-2147483648, 2147483647]"
        self._error = value

    @builtins.property
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'position' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'position' field must be an integer in [-32768, 32767]"
        self._position = value

    @builtins.property
    def velocity(self):
        """Message field 'velocity'."""
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'velocity' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'velocity' field must be an integer in [-32768, 32767]"
        self._velocity = value

    @builtins.property
    def torque(self):
        """Message field 'torque'."""
        return self._torque

    @torque.setter
    def torque(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'torque' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'torque' field must be an integer in [-32768, 32767]"
        self._torque = value

    @builtins.property
    def temp_mos(self):
        """Message field 'temp_mos'."""
        return self._temp_mos

    @temp_mos.setter
    def temp_mos(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'temp_mos' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'temp_mos' field must be an integer in [-128, 127]"
        self._temp_mos = value

    @builtins.property
    def temp_rotor(self):
        """Message field 'temp_rotor'."""
        return self._temp_rotor

    @temp_rotor.setter
    def temp_rotor(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'temp_rotor' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'temp_rotor' field must be an integer in [-128, 127]"
        self._temp_rotor = value

    @builtins.property
    def position_rad(self):
        """Message field 'position_rad'."""
        return self._position_rad

    @position_rad.setter
    def position_rad(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'position_rad' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'position_rad' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._position_rad = value

    @builtins.property
    def velocity_rad_s(self):
        """Message field 'velocity_rad_s'."""
        return self._velocity_rad_s

    @velocity_rad_s.setter
    def velocity_rad_s(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'velocity_rad_s' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'velocity_rad_s' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._velocity_rad_s = value

    @builtins.property
    def torque_nm(self):
        """Message field 'torque_nm'."""
        return self._torque_nm

    @torque_nm.setter
    def torque_nm(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'torque_nm' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'torque_nm' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._torque_nm = value

    @builtins.property
    def can_data(self):
        """Message field 'can_data'."""
        return self._can_data

    @can_data.setter
    def can_data(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint8, \
                "The 'can_data' numpy.ndarray() must have the dtype of 'numpy.uint8'"
            assert value.size == 8, \
                "The 'can_data' numpy.ndarray() must have a size of 8"
            self._can_data = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 8 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'can_data' field must be a set or sequence with length 8 and each value of type 'int' and each unsigned integer in [0, 255]"
        self._can_data = numpy.array(value, dtype=numpy.uint8)
