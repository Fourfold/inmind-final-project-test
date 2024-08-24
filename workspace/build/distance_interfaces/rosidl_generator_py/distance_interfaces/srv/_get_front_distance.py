# generated from rosidl_generator_py/resource/_idl.py.em
# with input from distance_interfaces:srv/GetFrontDistance.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetFrontDistance_Request(type):
    """Metaclass of message 'GetFrontDistance_Request'."""

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
            module = import_type_support('distance_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'distance_interfaces.srv.GetFrontDistance_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_front_distance__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_front_distance__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_front_distance__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_front_distance__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_front_distance__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetFrontDistance_Request(metaclass=Metaclass_GetFrontDistance_Request):
    """Message class 'GetFrontDistance_Request'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_GetFrontDistance_Response(type):
    """Metaclass of message 'GetFrontDistance_Response'."""

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
            module = import_type_support('distance_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'distance_interfaces.srv.GetFrontDistance_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_front_distance__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_front_distance__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_front_distance__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_front_distance__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_front_distance__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetFrontDistance_Response(metaclass=Metaclass_GetFrontDistance_Response):
    """Message class 'GetFrontDistance_Response'."""

    __slots__ = [
        '_front_distance',
    ]

    _fields_and_field_types = {
        'front_distance': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.front_distance = kwargs.get('front_distance', float())

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
        if self.front_distance != other.front_distance:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def front_distance(self):
        """Message field 'front_distance'."""
        return self._front_distance

    @front_distance.setter
    def front_distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'front_distance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'front_distance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._front_distance = value


class Metaclass_GetFrontDistance(type):
    """Metaclass of service 'GetFrontDistance'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('distance_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'distance_interfaces.srv.GetFrontDistance')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_front_distance

            from distance_interfaces.srv import _get_front_distance
            if _get_front_distance.Metaclass_GetFrontDistance_Request._TYPE_SUPPORT is None:
                _get_front_distance.Metaclass_GetFrontDistance_Request.__import_type_support__()
            if _get_front_distance.Metaclass_GetFrontDistance_Response._TYPE_SUPPORT is None:
                _get_front_distance.Metaclass_GetFrontDistance_Response.__import_type_support__()


class GetFrontDistance(metaclass=Metaclass_GetFrontDistance):
    from distance_interfaces.srv._get_front_distance import GetFrontDistance_Request as Request
    from distance_interfaces.srv._get_front_distance import GetFrontDistance_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
