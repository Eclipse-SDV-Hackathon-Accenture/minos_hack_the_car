# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: datatypes/ros/sensor_msgs/Bump.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from datatypes.ros.std_msgs import Header_pb2 as datatypes_dot_ros_dot_std__msgs_dot_Header__pb2
from datatypes.ros.sensor_msgs import CompressedImage_pb2 as datatypes_dot_ros_dot_sensor__msgs_dot_CompressedImage__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='datatypes/ros/sensor_msgs/Bump.proto',
  package='ros.sensor_msgs',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=b'\n$datatypes/ros/sensor_msgs/Bump.proto\x12\x0fros.sensor_msgs\x1a#datatypes/ros/std_msgs/Header.proto\x1a/datatypes/ros/sensor_msgs/CompressedImage.proto\"\xa1\x01\n\x04\x42ump\x12$\n\x06header\x18\x01 \x01(\x0b\x32\x14.ros.std_msgs.Header\x12\x10\n\x08latitude\x18\x02 \x01(\x01\x12\x11\n\tlongitude\x18\x03 \x01(\x01\x12\x10\n\x08\x61ltitude\x18\x04 \x01(\x01\x12\r\n\x05level\x18\x05 \x01(\x05\x12-\n\x03img\x18\x06 \x01(\x0b\x32 .ros.sensor_msgs.CompressedImageb\x06proto3'
  ,
  dependencies=[datatypes_dot_ros_dot_std__msgs_dot_Header__pb2.DESCRIPTOR,datatypes_dot_ros_dot_sensor__msgs_dot_CompressedImage__pb2.DESCRIPTOR,])




_BUMP = _descriptor.Descriptor(
  name='Bump',
  full_name='ros.sensor_msgs.Bump',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='ros.sensor_msgs.Bump.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='latitude', full_name='ros.sensor_msgs.Bump.latitude', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='longitude', full_name='ros.sensor_msgs.Bump.longitude', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='altitude', full_name='ros.sensor_msgs.Bump.altitude', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='level', full_name='ros.sensor_msgs.Bump.level', index=4,
      number=5, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='img', full_name='ros.sensor_msgs.Bump.img', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=144,
  serialized_end=305,
)

_BUMP.fields_by_name['header'].message_type = datatypes_dot_ros_dot_std__msgs_dot_Header__pb2._HEADER
_BUMP.fields_by_name['img'].message_type = datatypes_dot_ros_dot_sensor__msgs_dot_CompressedImage__pb2._COMPRESSEDIMAGE
DESCRIPTOR.message_types_by_name['Bump'] = _BUMP
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Bump = _reflection.GeneratedProtocolMessageType('Bump', (_message.Message,), {
  'DESCRIPTOR' : _BUMP,
  '__module__' : 'datatypes.ros.sensor_msgs.Bump_pb2'
  # @@protoc_insertion_point(class_scope:ros.sensor_msgs.Bump)
  })
_sym_db.RegisterMessage(Bump)


# @@protoc_insertion_point(module_scope)