# A single package with the action interface and the client and server.

This package contains an few example which show how to create a single package with the interface, client and server, all in one.

The main difference between the msg/srv interface that was included into a package with it's client and service is that an action client and server needs to have both the library and executable included in the rosidl_typesupport_cpp in the CMakeLists file.

