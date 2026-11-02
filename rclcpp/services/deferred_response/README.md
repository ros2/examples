# Deferred service response

This package demonstrates how a service can defer sending its response. The
server waits 5 seconds after receiving an `AddTwoInts` request, then sends the
response using the service handle and request header passed to its callback.
