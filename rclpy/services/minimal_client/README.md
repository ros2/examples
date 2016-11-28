# Minimal service client cookbook recipes

This package contains a few strategies to create service clients.
The `client` recipe shows how to request data from a service with a blocking call.
The `client_async` recipe shows how to request data from a service with a non blocking call. The user has to check if a response has been received in the main loop
The `client_async_member_function` recipe is analog to `client_async` but sends the request inside a MinimalClient class
