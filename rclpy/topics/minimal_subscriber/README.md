# Minimal "subscriber" cookbook recipes

This package contains a few different strategies for creating short nodes that display received messages.
The `subscriber_lambda` recipe shows how to embed the callback functions inside your main.
The `subscriber_member_function` recipe creates a class MinimalSubscriber that contains the callback, keeping the main simple.
