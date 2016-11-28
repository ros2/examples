# Minimal "subscriber" cookbook recipes

This package contains a few different strategies for creating short nodes that display received messages.
The `subscriber_old_school` recipe creates a listener node very similar to how it would be done in ROS1 using rospy.
The `subscriber_lambda` and `subscriber_local_function` shows how to embed the callback functions inside your main.
The `subscriber_member_function` created a class MinimalSubscriber that contains the callback keeping the main simple.
