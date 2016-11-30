# Minimal "publisher" cookbook recipes

This package contains a few different strategies for creating short nodes that blast out messages.
The `publisher_old_school` recipe creates a talker node very similar to how it would be done in ROS 1 using rospy.
The `publisher_local_function` recipe shows how to leverage the timers provided by ROS 2 to trigger message publication.
The `publisher_member_function` recipe creates a class MinimalPublisher that sends messages periodically.
