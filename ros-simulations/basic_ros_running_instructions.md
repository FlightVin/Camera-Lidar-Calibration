# Basic Instructions for running ROS

**NOTE** Do not use conda environments

## Initializing ros
1. Source ros noetic - `source /opt/ros/noetic/setup.bash`

## Ros Nodes
1. Run master using `roscore`
2. Run nodes using `rosrun <package_name> <node>`
3. `rqt_graph` in another terminal to show ros graph (all nodes and topics)

## catkin workspaces
1. create `~/catkin_ws/src`
2. move into `catkin_ws` and run `catkin_make`
3. `source ~/catkin_ws/devel/setup.bash`

## creating ros packages
1. go to src folder
2. use `catkin_create_pkg <pkg_name> <dependency_pkg1> <dependency_pkg2>` dependency packages are like rospy, turtlesim

## writing nodes
1. go into `catkin_ws/src/<package_name>` and create a a folder for python scripts say `scripts`
2. create a python file and do `chmod +x *.py`
3. define at top - #!/usr/bin... (path of python)

Example code
```py
import rospy

if __name__ == "__main__":
	rospy.init_node("test_node")

	rospy.loginfo("hello from test node")

	# also have logerr, logwarn

	rospy.sleep(1)

# this program exits after sleeping
```
4. start roscore and just do `python3 file.py`
5. can also do `rosrun <pkg_name> file.py`

Example code
```py
import rospy

if __name__ == "__main__":
        rospy.init_node("test_node")

	rate = rospy.Rate(10) # number of times per second

	while not rospy.is_shutdown():
		# code

		rate.sleep()
```
6. `rosnode list` lists all nodes running
7. `rosnode kill <node_name>` for killing the node

## Ros topics
1. `rostopic list`
2. `rostopic info <topic_name>`
3. `rostopic ehco <topic_name>` - does the same thing as a listener

## Ros Publishers
1. create a new script in the package

Example for making turtlesium to move in a circle
```py
import rospy
from geometry_msgs.msg import Twist

if __name__ == "__main__":
        rospy.init_node("test_node")

	pub = rospy.Publisher("<get_exact_topic_name>", data_type)
	# get the first from rostopic list and the data type from rostopic info topic_name
	# if the message is complex chec out rosmsg show geometry_msg/twist (you get the name from info)
	

	# here it would be
	pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 10) # queue size is basically buffer

	# remember to add 
	# <build_depend>geometry_msgs<build_depend>
	# and same for <export...> and <exec...>
	# in src of catkin_ws


	# make a loop and a rate and use pub.publish(mgs)
	# the msg can be done by msg = Twist(); msg.linear.x = ... etc


```

## Ros subscriber
1. in a new script use `sub = rospy.Subscriber("/turtle1/pose", Pose, callback = call_back_func)
2. This will only print once. Add `rospy.spin()` that causes an infinite loop - more passive as it creates threads

**NOTE** We can combine pubs and subs within the same script - you can publish in the subscriber's call back function


Followed this - https://www.youtube.com/watch?v=GAJ3c5XmJSA&list=PLLSegLrePWgIbIrA4iehUQ-impvIXdd9Q&index=6

