## SAVE
### Simulated Autonomous Vehicle Engine

### Installation

How to install ROS on Windows:
http://wiki.ros.org/Installation/Windows

How to use AirSim with ROS:
https://microsoft.github.io/AirSim/docs/ros/


All commands below must be run in a ROS console. A ROS console is one which has been setup with the appropriate setup scripts and has the necessary environment variables setup.

It is recommended that developers of the system use Visual Studio Code. It is an open source IDE that contains excellent support Python and ROS APIs.

**`roscore` is the first thing you should run when using ROS.**

### ROS Tutorial [http://wiki.ros.org/ROS/Tutorials]: ###

#### Frequently used commands:

1. Creating or building a ROS workspace:
`catkin_make when in project folder...`

2. Setup the project
`.\devel\setup.bat`

	Ensure package path is setup:
	echo %ROS_PACKAGE_PATH%

Getting to the log package where the logs are stored:
`roscd log`

`rosls` looks at package without absolute path.

Packages are stored in the src of the project.

Creating a new package:
Ensure that you are in the src directory
`catkin_create_pkg <package name>`

Build package:
Run steps 1 and 2

Enable AirSim to be used with ROS
```
# From https://microsoft.github.io/AirSim/docs/ros/
# copy package
mkdir -p ../catkin_ws/src/airsim/scripts/airsim
cp AirSim/PythonClient/airsim/*.py ../catkin_ws/src/airsim/scripts/airsim

# copy ROS examples
cp AirSim/PythonClient/ros/*.py ../catkin_ws/src/airsim/scripts
```

#### ROS Nodes:

From http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes:
`rosnode list` - lists all nodes running in ROS
`rosnode info /<node>` - provide more information on the given node

Running a new node:
`rosrun <package> <node>`

View nodes and their connections
`rosrun rqt_graph rqt_graph`

#### Topics and msg [http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics]:
```
rostopic -h - list available commands for rostopic command
	echo [topic] - Shows the data published for a topic

	list [/topic] - returns all topics currently subed and pubed. (-h for more details)

	type [/topic] - returns the message type of the published topic
			Can also use 'rosmsg show <msg type>'
```

Publish to a topic from the command line for testing purposes:
`rostopic pub [topic] [msg_type] [args]`
Ex:
`rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'`
http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics

#### ROS Services [http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams]:
`rosservice -h` for further details.

##### Process of making Services and their messages [http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv]:
The link above contains the valid type fields of messages and also here: http://wiki.ros.org/msg

Messages are used with Messages are contained in a `msg` folder in ROS package directories.

The `package.xml` and the `CMakeLists.txt` of the package must be updated to allow messages and services in the package.

`rosmsg` - Command to interact with messages
`rossrv` - Command to interact with services

Creating Publisher/Subscriber in Rospy: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
Creating services in Rospy: http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29

#### Logging in ROS [http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch]:

The link above contains an example on how to use the RQT Console and Logger.
`rosrun rqt_console rqt_console` - Open the console for logging of ROS nodes
`rosrun rqt_logger_level rqt_logger_level` - Open an application to control verbosity of logging

The link above also contains information on how to create a launch file to start multiple nodes using the `roslaunch` command and a `.launch` file.

### Troubleshooting
If you are unable to find the SAVE packages in the windows workspace, please ensure that `ROS_PACKAGE_PATH` in the devel/setup.bat is setup correctly and there are no spaces in the path. Spaces may create issues for users.
