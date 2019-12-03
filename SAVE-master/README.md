## SAVE
Simulated Autonomous Vehicle Engine

**roscore is the first thing you should run when using ROS.**

### ROS Tutorial [http://wiki.ros.org/ROS/Tutorials]: ###

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

From http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes:
`rosnode list` - lists all nodes running in ROS
`rosnode info /<node>` - provide more information on the given node

Running a new node:
`rosrun <package> <node>`

View nodes and their connections
`rosrun rqt_graph rqt_graph`

Topics and msg [http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics]:
```
rostopic -h - list available commands for rostopic command
	echo [topic] - Shows the data published for a topic

	list [/topic] - returns all topics currently subed and pubed. (-h for more details)

	type [/topic] - returns the message type of the published topic
			Can also use 'rosmsg show <msg type>'
```
