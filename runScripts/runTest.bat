:: Run ROSCORE
start roscore

:: RUN THE SIMULATOR
D:
cd D:\Local Documents\Utilities\AirSim\Neighborhood
start run.bat

:: WAIT FOR THE SIMULATOR TO START
TIMEOUT 10

:: RUN THE CONTROLLER
D:
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\controls\scripts
start python controller.py

:: RUN THE ACTION TRANSLATOR
D:
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\translators\scripts
start python actionTranslatorNavAgent.py

:: RUN THE ACC
D:
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\acc\scripts
start python acc.py

:: RUN SPEEDOMETER
D:
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\sensors\scripts
start python speedometer.py

:: RUN GPS SENSOR
D:
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\navigation\scripts
start python gpsSensor.py

:: RUN COMPASS
D:
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\sensors\scripts
start python compass.py

:: RUN CAMERA
D:
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\sensors\scripts
start python car_image_raw.py

:: RUN OBSTACLE AVOID
D:
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\obstacle_avoid\scripts
start python obstacle_avoid.py

:: RUN LANE KEEP ASSIST
D:
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\lka\scripts
start python lane_keep.py

:: RUN PERCEPTION TRANSLATOR
D:
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\translators\scripts
start python perceptionTranslatorNavAgent.py

:: RUN THE REASONER
D:
cd D:\Local Documents\ROS_Workspaces\SAVI_ROS\rosjavaWorkspace\src\savi_ros_java\savi_ros_bdi\build\install\savi_ros_bdi\bin
start .\savi_ros_bdi.bat savi_ros_java.savi_ros_bdi.SAVI_Main

:: Run THE PYTHON REASONER
::D:
::cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
::call setup.bat
::cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\controls\scripts
::start python traditionalAgent.py

:: RUN USER INTERFACE
D:
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\translators\scripts
start python userInterface.py

:: RUN LOGGER
D:
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd D:\Local Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\translators\scripts
start python logger.py