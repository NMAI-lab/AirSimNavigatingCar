:: Run ROSCORE
start roscore

:: RUN THE SIMULATOR
cd C:\Documents\Utilities\AirSim\Neighborhood
start run.bat

:: WAIT FOR THE SIMULATOR TO START
TIMEOUT 20

:: RUN THE CONTROLLER
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\controls\scripts
start python controller.py
TIMEOUT 2

:: RUN THE ACTION TRANSLATOR
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\translators\scripts
start python actionTranslatorNavAgent.py
TIMEOUT 2

:: RUN THE ACC
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\acc\scripts
start python acc.py
TIMEOUT 2

:: RUN SPEEDOMETER
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\sensors\scripts
start python speedometer.py
TIMEOUT 2

:: RUN GPS SENSOR
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\navigation\scripts
start python gpsSensor.py
TIMEOUT 2

:: RUN COMPASS
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\sensors\scripts
start python compass.py
TIMEOUT 2

:: RUN CAMERA
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\sensors\scripts
start python car_image_raw.py
TIMEOUT 2

:: RUN OBSTACLE AVOID
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\obstacle_avoid\scripts
start python obstacle_avoid.py
TIMEOUT 2

:: RUN LANE KEEP ASSIST
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\lka\scripts
start python lane_keep.py
TIMEOUT 2

:: RUN PERCEPTION TRANSLATOR
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\translators\scripts
start python perceptionTranslatorNavAgent.py
TIMEOUT 2

:: RUN THE REASONER
::cd C:\Users\Patrick\Documents\ROS_Workspaces\SAVI_ROS\rosjavaWorkspace\src\savi_ros_java\savi_ros_bdi\build\install\savi_ros_bdi\bin
::start .\savi_ros_bdi.bat savi_ros_java.savi_ros_bdi.SAVI_Main
::TIMEOUT 2

:: Run THE PYTHON REASONER
::cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
::call setup.bat
::cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\controls\scripts
::start python traditionalAgent.py
TIMEOUT 2

:: Run THE STATE MACHINE REASONER
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\controls\scripts
start python stateMachineAgent.py
TIMEOUT 2

:: RUN USER INTERFACE
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\translators\scripts
start python userInterface.py
TIMEOUT 2

:: RUN LOGGER
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\devel
call setup.bat
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\catkin_ws\src\translators\scripts
start python logger.py
TIMEOUT 2

:: Get back to the start directory
cd C:\Users\Patrick\Documents\ROS_Workspaces\AirSimNavigatingCar\runScripts