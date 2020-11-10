REM Start roscore
REM C:\Windows\System32\cmd.exe /k "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64&& set ChocolateyInstall=c:\opt\chocolatey&& c:\opt\ros\melodic\x64\setup.bat \k roscore
start cmd.exe @cmd /k "runRosCore.bat"

C:\Users\Patrick\AppData\Roaming\Microsoft\Windows\Start Menu\Programs\ROS Melodic.lnk /k roscore


REM start savi_ros_bdi
REM D:
REM cd D:\Local Documents\ROS_Workspaces\SAVI_ROS\rosjavaWorkspace\src\savi_ros_java\savi_ros_bdi\build\install\savi_ros_bdi\bin
REM start cmd.exe @cmd /k ".\savi_ros_bdi.bat savi_ros_java.savi_ros_bdi.SAVI_Main"

REM
