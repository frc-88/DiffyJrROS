@echo off

SET BASE_DIR=%~dp0

SET GEN_DIR=%BASE_DIR%\ros_networktables_bridge_genmsg
SET GEN_MSG_ROOT=%GEN_DIR%\genmsg

call %GEN_DIR%\build-rospy-messages.bat %userprofile%\tj2_ros\src\tj2_interfaces
set PYTHONPATH=%GEN_MSG_ROOT%\tj2_interfaces:%PYTHONPATH%
call "%GEN_DIR%\venv\Scripts\activate"
SET BASE_DIR=%~dp0
python "%GEN_DIR%\clone_repos.py" "%BASE_DIR%\source_list.json" "%GEN_MSG_ROOT%"
python "%GEN_DIR%\main.py" "-r" "src/main/java" "-m" "frc/robot/ros/messages" "-s" "%BASE_DIR%\source_list.json"
