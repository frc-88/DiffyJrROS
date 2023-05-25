@echo off

SET BASE_DIR=%~dp0

SET GEN_DIR=%BASE_DIR%ros_networktables_bridge_genmsg
SET GEN_MSG_ROOT=%GEN_DIR%\genmsg
SET SOURCES_PATH=%BASE_DIR%source_list.json

call "%GEN_DIR%\build-rospy-messages.bat" "%SOURCES_PATH%"
call "%GEN_DIR%\venv\Scripts\activate"
call "%GEN_MSG_ROOT%\set_build_python_path.bat"
python "%GEN_DIR%\main.py" "-r" "src/main/java" "-m" "frc/robot/ros/messages" "-s" "%SOURCES_PATH%"
