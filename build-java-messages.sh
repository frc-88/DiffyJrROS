BASE_DIR=$(realpath "$(dirname $0)")

GEN_DIR=${BASE_DIR}/ros_networktables_bridge_genmsg
GEN_MSG_ROOT=${GEN_DIR}/genmsg

${GEN_DIR}/build-rospy-messages.sh ~/tj2_ros/src/tj2_interfaces
export PYTHONPATH=${GEN_MSG_ROOT}/tj2_interfaces:$PYTHONPATH
source ${GEN_DIR}/venv/bin/activate
python3 ${GEN_DIR}/clone_repos.py ${BASE_DIR}/source_list.json ${GEN_MSG_ROOT}
python3 ${GEN_DIR}/main.py -r 'src/main/java' -m 'frc/robot/ros/messages' -s ${BASE_DIR}/source_list.json
