BASE_DIR=$(realpath "$(dirname $0)")

GEN_DIR=${BASE_DIR}/ros_networktables_bridge_genmsg
GEN_MSG_ROOT=${GEN_DIR}/genmsg
SOURCES_PATH=${BASE_DIR}/source_list.json

${GEN_DIR}/build-rospy-messages.sh ${SOURCES_PATH}
source ${GEN_DIR}/venv/bin/activate
source ${GEN_MSG_ROOT}/set_build_python_path.sh
python3 ${GEN_DIR}/main.py -r 'src/main/java' -m 'frc/robot/ros/messages' -s ${SOURCES_PATH}
