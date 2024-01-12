package frc.robot.ros.bridge;

import java.util.HashMap;
import java.util.Map;

import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.BridgeSubscriber;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.std_msgs.RosFloat64;

public class JointManager {
    private final ROSNetworkTablesBridge bridge;
    private final Map<String, BridgePublisher<RosFloat64>> jointSendTopics = new HashMap<>();
    private final Map<String, BridgeSubscriber<RosFloat64>> jointReceiveTopics = new HashMap<>();
    private final Map<String, Double> jointCommands = new HashMap<>();
    private final String namespace;

    public JointManager(ROSNetworkTablesBridge bridge, String namespace) {
        this.bridge = bridge;
        if (namespace.charAt(namespace.length() - 1) != '/') {
            namespace += "/";
        }
        this.namespace = namespace;
    }

    public void addJoint(String name) {
        if (name.charAt(0) == '/') {
            name = name.substring(1, name.length());
        }
        String sendTopic = namespace + name;
        jointSendTopics.put(name, new BridgePublisher<>(bridge, sendTopic));
        String commandTopic = namespace + name + "/command";
        jointReceiveTopics.put(name, new BridgeSubscriber<>(bridge, commandTopic, RosFloat64.class));
        jointCommands.put(name, 0.0);
    }

    public boolean isJoint(String name) {
        return jointSendTopics.containsKey(name);
    }

    public void sendJointPosition(String name, double position) {
        if (!isJoint(name)) {
            addJoint(name);
        }
        jointSendTopics.get(name).send(new RosFloat64(position));
    }

    public void update() {
        for (String name : jointReceiveTopics.keySet()) {
            RosFloat64 command;
            if ((command = jointReceiveTopics.get(name).receive()) != null) {
                jointCommands.put(name, command.getData());
            }
        }
    }

    public double getJointCommand(String name) {
        update();
        return jointCommands.get(name);
    }
}
