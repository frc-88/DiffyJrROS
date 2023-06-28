package frc.robot.ros.bridge;

import java.util.HashMap;
import java.util.Map;

import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.std_msgs.Float64;

public class JointManager {
    private final ROSNetworkTablesBridge bridge;
    private final Map<String, BridgePublisher<Float64>> joints = new HashMap<>();
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
        String topic = namespace + name;
        joints.put(name, new BridgePublisher<>(bridge, topic));
    }

    public boolean isJoint(String name) {
        return joints.containsKey(name);
    }

    public void sendJointPosition(String name, double position) {
        joints.get(name).send(new Float64(position));
    }
}
