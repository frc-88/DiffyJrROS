package frc.robot.ros.bridge;

import java.util.Base64;

import edu.wpi.first.networktables.StringPublisher;
import frc.robot.ros.messages.RosMessage;

public class BridgePublisher<T extends RosMessage> {
    private ROSNetworkTablesBridge bridge;
    private String topicName;
    private StringPublisher pub;

    public BridgePublisher(ROSNetworkTablesBridge bridge, String topicName) {
        this.bridge = bridge;
        this.topicName = topicName;
        this.pub = this.bridge.advertise(this.topicName);
    }

    public void send(T msg) {
        String input = msg.toString();
        String encodedString = Base64.getEncoder().encodeToString(input.getBytes());
        this.pub.set(encodedString);
    }
}
