package frc.robot.ros.bridge;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;

public class ROSNetworkTablesBridge {
    private NetworkTableInstance instance;
    private NetworkTable rosToNtSubtable;
    private NetworkTable ntToRosSubtable;
    private double updateInterval;

    public ROSNetworkTablesBridge(String address, int port, double updateInterval) {
        instance = NetworkTableInstance.create();
        instance.startClient3("bridge");
        instance.setServer(address, port);
        this.updateInterval = updateInterval;

        rosToNtSubtable = instance.getTable("ros_to_nt");
        ntToRosSubtable = instance.getTable("nt_to_ros");
    }

    public StringPublisher advertise(String topicName) {
        StringPublisher pub = ntToRosSubtable.getStringTopic(topicName)
                .publish(PubSubOption.periodic(this.updateInterval));
        pub.set("");
        return pub;
    }

    public StringSubscriber subscribe(String topicName) {
        return rosToNtSubtable.getStringTopic(topicName).subscribe("", PubSubOption.sendAll(true),
                PubSubOption.periodic(this.updateInterval));
    }
}
