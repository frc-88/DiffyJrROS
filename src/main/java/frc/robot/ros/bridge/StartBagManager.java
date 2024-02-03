package frc.robot.ros.bridge;

import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.TimePrimitive;
import frc.team88.ros.messages.std_msgs.RosTime;

public class StartBagManager {
    private final BridgePublisher<RosTime> startBagPub;

    public StartBagManager(ROSNetworkTablesBridge bridge) {
        startBagPub = new BridgePublisher<>(bridge, "start_bag");
    }

    public void startBag() {
        TimePrimitive now = startBagPub.getNow();
        startBagPub.send(new RosTime(now));
    }
}
