package frc.robot.ros.bridge;

import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.TimePrimitive;
import frc.team88.ros.messages.std_msgs.RosTime;

public class BagManager {
    private final BridgePublisher<RosTime> startBagPub;
    private final BridgePublisher<RosTime> startSvoPub;
    private final BridgePublisher<RosTime> stopBagPub;
    private boolean isRecording = false;

    public BagManager(ROSNetworkTablesBridge bridge) {
        startBagPub = new BridgePublisher<>(bridge, "start_bag");
        startSvoPub = new BridgePublisher<>(bridge, "start_svo");
        stopBagPub = new BridgePublisher<>(bridge, "stop_bag");
    }

    public void startBag() {
        isRecording = true;
        TimePrimitive now = startBagPub.getNow();
        startBagPub.send(new RosTime(now));
    }

    public void stopBag() {
        isRecording = false;
        TimePrimitive now = stopBagPub.getNow();
        stopBagPub.send(new RosTime(now));
    }

    public void startSvo() {
        isRecording = true;
        TimePrimitive now = startSvoPub.getNow();
        startSvoPub.send(new RosTime(now));
    }

    public boolean isRecording() {
        return isRecording;
    }
}
