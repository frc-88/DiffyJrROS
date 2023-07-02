package frc.robot.ros.bridge;

import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.BridgeSubscriber;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.std_msgs.Float64;

public class PingPublisher implements Publisher {
    private final BridgeSubscriber<Float64> pingSendSub;
    private final BridgePublisher<Float64> pingReturnPub;

    public PingPublisher(ROSNetworkTablesBridge bridge) {
        pingSendSub = new BridgeSubscriber<>(bridge, "ping_send", Float64.class);
        pingReturnPub = new BridgePublisher<>(bridge, "ping_return");
    }

    public void publish() {
        Float64 ping;
        if ((ping = pingSendSub.receive()) != null) {
            pingReturnPub.send(ping);
        }
    }
}
