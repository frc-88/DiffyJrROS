package frc.robot.ros.bridge;

import java.util.Optional;

import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.BridgeSubscriber;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.std_msgs.RosFloat64;

public class PingPublisher implements Publisher {
    private final BridgeSubscriber<RosFloat64> pingSendSub;
    private final BridgePublisher<RosFloat64> pingReturnPub;

    public PingPublisher(ROSNetworkTablesBridge bridge) {
        pingSendSub = new BridgeSubscriber<>(bridge, "ping_send", RosFloat64.class);
        pingReturnPub = new BridgePublisher<>(bridge, "ping_return");
    }

    public void publish() {
        Optional<RosFloat64> ping;
        if ((ping = pingSendSub.receive()).isPresent()) {
            pingReturnPub.send(ping.get());
        }
    }
}
