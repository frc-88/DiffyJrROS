package frc.robot.ros.bridge;

import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.BridgeSubscriber;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.std_msgs.RosFloat64;
import frc.team88.ros.messages.std_msgs.RosInt32;

public class PingPublisher implements Publisher {
    private final BridgeSubscriber<RosFloat64> pingSendSub;
    private final BridgePublisher<RosFloat64> pingReturnPub;
    private final BridgePublisher<RosInt32> powerModePub;
    private int powerMode = 0;

    public PingPublisher(ROSNetworkTablesBridge bridge, int powerMode) {
        pingSendSub = new BridgeSubscriber<>(bridge, "ping_send", RosFloat64.class);
        pingReturnPub = new BridgePublisher<>(bridge, "ping_return");
        powerModePub = new BridgePublisher<>(bridge, "power_mode");
        this.powerMode = powerMode;
    }

    public void publish() {
        RosFloat64 ping;
        if ((ping = pingSendSub.receive()) != null) {
            powerModePub.send(new RosInt32(this.powerMode));
            pingReturnPub.send(ping);
        }
    }
}
