package frc.robot.ros.bridge;

import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.BridgeSubscriber;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.std_msgs.Float64;

public class PingPublisher implements Publisher {
    private final BridgeSubscriber<Float64> m_pingSendSub;
    private final BridgePublisher<Float64> m_pingReturnPub;

    public PingPublisher(ROSNetworkTablesBridge bridge) {
        m_pingSendSub = new BridgeSubscriber<>(bridge, "ping_send", Float64.class);
        m_pingReturnPub = new BridgePublisher<>(bridge, "ping_return");
    }

    public void publish() {
        Float64 ping;
        if ((ping = m_pingSendSub.receive()) != null) {
            m_pingReturnPub.send(ping);
        }
    }
}
