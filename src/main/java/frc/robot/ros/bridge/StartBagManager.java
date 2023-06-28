package frc.robot.ros.bridge;

import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.TimePrimitive;
import frc.team88.ros.messages.std_msgs.Time;

public class StartBagManager {
    private final BridgePublisher<Time> m_startBagPub;

    public StartBagManager(ROSNetworkTablesBridge bridge) {
        m_startBagPub = new BridgePublisher<>(bridge, "start_bag");
    }

    public void startBag() {
        TimePrimitive now = m_startBagPub.getNow();
        m_startBagPub.send(new Time(now));
    }
}
