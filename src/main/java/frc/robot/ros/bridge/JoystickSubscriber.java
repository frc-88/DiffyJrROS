package frc.robot.ros.bridge;

import java.util.Optional;

import frc.team88.ros.bridge.BridgeSubscriber;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.sensor_msgs.Joy;

public class JoystickSubscriber implements Subscriber<Joy> {
    private final BridgeSubscriber<Joy> m_joySub;

    public JoystickSubscriber(ROSNetworkTablesBridge bridge) {
        m_joySub = new BridgeSubscriber<>(bridge, "/joy", Joy.class);
    }

    public Optional<Joy> receive() {
        return Optional.ofNullable(m_joySub.receive());
    }
}
