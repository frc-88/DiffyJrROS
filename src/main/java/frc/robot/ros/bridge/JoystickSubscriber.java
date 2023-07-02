package frc.robot.ros.bridge;

import java.util.Optional;

import frc.team88.ros.bridge.BridgeSubscriber;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.sensor_msgs.Joy;

public class JoystickSubscriber implements Subscriber<Joy> {
    private final BridgeSubscriber<Joy> joySub;

    public JoystickSubscriber(ROSNetworkTablesBridge bridge) {
        joySub = new BridgeSubscriber<>(bridge, "/joy", Joy.class);
    }

    public Optional<Joy> receive() {
        return Optional.ofNullable(joySub.receive());
    }
}
