package frc.robot.ros.bridge;

import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.std_msgs.Bool;

public class MotorEnablePublisher implements Publisher {
    private final BridgePublisher<Bool> setMotorEnablePub;
    private boolean enable = false;

    public MotorEnablePublisher(ROSNetworkTablesBridge bridge) {
        setMotorEnablePub = new BridgePublisher<>(bridge, "motor_enable");
    }

    public void publish() {
        setMotorEnablePub.send(new Bool(enable));
    }

    public void setMotorEnable(boolean enable) {
        this.enable = enable;
    }
}
