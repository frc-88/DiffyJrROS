package frc.robot.ros.bridge;

import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.std_msgs.RosBool;

public class MotorEnablePublisher implements Publisher {
    private final BridgePublisher<RosBool> setMotorEnablePub;
    private boolean enable = false;

    public MotorEnablePublisher(ROSNetworkTablesBridge bridge) {
        setMotorEnablePub = new BridgePublisher<>(bridge, "motor_enable");
    }

    public void publish() {
        setMotorEnablePub.send(new RosBool(enable));
    }

    public void setMotorEnable(boolean enable) {
        this.enable = enable;
    }
}
