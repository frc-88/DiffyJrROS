package frc.robot.ros.bridge;

import edu.wpi.first.wpilibj.RobotController;
import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.sensor_msgs.BatteryState;

public class BatteryStatePublisher implements Publisher {
    private final BridgePublisher<BatteryState> batteryPub;

    public BatteryStatePublisher(ROSNetworkTablesBridge bridge) {
        batteryPub = new BridgePublisher<>(bridge, "battery");
    }

    @Override
    public void publish() {
        BatteryState state = new BatteryState();
        state.setHeader(batteryPub.getHeader(""));
        state.setVoltage((float) RobotController.getBatteryVoltage());
        batteryPub.send(state);
    }

}
