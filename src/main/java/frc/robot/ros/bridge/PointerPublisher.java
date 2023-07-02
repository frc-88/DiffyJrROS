package frc.robot.ros.bridge;

import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.TimePrimitive;
import frc.team88.ros.messages.geometry_msgs.PointStamped;
import frc.team88.ros.messages.std_msgs.Time;

public class PointerPublisher {
    private final BridgePublisher<PointStamped> goalPub;
    private final BridgePublisher<Time> recordCalibrationPub;

    public PointerPublisher(ROSNetworkTablesBridge bridge) {
        goalPub = new BridgePublisher<>(bridge, "pointer/goal");
        recordCalibrationPub = new BridgePublisher<>(bridge, "pointer/record");
    }

    public void publishGoal(PointStamped goal) {
        goalPub.send(goal);
    }

    public void recordCalibration() {
        TimePrimitive now = recordCalibrationPub.getNow();
        recordCalibrationPub.send(new Time(now));
    }
}
