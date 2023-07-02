package frc.robot.ros.bridge;

import java.util.Optional;

import frc.robot.ros.messages.apriltag_ros.AprilTagDetectionArray;
import frc.team88.ros.bridge.BridgeSubscriber;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;

public class TagSubscriber implements Subscriber<AprilTagDetectionArray> {
    private final BridgeSubscriber<AprilTagDetectionArray> tagSub;
    private AprilTagDetectionArray lastTag = new AprilTagDetectionArray();

    public TagSubscriber(ROSNetworkTablesBridge bridge) {
        tagSub = new BridgeSubscriber<>(bridge, "/northstar/tag_detections", AprilTagDetectionArray.class);
    }

    public Optional<AprilTagDetectionArray> receive() {
        AprilTagDetectionArray msg;
        if ((msg = tagSub.receive()) != null) {
            lastTag = msg;
        }
        return Optional.ofNullable(msg);
    }

    public AprilTagDetectionArray getLastTag() {
        receive();
        return lastTag;
    }
}
