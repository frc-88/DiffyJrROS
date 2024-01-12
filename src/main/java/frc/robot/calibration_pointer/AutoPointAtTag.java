package frc.robot.calibration_pointer;

import java.util.Optional;

import frc.robot.ros.messages.apriltag_ros.AprilTagDetection;
import frc.robot.ros.messages.apriltag_ros.AprilTagDetectionArray;
import frc.team88.ros.messages.geometry_msgs.Point;
import frc.team88.ros.messages.geometry_msgs.PointStamped;
import frc.team88.ros.messages.std_msgs.RosHeader;

public class AutoPointAtTag {
    public static Optional<PointStamped> compute(AprilTagDetectionArray detections) {
        if (detections.getDetections().size() == 0) {
            return Optional.empty();
        }
        AprilTagDetection closestTag = new AprilTagDetection();
        double closestDistance = Double.MAX_VALUE;
        for (AprilTagDetection tag : detections.getDetections()) {
            double distance = computeTagDistance(tag);
            if (distance < closestDistance) {
                closestTag = tag;
                closestDistance = distance;
            }
        }
        if (closestDistance != Double.MAX_VALUE) {
            RosHeader header = detections.getHeader();
            Point point = closestTag.getPose().getPose().getPose().getPosition();
            return Optional.of(new PointStamped(header, point));
        } else {
            return Optional.empty();
        }
    }

    private static double computeTagDistance(AprilTagDetection tag) {
        Point point = tag.getPose().getPose().getPose().getPosition();
        return Math.sqrt(Math.pow(point.getX(), 2) + Math.pow(point.getY(), 2) + Math.pow(point.getZ(), 2));
    }
}
