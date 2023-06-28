package frc.robot.ros.bridge;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.trajectory.RotationSequence;
import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.conversions.ROSConversions;
import frc.team88.ros.messages.geometry_msgs.Pose;
import frc.team88.ros.messages.geometry_msgs.PoseStamped;
import frc.team88.ros.messages.nav_msgs.Path;
import frc.team88.ros.messages.std_msgs.Header;

public class AutoPathManager {
    private final BridgePublisher<Path> m_autoPathPub;
    private final BridgePublisher<PoseStamped> m_autoStartPub;

    public AutoPathManager(ROSNetworkTablesBridge bridge) {
        m_autoPathPub = new BridgePublisher<>(bridge, "auto/path");
        m_autoStartPub = new BridgePublisher<>(bridge, "auto/start");
    }

    public void sendAutoInfo(Trajectory trajectory, RotationSequence rotationSequence) {
        List<Trajectory.State> traj_states = trajectory.getStates();
        List<RotationSequence.State> rotate_states = rotationSequence.getStates();
        if (traj_states.size() != rotate_states.size()) {
            System.out.println("Trajectory and rotation sequences don't match! Not sending info.");
            return;
        }
        System.out.println("Sending auto info. Path length is " + traj_states.size());

        Path path = new Path();
        Header header = m_autoPathPub.getHeader(Frames.MAP_FRAME);
        path.setHeader(header);
        for (int index = 0; index < traj_states.size(); index++) {
            Pose2d pose2d = new Pose2d(traj_states.get(index).poseMeters.getTranslation(),
                    rotate_states.get(index).position);
            Pose pose3d = ROSConversions.wpiToRosPose(new Pose3d(pose2d));
            path.getPoses().add(new PoseStamped(header, pose3d));
        }

        PoseStamped startPose = path.getPoses().get(0);

        m_autoPathPub.send(path);
        m_autoStartPub.send(startPose);
    }
}
