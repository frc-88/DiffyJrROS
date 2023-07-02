package frc.robot.ros.bridge;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team88.ros.bridge.BridgeSubscriber;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.conversions.ROSConversions;
import frc.team88.ros.messages.geometry_msgs.PoseStamped;

public class NearestConeSubscriber implements Subscriber<PoseStamped> {
    private final BridgeSubscriber<PoseStamped> nearestConeSub;

    public NearestConeSubscriber(ROSNetworkTablesBridge bridge) {
        nearestConeSub = new BridgeSubscriber<>(bridge, "nearest_cone", PoseStamped.class);
    }

    @Override
    public Optional<PoseStamped> receive() {
        PoseStamped msg;
        if ((msg = nearestConeSub.receive()) != null) {
            Pose3d cone = ROSConversions.rosToWpiPose(msg.getPose());

            double roll = cone.getRotation().getX();
            double yaw = cone.getRotation().getZ();
            boolean is_standing = Math.abs(roll) < Math.PI / 4;

            SmartDashboard.putNumber("Nearest cone x", cone.getX());
            SmartDashboard.putNumber("Nearest cone y", cone.getY());
            SmartDashboard.putNumber("Nearest cone angle (degrees)", Units.radiansToDegrees(yaw));
            SmartDashboard.putBoolean("Nearest cone is standing", is_standing);
            return Optional.of(msg);
        } else {
            return Optional.empty();
        }
    }

}
