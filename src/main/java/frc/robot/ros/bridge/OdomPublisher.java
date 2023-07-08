package frc.robot.ros.bridge;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.drive_subsystem.DriveSubsystem;
import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.conversions.ROSConversions;
import frc.team88.ros.messages.TimePrimitive;
import frc.team88.ros.messages.geometry_msgs.Point;
import frc.team88.ros.messages.geometry_msgs.Pose;
import frc.team88.ros.messages.geometry_msgs.PoseWithCovariance;
import frc.team88.ros.messages.geometry_msgs.Quaternion;
import frc.team88.ros.messages.geometry_msgs.Twist;
import frc.team88.ros.messages.geometry_msgs.TwistWithCovariance;
import frc.team88.ros.messages.geometry_msgs.Vector3;
import frc.team88.ros.messages.nav_msgs.Odometry;
import frc.team88.ros.messages.std_msgs.Header;

public class OdomPublisher implements Publisher {
    private final DriveSubsystem driveSubsystem;
    private final BridgePublisher<Odometry> odomPub;

    public OdomPublisher(DriveSubsystem drive, ROSNetworkTablesBridge bridge) {
        driveSubsystem = drive;
        odomPub = new BridgePublisher<>(bridge, "odom");
    }

    private final Odometry odomMsg = new Odometry(new Header(0, new TimePrimitive(), Frames.ODOM_FRAME),
            Frames.BASE_FRAME,
            new PoseWithCovariance(new Pose(new Point(0, 0, 0), new Quaternion(0, 0, 0, 1)), new Double[] {
                    5e-4, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 5e-4, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 5e-4, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 5e-4, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 5e-4, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 5e-4
            }),
            new TwistWithCovariance(new Twist(new Vector3(0, 0, 0), new Vector3(0, 0, 0)), new Double[] {
                    1e-4, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1e-4, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 1e-4, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 1e-4, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 1e-4, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 1e-4
            }));

    public void publish() {
        Pose2d pose = driveSubsystem.getSwerve().getOdometryPose();
        ChassisSpeeds velocity = driveSubsystem.getSwerve().getChassisSpeeds();

        odomMsg.setHeader(odomPub.getHeader(Frames.ODOM_FRAME));
        odomMsg.getPose().setPose(ROSConversions.wpiToRosPose(new Pose3d(pose)));
        odomMsg.getTwist().getTwist()
                .setLinear(new Vector3(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, 0.0));
        odomMsg.getTwist().getTwist().setAngular(new Vector3(0.0, 0.0, velocity.omegaRadiansPerSecond));

        odomPub.send(odomMsg);
    }

}
