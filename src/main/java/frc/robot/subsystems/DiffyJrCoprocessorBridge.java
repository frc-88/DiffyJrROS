package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ros.bridge.BridgePublisher;
import frc.robot.ros.bridge.BridgeSubscriber;
import frc.robot.ros.bridge.ROSConversions;
import frc.robot.ros.bridge.ROSNetworkTablesBridge;
import frc.robot.ros.messages.Time;
import frc.robot.ros.messages.geometry_msgs.Point;
import frc.robot.ros.messages.geometry_msgs.Pose;
import frc.robot.ros.messages.geometry_msgs.PoseWithCovariance;
import frc.robot.ros.messages.geometry_msgs.Quaternion;
import frc.robot.ros.messages.geometry_msgs.Twist;
import frc.robot.ros.messages.geometry_msgs.TwistWithCovariance;
import frc.robot.ros.messages.geometry_msgs.Vector3;
import frc.robot.ros.messages.nav_msgs.Odometry;
import frc.robot.ros.messages.std_msgs.Float64;
import frc.robot.ros.messages.std_msgs.Header;

public class DiffyJrCoprocessorBridge extends SubsystemBase {
    private final DriveSubsystem m_drive;
    private final ROSNetworkTablesBridge m_ros_interface = new ROSNetworkTablesBridge(
            Constants.COPROCESSOR_ADDRESS,
            Constants.COPROCESSOR_PORT,
            Constants.COPROCESSOR_TABLE_UPDATE_DELAY);
    private final BridgeSubscriber<Twist> m_twistSub = new BridgeSubscriber<>(m_ros_interface, "/tj2/cmd_vel",
            Twist.class);
    private final BridgePublisher<Odometry> m_odomPub = new BridgePublisher<>(m_ros_interface, "/tj2/odom");
    private final BridgeSubscriber<Float64> m_pingSendSub = new BridgeSubscriber<>(m_ros_interface, "/tj2/ping_send",
            Float64.class);
    private final BridgePublisher<Float64> m_pingReturnPub = new BridgePublisher<>(m_ros_interface, "/tj2/ping_return");

    private final String frame_id = "odom";
    private final String child_frame_id = "base_link";
    private final Odometry odomMsg = new Odometry(new Header(0, new Time(), frame_id), child_frame_id,
            new PoseWithCovariance(new Pose(new Point(0, 0, 0), new Quaternion(0, 0, 0, 1)), new Double[] {
                    5e-2, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 5e-2, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 5e-2, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 5e-2, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 5e-2, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 5e-2
            }),
            new TwistWithCovariance(new Twist(new Vector3(0, 0, 0), new Vector3(0, 0, 0)), new Double[] {
                    10e-2, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 10e-2, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 10e-2, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 10e-2, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 10e-2, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 10e-2
            }));

    public DiffyJrCoprocessorBridge(
            DriveSubsystem drive) {
        m_drive = drive;
    }

    public BridgeSubscriber<Twist> getTwistSub() {
        return m_twistSub;
    }

    private void checkPing() {
        if (m_pingSendSub.didUpdate()) {
            Float64 ping = m_pingSendSub.receive();
            m_pingReturnPub.send(ping);
        }
    }

    private void sendOdom() {
        Pose2d pose = m_drive.getSwerve().getOdometryPose();
        ChassisSpeeds velocity = m_drive.getSwerve().getChassisSpeeds();

        odomMsg.setHeader(m_odomPub.getHeader(frame_id));
        odomMsg.getPose().setPose(ROSConversions.wpiToRosPose(new Pose3d(pose)));
        odomMsg.getTwist().getTwist()
                .setLinear(new Vector3(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, 0.0));
        odomMsg.getTwist().getTwist().setAngular(new Vector3(0.0, 0.0, velocity.omegaRadiansPerSecond));

        m_odomPub.send(odomMsg);
    }

    @Override
    public void periodic() {
        super.periodic();
        sendOdom();
        checkPing();
    }
}
