package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ros.messages.tj2_interfaces.NavX;
import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.BridgeSubscriber;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.conversions.ROSConversions;
import frc.team88.ros.conversions.TFListenerCompact;
import frc.team88.ros.messages.Time;
import frc.team88.ros.messages.geometry_msgs.Point;
import frc.team88.ros.messages.geometry_msgs.Pose;
import frc.team88.ros.messages.geometry_msgs.PoseWithCovariance;
import frc.team88.ros.messages.geometry_msgs.Quaternion;
import frc.team88.ros.messages.geometry_msgs.Twist;
import frc.team88.ros.messages.geometry_msgs.TwistWithCovariance;
import frc.team88.ros.messages.geometry_msgs.Vector3;
import frc.team88.ros.messages.nav_msgs.Odometry;
import frc.team88.ros.messages.std_msgs.Float64;
import frc.team88.ros.messages.std_msgs.Header;

public class DiffyJrCoprocessorBridge extends SubsystemBase {
    private final DriveSubsystem m_drive;
    private final frc.robot.diffswerve.NavX m_imu;

    private final ROSNetworkTablesBridge m_ros_interface = new ROSNetworkTablesBridge(
            Constants.COPROCESSOR_ADDRESS,
            Constants.COPROCESSOR_PORT,
            Constants.COPROCESSOR_TABLE_UPDATE_DELAY);

    private final BridgeSubscriber<Twist> m_twistSub = new BridgeSubscriber<>(m_ros_interface, "/tj2/cmd_vel",
            Twist.class);
    private final BridgeSubscriber<Float64> m_pingSendSub = new BridgeSubscriber<>(m_ros_interface,
            "/tj2/ping_send",
            Float64.class);

    private final BridgePublisher<Odometry> m_odomPub = new BridgePublisher<>(m_ros_interface, "/tj2/odom");
    private final BridgePublisher<NavX> m_imuPub = new BridgePublisher<>(m_ros_interface, "/tj2/imu");
    private final BridgePublisher<Float64> m_pingReturnPub = new BridgePublisher<>(m_ros_interface,
            "/tj2/ping_return");

    private final TFListenerCompact m_tfListenerCompact = new TFListenerCompact(m_ros_interface, "/tf_compact");

    public final String MAP_FRAME = "map";
    public final String ODOM_FRAME = "odom";
    public final String BASE_FRAME = "base_link";
    public final String IMU_FRAME = "imu";

    private final Odometry odomMsg = new Odometry(new Header(0, new Time(), ODOM_FRAME), BASE_FRAME,
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
        m_imu = m_drive.getImu();
    }

    public BridgeSubscriber<Twist> getTwistSub() {
        return m_twistSub;
    }

    public TFListenerCompact getTFListener() {
        return m_tfListenerCompact;
    }

    private void checkPing() {
        Float64 ping;
        if ((ping = m_pingSendSub.receive()) != null) {
            m_pingReturnPub.send(ping);
        }
    }

    private void sendOdom() {
        Pose2d pose = m_drive.getSwerve().getOdometryPose();
        ChassisSpeeds velocity = m_drive.getSwerve().getChassisSpeeds();

        odomMsg.setHeader(m_odomPub.getHeader(ODOM_FRAME));
        odomMsg.getPose().setPose(ROSConversions.wpiToRosPose(new Pose3d(pose)));
        odomMsg.getTwist().getTwist()
                .setLinear(new Vector3(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, 0.0));
        odomMsg.getTwist().getTwist().setAngular(new Vector3(0.0, 0.0, velocity.omegaRadiansPerSecond));

        m_odomPub.send(odomMsg);
    }

    private void sendImu() {
        m_imuPub.send(new NavX(m_imuPub.getHeader(IMU_FRAME),
                ROSConversions.wpiToRosRotation(new Rotation3d(m_imu.getRoll(), m_imu.getPitch(), m_imu.getYaw())),
                new Vector3(0.0, 0.0, m_imu.getYawRate()),
                new Vector3(m_imu.getAccelX(), m_imu.getAccelY(), m_imu.getAccelZ())));
    }

    @Override
    public void periodic() {
        super.periodic();
        sendOdom();
        sendImu();
        checkPing();
        m_tfListenerCompact.update();
    }
}
