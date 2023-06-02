package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ros.bridge.JointPublisher;
import frc.robot.ros.messages.tj2_interfaces.NavX;
import frc.robot.trajectory.RotationSequence;
import frc.robot.ros.messages.tj2_interfaces.Match;
import frc.robot.ros.messages.tj2_interfaces.MatchPeriod;
import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.BridgeSubscriber;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.conversions.ROSConversions;
import frc.team88.ros.conversions.TFListenerCompact;
import frc.team88.ros.messages.std_msgs.Time;
import frc.team88.ros.messages.TimePrimitive;
import frc.team88.ros.messages.geometry_msgs.Point;
import frc.team88.ros.messages.geometry_msgs.Pose;
import frc.team88.ros.messages.geometry_msgs.PoseStamped;
import frc.team88.ros.messages.geometry_msgs.PoseWithCovariance;
import frc.team88.ros.messages.geometry_msgs.Quaternion;
import frc.team88.ros.messages.geometry_msgs.Twist;
import frc.team88.ros.messages.geometry_msgs.TwistWithCovariance;
import frc.team88.ros.messages.geometry_msgs.Vector3;
import frc.team88.ros.messages.nav_msgs.Odometry;
import frc.team88.ros.messages.nav_msgs.Path;
import frc.team88.ros.messages.std_msgs.Bool;
import frc.team88.ros.messages.std_msgs.Float64;
import frc.team88.ros.messages.std_msgs.Header;

public class DiffyJrCoprocessorBridge extends SubsystemBase {
    private final DriveSubsystem m_drive;
    private final frc.robot.diffswerve.NavX m_imu;

    private final ROSNetworkTablesBridge m_ros_interface;
    private final long SLOW_INTERVAL = 5; // every 5 periodic ticks, slow update is called
    private long m_updateCounter = 0;
    private MatchPeriod m_matchPeriod = new MatchPeriod(MatchPeriod.DISABLED);

    private final BridgeSubscriber<Twist> m_twistSub;
    private final BridgeSubscriber<Float64> m_pingSendSub;
    private final BridgeSubscriber<Bool> m_fieldRelativeSub;
    private final BridgeSubscriber<PoseStamped> m_nearestConeSub;

    private final BridgePublisher<Odometry> m_odomPub;
    private final BridgePublisher<NavX> m_imuPub;
    private final BridgePublisher<Float64> m_pingReturnPub;
    private final BridgePublisher<Match> m_matchPub;
    private final BridgePublisher<MatchPeriod> m_matchPeriodPub;
    private final BridgePublisher<Time> m_startBagPub;
    private final BridgePublisher<Path> m_autoPathPub;
    private final BridgePublisher<PoseStamped> m_autoStartPub;

    private final TFListenerCompact m_tfListenerCompact;
    private final JointPublisher m_jointPublisher;

    public static final String MAP_FRAME = "map";
    public static final String ODOM_FRAME = "odom";
    public static final String BASE_FRAME = "base_link";
    public static final String IMU_FRAME = "imu";

    private final Odometry m_odomMsg = new Odometry(new Header(0, new TimePrimitive(), ODOM_FRAME), BASE_FRAME,
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

    private final String[] JOINT_NAMES = new String[] {
            "base_link_to_wheel_0_joint",
            "base_link_to_wheel_1_joint",
            "base_link_to_wheel_2_joint",
            "base_link_to_wheel_3_joint"
    };

    public DiffyJrCoprocessorBridge(
            DriveSubsystem drive) {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();

        m_ros_interface = new ROSNetworkTablesBridge(instance.getTable(""), 0.02);

        m_twistSub = new BridgeSubscriber<>(m_ros_interface, "cmd_vel", Twist.class);
        m_pingSendSub = new BridgeSubscriber<>(m_ros_interface, "ping_send", Float64.class);
        m_fieldRelativeSub = new BridgeSubscriber<>(m_ros_interface, "field_relative", Bool.class);
        m_nearestConeSub = new BridgeSubscriber<>(m_ros_interface, "nearest_cone", PoseStamped.class);

        m_odomPub = new BridgePublisher<>(m_ros_interface, "odom");
        m_imuPub = new BridgePublisher<>(m_ros_interface, "imu");
        m_pingReturnPub = new BridgePublisher<>(m_ros_interface, "ping_return");
        m_matchPub = new BridgePublisher<>(m_ros_interface, "match");
        m_matchPeriodPub = new BridgePublisher<>(m_ros_interface, "match_period");
        m_startBagPub = new BridgePublisher<>(m_ros_interface, "start_bag");
        m_autoPathPub = new BridgePublisher<>(m_ros_interface, "auto/path");
        m_autoStartPub = new BridgePublisher<>(m_ros_interface, "auto/start");

        m_tfListenerCompact = new TFListenerCompact(m_ros_interface, "/tf_compact");
        m_jointPublisher = new JointPublisher(m_ros_interface, "joint");

        m_drive = drive;
        m_imu = m_drive.getImu();

        for (String name : JOINT_NAMES) {
            m_jointPublisher.addJoint(name);
        }
    }

    // ---
    // Subscription updates
    // ---

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

    private void checkFieldRelative() {
        Bool msg;
        if ((msg = m_fieldRelativeSub.receive()) != null) {
            m_drive.getSwerve().setFieldRelativeCommands(msg.getData());
            m_drive.getSwerve().resetFieldOffset();
        }
    }

    private void checkNearestCone() {
        PoseStamped msg;
        if ((msg = m_nearestConeSub.receive()) != null) {
            Pose3d cone = ROSConversions.rosToWpiPose(msg.getPose());

            double roll = cone.getRotation().getX();
            double yaw = cone.getRotation().getZ();
            boolean is_standing = Math.abs(roll) < Math.PI / 4;

            SmartDashboard.putNumber("Nearest cone x", cone.getX());
            SmartDashboard.putNumber("Nearest cone y", cone.getY());
            SmartDashboard.putNumber("Nearest cone angle (degrees)", Units.radiansToDegrees(yaw));
            SmartDashboard.putBoolean("Nearest cone is standing", is_standing);
        }
    }

    // ---
    // Publication updates
    // ---

    private void sendOdom() {
        Pose2d pose = m_drive.getSwerve().getOdometryPose();
        ChassisSpeeds velocity = m_drive.getSwerve().getChassisSpeeds();

        m_odomMsg.setHeader(m_odomPub.getHeader(ODOM_FRAME));
        m_odomMsg.getPose().setPose(ROSConversions.wpiToRosPose(new Pose3d(pose)));
        m_odomMsg.getTwist().getTwist()
                .setLinear(new Vector3(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, 0.0));
        m_odomMsg.getTwist().getTwist().setAngular(new Vector3(0.0, 0.0, velocity.omegaRadiansPerSecond));

        m_odomPub.send(m_odomMsg);
    }

    private void sendImu() {
        m_imuPub.send(new NavX(m_imuPub.getHeader(IMU_FRAME),
                ROSConversions.wpiToRosRotation(new Rotation3d(m_imu.getRoll(), m_imu.getPitch(), m_imu.getYaw())),
                new Vector3(0.0, 0.0, m_imu.getYawRate()),
                new Vector3(m_imu.getAccelX(), m_imu.getAccelY(), m_imu.getAccelZ())));
    }

    private void sendJoints() {
        SwerveModuleState[] states = m_drive.getSwerve().getModuleStates();
        for (int moduleIndex = 0; moduleIndex < states.length; moduleIndex++) {
            SwerveModuleState state = states[moduleIndex];
            m_jointPublisher.sendJointPosition(JOINT_NAMES[moduleIndex], state.angle.getRadians());
        }
    }

    public void sendDisableMatchPeriod() {
        m_matchPeriod = new MatchPeriod(MatchPeriod.DISABLED);
        m_matchPeriodPub.send(m_matchPeriod);
    }

    public void sendAutonomousMatchPeriod() {
        m_matchPeriod = new MatchPeriod(MatchPeriod.AUTONOMOUS);
        m_matchPeriodPub.send(m_matchPeriod);
    }

    public void sendTeleopMatchPeriod() {
        m_matchPeriod = new MatchPeriod(MatchPeriod.TELEOP);
        m_matchPeriodPub.send(m_matchPeriod);
    }

    private void sendMatch() {
        m_matchPub.send(new Match(
                DriverStation.getMatchTime(),
                DriverStation.getAlliance().name(),
                (byte) DriverStation.getLocation(),
                m_matchPeriod));
    }

    public void startBag() {
        TimePrimitive now = m_startBagPub.getNow();
        m_startBagPub.send(new Time(now));
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
        Header header = m_autoPathPub.getHeader(MAP_FRAME);
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

    // ---
    // Periodics
    // ---

    public void slowPeriodic() {
        sendJoints();
        sendMatch();
    }

    @Override
    public void periodic() {
        super.periodic();
        sendOdom();
        sendImu();
        checkPing();
        checkFieldRelative();
        checkNearestCone();
        m_tfListenerCompact.update();
        if (m_updateCounter % SLOW_INTERVAL == 0) {
            slowPeriodic();
        }
        m_updateCounter++;
    }
}
