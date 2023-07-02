package frc.robot.ros;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drive_subsystem.DriveSubsystem;
import frc.robot.ros.bridge.AutoPathManager;
import frc.robot.ros.bridge.ImuPublisher;
import frc.robot.ros.bridge.JointManager;
import frc.robot.ros.bridge.JoystickSubscriber;
import frc.robot.ros.bridge.MatchManager;
import frc.robot.ros.bridge.MotorEnablePublisher;
import frc.robot.ros.bridge.NearestConeSubscriber;
import frc.robot.ros.bridge.OdomPublisher;
import frc.robot.ros.bridge.PingPublisher;
import frc.robot.ros.bridge.PointerPublisher;
import frc.robot.ros.bridge.Publisher;
import frc.robot.ros.bridge.StartBagManager;
import frc.robot.ros.bridge.TagSubscriber;
import frc.robot.ros.bridge.TwistSubscriber;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.conversions.TFListenerCompact;

public class DiffyJrCoprocessorBridge extends SubsystemBase {
    private final DriveSubsystem drive;

    private final ROSNetworkTablesBridge bridge;
    private final long SLOW_INTERVAL = 5; // every 5 periodic ticks, slow update is called
    private long updateCounter = 0;

    public final AutoPathManager autoPathManager;
    public final ImuPublisher imuPublisher;
    public final JointManager jointManager;
    public final JoystickSubscriber joystickSubscriber;
    public final MatchManager matchManager;
    public final MotorEnablePublisher motorEnablePublisher;
    public final NearestConeSubscriber nearestConeSubscriber;
    public final OdomPublisher odometryPublisher;
    public final PingPublisher pingPublisher;
    public final StartBagManager startBagPublisher;
    public final TwistSubscriber twistSubscriber;
    public final TFListenerCompact tfListenerCompact;
    public final PointerPublisher pointerPublisher;
    public final TagSubscriber tagSubscriber;

    private final String[] JOINT_NAMES = new String[] {
            "base_link_to_wheel_0_joint",
            "base_link_to_wheel_1_joint",
            "base_link_to_wheel_2_joint",
            "base_link_to_wheel_3_joint"
    };

    private final Publisher[] publishers;

    public DiffyJrCoprocessorBridge(DriveSubsystem drive) {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();

        bridge = new ROSNetworkTablesBridge(instance.getTable(""), 0.02);

        autoPathManager = new AutoPathManager(bridge);
        imuPublisher = new ImuPublisher(drive, bridge);
        jointManager = new JointManager(bridge, "joint");
        joystickSubscriber = new JoystickSubscriber(bridge);
        matchManager = new MatchManager(bridge);
        motorEnablePublisher = new MotorEnablePublisher(bridge);
        nearestConeSubscriber = new NearestConeSubscriber(bridge);
        odometryPublisher = new OdomPublisher(drive, bridge);
        pingPublisher = new PingPublisher(bridge);
        startBagPublisher = new StartBagManager(bridge);
        twistSubscriber = new TwistSubscriber(bridge);
        tfListenerCompact = new TFListenerCompact(bridge, "/tf_compact");
        pointerPublisher = new PointerPublisher(bridge);
        tagSubscriber = new TagSubscriber(bridge);

        publishers = new Publisher[] {
                imuPublisher,
                motorEnablePublisher,
                odometryPublisher,
                pingPublisher
        };

        this.drive = drive;

        for (String name : JOINT_NAMES) {
            jointManager.addJoint(name);
        }
    }

    // ---
    // Publication updates
    // ---

    private void sendJoints() {
        SwerveModuleState[] states = drive.getSwerve().getModuleStates();
        for (int moduleIndex = 0; moduleIndex < states.length; moduleIndex++) {
            SwerveModuleState state = states[moduleIndex];
            jointManager.sendJointPosition(JOINT_NAMES[moduleIndex], state.angle.getRadians());
        }
    }

    // ---
    // Periodics
    // ---

    public void slowPeriodic() {
        sendJoints();
        matchManager.sendMatch();
    }

    @Override
    public void periodic() {
        tfListenerCompact.update();
        for (Publisher publisher : publishers) {
            publisher.publish();
        }
        if (updateCounter % SLOW_INTERVAL == 0) {
            slowPeriodic();
        }
        updateCounter++;
    }
}
