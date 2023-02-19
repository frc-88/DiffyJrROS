package frc.robot.util.coprocessor.networktables;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import frc.robot.util.coprocessor.ChassisInterface;
import frc.robot.util.coprocessor.MessageTimer;
import frc.robot.util.diffswerve.DiffSwerveChassis;
import frc.robot.util.diffswerve.DiffSwerveModule;
import frc.robot.util.diffswerve.NavX;


public class DiffyJrTable extends CoprocessorTable {
    private DiffSwerveChassis swerve;
    private NavX imu;
    protected Pose2d tagGlobalPose = new Pose2d();
    protected MessageTimer tagGlobalPoseTimer = new MessageTimer(DEFAULT_MESSAGE_TIMEOUT);
    private DoubleArraySubscriber tagGlobalPoseSub;

    private final double kGravity = 9.81;

    public DiffyJrTable(DiffSwerveChassis swerve, NavX imu, String address, int port, double updateInterval) {
        super((ChassisInterface)swerve, address, port, updateInterval);
        this.swerve = swerve;
        this.imu = imu;

        tagGlobalPoseSub = rootTable.getDoubleArrayTopic("tag_global").subscribe(new double []{0.0, 0.0, 0.0, 0.0}, PubSubOption.sendAll(true), PubSubOption.periodic(this.updateInterval));
    }

    // ---
    // Tag global pose
    // ---

    public Pose2d getTagGlobalPose() {
        updateTagGlobalPose();
        return tagGlobalPose;
    }

    public boolean isTagGlobalPoseActive() {
        return tagGlobalPoseTimer.isActive();
    }

    private void updateTagGlobalPose() {
        TimestampedDoubleArray pose = tagGlobalPoseSub.getAtomic();
        if (pose.timestamp == 0.0) {
            return;
        }
        if (pose.value.length != 5) {
            System.out.println("Warning: Received tag global pose is not of length 5. Ignoring.");
            return;
        }
        // index 0 is timestamp
        double x = pose.value[1];
        double y = pose.value[2];
        // double z = pose.value[3];
        double theta = pose.value[4];
        tagGlobalPose = new Pose2d(x, y, new Rotation2d(theta));
        tagGlobalPoseTimer.reset();
    }

    @Override
    public void update() {
        super.update();
        sendImu(
            Units.degreesToRadians(imu.getRoll()),
            Units.degreesToRadians(imu.getPitch()),
            Units.degreesToRadians(imu.getYaw()),
            Units.degreesToRadians(imu.getYawRate()),
            imu.getAccelX() * kGravity,
            imu.getAccelY() * kGravity
        );
    }

    public void updateSlow() {
        DiffSwerveModule[] modules = this.swerve.getModules();
        for (int index = 0; index < modules.length; index++) {
            DiffSwerveModule module = modules[index];
            SwerveModuleState state = module.getState();
            setJointPosition(index, state.angle.getRadians());
        }
        setNoGoZones(new String[] {"<!team>_score_zone", "<!team>_zone", "<!team>_safe_zone"});
    }
}
