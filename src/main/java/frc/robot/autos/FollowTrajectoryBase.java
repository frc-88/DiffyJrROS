package frc.robot.autos;

import java.util.List;
import java.util.TreeMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.diffswerve.Constants;
import frc.robot.drive_subsystem.DriveSubsystem;
import frc.robot.localization.Localization;
import frc.robot.preferenceconstants.DoublePreferenceConstant;
import frc.robot.preferenceconstants.PIDPreferenceConstants;
import frc.robot.trajectory.CustomHolonomicDriveController;
import frc.robot.trajectory.CustomTrajectoryGenerator;
import frc.robot.trajectory.RotationSequence;

public class FollowTrajectoryBase extends CommandBase {
    protected final DriveSubsystem drive;
    protected final Localization localization;
    protected Trajectory trajectory = new Trajectory();
    protected RotationSequence rotationSequence = new RotationSequence(new TreeMap<>());

    private PIDPreferenceConstants linearConstants = new PIDPreferenceConstants("TrajectoryLinearPID");
    private PIDPreferenceConstants angularConstants = new PIDPreferenceConstants("TrajectoryAngularPID");
    private DoublePreferenceConstant distanceStartThreshold = new DoublePreferenceConstant("distanceStartThreshold",
            0.25);
    private DoublePreferenceConstant angleStartThreshold = new DoublePreferenceConstant("angleStartThreshold",
            Units.degreesToRadians(5.0));

    private final Timer timer = new Timer();
    private boolean failed = false;

    private final PIDController xController = new PIDController(0.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(0.0, 0.0, 0.0);
    private final PIDController thetaController = new PIDController(0.0, 0.0, 0.0);

    private final CustomHolonomicDriveController driveController = new CustomHolonomicDriveController(
            xController, yController, thetaController);

    public FollowTrajectoryBase(
            DriveSubsystem drive,
            Localization localization) {
        this.drive = drive;
        this.localization = localization;
        this.trajectory = new Trajectory();
        this.rotationSequence = new RotationSequence(new TreeMap<>());
        addRequirements(drive);
    }

    protected void setTrajectory(Trajectory trajectory, RotationSequence rotationSequence) {
        this.trajectory = trajectory;
        this.rotationSequence = rotationSequence;
    }

    protected void setTrajectoryFromGenerator(CustomTrajectoryGenerator generator) {
        trajectory = generator.getDriveTrajectory();
        rotationSequence = generator.getHolonomicRotationSequence();
    }

    @Override
    public void initialize() {
        Pose2d startPose = trajectory.sample(0.0).poseMeters;
        Rotation2d startRotation = rotationSequence.sample(0.0).position;
        double distanceToStart = getPose().getTranslation().getDistance(startPose.getTranslation());
        double angleToStart = Math.abs(getPose().getRotation().minus(startRotation).getRadians());
        if (distanceToStart > distanceStartThreshold.getValue()) {
            setHasFailed(true);
            System.out.println("Distance to start point exceeds threshold: " + distanceToStart);
            return;
        }
        if (angleToStart > angleStartThreshold.getValue()) {
            setHasFailed(true);
            System.out.println("Angle to start point exceeds threshold: " + angleToStart);
            return;
        }
        setHasFailed(false);
        xController.setPID(
                linearConstants.getKP().getValue(),
                linearConstants.getKI().getValue(),
                linearConstants.getKD().getValue());
        yController.setPID(
                linearConstants.getKP().getValue(),
                linearConstants.getKI().getValue(),
                linearConstants.getKD().getValue());
        thetaController.setPID(
                angularConstants.getKP().getValue(),
                angularConstants.getKI().getValue(),
                angularConstants.getKD().getValue());

        driveController.reset();

        timer.reset();
        timer.start();
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

    public RotationSequence getRotationSequence() {
        return rotationSequence;
    }

    @Override
    public void execute() {
        Trajectory.State driveState = trajectory.sample(timer.get());
        RotationSequence.State holonomicRotationState = rotationSequence.sample(timer.get());

        if (isPoseValid()) {
            ChassisSpeeds nextDriveState = driveController.calculate(getPose(), driveState, holonomicRotationState);
            drive.drive(nextDriveState);
        } else {
            setHasFailed(true);
            drive.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return hasFailed() || timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    public boolean hasFailed() {
        return failed;
    }

    public void setHasFailed(boolean failed) {
        this.failed = failed;
    }

    protected boolean isPoseValid() {
        boolean state = this.localization.isValid();
        if (!state) {
            DriverStation.reportError("Failed to get transform from ROS!!", true);
        }
        return state;
    }

    protected Pose2d getPose() {
        return this.localization.getPose();
    }

    protected TrajectoryConfig makeTrajectoryConfig(double startVelocity, List<TrajectoryConstraint> extraConstraints) {
        return new TrajectoryConfig(Constants.DriveTrain.MAX_CHASSIS_SPEED,
                Constants.DriveTrain.MAX_CHASSIS_LINEAR_ACCEL)
                .setKinematics(drive.getSwerve().getSwerveKinematics())
                .setStartVelocity(startVelocity)
                .setEndVelocity(0.0)
                .addConstraint(
                        new CentripetalAccelerationConstraint(Constants.DriveTrain.MAX_CHASSIS_CENTRIPITAL_ACCEL))
                .addConstraints(extraConstraints);
    }
}
