// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.TreeMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.diffswerve.Constants;
import frc.robot.localization.Localization;
import frc.robot.preferenceconstants.PIDPreferenceConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.trajectory.CustomHolonomicDriveController;
import frc.robot.trajectory.CustomTrajectoryGenerator;
import frc.robot.trajectory.RotationSequence;
import frc.robot.trajectory.Waypoint;

public class FollowTrajectory extends CommandBase {
    private final DriveSubsystem drive;
    private final Localization localization;
    private final Trajectory trajectory;
    private final RotationSequence rotationSequence;

    private PIDPreferenceConstants linearConstants = new PIDPreferenceConstants("TrajectoryLinearPID");
    private PIDPreferenceConstants angularConstants = new PIDPreferenceConstants("TrajectoryAngularPID");

    private final Timer timer = new Timer();

    private final PIDController xController = new PIDController(0.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(0.0, 0.0, 0.0);
    private final PIDController thetaController = new PIDController(0.0, 0.0, 0.0);

    private final CustomHolonomicDriveController driveController = new CustomHolonomicDriveController(
            xController, yController, thetaController);

    public FollowTrajectory(
            DriveSubsystem drive,
            Localization localization,
            Trajectory trajectory,
            RotationSequence rotationSequence) {
        this.drive = drive;
        this.localization = localization;
        this.trajectory = trajectory;
        this.rotationSequence = rotationSequence;
        addRequirements(drive);
    }

    public FollowTrajectory(
            DriveSubsystem drive,
            Localization localization,
            List<Waypoint> waypoints,
            double startVelocity,
            List<TrajectoryConstraint> extraConstraints) {
        this.drive = drive;
        this.localization = localization;
        TrajectoryConfig config = makeTrajectoryConfig(startVelocity, extraConstraints);
        CustomTrajectoryGenerator generator = new CustomTrajectoryGenerator();
        generator.generate(config, waypoints);
        trajectory = generator.getDriveTrajectory();
        rotationSequence = generator.getHolonomicRotationSequence();
        addRequirements(drive);
    }

    public FollowTrajectory(DriveSubsystem drive, Localization localization, List<Waypoint> waypoints) {
        this(drive, localization, waypoints, 0.0, List.of());
    }

    public FollowTrajectory(DriveSubsystem drive, Localization localization, Pose2d relativeGoalPose) {
        this.drive = drive;
        this.localization = localization;

        Pose2d currentPose = getPose();
        Transform2d transform = new Transform2d(new Pose2d(), currentPose);
        Pose2d goalPose = relativeGoalPose.transformBy(transform);

        ArrayList<Waypoint> waypoints = new ArrayList<>();
        waypoints.add(new Waypoint(currentPose.getTranslation()));
        waypoints.add(new Waypoint(goalPose.getTranslation()));

        TrajectoryConfig config = makeTrajectoryConfig(0.0, List.of());
        CustomTrajectoryGenerator generator = new CustomTrajectoryGenerator();
        generator.generate(config, waypoints);
        trajectory = generator.getDriveTrajectory();
        rotationSequence = generator.getHolonomicRotationSequence();
        addRequirements(drive);
    }

    public static FollowTrajectory fromJSON(DriveSubsystem drive, Localization localization, String filePath) {
        filePath = "pathplanner/generatedJSON/" + filePath;
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filePath);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + filePath, ex.getStackTrace());
        }
        TreeMap<Double, Rotation2d> holonomicWaypoints = new TreeMap<>();

        for (State state : trajectory.getStates()) {
            holonomicWaypoints.put(state.timeSeconds, state.poseMeters.getRotation());
        }

        RotationSequence rotationSequence = new RotationSequence(holonomicWaypoints);
        return new FollowTrajectory(drive, localization, trajectory, rotationSequence);
    }

    private TrajectoryConfig makeTrajectoryConfig(double startVelocity, List<TrajectoryConstraint> extraConstraints) {
        return new TrajectoryConfig(Constants.DriveTrain.MAX_CHASSIS_SPEED,
                Constants.DriveTrain.MAX_CHASSIS_LINEAR_ACCEL)
                .setKinematics(drive.getSwerve().getSwerveKinematics())
                .setStartVelocity(startVelocity)
                .setEndVelocity(0.0)
                .addConstraint(
                        new CentripetalAccelerationConstraint(Constants.DriveTrain.MAX_CHASSIS_CENTRIPITAL_ACCEL))
                .addConstraints(extraConstraints);
    }

    private Pose2d getPose() {
        return this.localization.getPose();
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

    public RotationSequence getRotationSequence() {
        return rotationSequence;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
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

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Trajectory.State driveState = trajectory.sample(timer.get());
        RotationSequence.State holonomicRotationState = rotationSequence.sample(timer.get());

        ChassisSpeeds nextDriveState = driveController.calculate(getPose(), driveState, holonomicRotationState);
        drive.drive(nextDriveState);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}
