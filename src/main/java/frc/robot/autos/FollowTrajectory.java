// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.TreeMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.drive_subsystem.DriveSubsystem;
import frc.robot.localization.Localization;
import frc.robot.trajectory.CustomTrajectoryGenerator;
import frc.robot.trajectory.RotationSequence;
import frc.robot.trajectory.Waypoint;
import frc.robot.trajectory.custompathweaver.CustomPathweaverLoader;

public class FollowTrajectory extends FollowTrajectoryBase {
    public FollowTrajectory(
            DriveSubsystem drive,
            Localization localization,
            Trajectory trajectory,
            RotationSequence rotationSequence) {
        super(drive, localization);
        setTrajectory(trajectory, rotationSequence);
    }

    public FollowTrajectory(
            DriveSubsystem drive,
            Localization localization,
            List<Waypoint> waypoints,
            double startVelocity,
            List<TrajectoryConstraint> extraConstraints) {
        super(drive, localization);
        TrajectoryConfig config = makeTrajectoryConfig(startVelocity, extraConstraints);
        CustomTrajectoryGenerator generator = new CustomTrajectoryGenerator();
        generator.generate(config, waypoints);
        setTrajectoryFromGenerator(generator);
        addRequirements(drive);
    }

    public FollowTrajectory(DriveSubsystem drive, Localization localization, List<Waypoint> waypoints) {
        this(drive, localization, waypoints, 0.0, List.of());
    }

    public static FollowTrajectory fromJSON(DriveSubsystem drive, Localization localization, String filePath,
            Pose2d origin) {
        filePath = "pathplanner/generatedJSON/" + filePath;
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filePath);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + filePath, ex.getStackTrace());
            return new FollowTrajectory(drive, localization, new Trajectory(), new RotationSequence(new TreeMap<>()));
        }
        trajectory = trajectory.transformBy(new Transform2d(new Pose2d(), origin));
        TreeMap<Double, Rotation2d> holonomicWaypoints = new TreeMap<>();

        for (State state : trajectory.getStates()) {
            holonomicWaypoints.put(state.timeSeconds, state.poseMeters.getRotation());
        }

        RotationSequence rotationSequence = new RotationSequence(holonomicWaypoints);
        return new FollowTrajectory(drive, localization, trajectory, rotationSequence);
    }

    public static FollowTrajectory fromJSONHolonomic(DriveSubsystem drive, Localization localization, String filePath,
            Pose2d origin) {
        filePath = "pathplanner/generatedJSON/" + filePath;
        Pair<Trajectory, RotationSequence> data;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filePath);
            data = CustomPathweaverLoader.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + filePath, ex.getStackTrace());
            return new FollowTrajectory(drive, localization, new Trajectory(), new RotationSequence(new TreeMap<>()));
        }
        Trajectory trajectory = data.getFirst().relativeTo(origin);
        RotationSequence rotationSequence = data.getSecond();
        return new FollowTrajectory(drive, localization, trajectory, rotationSequence);
    }
}
