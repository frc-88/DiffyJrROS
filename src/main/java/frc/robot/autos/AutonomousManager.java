// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.drive_subsystem.DriveSubsystem;
import frc.robot.localization.Localization;
import frc.robot.preferenceconstants.StringPreferenceConstant;
import frc.robot.ros.bridge.AutoPathManager;
import frc.robot.trajectory.RotationSequence;

public class AutonomousManager extends SubsystemBase {
    private final long SLOW_INTERVAL = 5; // every 5 periodic ticks, slowPeriodic is called
    private long updateCounter = 0;
    private Map<String, FollowTrajectory> trajectories;

    private final StringPreferenceConstant autonomousPreferenceConstant = new StringPreferenceConstant("auto", "");
    private String selectedAutonomous = "";
    private Map<String, Command> autos = new HashMap<>();

    public final Pose2d garageOrigin = new Pose2d(39.0, 26.2, new Rotation2d());
    public final Pose2d apartmentOrigin = new Pose2d(4.2, 1.69, new Rotation2d());
    public final Pose2d chargedUpOrigin = new Pose2d(8.25, 4.0, new Rotation2d());

    private final DriveSubsystem driveSubsystem;
    private final Localization rosLocalization;
    private final Localization odomLocalization;
    private final AutoPathManager autoPathManager;

    /** Creates a new AutonomousManager. */
    public AutonomousManager(DriveSubsystem driveSubsystem, Localization rosLocalization, Localization odomLocalization,
            AutoPathManager autoPathManager) {
        this.driveSubsystem = driveSubsystem;
        this.rosLocalization = rosLocalization;
        this.odomLocalization = odomLocalization;
        this.autoPathManager = autoPathManager;

        // All auto selector stuff here is kinda hacky. This is in service of pushing
        // the active trajectory to ROS. An autonomous builder object should be created
        // to do this in future
        trajectories = Map.of(
                "square",
                makeTrajectory("SquareGarage.wpilib.json", garageOrigin),
                "apartment",
                makeTrajectory("Apartment.wpilib.json", apartmentOrigin),
                "backwards",
                makeTrajectory("Backwards Apartment.wpilib.json", apartmentOrigin),
                "test",
                makeTrajectory("Test Auto.wpilib.json", chargedUpOrigin));
        for (String key : trajectories.keySet()) {
            autos.put(key,
                    new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                Pose2d resetPose = rosLocalization.getPose();
                                System.out.println("Resetting odometry to " + resetPose);
                                this.odomLocalization.reset(resetPose);
                            }),
                            trajectories.get(key)));
        }
        autos.put("", new WaitCommand(15.0));
    }

    public Command getAutonomousCommand() {
        return autos.get(selectedAutonomous);
    }

    private FollowTrajectory makeTrajectory(String filePath, Pose2d origin) {
        return FollowTrajectory.fromJSONHolonomic(driveSubsystem, rosLocalization, filePath, origin);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (updateCounter % SLOW_INTERVAL == 0) {
            slowPeriodic();
        }
        updateCounter++;
    }

    private void slowPeriodic() {
        updateSelectedAuto();
    }

    private void updateSelectedAuto() {
        String value = autonomousPreferenceConstant.getValue();
        if (!setSelectedAuto(value)) {
            System.out.println("Invalid auto supplied: " + value);
        }
    }

    private boolean setSelectedAuto(String value) {
        if (!autos.containsKey(value)) {
            return false;
        }
        if (!selectedAutonomous.equals(value)) {
            System.out.println(String.format("Selecting auto %s. Was %s", value, selectedAutonomous));
            selectedAutonomous = value;
            if (trajectories.containsKey(selectedAutonomous)) {
                FollowTrajectory follow = trajectories.get(selectedAutonomous);
                autoPathManager.sendAutoInfo(follow.getTrajectory(), follow.getRotationSequence());
            } else {
                autoPathManager.sendAutoInfo(new Trajectory(), new RotationSequence(new TreeMap<>()));
            }
        }
        return true;
    }
}
