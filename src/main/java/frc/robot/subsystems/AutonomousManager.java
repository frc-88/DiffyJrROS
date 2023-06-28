// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FollowTrajectory;
import frc.robot.localization.Localization;
import frc.robot.preferenceconstants.StringPreferenceConstant;
import frc.robot.ros.bridge.AutoPathManager;
import frc.robot.trajectory.RotationSequence;

public class AutonomousManager extends SubsystemBase {
    private final long SLOW_INTERVAL = 5; // every 5 periodic ticks, slowPeriodic is called
    private long updateCounter = 0;
    private Map<String, FollowTrajectory> m_trajectories;

    private final StringPreferenceConstant m_autonomousPreferenceConstant = new StringPreferenceConstant("auto", "");
    private String m_selectedAutonomous = "";
    private Map<String, CommandBase> m_autos = new HashMap<>();

    public final Pose2d garageOrigin = new Pose2d(39.0, 26.2, new Rotation2d());
    public final Pose2d apartmentOrigin = new Pose2d(4.2, 1.69, new Rotation2d());
    public final Pose2d chargedUpOrigin = new Pose2d(8.25, 4.0, new Rotation2d());

    private final DriveSubsystem m_drive;
    private final Localization m_ros_localization;
    private final Localization m_odom_localization;
    private final AutoPathManager m_autoPathManager;

    /** Creates a new AutonomousManager. */
    public AutonomousManager(DriveSubsystem drive, Localization ros_localization, Localization odom_localization,
            AutoPathManager autoPathManager) {
        m_drive = drive;
        m_ros_localization = ros_localization;
        m_odom_localization = odom_localization;
        m_autoPathManager = autoPathManager;

        // All auto selector stuff here is kinda hacky. This is in service of pushing
        // the active trajectory to ROS. An autonomous builder object should be created
        // to do this in future
        m_trajectories = Map.of(
                "square",
                makeTrajectory("SquareGarage.wpilib.json", garageOrigin),
                "apartment",
                makeTrajectory("Apartment.wpilib.json", apartmentOrigin),
                "backwards",
                makeTrajectory("Backwards Apartment.wpilib.json", apartmentOrigin),
                "test",
                makeTrajectory("Test Auto.wpilib.json", chargedUpOrigin));
        for (String key : m_trajectories.keySet()) {
            m_autos.put(key,
                    new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                Pose2d resetPose = m_ros_localization.getPose();
                                System.out.println("Resetting odometry to " + resetPose);
                                m_odom_localization.reset(resetPose);
                            }),
                            m_trajectories.get(key)));
        }
        m_autos.put("", new WaitCommand(15.0));
    }

    public Command getAutonomousCommand() {
        return m_autos.get(m_selectedAutonomous);
    }

    private FollowTrajectory makeTrajectory(String filePath, Pose2d origin) {
        return FollowTrajectory.fromJSONHolonomic(m_drive, m_ros_localization, filePath, origin);
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
        String value = m_autonomousPreferenceConstant.getValue();
        if (!setSelectedAuto(value)) {
            System.out.println("Invalid auto supplied: " + value);
        }
    }

    private boolean setSelectedAuto(String value) {
        if (!m_autos.containsKey(value)) {
            return false;
        }
        if (!m_selectedAutonomous.equals(value)) {
            System.out.println(String.format("Selecting auto %s. Was %s", value, m_selectedAutonomous));
            m_selectedAutonomous = value;
            if (m_trajectories.containsKey(m_selectedAutonomous)) {
                FollowTrajectory follow = m_trajectories.get(m_selectedAutonomous);
                m_autoPathManager.sendAutoInfo(follow.getTrajectory(), follow.getRotationSequence());
            } else {
                m_autoPathManager.sendAutoInfo(new Trajectory(), new RotationSequence(new TreeMap<>()));
            }
        }
        return true;
    }
}
