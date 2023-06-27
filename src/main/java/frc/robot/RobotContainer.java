// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.CoastDriveMotors;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.SwitchableJoystickCommand;
import frc.robot.commands.SwitchableJoystickCommand.JoystickType;
import frc.robot.localization.Localization;
import frc.robot.localization.OdometryLocalization;
import frc.robot.localization.ROSLocalization;
import frc.robot.preferenceconstants.IntPreferenceConstant;
import frc.robot.preferenceconstants.StringPreferenceConstant;
import frc.robot.subsystems.DiffyJrCoprocessorBridge;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LaserTurret;
import frc.robot.subsystems.SwerveJoystick;
import frc.robot.subsystems.SwerveJoystick.SwerveControllerType;
import frc.robot.trajectory.RotationSequence;

import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem m_drive = new DriveSubsystem();
    private final SwerveJoystick m_joystick = new SwerveJoystick(SwerveControllerType.XBOX);
    private final DiffyJrCoprocessorBridge m_bridge = new DiffyJrCoprocessorBridge(m_drive);
    private final Localization m_ros_localization = new ROSLocalization(m_drive, m_bridge);
    private final Localization m_odom_localization = new OdometryLocalization(m_drive);
    private final LaserTurret m_laser_turret = new LaserTurret(SerialPort.Port.kMXP);

    private final IntPreferenceConstant joystickPreferenceConstant = new IntPreferenceConstant("joystick", 0);
    private final CommandBase m_switchableJoystickCommand = new SwitchableJoystickCommand(m_drive,
            m_bridge.getTwistSub(), m_joystick, this::getJoystickType);
    private Trigger userButton = new Trigger(() -> RobotController.getUserButton());

    private final StringPreferenceConstant m_autonomousPreferenceConstant = new StringPreferenceConstant("auto", "");
    private String m_selectedAutonomous = "";
    private Map<String, CommandBase> m_autos = new HashMap<>();
    private Map<String, FollowTrajectory> m_trajectories;

    private final Pose2d garageOrigin = new Pose2d(39.0, 26.2, new Rotation2d());
    private final Pose2d apartmentOrigin = new Pose2d(4.2, 1.69, new Rotation2d());
    private final Pose2d chargedUpOrigin = new Pose2d(8.25, 4.0, new Rotation2d());

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(Robot robot) {
        System.out.println("RobotContainer initializing");
        configureCommands();
        configurePeriodics(robot);
        m_drive.getSwerve().setAngleControllerEnabled(false);
        System.out.println("RobotContainer initialization complete");
    }

    private JoystickType getJoystickType() {
        switch (joystickPreferenceConstant.getValue()) {
            case 0:
                return JoystickType.ROS;
            case 1:
                return JoystickType.LOCAL;
            default:
                return JoystickType.ROS;
        }
    }

    private void configureCommands() {
        m_drive.setDefaultCommand(m_switchableJoystickCommand);
        userButton.whileTrue(new CoastDriveMotors(m_drive));

        Trigger toggleFieldRelative = m_joystick.getBButton();
        toggleFieldRelative.onTrue(new InstantCommand(() -> {
            boolean new_state = !m_drive.getSwerve().commandsAreFieldRelative();
            System.out.println("Setting field relative commands to " + new_state);
            m_drive.getSwerve().setFieldRelativeCommands(new_state);
        }));
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
                makeTrajectory("Test Auto.wpilib.json", chargedUpOrigin),
                "straight", new FollowTrajectory(m_drive, m_ros_localization, new Pose2d(1.0, 0.0, new Rotation2d())));
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

    private FollowTrajectory makeTrajectory(String filePath, Pose2d origin) {
        return FollowTrajectory.fromJSONHolonomic(m_drive, m_ros_localization, filePath, origin);
    }

    private void configurePeriodics(Robot robot) {
        robot.addPeriodic(m_drive.getSwerve()::controllerPeriodic,
                frc.robot.diffswerve.Constants.DifferentialSwerveModule.kDt, 0.0025);
        robot.addPeriodic(this::updateSelectedAuto, 0.5);
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
                m_bridge.sendAutoInfo(follow.getTrajectory(), follow.getRotationSequence());
            } else {
                m_bridge.sendAutoInfo(new Trajectory(), new RotationSequence(new TreeMap<>()));
            }
        }
        return true;
    }

    public void setEnableDrive(boolean enabled) {
        System.out.println("Set drive motors to " + enabled);
        m_drive.setEnabled(enabled);
        m_drive.setCoast(!enabled);
    }

    public void disabledInit() {
        m_bridge.sendDisableMatchPeriod();
    }

    public void autonomousInit() {
        // m_bridge.startBag();
        m_bridge.sendAutonomousMatchPeriod();
    }

    public void teleopInit() {
        m_bridge.sendTeleopMatchPeriod();
    }

    public void disabledPeriodic() {

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_autos.get(m_selectedAutonomous);
    }
}
