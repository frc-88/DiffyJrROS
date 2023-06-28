// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.CoastDriveMotors;
import frc.robot.commands.DriveSwerveJoystickCommand;
import frc.robot.driverinput.JoystickInterface;
import frc.robot.driverinput.ROSJoystick;
import frc.robot.localization.Localization;
import frc.robot.localization.OdometryLocalization;
import frc.robot.localization.ROSLocalization;
import frc.robot.subsystems.AutonomousManager;
import frc.robot.subsystems.DiffyJrCoprocessorBridge;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LaserTurret;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    private final DiffyJrCoprocessorBridge m_bridge = new DiffyJrCoprocessorBridge(m_drive);
    private final JoystickInterface m_joystick = new ROSJoystick(m_bridge.joystickSubscriber);
    private final Localization m_ros_localization = new ROSLocalization(m_drive, m_bridge.tfListenerCompact);
    private final Localization m_odom_localization = new OdometryLocalization(m_drive);
    private final LaserTurret m_laser_turret = new LaserTurret(SerialPort.Port.kMXP);
    private final AutonomousManager m_auto_manager = new AutonomousManager(m_drive, m_ros_localization,
            m_odom_localization, m_bridge.autoPathManager);

    private Trigger userButton = new Trigger(() -> RobotController.getUserButton());

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

    private void configureCommands() {
        m_drive.setDefaultCommand(new DriveSwerveJoystickCommand(m_drive, m_joystick, m_bridge.motorEnablePublisher));
        userButton.whileTrue(new CoastDriveMotors(m_drive));

        Trigger toggleFieldRelative = new Trigger(() -> this.m_joystick.isButtonBPressed());
        toggleFieldRelative.onTrue(new InstantCommand(() -> {
            boolean new_state = !m_drive.getSwerve().commandsAreFieldRelative();
            System.out.println("Setting field relative commands to " + new_state);
            m_drive.getSwerve().setFieldRelativeCommands(new_state);
            m_drive.getSwerve().resetFieldOffset();
        }));
    }

    private void configurePeriodics(Robot robot) {
        robot.addPeriodic(m_drive.getSwerve()::controllerPeriodic,
                frc.robot.diffswerve.Constants.DifferentialSwerveModule.kDt, 0.0025);
    }

    public void setEnableDrive(boolean enabled) {
        System.out.println("Set drive motors to " + enabled);
        m_drive.setEnabled(enabled);
        m_drive.setCoast(!enabled);
    }

    public void disabledInit() {
        m_bridge.matchManager.sendDisableMatchPeriod();
    }

    public void autonomousInit() {
        // m_bridge.startBag();
        m_bridge.matchManager.sendAutonomousMatchPeriod();
    }

    public void teleopInit() {
        m_bridge.matchManager.sendTeleopMatchPeriod();
    }

    public void disabledPeriodic() {

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_auto_manager.getAutonomousCommand();
    }
}
