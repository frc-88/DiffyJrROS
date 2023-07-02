// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.autos.AutonomousManager;
import frc.robot.calibration_pointer.AutoFocusLaser;
import frc.robot.calibration_pointer.CalibrateLaser;
import frc.robot.calibration_pointer.CalibrationPointer;
import frc.robot.drive_subsystem.CoastDriveMotors;
import frc.robot.drive_subsystem.DriveSubsystem;
import frc.robot.drive_subsystem.DriveSwerveJoystickCommand;
import frc.robot.driverinput.JoystickInterface;
import frc.robot.driverinput.ROSJoystick;
import frc.robot.localization.Localization;
import frc.robot.localization.OdometryLocalization;
import frc.robot.localization.ROSLocalization;
import frc.robot.ros.DiffyJrCoprocessorBridge;
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
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();

    private final DiffyJrCoprocessorBridge bridge = new DiffyJrCoprocessorBridge(driveSubsystem);
    private final JoystickInterface joystick = new ROSJoystick(bridge.joystickSubscriber);
    private final Localization rosLocalization = new ROSLocalization(driveSubsystem, bridge.tfListenerCompact);
    private final Localization odomLocalization = new OdometryLocalization(driveSubsystem);
    private final CalibrationPointer calibrationPointer = new CalibrationPointer(SerialPort.Port.kMXP,
            bridge.jointManager);
    private final AutonomousManager autoManager = new AutonomousManager(driveSubsystem, rosLocalization,
            odomLocalization, bridge.autoPathManager);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(Robot robot) {
        System.out.println("RobotContainer initializing");
        configureCommands();
        configurePeriodics(robot);
        driveSubsystem.getSwerve().setAngleControllerEnabled(false);
        System.out.println("RobotContainer initialization complete");
    }

    private void configureCommands() {
        driveSubsystem.setDefaultCommand(
                new DriveSwerveJoystickCommand(driveSubsystem, joystick, bridge.motorEnablePublisher));

        Trigger userButton = new Trigger(() -> RobotController.getUserButton());
        userButton.whileTrue(new CoastDriveMotors(driveSubsystem));

        Trigger toggleFieldRelative = new Trigger(() -> this.joystick.isButtonBPressed());
        toggleFieldRelative.onTrue(new InstantCommand(() -> {
            boolean new_state = !driveSubsystem.getSwerve().commandsAreFieldRelative();
            System.out.println("Setting field relative commands to " + new_state);
            driveSubsystem.getSwerve().setFieldRelativeCommands(new_state);
            driveSubsystem.getSwerve().resetFieldOffset();
        }));

        Trigger togglePointer = new Trigger(() -> this.joystick.isButtonXPressed());
        // togglePointer.toggleOnTrue(new CalibrateLaser(driveSubsystem,
        // calibrationPointer, joystick,
        // bridge.pointerPublisher, bridge.tagSubscriber));
        togglePointer
                .toggleOnTrue(new AutoFocusLaser(calibrationPointer, bridge.pointerPublisher,
                        bridge.tagSubscriber));

    }

    private void configurePeriodics(Robot robot) {
        robot.addPeriodic(driveSubsystem.getSwerve()::controllerPeriodic,
                frc.robot.diffswerve.Constants.DifferentialSwerveModule.kDt, 0.0025);
    }

    public void setEnableDrive(boolean enabled) {
        System.out.println("Set drive motors to " + enabled);
        driveSubsystem.setEnabled(enabled);
        driveSubsystem.setCoast(!enabled);
    }

    public void disabledInit() {
        bridge.matchManager.sendDisableMatchPeriod();
    }

    public void autonomousInit() {
        // bridge.startBag();
        bridge.matchManager.sendAutonomousMatchPeriod();
    }

    public void teleopInit() {
        bridge.matchManager.sendTeleopMatchPeriod();
    }

    public void disabledPeriodic() {

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoManager.getAutonomousCommand();
    }
}
