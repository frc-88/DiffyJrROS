// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.CoastDriveMotors;
import frc.robot.commands.DriveSwerveJoystickCommand;
import frc.robot.commands.PassthroughRosCommand;
import frc.robot.ros.bridge.BridgePublisher;
import frc.robot.ros.bridge.BridgeSubscriber;
import frc.robot.ros.bridge.ROSNetworkTablesBridge;
import frc.robot.ros.messages.geometry_msgs.Twist;
import frc.robot.ros.messages.nav_msgs.Odometry;
import frc.robot.subsystems.DiffyJrCoprocessorBridge;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveJoystick;
import frc.robot.subsystems.SwerveJoystick.SwerveControllerType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
  private final SwerveJoystick m_joystick = new SwerveJoystick(SwerveControllerType.NT);
  private final DiffyJrCoprocessorBridge m_bridge = new DiffyJrCoprocessorBridge(m_drive);

  private final CommandBase m_joystickDriveCommand = new DriveSwerveJoystickCommand(m_drive, m_joystick);
  private final CommandBase m_passthroughRosCommand = new PassthroughRosCommand(m_drive, m_bridge.getTwistSub());
  private Trigger userButton = new Trigger(() -> RobotController.getUserButton());

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Robot robot) {
    System.out.println("RobotContainer initializing");
    configureDriveCommand();
    configurePeriodics(robot);
    m_drive.getSwerve().setAngleControllerEnabled(false);
    System.out.println("RobotContainer initialization complete");
  }

  private void configureDriveCommand() {
    m_drive.setDefaultCommand(m_passthroughRosCommand);
    // m_drive.setDefaultCommand(m_joystickDriveCommand);
    m_joystick.getLeftTriggerButton().whileTrue(m_joystickDriveCommand);
    // m_joystick.getRightTriggerButton().whileTrue(m_passthroughRosCommand);
    // m_joystick.getLeftTriggerButton().whileTrue(new PointToCenterDriveCommand(
    // m_drive, m_ros_interface, m_joystick,
    // frc.robot.util.diffswerve.Constants.DriveTrain.MAX_CHASSIS_ANG_VEL * 0.75)
    // );
    userButton.whileTrue(new CoastDriveMotors(m_drive));
    // userButton.whileTrue(new TestDiffSwerveMotors(m_drive));
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

  public void disabledPeriodic() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new WaitCommand(15.0);
  }
}
