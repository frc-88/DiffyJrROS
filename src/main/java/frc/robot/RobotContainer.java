// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.CoastDriveMotors;
import frc.robot.commands.DriveSwerveJoystickCommand;
import frc.robot.commands.PassthroughRosCommand;
import frc.robot.commands.SetGlobalPoseToTagGlobalPose;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveJoystick;
import frc.robot.subsystems.SwerveJoystick.SwerveControllerType;
import frc.robot.util.coprocessor.networktables.DiffyJrTable;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final SwerveJoystick m_joystick = new SwerveJoystick(SwerveControllerType.NT);
  private final DiffyJrTable m_ros_interface = new DiffyJrTable(
    m_drive.getSwerve(),
    m_drive.getImu(),
    Robot.isSimulation() ? Constants.COPROCESSOR_ADDRESS_SIMULATED : Constants.COPROCESSOR_ADDRESS,
    Constants.COPROCESSOR_PORT,
    Constants.COPROCESSOR_TABLE_UPDATE_DELAY);

  private final CommandBase m_joystickDriveCommand = new DriveSwerveJoystickCommand(m_drive, m_ros_interface, m_joystick);
  private final CommandBase m_passthroughRosCommand = new PassthroughRosCommand(m_drive, m_ros_interface);
  private Trigger userButton = new Trigger(() -> RobotController.getUserButton());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
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
    //   m_drive, m_ros_interface, m_joystick,
    //   frc.robot.util.diffswerve.Constants.DriveTrain.MAX_CHASSIS_ANG_VEL * 0.75)
    // );
    userButton.whileTrue(new CoastDriveMotors(m_drive));
    // userButton.whileTrue(new TestDiffSwerveMotors(m_drive));
  }

  private void configurePeriodics(Robot robot) {
    robot.addPeriodic(m_ros_interface::update, Constants.COPROCESSOR_PERIODIC_UPDATE_DELAY, Constants.COPROCESSOR_PERIODIC_UPDATE_OFFSET);
    robot.addPeriodic(m_ros_interface::updateSlow, Constants.COPROCESSOR_SLOW_PERIODIC_UPDATE_DELAY, Constants.COPROCESSOR_SLOW_PERIODIC_UPDATE_OFFSET);
    // robot.addPeriodic(m_ros_interface::updateModules, 1.0 / 15.0, 0.03);
    robot.addPeriodic(m_drive.getSwerve()::controllerPeriodic, frc.robot.util.diffswerve.Constants.DifferentialSwerveModule.kDt, 0.0025);
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
    return new SetGlobalPoseToTagGlobalPose(m_ros_interface);
  }
}
