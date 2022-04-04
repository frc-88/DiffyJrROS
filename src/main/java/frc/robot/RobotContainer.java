// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveSwerveJoystickCommand;
import frc.robot.commands.DriveWithWaypointsPlan;
import frc.robot.commands.PassthroughRosCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.SwerveJoystick;
import frc.robot.subsystems.SwerveJoystick.SwerveControllerType;
import frc.robot.util.roswaypoints.Waypoint;
import frc.robot.util.roswaypoints.WaypointsPlan;
import frc.robot.util.coprocessortable.DiffyJrTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
    Robot.isSimulation() ? Constants.COPROCESSOR_ADDRESS_SIMULATED : Constants.COPROCESSOR_ADDRESS,
    Constants.COPROCESSOR_PORT,
    Constants.COPROCESSOR_TABLE_UPDATE_DELAY);
  private final Navigation m_nav = new Navigation(m_ros_interface);

  private final CommandBase m_joystickDriveCommand = new DriveSwerveJoystickCommand(m_drive, m_joystick);
  private final CommandBase m_passthroughRosCommand = new PassthroughRosCommand(m_drive, m_ros_interface);
  private CommandBase m_autoCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Robot robot) {
    configureDriveCommand();
    configureAutoCommand();
    configurePeriodics(robot);
  }

  private void configureDriveCommand() {
    m_drive.setDefaultCommand(m_passthroughRosCommand);
    // m_drive.setDefaultCommand(m_joystickDriveCommand);
    m_joystick.getAllowRosButton().whileHeld(m_joystickDriveCommand);
    // m_joystick.getAllowRosButton().whileHeld(m_passthroughRosCommand);
  }

  private void configureAutoCommand() {
    WaypointsPlan autoPlan = new WaypointsPlan(m_ros_interface);
    autoPlan.addWaypoint(new Waypoint("<team>_" + "_point_a"));
    autoPlan.addWaypoint(new Waypoint("<team>_" + "_point_b"));
    autoPlan.addWaypoint(new Waypoint("<team>_" + "_end"));
    autoPlan.addWaypoint(new Waypoint("powercell"));

    m_autoCommand = new SequentialCommandGroup(
      // new DriveDistanceMeters(m_drive, 0.5, 0.5),
      new DriveWithWaypointsPlan(m_nav, m_drive, autoPlan)
    );
  }

  private void configurePeriodics(Robot robot) {
    robot.addPeriodic(m_ros_interface::update, Constants.COPROCESSOR_PERIODIC_UPDATE_DELAY, Constants.COPROCESSOR_PERIODIC_UPDATE_OFFSET);
    robot.addPeriodic(m_ros_interface::updateSlow, Constants.COPROCESSOR_SLOW_PERIODIC_UPDATE_DELAY, Constants.COPROCESSOR_SLOW_PERIODIC_UPDATE_OFFSET);
    robot.addPeriodic(m_drive.getSwerve()::controllerPeriodic, frc.robot.util.diffswerve.Constants.DifferentialSwerveModule.kDt, 0.0025);
  }

  public void setEnableDrive(boolean enabled) {
    m_drive.setEnabled(enabled);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
