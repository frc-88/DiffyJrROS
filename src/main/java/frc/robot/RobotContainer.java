// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.CoastDriveMotors;
import frc.robot.commands.DriveSwerveJoystickCommand;
import frc.robot.commands.DriveToPowercell;
import frc.robot.commands.DriveWithWaypointsPlan;
import frc.robot.commands.PassthroughRosCommand;
import frc.robot.commands.PointToCenterDriveCommand;
import frc.robot.commands.TestDiffSwerveMotors;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.SwerveJoystick;
import frc.robot.subsystems.SwerveJoystick.SwerveControllerType;
import frc.robot.util.roswaypoints.Waypoint;
import frc.robot.util.roswaypoints.WaypointsPlan;
import frc.robot.util.sensors.Limelight;
// import frc.robot.util.coprocessor.networktables.DiffyJrTable;
import frc.robot.util.coprocessor.serial.DiffyJrSerial;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final SwerveJoystick m_joystick = new SwerveJoystick(SwerveControllerType.XBOX);
  // private final DiffyJrTable m_ros_interface = new DiffyJrTable(
  //   m_drive.getSwerve(),
  //   Robot.isSimulation() ? Constants.COPROCESSOR_ADDRESS_SIMULATED : Constants.COPROCESSOR_ADDRESS,
  //   Constants.COPROCESSOR_PORT,
  //   Constants.COPROCESSOR_TABLE_UPDATE_DELAY);
  private final DiffyJrSerial m_ros_interface = new DiffyJrSerial(
    m_drive.getSwerve(),
    m_drive.getImu()
  );
  private final Navigation m_nav = new Navigation(m_ros_interface);
  private final Limelight m_limelight = new Limelight();

  private final CommandBase m_joystickDriveCommand = new DriveSwerveJoystickCommand(m_drive, m_joystick);
  private final CommandBase m_passthroughRosCommand = new PassthroughRosCommand(m_drive, m_ros_interface);
  private CommandBase m_autoCommand;
  private Button userButton = new Button(() -> RobotController.getUserButton());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Robot robot) {
    configureDriveCommand();
    m_autoCommand = configureAutoCommand();
    configurePeriodics(robot);
    m_limelight.ledOff();
  }

  private void configureDriveCommand() {
    m_drive.setDefaultCommand(m_passthroughRosCommand);
    // m_drive.setDefaultCommand(m_joystickDriveCommand);
    m_joystick.getAllowRosButton().whileHeld(m_joystickDriveCommand);
    // m_joystick.getAllowRosButton().whileHeld(m_passthroughRosCommand);
    m_joystick.getPointToCenterButton().whileHeld(new PointToCenterDriveCommand(
      m_drive, m_ros_interface, m_joystick,
      frc.robot.util.diffswerve.Constants.DriveTrain.MAX_CHASSIS_ANG_VEL * 0.75)
    );
    // userButton.whileHeld(new CoastDriveMotors(m_drive));
    userButton.whileHeld(new TestDiffSwerveMotors(m_drive));
  }

  private CommandBase configureAutoCommand() {
    WaypointsPlan autoPlan = new WaypointsPlan(m_ros_interface);
    autoPlan.addWaypoint(new Waypoint("<team>_" + "_point_a"));
    autoPlan.addWaypoint(new Waypoint("<team>_" + "_point_b"));
    autoPlan.addWaypoint(new Waypoint("<team>_" + "_end"));
    autoPlan.addWaypoint(new Waypoint("powercell"));

    CommandBase auto1 = new SequentialCommandGroup(
      // new DriveDistanceMeters(m_drive, 0.5, 0.5),
      new DriveWithWaypointsPlan(m_nav, m_drive, autoPlan)
    );
    CommandBase auto2 = new DriveToPowercell(m_nav, m_drive);

    // return auto1;
    return auto2;
  }

  private void configurePeriodics(Robot robot) {
    robot.addPeriodic(m_ros_interface::update, Constants.COPROCESSOR_PERIODIC_UPDATE_DELAY, Constants.COPROCESSOR_PERIODIC_UPDATE_OFFSET);
    robot.addPeriodic(m_ros_interface::updateSlow, Constants.COPROCESSOR_SLOW_PERIODIC_UPDATE_DELAY, Constants.COPROCESSOR_SLOW_PERIODIC_UPDATE_OFFSET);
    robot.addPeriodic(m_drive.getSwerve()::controllerPeriodic, frc.robot.util.diffswerve.Constants.DifferentialSwerveModule.kDt, 0.0025);
  }

  public void setEnableDrive(boolean enabled) {
    System.out.println("Set drive motors to " + enabled);
    m_drive.setEnabled(enabled);
    m_limelight.ledOff();
    m_drive.setCoast(!enabled);
  }
  
  public void disabledPeriodic() {
    m_limelight.ledOff();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}
