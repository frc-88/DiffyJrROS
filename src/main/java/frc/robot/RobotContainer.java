// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ChaseObject;
import frc.robot.commands.CoastDriveMotors;
import frc.robot.commands.DriveSwerveJoystickCommand;
import frc.robot.commands.DriveWithWaypointsPlan;
import frc.robot.commands.PassthroughRosCommand;
// import frc.robot.commands.PointToCenterDriveCommand;
// import frc.robot.commands.TestDiffSwerveMotors;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.SwerveJoystick;
import frc.robot.subsystems.SwerveJoystick.SwerveControllerType;
import frc.robot.util.sensors.Limelight;
import frc.robot.util.coprocessor.networktables.DiffyJrTable;
import frc.robot.util.coprocessor.roswaypoints.Waypoint;
import frc.robot.util.coprocessor.roswaypoints.WaypointsPlan;
// import frc.robot.util.coprocessor.serial.DiffyJrSerial;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private final DiffyJrTable m_ros_interface = new DiffyJrTable(
    m_drive.getSwerve(),
    m_drive.getImu(),
    Robot.isSimulation() ? Constants.COPROCESSOR_ADDRESS_SIMULATED : Constants.COPROCESSOR_ADDRESS,
    Constants.COPROCESSOR_PORT,
    Constants.COPROCESSOR_TABLE_UPDATE_DELAY);
  // private final DiffyJrSerial m_ros_interface = new DiffyJrSerial(
  //   m_drive.getSwerve(),
  //   m_drive.getImu()
  // );
  private final Navigation m_nav = new Navigation(m_ros_interface);
  private final Limelight m_limelight = new Limelight();

  private final CommandBase m_joystickDriveCommand = new DriveSwerveJoystickCommand(m_drive, m_ros_interface, m_joystick);
  private final CommandBase m_passthroughRosCommand = new PassthroughRosCommand(m_drive, m_ros_interface);
  private Button userButton = new Button(() -> RobotController.getUserButton());

  private final CommandBase staticAutoCommand = configureStaticAutoCommand();
  private final CommandBase chaseAutoCommand = configureChaseAutoCommand();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Robot robot) {
    configureDriveCommand();
    configurePeriodics(robot);
    m_limelight.ledOff();
    m_drive.getSwerve().setAngleControllerEnabled(false);
    SmartDashboard.putString("Auto", "chase");
  }

  private void configureDriveCommand() {
    // m_drive.setDefaultCommand(m_passthroughRosCommand);
    m_drive.setDefaultCommand(m_joystickDriveCommand);
    // m_joystick.getAllowRosButton().whileHeld(m_joystickDriveCommand);
    m_joystick.getRightTriggerButton().whileHeld(m_passthroughRosCommand);
    // m_joystick.getLeftTriggerButton().whileHeld(new PointToCenterDriveCommand(
    //   m_drive, m_ros_interface, m_joystick,
    //   frc.robot.util.diffswerve.Constants.DriveTrain.MAX_CHASSIS_ANG_VEL * 0.75)
    // );
    m_joystick.getLeftTriggerButton().whileHeld(
      new ChaseObject(m_drive, m_ros_interface, "cargo_<team>")
    );
    userButton.whileHeld(new CoastDriveMotors(m_drive));
    // userButton.whileHeld(new TestDiffSwerveMotors(m_drive));
  }

  private CommandBase configureStaticAutoCommand() {
    WaypointsPlan autoPlan = new WaypointsPlan(m_ros_interface);
    autoPlan.addWaypoint(new Waypoint("<team>" + "_a"));
    autoPlan.addWaypoint(new Waypoint("<team>_1"));
    autoPlan.addWaypoint(new Waypoint("<team>_2").makeContinuous(true));
    autoPlan.addWaypoint(new Waypoint("<team>_3").makeContinuous(true));
    autoPlan.addWaypoint(new Waypoint("<team>_4").makeIgnoreOrientation(false));

    CommandBase staticAuto = new SequentialCommandGroup(
      // new DriveDistanceMeters(m_drive, 0.5, 0.5),
      new InstantCommand(() -> m_ros_interface.setNoGoZones(new String[] {"<!team>"})),
      new DriveWithWaypointsPlan(m_nav, m_drive, autoPlan),
      new ChaseObject(m_drive, m_ros_interface, "power_cell")
    );
    
    return staticAuto;
  }

  private CommandBase configureChaseAutoCommand() {
    return new ChaseObject(m_drive, m_ros_interface, "cargo_<team>");
  }

  private void configurePeriodics(Robot robot) {
    robot.addPeriodic(m_ros_interface::update, Constants.COPROCESSOR_PERIODIC_UPDATE_DELAY, Constants.COPROCESSOR_PERIODIC_UPDATE_OFFSET);
    // robot.addPeriodic(m_ros_interface::updateModules, 1.0 / 15.0, 0.03);
    robot.addPeriodic(m_ros_interface::updateSlow, Constants.COPROCESSOR_SLOW_PERIODIC_UPDATE_DELAY, Constants.COPROCESSOR_SLOW_PERIODIC_UPDATE_OFFSET);
    robot.addPeriodic(m_drive.getSwerve()::controllerPeriodic, frc.robot.util.diffswerve.Constants.DifferentialSwerveModule.kDt, 0.0025);
  }

  public void setEnableDrive(boolean enabled) {
    System.out.println("Set drive motors to " + enabled);
    m_ros_interface.setNoGoZones(new String[] {"<!team>"});
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
    String autoName = SmartDashboard.getString("Auto", "chase");
    if (autoName.equals("static")) {
      return staticAutoCommand;
    }
    else if (autoName.equals("chase")) {
      return chaseAutoCommand;
    }
    else {
      return new WaitCommand(1);
    }
  }
}
