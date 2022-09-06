// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveJoystick;
import frc.robot.util.coprocessor.CoprocessorBase;
import frc.robot.util.coprocessor.VelocityCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveSwerveJoystickCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_drive;
  private final SwerveJoystick m_joystick;
  private final CoprocessorBase m_coprocessor;

  /**
   * Creates a new DriveSwerveJoystickCommand.
   *
   * @param drive The subsystem used by this command.
   */
  public DriveSwerveJoystickCommand(DriveSubsystem drive, CoprocessorBase coprocessor, SwerveJoystick joystick) {
    m_drive = drive;
    m_coprocessor = coprocessor;
    m_joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    VelocityCommand command = m_joystick.getCommand();
    double speed = m_drive.getSwerve().getChassisSpeed();
    if (m_coprocessor.getLaserScanObstacles().isObstacleWithinBounds(speed)) {
      System.out.println("Obstacle detected within bounds!");
      if (m_coprocessor.getLaserScanObstacles().isDirectionAllowed(command.getHeading(), speed)) {
        m_drive.drive(command);
      } else {
        System.out.println("Velocity command doesn't move robot away from obstacle! Ignoring.");
        m_drive.stop();
      }
    } else {
      m_drive.drive(command);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
