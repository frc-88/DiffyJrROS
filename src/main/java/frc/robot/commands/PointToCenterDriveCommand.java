// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveJoystick;
import frc.robot.util.coprocessor.VelocityCommand;
import frc.robot.util.coprocessor.serial.DiffyJrSerial;
import frc.robot.util.coprocessor.networktables.DiffyJrTable;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class PointToCenterDriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_drive;
  // private final DiffyJrSerial m_ros_interface;
  private final DiffyJrTable m_ros_interface;
  private final SwerveJoystick m_joystick;
  private double m_normalizeError = 1.0;
  private double m_maxAngVel = 1.0;
  private final PIDController controller;

  /**
   * Creates a new PointToCenterDriveCommand.
   *
   * @param drive The subsystem used by this command.
   */
  public PointToCenterDriveCommand(DriveSubsystem drive, DiffyJrTable ros_interface, SwerveJoystick joystick, double maxAngVel) {
  // public PointToCenterDriveCommand(DriveSubsystem drive, DiffyJrSerial ros_interface, SwerveJoystick joystick, double maxAngVel) {
    m_drive = drive;
    m_ros_interface = ros_interface;
    m_joystick = joystick;
    m_maxAngVel = maxAngVel;
    controller = new PIDController(3.5, 0.0, 0.3);
    controller.setSetpoint(0.0);

    double error = getAngleError(Math.PI);
    if (error > 0.0) {
      m_normalizeError = 1.0 / error;
    }
    // else {
    //   System.out.println("Angle error function returned a value: " + error);
    // }

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private double getAngleError(double targetAngle) {
    return -Math.atan(targetAngle * 2.0) * m_normalizeError;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    VelocityCommand joyCommand = m_joystick.getCommand();
    VelocityCommand driveCommand = new VelocityCommand();

    if (m_ros_interface.isTargetValid()) {
      double targetAngle = m_ros_interface.getTargetAngle();
      double error = getAngleError(targetAngle);
      driveCommand.vt = controller.calculate(error);
      if (Math.abs(driveCommand.vt) > m_maxAngVel) {
        driveCommand.vt = Math.copySign(m_maxAngVel, driveCommand.vt);
      }
    }
    else {
      System.out.println("Can't find target! Not pointing to center!");
      driveCommand.vt = joyCommand.vt;
    }

    driveCommand.vx = joyCommand.vx;
    driveCommand.vy = joyCommand.vy;

    m_drive.drive(driveCommand);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
