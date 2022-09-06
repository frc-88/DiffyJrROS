// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.coprocessor.GameObject;
import frc.robot.util.coprocessor.VelocityCommand;
import frc.robot.util.coprocessor.networktables.CoprocessorTable;

public class ChaseObject extends CommandBase {
  private DriveSubsystem m_drive;
  private CoprocessorTable m_coprocessor;
  private String m_objectName = "";
  private PIDController m_controller;
  private double m_maxVelocity = 1.0;
  private double m_maxAngularVelocity = 1.0;
  private double m_goalDistance = Double.NaN;
  private double m_distanceThreshold = 1.0;
  private double k_distanceRamp = 2.0;
  private double kP = 2.0;
  private double kD = 0.0;
  private double kI = 0.01;

  /** Creates a new ChaseObject. */
  public ChaseObject(
      DriveSubsystem drive, CoprocessorTable coprocessor,
      String objectName,
      double maxVelocityMetersPerSecond, double maxAngularVelocityRadiansPerSecond,
      double distanceThresholdMeters) {
    m_drive = drive;
    m_coprocessor = coprocessor;
    m_objectName = objectName;
    m_maxVelocity = maxVelocityMetersPerSecond;
    m_maxAngularVelocity = maxAngularVelocityRadiansPerSecond;
    m_distanceThreshold = distanceThresholdMeters;

    m_controller = new PIDController(kP, kI, kD);
    m_controller.setSetpoint(0.0);

    SmartDashboard.putNumber("chase_kP", kP);
    SmartDashboard.putNumber("chase_kI", kI);
    SmartDashboard.putNumber("chase_kD", kD);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.setP(SmartDashboard.getNumber("chase_kP", kP));
    m_controller.setI(SmartDashboard.getNumber("chase_kI", kI));
    m_controller.setD(SmartDashboard.getNumber("chase_kD", kD));
    m_controller.reset();
    m_goalDistance = Double.NaN;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    GameObject gameObject = m_coprocessor.getNearestGameObject(m_objectName);
    if (!gameObject.isValid()) {
      m_drive.stop();
      System.out.println(String.format("Requested game object %s is not available", m_objectName));
      return;
    }
    
    Pose2d relativeGoalPose = gameObject.get();

    m_goalDistance = relativeGoalPose.getX();  // ignoring Y component since we don't directly control it here
    double error = new Rotation2d(relativeGoalPose.getX(), relativeGoalPose.getY()).getRadians();
    error *= -1.0;
    
    VelocityCommand driveCommand = new VelocityCommand();
    driveCommand.vt = m_controller.calculate(error);
    if (Math.abs(driveCommand.vt) > m_maxAngularVelocity) {
      driveCommand.vt = Math.copySign(m_maxAngularVelocity, driveCommand.vt);
    }
    
    driveCommand.vx = m_goalDistance * k_distanceRamp;
    if (Math.abs(driveCommand.vx) > m_maxVelocity) {
      driveCommand.vx = Math.copySign(m_maxVelocity, driveCommand.vx);
    }

    driveCommand.vy = 0.0;

    m_drive.drive(driveCommand);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.getSwerve().stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !Double.isNaN(m_goalDistance) && Math.abs(m_goalDistance) < m_distanceThreshold;
  }
}
