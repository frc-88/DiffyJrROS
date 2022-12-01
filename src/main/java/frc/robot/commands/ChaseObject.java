// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.coprocessor.GameObject;
import frc.robot.util.coprocessor.VelocityCommand;
import frc.robot.util.coprocessor.ZoneInfo;
import frc.robot.util.coprocessor.networktables.CoprocessorTable;

public class ChaseObject extends CommandBase {
  private DriveSubsystem m_drive;
  private CoprocessorTable m_coprocessor;
  private String m_objectName = "";
  private PIDController m_controller;
  private double m_maxVelocity = 2.0;
  private double m_maxAngularVelocity = 6.0;
  private double m_goalDistance = Double.NaN;
  private double m_distanceThreshold = 1.0;
  private double k_distanceRamp = 2.0;
  private double kP = 2.3;
  private double kD = 0.1;
  private double kI = 0.01;
  private double m_noGoRange = 0.5;
  private final double kReverseFanRadians = Units.degreesToRadians(180.0);  // Range of angles to accept as a valid reverse command

  /** Creates a new ChaseObject. */
  public ChaseObject(
      DriveSubsystem drive, CoprocessorTable coprocessor, String objectName) {
    m_drive = drive;
    m_coprocessor = coprocessor;
    m_objectName = objectName;

    m_controller = new PIDController(kP, kI, kD);
    m_controller.setSetpoint(0.0);

    SmartDashboard.putNumber("chase_kP", kP);
    SmartDashboard.putNumber("chase_kI", kI);
    SmartDashboard.putNumber("chase_kD", kD);
    SmartDashboard.putNumber("chase_maxVelocity", m_maxVelocity);
    SmartDashboard.putNumber("chase_maxAngularVelocity", m_maxAngularVelocity);
    SmartDashboard.putNumber("chase_distanceThreshold", m_distanceThreshold);
    SmartDashboard.putNumber("chase_noGoRange", m_noGoRange);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.setP(SmartDashboard.getNumber("chase_kP", kP));
    m_controller.setI(SmartDashboard.getNumber("chase_kI", kI));
    m_controller.setD(SmartDashboard.getNumber("chase_kD", kD));
    m_maxVelocity = SmartDashboard.getNumber("chase_maxVelocity", m_maxVelocity);
    m_maxAngularVelocity = SmartDashboard.getNumber("chase_maxAngularVelocity", m_maxAngularVelocity);
    m_distanceThreshold = SmartDashboard.getNumber("chase_distanceThreshold", m_distanceThreshold);
    m_noGoRange = SmartDashboard.getNumber("chase_noGoRange", m_noGoRange);

    m_controller.reset();
    m_goalDistance = Double.NaN;
    System.out.println("Chasing " + m_objectName);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_coprocessor.areZonesValid()) {
      m_drive.stop();
      System.out.println("Coprocessor zones are not valid. Doing nothing for chase object.");
      return;
    }

    ZoneInfo zone = m_coprocessor.getNearestNoGoZone();
    if (zone.getIsInside()) {
      m_drive.stop();
      System.out.println("Robot is inside no-go zone!");
      return;
    }

    GameObject gameObject = m_coprocessor.getNearestGameObject(m_objectName);
    if (!gameObject.isValid()) {
      m_drive.stop();
      System.out.println(String.format("Requested game object %s is not available. Doing nothing for chase object.", m_objectName));
      return;
    }
    
    Pose2d relativeGoalPose = gameObject.getPose();

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

    if (!m_coprocessor.isDirectionTowardNoGoZonesAllowed(driveCommand.getHeading(), kReverseFanRadians, m_noGoRange)) {
      System.out.println("Velocity command doesn't move robot away from no go zone! Stopping motors.");
      m_drive.stop();
    }
    else {
      m_drive.drive(driveCommand);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Finished chasing " + m_objectName);
    m_drive.getSwerve().stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !Double.isNaN(m_goalDistance) && Math.abs(m_goalDistance) < m_distanceThreshold;
  }
}
