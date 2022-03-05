// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.NavigationSubsystem.RosAutoState;
import frc.robot.util.roswaypoints.WaypointsPlan;
import frc.robot.util.tunnel.VelocityCommand;

public class DriveWithWaypointsPlan extends CommandBase {
  private final NavigationSubsystem m_nav;
  private final DriveSubsystem m_drive;
  private WaypointsPlan m_plan;
  private long m_is_finished_timeout = 0;

  /** Creates a new DriveWithWaypointsPlan. */
  public DriveWithWaypointsPlan(NavigationSubsystem nav, DriveSubsystem drive, WaypointsPlan plan, long is_finished_timeout) {
    m_nav = nav;
    m_drive = drive;
    m_plan = plan;
    m_is_finished_timeout = is_finished_timeout;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(nav);
  }
  public DriveWithWaypointsPlan(NavigationSubsystem nav, DriveSubsystem drive, WaypointsPlan plan) {
    this(nav, drive, plan, 15_000_000);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_nav.setWaypointsPlan(m_plan, m_is_finished_timeout);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    VelocityCommand command = m_nav.getAutoCommand();
    if (Objects.isNull(command)) {
      return;
    }
    m_drive.drive(command);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_nav.cancelAutoGoal();  // Confirm whether cancelGoal should be called in all cases
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_nav.getRosAutoState() == RosAutoState.FINISHED;
  }
}
