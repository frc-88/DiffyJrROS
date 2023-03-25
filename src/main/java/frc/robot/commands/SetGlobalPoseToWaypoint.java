// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.coprocessor.CoprocessorBase;

public class SetGlobalPoseToWaypoint extends CommandBase {
  private final CoprocessorBase m_coprocessor;
  private final String m_waypointName;
  private boolean is_set = false;

  /** Creates a new SetGlobalPoseToWaypoint. */
  public SetGlobalPoseToWaypoint(CoprocessorBase coprocessor, String waypointName) {
    m_coprocessor = coprocessor;
    m_waypointName = waypointName;
    // addRequirements(m_nav);
    // // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    is_set = false;

  }

  @Override
  public void execute() {
    Pose2d pose = m_coprocessor.getWaypoint(m_waypointName);
    if (m_coprocessor.isPoseValid(pose)) {
      m_coprocessor.sendPoseEstimate(pose);
      is_set = true;
      System.out.println("Set pose to waypoint " + m_waypointName);
    } else {
      System.out.println("Warning: " + m_waypointName + " is not a valid waypoint name");
    }
  }

  @Override
  public boolean isFinished() {
    return is_set;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
