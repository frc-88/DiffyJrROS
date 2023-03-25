// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.coprocessor.networktables.DiffyJrTable;

public class SetGlobalPoseToTagGlobalPose extends CommandBase {
  private final DiffyJrTable m_coprocessor;
  private boolean is_set = false;

  /** Creates a new SetGlobalPoseToTagGlobalPose. */
  public SetGlobalPoseToTagGlobalPose(DiffyJrTable coprocessor) {
    m_coprocessor = coprocessor;
    // // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    is_set = false;

  }

  @Override
  public void execute() {
    Pose2d pose = m_coprocessor.getTagGlobalPose();
    if (m_coprocessor.isPoseValid(pose) && m_coprocessor.isGlobalPoseActive()) {
      m_coprocessor.sendPoseEstimate(pose);
      is_set = true;
      System.out.println("Set pose to tag global pose");
    } else {
      System.out.println("Warning: global tag pose is not valid!");
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
