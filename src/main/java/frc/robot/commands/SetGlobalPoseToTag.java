// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Navigation;
import frc.robot.util.coprocessor.GameObject;

public class SetGlobalPoseToTag extends CommandBase {
  private final Navigation m_nav;
  private final String m_waypointName;
  private final String m_gameObjectName;
  /** Creates a new SetGlobalPoseToTag. */
  public SetGlobalPoseToTag(Navigation nav, String gameObjectName, String waypointName) {
    m_nav = nav;
    m_waypointName = waypointName;
    m_gameObjectName = gameObjectName;
    // addRequirements(m_nav);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    GameObject gameObjectPose = m_nav.getCoprocessor().getNearestGameObject(m_gameObjectName);
    if (!gameObjectPose.isValid()) {
      System.out.println("Warning: " + m_gameObjectName + " is not a valid game object");
    }
    Pose2d resetPose = m_nav.getWaypointMap().getPoseRelativeToWaypoint(m_waypointName, gameObjectPose.getPose());
    if (m_nav.isPoseValid(resetPose)) {
      m_nav.setPoseEstimate(resetPose);
    }
    else {
      System.out.println("Warning: " + m_waypointName + " is not a valid waypoint");
    }
  }

  public boolean isFinished() {
    return true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
