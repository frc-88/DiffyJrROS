// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.coprocessor.tunnel.CoprocessorBase;

public class PassthroughRosCommand extends CommandBase {
  private final DriveSubsystem m_drive;
  private final CoprocessorBase m_coprocessor;
  /** Creates a new PassthroughRosCommand. */
  public PassthroughRosCommand(DriveSubsystem drive, CoprocessorBase coprocessor) {
    m_drive = drive;
    m_coprocessor = coprocessor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_coprocessor.isCommandActive()) {
      m_drive.drive(m_coprocessor.getCommand());
    }
    else {
      m_drive.stop();
    }
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
