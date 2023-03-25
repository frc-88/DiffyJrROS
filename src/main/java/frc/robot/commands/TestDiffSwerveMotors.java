// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.diffswerve.DiffSwerveModule;

public class TestDiffSwerveMotors extends CommandBase {
  private final DriveSubsystem m_drive;
  private long startTime = 0;

  private ArrayList<Double> loCommands = new ArrayList<>();
  private ArrayList<Double> hiCommands = new ArrayList<>();
  private long commandInterval = 100_000;
  private int currentIndex = 0;

  /** Creates a new TestDiffSwerveMotors. */
  public TestDiffSwerveMotors(DriveSubsystem drive) {
    m_drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);

    long segment_length = 2_000_000;
    addRamp(loCommands, segment_length, 0.0, 12.0);
    addRamp(hiCommands, segment_length, 0.0, 0.0);

    addRamp(loCommands, segment_length, 12.0, 12.0);
    addRamp(hiCommands, segment_length, 0.0, 12.0);

    addRamp(loCommands, segment_length, 12.0, -12.0);
    addRamp(hiCommands, segment_length, 12.0, 12.0);

    addRamp(loCommands, segment_length, -12.0, -12.0);
    addRamp(hiCommands, segment_length, 12.0, -12.0);

    addRamp(loCommands, segment_length, -12.0, 12.0);
    addRamp(hiCommands, segment_length, -12.0, -12.0);

    addRamp(loCommands, segment_length, 12.0, 12.0);
    addRamp(hiCommands, segment_length, -12.0, 12.0);

    addRamp(loCommands, segment_length, 12.0, 0.0);
    addRamp(hiCommands, segment_length, 12.0, 0.0);
  }

  private void addRamp(ArrayList<Double> commands, long duration, double startVoltage, double stopVoltage) {
    int max_index = timeToIndex(duration);
    for (int index = 0; index < max_index; index++) {
      double voltage = (stopVoltage - startVoltage) / max_index * index + startVoltage;
      commands.add(voltage);
    }
  }

  private int timeToIndex(long time) {
    return (int) (time / commandInterval);
  }

  private long getTime() {
    return RobotController.getFPGATime();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = getTime();
    currentIndex = 0;
    m_drive.setEnabled(false);
  }

  private double getVoltage(ArrayList<Double> commands, int index) {
    if (index >= commands.size()) {
      return commands.get(commands.size() - 1);
    } else {
      return commands.get(index);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (int module_index = 0; module_index < m_drive.getSwerve().getNumModules(); module_index++) {
      DiffSwerveModule module = m_drive.getSwerve().getModule(module_index);
      currentIndex = timeToIndex(getTime() - startTime);
      module.setHiMotorVoltage(getVoltage(hiCommands, currentIndex));
      module.setLoMotorVoltage(getVoltage(loCommands, currentIndex));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
    m_drive.setEnabled(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentIndex >= hiCommands.size() && currentIndex >= loCommands.size();
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
}
