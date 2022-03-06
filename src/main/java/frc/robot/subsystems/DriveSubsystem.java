// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.diffswerve.DiffSwerveChassis;
import frc.robot.util.coprocessortable.VelocityCommand;

public class DriveSubsystem extends SubsystemBase {
  private DiffSwerveChassis swerve;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    swerve = new DiffSwerveChassis();
  }

  @Override
  public void periodic() {
    swerve.periodic();
  }

  public DiffSwerveChassis getSwerve() {
    return swerve;
  }

  public void setEnabled(boolean enabled) {
    swerve.setEnabled(enabled);
  }

  public void drive(VelocityCommand command) {
    swerve.drive(command);
  }

  public void stop() {
    swerve.stop();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
