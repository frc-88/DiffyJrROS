// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive_subsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CoastDriveMotors extends CommandBase {
    private final DriveSubsystem driveSubsystem;

    /** Creates a new CoastDriveMotors. */
    public CoastDriveMotors(DriveSubsystem drive) {
        driveSubsystem = drive;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveSubsystem.stop();
        driveSubsystem.setCoast(true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setCoast(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
