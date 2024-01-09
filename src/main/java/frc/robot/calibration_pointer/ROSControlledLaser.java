// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.calibration_pointer;

import edu.wpi.first.wpilibj2.command.Command;

public class ROSControlledLaser extends Command {
    private final CalibrationPointer calibrationPointer;

    /** Creates a new AutoFocusLaserTurret. */
    public ROSControlledLaser(CalibrationPointer calibrationPointer) {
        this.calibrationPointer = calibrationPointer;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(calibrationPointer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        calibrationPointer.setEnableROSJoints(true);
        calibrationPointer.setLaser(true);
        calibrationPointer.setToDefaultAngles();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        calibrationPointer.setLaser(false);
        calibrationPointer.setEnableROSJoints(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public boolean runsWhenDisabled() {
        return true;
    }
}
