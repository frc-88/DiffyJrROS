// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.driverinput.JoystickInterface;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LaserTurret;

public class CalibrateLaserTurret extends CommandBase {
    private final JoystickInterface m_joystick;
    private final LaserTurret m_laser_turret;
    private final double speedMultiplier = 2.0;
    private final double deadzone = 0.1;

    /** Creates a new CalibrateLaserTurret. */
    public CalibrateLaserTurret(DriveSubsystem drive, LaserTurret laser_turret, JoystickInterface joystick) {
        m_joystick = joystick;
        m_laser_turret = laser_turret;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(laser_turret, drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_laser_turret.setLaser(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        updatePositions();

    }

    private void updatePositions() {
        double panValue = speedMultiplier * m_joystick.getRightStickX();
        double tiltValue = -speedMultiplier * m_joystick.getRightStickY();

        if (Math.abs(panValue) > deadzone) {
            m_laser_turret.setPanPosition(limitServo(m_laser_turret.getPanPosition() + panValue));
        }
        if (Math.abs(tiltValue) > deadzone) {
            m_laser_turret.setTiltPosition(limitServo(m_laser_turret.getTiltPosition() + tiltValue));
        }
    }

    private double limitServo(double value) {
        return limit(value, 0, 180);
    }

    private double limit(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
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
