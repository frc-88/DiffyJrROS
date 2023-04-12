// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team88.ros.bridge.BridgeSubscriber;
import frc.team88.ros.conversions.ROSConversions;
import frc.team88.ros.messages.geometry_msgs.Twist;
import frc.robot.subsystems.DriveSubsystem;

public class PassthroughRosCommand extends CommandBase {
    private final DriveSubsystem m_drive;
    private final BridgeSubscriber<Twist> m_twistSub;
    private long prevTime = 0;
    private final long TIMEOUT = 250_000;
    private ChassisSpeeds cached = new ChassisSpeeds();

    /** Creates a new PassthroughRosCommand. */
    public PassthroughRosCommand(DriveSubsystem drive, BridgeSubscriber<Twist> twistSub) {
        m_drive = drive;
        m_twistSub = twistSub;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    private long getTime() {
        return RobotController.getFPGATime();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Twist msg;
        long now = getTime();
        if ((msg = m_twistSub.receive()) != null) {
            cached = ROSConversions.rosToWpiTwist(msg);
            prevTime = now;
        }
        if (getTime() - prevTime < TIMEOUT) {
            m_drive.drive(cached);
        } else {
            m_drive.stop();
        }

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
}
