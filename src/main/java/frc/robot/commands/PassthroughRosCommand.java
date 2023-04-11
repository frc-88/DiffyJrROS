// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.ros.bridge.BridgeSubscriber;
import frc.robot.ros.messages.geometry_msgs.Twist;
import frc.robot.subsystems.DriveSubsystem;

public class PassthroughRosCommand extends CommandBase {
    private final DriveSubsystem m_drive;
    private final BridgeSubscriber<Twist> m_twistSub;

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

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_twistSub.didUpdate()) {
            Twist msg = m_twistSub.receive();
            m_drive.drive(
                    new ChassisSpeeds(
                            msg.getLinear().getX(),
                            msg.getLinear().getY(),
                            msg.getAngular().getZ()));
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
