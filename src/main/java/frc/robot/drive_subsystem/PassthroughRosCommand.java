// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive_subsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team88.ros.bridge.BridgeSubscriber;
import frc.team88.ros.conversions.ROSConversions;
import frc.team88.ros.messages.geometry_msgs.Twist;

public class PassthroughRosCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final BridgeSubscriber<Twist> twistSub;
    private long prevTime = 0;
    private final long TIMEOUT = 30_000_000;
    private ChassisSpeeds cached = new ChassisSpeeds();

    /** Creates a new PassthroughRosCommand. */
    public PassthroughRosCommand(DriveSubsystem driveSubsystem, BridgeSubscriber<Twist> twistSub) {
        this.driveSubsystem = driveSubsystem;
        this.twistSub = twistSub;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
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
        if ((msg = twistSub.receive()) != null) {
            cached = ROSConversions.rosToWpiTwist(msg);
            prevTime = now;
        }
        if (getTime() - prevTime < TIMEOUT) {
            driveSubsystem.drive(cached);
        } else {
            driveSubsystem.stop();
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
