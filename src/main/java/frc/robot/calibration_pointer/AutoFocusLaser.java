// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.calibration_pointer;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ros.bridge.PointerPublisher;
import frc.robot.ros.bridge.TagSubscriber;
import frc.robot.ros.messages.apriltag_ros.AprilTagDetectionArray;
import frc.team88.ros.messages.geometry_msgs.PointStamped;

public class AutoFocusLaser extends CommandBase {
    private final CalibrationPointer calibrationPointer;
    private final PointerPublisher pointerPublisher;
    private final TagSubscriber tagSubscriber;

    /** Creates a new AutoFocusLaserTurret. */
    public AutoFocusLaser(CalibrationPointer calibrationPointer, PointerPublisher pointerPublisher,
            TagSubscriber tagSubscriber) {
        this.calibrationPointer = calibrationPointer;
        this.pointerPublisher = pointerPublisher;
        this.tagSubscriber = tagSubscriber;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(calibrationPointer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        calibrationPointer.setEnableROSJoints(true);
        calibrationPointer.setLaser(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        updatePositions();
    }

    private void updatePositions() {
        Optional<AprilTagDetectionArray> detections = tagSubscriber.receive();
        if (!detections.isEmpty()) {
            Optional<PointStamped> goal = AutoPointAtTag.compute(detections.get());
            if (!goal.isEmpty()) {
                pointerPublisher.publishGoal(goal.get());
            }
        }
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
