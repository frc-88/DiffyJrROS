// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.calibration_pointer;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive_subsystem.DriveSubsystem;
import frc.robot.driverinput.JoystickInterface;
import frc.robot.ros.bridge.PointerPublisher;
import frc.robot.ros.bridge.TagSubscriber;
import frc.robot.ros.messages.apriltag_ros.AprilTagDetectionArray;
import frc.team88.ros.messages.geometry_msgs.PointStamped;

public class CalibrateLaser extends CommandBase {
    private final JoystickInterface joystick;
    private final CalibrationPointer calibrationPointer;
    private final double speedMultiplier = 2.0;
    private final double deadzone = 0.1;
    private final PointerPublisher pointerPublisher;
    private final TagSubscriber tagSubscriber;

    private boolean prevAButton = false;
    private boolean prevYButton = false;

    /** Creates a new CalibrateLaserTurret. */
    public CalibrateLaser(DriveSubsystem drive, CalibrationPointer calibrationPointer, JoystickInterface joystick,
            PointerPublisher pointerPublisher, TagSubscriber tagSubscriber) {
        this.joystick = joystick;
        this.calibrationPointer = calibrationPointer;
        this.pointerPublisher = pointerPublisher;
        this.tagSubscriber = tagSubscriber;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(calibrationPointer, drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        calibrationPointer.setEnableROSJoints(false);
        calibrationPointer.setLaser(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        updatePositions();
        checkRecordData();
        checkLaserToggle();
    }

    private void checkRecordData() {
        boolean isAPressed = joystick.isButtonAPressed();
        if (isAPressed && prevAButton != isAPressed) {
            pointerPublisher.recordCalibration();
        }
        prevAButton = isAPressed;
    }

    private void checkLaserToggle() {
        boolean isYPressed = joystick.isButtonYPressed();
        if (isYPressed && prevYButton != isYPressed) {
            calibrationPointer.setLaser(!calibrationPointer.isLaserOn());
        }
        prevYButton = isYPressed;
    }

    private void updatePositions() {
        double panValue = speedMultiplier * joystick.getRightStickX();
        double tiltValue = -speedMultiplier * joystick.getRightStickY();

        if (Math.abs(panValue) > deadzone) {
            calibrationPointer.setPanAngle(limitServo(calibrationPointer.getPanValue() + panValue));
        }
        if (Math.abs(tiltValue) > deadzone) {
            calibrationPointer.setTiltAngle(limitServo(calibrationPointer.getTiltValue() + tiltValue));
        }

        Optional<AprilTagDetectionArray> detections = tagSubscriber.receive();
        if (!detections.isEmpty()) {
            Optional<PointStamped> goal = AutoPointAtTag.compute(detections.get());
            if (!goal.isEmpty()) {
                pointerPublisher.publishGoal(goal.get());
            }
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
        calibrationPointer.setEnableROSJoints(true);
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
