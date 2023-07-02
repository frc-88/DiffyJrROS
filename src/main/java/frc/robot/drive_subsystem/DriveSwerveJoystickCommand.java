// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive_subsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.diffswerve.Constants;
import frc.robot.driverinput.Helpers;
import frc.robot.driverinput.JoystickInterface;
import frc.robot.ros.bridge.MotorEnablePublisher;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveSwerveJoystickCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem driveSubsystem;
    private final JoystickInterface joystick;
    private final MotorEnablePublisher motorEnablePublisher;
    private final double JOYSTICK_DEADBAND = 0.1;
    private final double[] LINEAR_MULTIPLIERS = { 0.1, 0.25, 0.5, 0.9, 1.0 };
    private final double[] ANGULAR_MULTIPLIERS = { 0.1, 0.125, 0.15, 0.2, 1.0 };
    private int speed_selector = 0;
    private int prevDpadState = 0;

    /**
     * Creates a new DriveSwerveJoystickCommand.
     *
     * @param driveSubsystem The subsystem used by this command.
     */
    public DriveSwerveJoystickCommand(DriveSubsystem driveSubsystem, JoystickInterface joystick,
            MotorEnablePublisher motorEnablePublisher) {
        this.driveSubsystem = driveSubsystem;
        this.joystick = joystick;
        this.motorEnablePublisher = motorEnablePublisher;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    private void setSpeedMode(int index) {
        speed_selector = Math.max(Math.min(index, LINEAR_MULTIPLIERS.length - 1), 0);
    }

    private double getLinearMultiplier() {
        return LINEAR_MULTIPLIERS[speed_selector];
    }

    private double getAngularMultiplier() {
        return ANGULAR_MULTIPLIERS[speed_selector];
    }

    public double getVelocityX() {
        return joystick.getLeftStickY();
    }

    public double getVelocityY() {
        return joystick.getLeftStickX();
    }

    public double getVelocityTheta() {
        return joystick.getRightStickX();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        updateMotorEnable();
        updateSpeedMode();
        driveSubsystem.drive(getChassisSpeeds());
    }

    private void updateMotorEnable() {
        if (joystick.isButtonLeftBumperPressed() && joystick.isButtonStartPressed()) {
            motorEnablePublisher.setMotorEnable(true);
        } else if (joystick.isButtonLeftBumperPressed() || joystick.isButtonRightBumperPressed()) {
            motorEnablePublisher.setMotorEnable(false);
        }
    }

    private void updateSpeedMode() {
        int dpadState = getDpadState();
        if (dpadState != prevDpadState) {
            if (dpadState == 1) {
                setSpeedMode(speed_selector + 1);
            } else if (dpadState == -1) {
                setSpeedMode(speed_selector - 1);
            }
        }
        prevDpadState = dpadState;
    }

    private int getDpadState() {
        if (joystick.getDpadVertical() > 0.5) {
            return 1;
        } else if (joystick.getDpadVertical() < -0.5) {
            return -1;
        } else {
            return 0;
        }
    }

    private ChassisSpeeds getChassisSpeeds() {
        double vx = Helpers.applyDeadband(this.getVelocityX(), JOYSTICK_DEADBAND);
        double vy = Helpers.applyDeadband(this.getVelocityY(), JOYSTICK_DEADBAND);
        double vt = Helpers.applyDeadband(this.getVelocityTheta(), JOYSTICK_DEADBAND);

        vx *= Constants.DriveTrain.MAX_CHASSIS_SPEED * getLinearMultiplier();
        vy *= Constants.DriveTrain.MAX_CHASSIS_SPEED * getLinearMultiplier();
        vt *= Constants.DriveTrain.MAX_CHASSIS_ANG_VEL * getAngularMultiplier();

        // If magnitude of translation is in the "no-go" zone
        // (_zero_epsilon..._min_linear_cmd),
        // set vx, vy to _min_linear_cmd with heading applied
        double trans_vel = Math.sqrt(vx * vx + vy * vy);
        if (Constants.EPSILON < Math.abs(trans_vel) && Math.abs(trans_vel) < Constants.DriveTrain.MIN_CHASSIS_SPEED) {
            double trans_angle = Math.atan2(vy, vx);
            vx = Constants.DriveTrain.MIN_CHASSIS_SPEED * Math.cos(trans_angle);
            vy = Constants.DriveTrain.MIN_CHASSIS_SPEED * Math.sin(trans_angle);
        }
        // If magnitude of translation is in the "zero" zone (<Constants.EPSILON),
        // Set translation velocity to zero
        // If angular velocity is in the "no-go" zone,
        // set vt to Constants.DriveTrain.MIN_CHASSIS_ANG_VEL with direction applied
        // If angular velocity is in the "zero" zone,
        // set vt to zero
        else if (Math.abs(trans_vel) < Constants.EPSILON) {
            vx = 0.0;
            vy = 0.0;
            if (Constants.EPSILON < Math.abs(vt) && Math.abs(vt) < Constants.DriveTrain.MIN_CHASSIS_ANG_VEL) {
                vt = Math.signum(vt) * Constants.DriveTrain.MIN_CHASSIS_ANG_VEL;
            } else if (Math.abs(vt) < Constants.EPSILON) {
                vt = 0.0;
            }
        }
        return new ChassisSpeeds(vx, vy, vt);
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

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
