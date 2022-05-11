// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.util.diffswerve.Constants;
import frc.robot.util.diffswerve.Helpers;
import frc.robot.util.controllers.NetworkTablesJoystick;
import frc.robot.util.controllers.XboxController;
import frc.robot.util.coprocessor.VelocityCommand;

public class SwerveJoystick extends SubsystemBase {
  public enum SwerveControllerType {
    XBOX,
    NT
  }
  private final SwerveControllerType m_type;

  private final double JOYSTICK_DEADBAND = 0.1;
  private NetworkTablesJoystick nt_gamepad;
  private XboxController xbox_gamepad;

  private VelocityCommand command = new VelocityCommand();
  
  /** Creates a new BasicJoystick. */
  public SwerveJoystick(SwerveControllerType type) {
    m_type = type;
    if (type == SwerveControllerType.NT) {
      this.nt_gamepad = new NetworkTablesJoystick();
    }
    else {
      this.xbox_gamepad = new XboxController(0);
    }
  }

  public Button getAllowRosButton() {
    if (m_type == SwerveControllerType.NT) {
      return this.nt_gamepad.getButton("RT");
    }
    else {
      return new Button(() -> this.xbox_gamepad.getRightTrigger() > 0.0);
    }
  }

  public Button getPointToCenterButton() {
    if (m_type == SwerveControllerType.NT) {
      return this.nt_gamepad.getButton("LT");
    }
    else {
      return new Button(() -> this.xbox_gamepad.getLeftTrigger() > 0.0);
    }
  }

  public double getVelocityX() {
    if (m_type == SwerveControllerType.NT) {
      return this.nt_gamepad.getX();
    }
    else {
      return this.xbox_gamepad.getLeftStickY();
    }
  }

  public double getVelocityY() {
    if (m_type == SwerveControllerType.NT) {
      return this.nt_gamepad.getY();
    }
    else {
      return -this.xbox_gamepad.getLeftStickX();
    }
  }

  public double getVelocityTheta() {
    if (m_type == SwerveControllerType.NT) {
      return this.nt_gamepad.getTheta();
    }
    else {
      return -this.xbox_gamepad.getRightStickX();
    }
  }

  @Override
  public void periodic() {
    double vx = Helpers.applyDeadband(this.getVelocityX(), JOYSTICK_DEADBAND);
    double vy = Helpers.applyDeadband(this.getVelocityY(), JOYSTICK_DEADBAND);
    double vt = Helpers.applyDeadband(this.getVelocityTheta(), JOYSTICK_DEADBAND);

    vx *= Constants.DriveTrain.MAX_CHASSIS_SPEED * 0.5;
    vy *= Constants.DriveTrain.MAX_CHASSIS_SPEED * 0.5;
    vt *= Constants.DriveTrain.MAX_CHASSIS_ANG_VEL * 0.15;

    // If magnitude of translation is in the "no-go" zone (_zero_epsilon..._min_linear_cmd),
    // set vx, vy to _min_linear_cmd with heading applied
    double trans_vel = Math.sqrt(vx * vx + vy * vy);
    if (Constants.EPSILON < Math.abs(trans_vel) && Math.abs(trans_vel) < Constants.DriveTrain.MIN_CHASSIS_SPEED)
    {
        double trans_angle = Math.atan2(vy, vx);
        vx = Constants.DriveTrain.MIN_CHASSIS_SPEED * Math.cos(trans_angle);
        vy = Constants.DriveTrain.MIN_CHASSIS_SPEED * Math.sin(trans_angle);
    }
    // If magnitude of translation is in the "zero" zone (<Constants.EPSILON),
    // Set translation velocity to zero
    //      If angular velocity is in the "no-go" zone,
    //      set vt to Constants.DriveTrain.MIN_CHASSIS_ANG_VEL with direction applied
    //      If angular velocity is in the "zero" zone,
    //      set vt to zero
    else if (Math.abs(trans_vel) < Constants.EPSILON) {
        vx = 0.0;
        vy = 0.0;
        if (Constants.EPSILON < Math.abs(vt) && Math.abs(vt) < Constants.DriveTrain.MIN_CHASSIS_ANG_VEL) {
            vt = Math.signum(vt) * Constants.DriveTrain.MIN_CHASSIS_ANG_VEL;
        }
        else if (Math.abs(vt) < Constants.EPSILON) {
            vt = 0.0;
        }
    }
    command.vx = vx;
    command.vy = vy;
    command.vt = vt;
  }

  public VelocityCommand getCommand() {
    return command;
  }
}
