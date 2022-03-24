// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.joystick;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.util.diffswerve.Constants;
import frc.robot.util.diffswerve.Helpers;
import frc.robot.util.coprocessortable.VelocityCommand;

public class SwerveDriverStationJoystick extends SubsystemBase implements SwerveJoystick {
  private final double JOYSTICK_DEADBAND = 0.1;
  private final Joystick gamepad;

  private static final int LEFT_HORIZ_AXIS = 0;
  private static final int LEFT_VERT_AXIS = 1;
  private static final int RIGHT_Z_AXIS = 3;
  private static final int LEFT_Z_AXIS = 2;
  private static final int RIGHT_HORIZ_AXIS = 4;
  private static final int RIGHT_VERT_AXIS = 5;
  private static final int BUTTON_A = 5;
  private static final int BUTTON_B = 2;
  private static final int BUTTON_X = 3;
  private static final int BUTTON_Y = 4;
  private static final int BUTTON_LEFT_BUMPER = 5;
  private static final int BUTTON_RIGHT_BUMPER = 6;
  private static final int BUTTON_BACK = 7;
  private static final int BUTTON_START = 8;
  private static final int BUTTON_LEFT_STICK = 9;
  private static final int BUTTON_RIGHT_STICK = 10;

  private VelocityCommand command = new VelocityCommand();
  
  /** Creates a new BasicJoystick. */
  public SwerveDriverStationJoystick(int port) {
    this.gamepad = new Joystick(port);
  }

  public Button getToggleCommandSourceButton() {
    return new JoystickButton(this.gamepad, BUTTON_RIGHT_BUMPER);
  }

  public double getX() {
    return -this.gamepad.getRawAxis(LEFT_VERT_AXIS);
  }

  public double getY() {
    return this.gamepad.getRawAxis(LEFT_HORIZ_AXIS);
  }

  public double getTheta() {
    return this.gamepad.getRawAxis(RIGHT_HORIZ_AXIS);
  }

  @Override
  public void periodic() {
    double vx = Helpers.applyDeadband(this.getX(), JOYSTICK_DEADBAND);
    double vy = Helpers.applyDeadband(this.getY(), JOYSTICK_DEADBAND);
    double vt = Helpers.applyDeadband(this.getTheta(), JOYSTICK_DEADBAND);

    vx *= Constants.DriveTrain.MAX_CHASSIS_SPEED * 0.75;
    vy *= Constants.DriveTrain.MAX_CHASSIS_SPEED * 0.75;
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
