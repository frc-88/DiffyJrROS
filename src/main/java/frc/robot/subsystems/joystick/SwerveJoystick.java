package frc.robot.subsystems.joystick;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.util.coprocessortable.VelocityCommand;

public interface SwerveJoystick {
    public Button getToggleCommandSourceButton();
    public double getX();
    public double getY();
    public double getTheta();
    public VelocityCommand getCommand();
}
