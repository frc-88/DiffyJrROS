package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveJoystick;
import frc.team88.ros.bridge.BridgeSubscriber;
import frc.team88.ros.messages.geometry_msgs.Twist;

public class SwitchableJoystickCommand extends CommandBase {
    public enum JoystickType {
        ROS,
        LOCAL
    };

    private final PassthroughRosCommand ros_command;
    private final DriveSwerveJoystickCommand local_command;
    private final Supplier<JoystickType> joystickTypeSupplier;

    public SwitchableJoystickCommand(DriveSubsystem drive, BridgeSubscriber<Twist> twistSub, SwerveJoystick joystick,
            Supplier<JoystickType> joystickTypeSupplier) {
        this.ros_command = new PassthroughRosCommand(drive, twistSub);
        this.local_command = new DriveSwerveJoystickCommand(drive, joystick);
        this.joystickTypeSupplier = joystickTypeSupplier;
        addRequirements(drive, joystick);
    }

    @Override
    public void initialize() {
        for (JoystickType type : JoystickType.values()) {
            getCommand(type).initialize();
        }
    }

    @Override
    public void execute() {
        getActiveCommand().execute();
    }

    public CommandBase getCommand(JoystickType type) {
        switch (type) {
            case ROS:
                return this.ros_command;
            case LOCAL:
                return this.local_command;
            default:
                DriverStation.reportError("Invalid joystick type supplied. Defaulting to ROS", false);
                return this.ros_command;
        }
    }

    public CommandBase getActiveCommand() {
        return getCommand(this.joystickTypeSupplier.get());
    }

    @Override
    public void end(boolean interrupted) {
        getActiveCommand().end(interrupted);
    }
}
