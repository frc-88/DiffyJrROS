package frc.robot.util.controllers;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Button;

public class NetworkTablesJoystick {
    private NetworkTable table;
    private NetworkTable axis_table;
    private NetworkTable button_table;
    
    Map<String, Button> buttons = new HashMap<>();

    public NetworkTablesJoystick() {
        table = NetworkTableInstance.getDefault().getTable("joystick");
        axis_table = table.getSubTable("axis");
        button_table = table.getSubTable("button");
    }
    public double getX() {
        return axis_table.getEntry("x").getDouble(0.0);
    }
    public double getY() {
        return axis_table.getEntry("y").getDouble(0.0);
    }
    public double getTheta() {
        return axis_table.getEntry("theta").getDouble(0.0);
    }

    public boolean getButtonValue(String name) {
        return button_table.getEntry(name).getBoolean(false);
    }

    public Button getButton(String name) {
        if (!buttons.containsKey(name)) {
            buttons.put(name, new Button(() -> this.getButtonValue(name)));
        }
        return buttons.get(name);
    }
}
