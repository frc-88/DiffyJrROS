package frc.robot.driverinput;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ros.bridge.JoystickSubscriber;
import frc.team88.ros.messages.sensor_msgs.Joy;

public class ROSJoystick extends SubsystemBase implements JoystickInterface {
    private enum ButtonMapping {
        A(0),
        B(1),
        X(2),
        Y(3),
        L(4),
        R(5),
        BACK(6),
        START(7),
        HOME(8),
        L_STICK(9),
        R_STICK(10);

        public final int index;

        private ButtonMapping(int index) {
            this.index = index;
        }
    }

    private enum AxisMapping {
        L_X(0),
        L_Y(1),
        R_X(3),
        R_Y(4),
        B_L(2),
        B_R(5),
        DPAD_VERT(7),
        DPAD_HORZ(6);

        public final int index;

        private AxisMapping(int index) {
            this.index = index;
        }
    }

    private JoystickSubscriber joySub;
    private Joy msg = new Joy();

    public ROSJoystick(JoystickSubscriber joySub) {
        this.joySub = joySub;
    }

    @Override
    public void periodic() {
        Optional<Joy> new_msg = joySub.receive();
        if (!new_msg.isEmpty()) {
            msg = new_msg.get();
        }
    }

    private boolean getButtonIndex(ButtonMapping button) {
        ArrayList<java.lang.Integer> buttons = msg.getButtons();
        if (buttons.size() <= button.index) {
            System.err.println("Button index out of range: " + button.index);
            return false;
        } else {
            return buttons.get(button.index) == 1;
        }
    }

    private double getAxisIndex(AxisMapping axis) {
        ArrayList<java.lang.Float> axes = msg.getAxes();
        if (axes.size() <= axis.index) {
            System.err.println("Axis index out of range: " + axis.index);
            return 0.0;
        } else {
            return axes.get(axis.index);
        }
    }

    @Override
    public boolean isButtonAPressed() {
        return getButtonIndex(ButtonMapping.A);
    }

    @Override
    public boolean isButtonBPressed() {
        return getButtonIndex(ButtonMapping.B);
    }

    @Override
    public boolean isButtonXPressed() {
        return getButtonIndex(ButtonMapping.X);
    }

    @Override
    public boolean isButtonYPressed() {
        return getButtonIndex(ButtonMapping.Y);
    }

    @Override
    public boolean isButtonLeftBumperPressed() {
        return getButtonIndex(ButtonMapping.L);
    }

    @Override
    public boolean isButtonRightBumperPressed() {
        return getButtonIndex(ButtonMapping.R);
    }

    @Override
    public boolean isButtonBackPressed() {
        return getButtonIndex(ButtonMapping.BACK);
    }

    @Override
    public boolean isButtonStartPressed() {
        return getButtonIndex(ButtonMapping.START);
    }

    @Override
    public boolean isButtonLeftStickPressed() {
        return getButtonIndex(ButtonMapping.L_STICK);
    }

    @Override
    public boolean isButtonRightStickPressed() {
        return getButtonIndex(ButtonMapping.R_STICK);
    }

    @Override
    public double getRightStickY() {
        return getAxisIndex(AxisMapping.R_Y);
    }

    @Override
    public double getRightStickX() {
        return getAxisIndex(AxisMapping.R_X);
    }

    @Override
    public double getLeftStickY() {
        return getAxisIndex(AxisMapping.L_Y);
    }

    @Override
    public double getLeftStickX() {
        return getAxisIndex(AxisMapping.L_X);
    }

    @Override
    public double getLeftTrigger() {
        return getAxisIndex(AxisMapping.B_L);
    }

    @Override
    public double getRightTrigger() {
        return getAxisIndex(AxisMapping.B_R);
    }

    @Override
    public double getLeftZ() {
        return 0.0;
    }

    @Override
    public double getRightZ() {
        return 0.0;
    }

    @Override
    public double getDpadVertical() {
        return getAxisIndex(AxisMapping.DPAD_VERT);
    }

    @Override
    public double getDpadHorizontal() {
        return getAxisIndex(AxisMapping.DPAD_HORZ);
    }
}
