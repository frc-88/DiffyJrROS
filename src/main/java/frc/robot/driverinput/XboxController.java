package frc.robot.driverinput;

import edu.wpi.first.wpilibj.Joystick;

public class XboxController extends Joystick implements JoystickInterface {
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

    public XboxController(int port) {
        super(port);
    }

    public boolean isButtonAPressed() {
        return this.getRawButton(BUTTON_A);
    }

    public boolean isButtonBPressed() {
        return this.getRawButton(BUTTON_B);
    }

    public boolean isButtonXPressed() {
        return this.getRawButton(BUTTON_X);
    }

    public boolean isButtonYPressed() {
        return this.getRawButton(BUTTON_Y);
    }

    public boolean isButtonLeftBumperPressed() {
        return this.getRawButton(BUTTON_LEFT_BUMPER);
    }

    public boolean isButtonRightBumperPressed() {
        return this.getRawButton(BUTTON_RIGHT_BUMPER);
    }

    public boolean isButtonBackPressed() {
        return this.getRawButton(BUTTON_BACK);
    }

    public boolean isButtonStartPressed() {
        return this.getRawButton(BUTTON_START);
    }

    public boolean isButtonLeftStickPressed() {
        return this.getRawButton(BUTTON_LEFT_STICK);
    }

    public boolean isButtonRightStickPressed() {
        return this.getRawButton(BUTTON_RIGHT_STICK);
    }

    public double getRightStickY() {
        return -this.getRawAxis(RIGHT_VERT_AXIS);
    }

    public double getRightStickX() {
        return this.getRawAxis(RIGHT_HORIZ_AXIS);
    }

    public double getLeftStickY() {
        return -this.getRawAxis(LEFT_VERT_AXIS);
    }

    public double getLeftStickX() {
        return this.getRawAxis(LEFT_HORIZ_AXIS);
    }

    public double getLeftTrigger() {
        return this.getRawAxis(LEFT_Z_AXIS);
    }

    public double getRightTrigger() {
        return this.getRawAxis(RIGHT_Z_AXIS);
    }

    public double getLeftZ() {
        return this.getRawAxis(LEFT_Z_AXIS);
    }

    public double getRightZ() {
        return this.getRawAxis(RIGHT_Z_AXIS);
    }

    public double getDpadVertical() {
        return 0.0; // TODO
    }

    public double getDpadHorizontal() {
        return 0.0; // TODO
    }

    public void rumble(double rumble) {
        this.setRumble(RumbleType.kLeftRumble, rumble);
        this.setRumble(RumbleType.kRightRumble, rumble);
    }

    public void rumbleLeft(double rumble) {
        this.setRumble(RumbleType.kLeftRumble, rumble);
    }

    public void rumbleRight(double rumble) {
        this.setRumble(RumbleType.kRightRumble, rumble);
    }

}
