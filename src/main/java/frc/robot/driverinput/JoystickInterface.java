package frc.robot.driverinput;

public interface JoystickInterface {
    public boolean isButtonAPressed();

    public boolean isButtonBPressed();

    public boolean isButtonXPressed();

    public boolean isButtonYPressed();

    public boolean isButtonLeftBumperPressed();

    public boolean isButtonRightBumperPressed();

    public boolean isButtonBackPressed();

    public boolean isButtonStartPressed();

    public boolean isButtonLeftStickPressed();

    public boolean isButtonRightStickPressed();

    public double getRightStickY();

    public double getRightStickX();

    public double getLeftStickY();

    public double getLeftStickX();

    public double getLeftTrigger();

    public double getRightTrigger();

    public double getLeftZ();

    public double getRightZ();

    public double getDpadVertical();

    public double getDpadHorizontal();

}
