package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaserTurret extends SubsystemBase {
    private final Servo panServo;
    private final Servo tiltServo;

    public LaserTurret(int pan_channel, int tilt_channel) {
        panServo = new Servo(pan_channel);
        tiltServo = new Servo(tilt_channel);
    }

    private void setRaw(double pan, double tilt) {
        panServo.set(pan);
        tiltServo.set(tilt);
    }
}
