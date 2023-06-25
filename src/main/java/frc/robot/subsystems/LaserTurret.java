package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaserTurret extends SubsystemBase {
    private final Servo panServo;
    private final Servo tiltServo;
    private final DigitalOutput laser;

    public LaserTurret(int pan_channel, int tilt_channel, int laser_channel) {
        panServo = new Servo(pan_channel);
        tiltServo = new Servo(tilt_channel);
        laser = new DigitalOutput(laser_channel);
    }

    private void setRaw(double pan, double tilt) {
        panServo.set(pan);
        tiltServo.set(tilt);
    }

    private void setLaser(double value) {
        // laser.setRaw((int) (value * 255));
        laser.set(value > 0.5);
    }

    private long prevTime = 0;
    private double value = 0.0;

    @Override
    public void periodic() {
        long now = RobotController.getFPGATime();
        if (now - prevTime > 250_000) {
            prevTime = now;
            value = (value + 0.1) % 1.0;
            System.out.println(String.format("Setting laser to %f", value));
            setRaw(value, value);
            setLaser(value);
        }
    }
}
