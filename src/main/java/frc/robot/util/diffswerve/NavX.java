package frc.robot.util.diffswerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;

public class NavX {

    // The navx object that this is based on.
    private AHRS base;

    // The offset to add to the yaw
    private double offset = 0;

    /** Constructor. Uses the default port of SPI on the MXP. */
    public NavX() {
        this.base = new AHRS();
    }

    /**
     * Constructor. Uses the given AHRS object instead of constructing a new one.
     *
     * @param base The AHRS object to base this object on.
     */
    public NavX(AHRS base) {
        this.base = base;
    }

    /**
     * Construct. Uses the given SPI port.
     *
     * @param port The SPI port
     */
    public NavX(SPI.Port port) {
        this.base = new AHRS(port);
    }

    /**
     * Constructor. Uses the given I2C port.
     *
     * @param port The I2C port
     */
    public NavX(I2C.Port port) {
        this.base = new AHRS(port);
    }

    /**
     * Constructor. Uses the given serial port.
     *
     * @param port The serial port
     */
    public NavX(SerialPort.Port port) {
        this.base = new AHRS(port);
    }

    public double getRoll() {
        return this.base.getRoll();
    }

    public double getPitch() {
        return this.base.getPitch();
    }

    public double getYaw() {
        return -this.base.getYaw() + this.offset;
    }

    public double getYawRate() {
        return -this.base.getRate();
    }

    public void calibrateYaw(double yaw) {
        this.offset = yaw - this.getYaw() + this.offset;
    }
    
    public double getVelocityX() {
        return this.base.getVelocityX();
    }

    public double getVelocityY() {
        return this.base.getVelocityY();
    }

    public double getVelocityZ() {
        return this.base.getVelocityZ();
    }

    public double getAccelX() {
        return this.base.getWorldLinearAccelX();
    }

    public double getAccelY() {
        return this.base.getWorldLinearAccelY();
    }

    public double getAccelZ() {
        return this.base.getWorldLinearAccelZ();
    }

    public void reset() {
        this.base.reset();
    }

    public AHRS getBase() {
        return this.base;
    }
}
