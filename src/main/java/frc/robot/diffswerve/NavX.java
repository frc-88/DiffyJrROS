package frc.robot.diffswerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;

public class NavX {

    // The navx object that this is based on.
    private AHRS base;
    private final double GRAVITY = 9.81;

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
        return Units.degreesToRadians(this.base.getRoll());
    }

    public double getPitch() {
        return Units.degreesToRadians(this.base.getPitch());
    }

    public double getYaw() {
        return Units.degreesToRadians(-this.base.getYaw());
    }

    public double getYawRate() {
        return Units.degreesToRadians(-this.base.getRate());
    }

    public double getAccelX() {
        return this.base.getWorldLinearAccelX() * GRAVITY;
    }

    public double getAccelY() {
        return this.base.getWorldLinearAccelY() * GRAVITY;
    }

    public double getAccelZ() {
        return this.base.getWorldLinearAccelZ() * GRAVITY;
    }

    public void reset() {
        this.base.reset();
    }

    public AHRS getBase() {
        return this.base;
    }

    // public FrcImu getMessage() {
    //     return new FrcImu(
    //         getRoll(),
    //         getPitch(),
    //         getYaw(),
    //         getYawRate(),
    //         getAccelX(),
    //         getAccelY(),
    //         getAccelZ()
    //     );
    // }

}
