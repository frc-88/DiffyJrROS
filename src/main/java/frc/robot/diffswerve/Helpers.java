package frc.robot.diffswerve;

public class Helpers {
    public static double limit(double value, double limitValLow, double limitValHigh) {
        if (value > limitValHigh) {
            return limitValHigh;
        }
        if (value < limitValLow) {
            return limitValLow;
        }
        return value;
    }

    /**
     * sets angle between -PI and PI.
     *
     * @param angle current to be changed.
     * @param radians determines if angle is radians or not.
     * @return changed angle.
     */
    public static double boundHalfAngle(double angle) {
        angle %= 2.0 * Math.PI;
        if (angle >= Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        if (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }
}
