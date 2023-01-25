package frc.robot.util.coprocessor.detections;

public class Orientation {
    public double w = 1.0;
    public double x = 0.0;
    public double y = 0.0;
    public double z = 0.0;

    public Orientation() {  }

    public Orientation(
        double w,
        double x,
        double y,
        double z
    ) {
        this.w = w;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public static Orientation fromEuler(double x, double y, double z) {
        double cr = Math.cos(x * 0.5);
        double sr = Math.sin(x * 0.5);
        double cp = Math.cos(y * 0.5);
        double sp = Math.sin(y * 0.5);
        double cy = Math.cos(z * 0.5);
        double sy = Math.sin(z * 0.5);

        Orientation q = new Orientation();
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;

        return q;
    }

    public EulerAngles asEuler() {
        EulerAngles angles = new EulerAngles();
        double sinr_cosp = 2.0 * (w * x + y * z);
        double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        angles.x = Math.atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = Math.sqrt(1.0 + 2.0 * (w * y - x * z));
        double cosp = Math.sqrt(1.0 - 2.0 * (w * y - x * z));
        angles.y = 2 * Math.atan2(sinp, cosp) - Math.PI / 2.0;

        // yaw (z-axis rotation)
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        angles.z = Math.atan2(siny_cosp, cosy_cosp);

        return angles;
    }
    
    public Orientation invert() {
        return new Orientation(
            w, -x, -y, -z
        );
    }

    public Orientation rotateBy(Orientation other) {
        return new Orientation(
            w * other.w - x * other.x - y * other.y - z * other.z,
            w * other.x + x * other.w + y * other.z - z * other.y,
            w * other.y + y * other.w + z * other.x - x * other.z,
            w * other.z + z * other.w + x * other.y - y * other.x
        );
    }
}
