package frc.robot.util.coprocessor.detections;

public class Position {
    public double x = 0.0;
    public double y = 0.0;
    public double z = 0.0;

    public Position()  {  }

    public Position(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Position invert() {
        return new Position(
            -x, -y, -z
        );
    }

    public Position translateBy(Position other) {
        return new Position(x + other.x, y + other.y, z + other.z);
    }

    public Position rotateBy(Orientation orientation) {
        Orientation vector = new Orientation(0.0, x, y, z);
        Orientation rotated = orientation.rotateBy(vector).rotateBy(orientation.invert());
        return new Position(
            rotated.x,
            rotated.y,
            rotated.z
        );
    }
}
